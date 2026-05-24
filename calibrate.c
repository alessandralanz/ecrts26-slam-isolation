/* calibrate.c: measure OV2SLAM per phase timing for dynamic controller
 *
 * gcc -O2 -Wall -o calibrate calibrate.c -lrt
 * sudo ./calibrate -p <slam_pid> -l /path/to/libov2slam.so -c 0,1 -s 10 -o calib.txt
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <stdint.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <linux/perf_event.h>

// ring buffer size (power of 2 pages)
#define RING_BUF_PAGES 8
#define PAGE_SIZE 4096
#define RING_BUF_SIZE (RING_BUF_PAGES * PAGE_SIZE)
#define MAX_CPUS 4
#define MAX_FDS 64

// memory barriers to prevent reordering
#ifdef __aarch64__
#define rmb() __asm__ __volatile__("dmb ishld" ::: "memory")
#define mb()  __asm__ __volatile__("dmb ish" ::: "memory")
#else
#define rmb() __asm__ __volatile__("" ::: "memory")
#define mb()  __asm__ __volatile__("" ::: "memory")
#endif

// uprobe points in ov2slam pipeline
// ordered by execution
enum checkpoint {
    CP_VT = 0, // visual tracking (frame entry)
    CP_PREPROC, // preprocessing image
    CP_KLT, // klt tracking (optical flow)
    CP_POSE, // compute pose (pose estimation)
    CP_DONE, // frame done (frame completed)
    CP_COUNT // for array sizing
};

// uprobe point names
static const char *cp_names[] = {
    "visualTracking", "preprocessImage", "kltTracking", "computePose", "frameDone"
};

// byte offsets inside libov2slam.so (from nm -D)
static unsigned long default_offsets[CP_COUNT] = {
    0x5d000, 0x58ae0, 0x58dc0, 0x5a8d0, 0x56750
};

// what we're accumulating per phase
struct phase_stats {
    double total_time_ms; // sum of elapsed times from VT 
    uint64_t total_insns; // sum of instructions retired from VT
    int count; // how many samples
};

// one per uprobe fd
struct probe_fd {
    int fd; // perf_event fd
    struct perf_event_mmap_page *meta; // first page of the mmap'd region, need for figuring out where the kernel has written up to and where we have read up to (used in drain())
    char *ring;
    size_t mmap_size; // how many bytes we need to unmap at the end
    enum checkpoint cp;
    int cpu;
};

// keyboard interrupt sets this to zero and main loop exits
static volatile int g_running = 1;
static void handle_signal(int sig) { (void)sig; g_running = 0; }

// monotonic clock 
static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static long sys_perf_event_open(struct perf_event_attr *a, pid_t pid, int cpu, int grp, unsigned long flags) {
    return syscall(SYS_perf_event_open, a, pid, cpu, grp, flags);
}

// read uprobe PMU dynamic type number from sysfs
// need for when we fill in the attribute type in perf event open call to specify that it's a uprobe event
static int read_uprobe_type(void) {
    int t; FILE *f = fopen("/sys/bus/event_source/devices/uprobe/type","r");
    if (!f) return -1;
    if (fscanf(f,"%d",&t)!=1) { 
        fclose(f); 
        return -1; 
    }
    fclose(f); return t;
}

// sum instruction counters across all of SLAM's CPUs
static uint64_t read_total_insns(int *fds, int n) {
    uint64_t total = 0;
    for (int i = 0; i < n; i++) {
        if (fds[i] >= 0) {
            uint64_t v = 0;
            if (read(fds[i], &v, sizeof(v)) == (ssize_t)sizeof(v))
                total += v;
        }
    }
    return total;
}

// convert the command line string into an int array to figure out the core assignment
static int parse_cpus(const char *str, int *cpus, int max) {
    int n = 0;
    char buf[64];
    strncpy(buf, str, sizeof(buf)-1); buf[sizeof(buf)-1] = 0;
    char *tok = strtok(buf, ",");
    while (tok && n < max) { cpus[n++] = atoi(tok); tok = strtok(NULL, ","); }
    return n;
}

// install one uprobe on one CPU
// put a breakpoint at specified offset in library and give a sample record every time it fires
static int setup_uprobe_on_cpu(struct probe_fd *p, int uprobe_type, const char *lib_path, unsigned long offset, enum checkpoint cp, int cpu) {
    struct perf_event_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.size = sizeof(attr);
    attr.type = uprobe_type; // uprobe PMU type from sysfs
    attr.config1 = (__u64)(unsigned long)lib_path;
    attr.config2 = offset; // byte offset within the ELF binary
    attr.sample_period = 1; // fire on every single call
    attr.sample_type = PERF_SAMPLE_TID | PERF_SAMPLE_TIME; // record pid + timestamp
    attr.disabled = 1; // start disabled and then enable batch later
    attr.wakeup_events = 1; // wake poll() on every event

    // pid = -1, cpu = cpu means system-wide on this core
    int fd = sys_perf_event_open(&attr, -1, cpu, -1, PERF_FLAG_FD_CLOEXEC);
    if (fd < 0) return -1;

    // mmap the ring buffer (1 metadata page + RING_PAGES data pages)
    size_t mmap_size = (1 + RING_BUF_PAGES) * PAGE_SIZE;
    void *base = mmap(NULL, mmap_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    if (base == MAP_FAILED) { 
        close(fd); 
        return -1; 
    }

    p->fd = fd;
    p->meta = (struct perf_event_mmap_page *)base; // metadata page at the start
    p->ring = (char *)base + PAGE_SIZE; // data ring starts one page in
    p->mmap_size = mmap_size;
    p->cp = cp;
    p->cpu = cpu;
    return 0;
}

// drain all pending events from one probe's ring buffer
// returns how many events matched our target PID
static int drain_probe(struct probe_fd *p, pid_t target_pid, uint64_t *time_out) {
    // read data_head (where the kernel has written up to)
    // read barrier to make sure we see the data the kernel wrote
    // walk records from tail to head
    // write barrier and update data_tail (telling the kernel we ate them)
    struct perf_event_mmap_page *meta = p->meta;
    __sync_synchronize(); // fence
    uint64_t head = meta->data_head; // read the kernel's write pointer
    rmb(); // read fence, can't read anything else until our read is complete and all data is written before data_head was upated is visible to us
    uint64_t tail = meta->data_tail; // read the last position
    if (head == tail) return 0; // nothing new

    int matched = 0;
    while (tail < head) {
        // tail wraps around the physical ring buffer
        uint64_t off = tail % RING_BUF_SIZE;
        struct perf_event_header *ehdr = (struct perf_event_header *)(p->ring + off);
        if (ehdr->type == PERF_RECORD_SAMPLE) {
            // only realy care if the PID matches
            char *ptr = (char *)ehdr + sizeof(*ehdr);
            uint32_t pid = *(uint32_t *)ptr; ptr += 4;
            ptr += 4;
            uint64_t ts = *(uint64_t *)ptr;
            if ((pid_t)pid == target_pid) {
                *time_out = ts; matched++;
            }
        }
        // move to the next record
        tail += ehdr->size; // different record types can have a different sizes and each record holds its own size
    }
    mb(); // fence, make sure all reads from the ring buffer are complete before we update data_tail
    meta->data_tail = tail; // ack
    return matched;
}

int main(int argc, char **argv) {
    pid_t slam_pid = -1;
    int skip_frames = 10;
    char *lib_path = NULL, *outfile = NULL, *cpu_str = "0,1";

    int c;
    while ((c = getopt(argc, argv, "p:l:c:s:o:h")) != -1) {
        switch (c) {
        case 'p': slam_pid = atoi(optarg); break;
        case 'l': lib_path = optarg; break;
        case 'c': cpu_str = optarg; break;
        case 's': skip_frames = atoi(optarg); break;
        case 'o': outfile = optarg; break;
        case 'h':
            printf("Usage: %s -p <slam_pid> -l <lib> [opts]\n", argv[0]);
            printf("  -c  SLAM CPU cores [0,1]\n");
            printf("  -s  frames to skip [10]\n");
            printf("  -o  output calib file [stdout]\n");
            return 0;
        }
    }

    if (slam_pid <= 0 || !lib_path) {
        fprintf(stderr, "need -p <slam_pid> -l <lib_path>\n");
        return 1;
    }
    if (kill(slam_pid, 0) != 0) {
        fprintf(stderr, "ERROR: pid %d not found\n", slam_pid);
        return 1;
    }

    int cpus[MAX_CPUS], n_cpus;
    n_cpus = parse_cpus(cpu_str, cpus, MAX_CPUS);

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // instruction counters
    int insn_fds[MAX_CPUS];
    struct perf_event_attr hw_attr;
    memset(&hw_attr, 0, sizeof(hw_attr));
    hw_attr.size = sizeof(hw_attr);
    hw_attr.type = PERF_TYPE_HARDWARE;
    hw_attr.config = PERF_COUNT_HW_INSTRUCTIONS;
    hw_attr.disabled = 0;
    hw_attr.exclude_kernel = 1;
    hw_attr.exclude_hv = 1;

    for (int i = 0; i < n_cpus; i++) {
        insn_fds[i] = sys_perf_event_open(&hw_attr, -1, cpus[i], -1, 0);
        if (insn_fds[i] < 0)
            fprintf(stderr, "WARNING: insn counter cpu%d failed (%s)\n", cpus[i], strerror(errno));
    }

    // install uprobes
    int uprobe_type = read_uprobe_type();
    if (uprobe_type < 0) {
        fprintf(stderr, "ERROR: uprobe PMU not available\n"); return 1;
    }

    struct probe_fd all_probes[CP_COUNT * MAX_CPUS];
    int n_fds = 0;

    for (int cp = 0; cp < CP_COUNT; cp++) {
        int ok = 0;
        for (int ci = 0; ci < n_cpus; ci++) {
            if (setup_uprobe_on_cpu(&all_probes[n_fds], uprobe_type, lib_path,
                    default_offsets[cp], cp, cpus[ci]) == 0) {
                n_fds++; ok++;
            }
        }
        fprintf(stderr, "  %s (0x%lx): %d/%d CPUs\n", cp_names[cp], default_offsets[cp], ok, n_cpus);
    }
    if (n_fds == 0) {
        fprintf(stderr, "ERROR: no probes installed\n"); return 1;
    }

    // enable all probes at the same time
    for (int i = 0; i < n_fds; i++)
        ioctl(all_probes[i].fd, PERF_EVENT_IOC_ENABLE, 0);

    fprintf(stderr, "profiling entire run (skipping first %d frames)\n", skip_frames);

    // event loop
    struct pollfd pfds[MAX_FDS];
    for (int i = 0; i < n_fds; i++) {
        pfds[i].fd = all_probes[i].fd;
        pfds[i].events = POLLIN;
    }

    struct phase_stats stats[CP_COUNT];
    memset(stats, 0, sizeof(stats));

    int total_frames = 0;
    double vt_time = 0; // when the current frame started
    uint64_t vt_insns = 0; // insn count when the current frame started

    while (g_running) {
        int ret = poll(pfds, n_fds, 500);
        if (ret < 0) { 
            if (errno == EINTR) continue; 
            break; 
        }
        if (ret == 0) {
            if (kill(slam_pid, 0) != 0 && errno == ESRCH) {
                fprintf(stderr, "slam exited\n"); break;
            }
            continue;
        }

        double now = now_ms();
        uint64_t cur_insns = read_total_insns(insn_fds, n_cpus);

        for (int i = 0; i < n_fds; i++) {
            if (!(pfds[i].revents & POLLIN)) continue;

            uint64_t ts;
            int hits = drain_probe(&all_probes[i], slam_pid, &ts);
            if (hits <= 0) continue;

            enum checkpoint cp = all_probes[i].cp;

            // VT = new frames starting
            if (cp == CP_VT) {
                total_frames++;
                vt_time = now;
                vt_insns = cur_insns;
                continue;
            }

            // only accumulate after skip period
            if (total_frames <= skip_frames) continue;

            // get the time and instructions since VT
            if (cp > CP_VT && cp <= CP_DONE) {
                double dt = now - vt_time;
                uint64_t di = cur_insns - vt_insns;
                stats[cp].total_time_ms += dt;
                stats[cp].total_insns += di;
                stats[cp].count++;
            }

            // progress every 200 frames
            if (cp == CP_DONE && total_frames % 200 == 0) {
                int measured = total_frames - skip_frames;
                fprintf(stderr, "  %d frames profiled...\n", measured);
            }
        }
    }

    // cleanup probes and counters
    for (int i = 0; i < n_fds; i++) {
        ioctl(all_probes[i].fd, PERF_EVENT_IOC_DISABLE, 0);
        munmap(all_probes[i].meta, all_probes[i].mmap_size);
        close(all_probes[i].fd);
    }
    for (int i = 0; i < n_cpus; i++)
        if (insn_fds[i] >= 0) close(insn_fds[i]);

    int measured = total_frames - skip_frames;
    if (measured < 0) measured = 0;

    fprintf(stderr, "\n=== calibration results (%d frames profiled) ===\n", measured);

    FILE *out = stdout;
    if (outfile) {
        out = fopen(outfile, "w");
        if (!out) { perror(outfile); return 1; }
    }

    fprintf(out, "# OV2SLAM calibration profile\n");
    fprintf(out, "# profiled over %d frames (skipped first %d)\n",
            measured, skip_frames);
    fprintf(out, "# format: phase_name  avg_time_ms  avg_insns_per_ms\n\n");

    for (int j = 1; j < CP_COUNT; j++) {
        struct phase_stats *s = &stats[j];
        if (s->count == 0) {
            fprintf(stderr, "  WARNING: %s: no samples!\n", cp_names[j]);
            continue;
        }
        double avg_time = s->total_time_ms / s->count;
        double avg_insns = (double)s->total_insns / s->count;
        double avg_rate = (avg_time > 0) ? avg_insns / avg_time : 0;

        fprintf(stderr, "  %s: avg %.1fms, %.0f insns, rate %.0f insns/ms (%d samples)\n",
                cp_names[j], avg_time, avg_insns, avg_rate, s->count);
        fprintf(out, "%-20s  %.1f  %.0f\n", cp_names[j], avg_time, avg_rate);
    }

    if (out != stdout) {
        fclose(out);
        fprintf(stderr, "\ncalibration written to %s\n", outfile);
    }

    return 0;
}
