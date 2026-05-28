// slam_common.h: definitions shared by calibrate.c and dyn_controller.c

#ifndef SLAM_COMMON_H
#define SLAM_COMMON_H

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <sys/types.h>
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

// uprobe points in ov2slam pipeline, ordered by execution
enum checkpoint {
    CP_VT = 0, // visual tracking (frame entry)
    CP_PREPROC, // preprocessing image
    CP_KLT, // klt tracking (optical flow)
    CP_POSE, // compute pose (pose estimation)
    CP_DONE, // frame done (frame completed)
    CP_COUNT // for array sizing
};

static const char *cp_names[] = {
    "visualTracking", "preprocessImage", "kltTracking", "computePose", "frameDone"
};

// byte offsets inside libov2slam.so (from nm -D); change every recompile
static unsigned long default_offsets[CP_COUNT] = {
    0x5d000, 0x58ae0, 0x58dc0, 0x5a8d0, 0x56750
};

// one per uprobe fd
struct probe_fd {
    int fd; // perf_event fd
    struct perf_event_mmap_page *meta; // first mmap page (holds data_head/data_tail)
    char *ring; // data ring, starts one page in
    size_t mmap_size; // bytes to munmap at cleanup
    enum checkpoint cp;
    int cpu;
};

// SIGINT/SIGTERM set this to zero and the main loop exits
static volatile int g_running = 1;
static void handle_signal(int sig) { (void)sig; g_running = 0; }

// monotonic clock in ms
static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static long sys_perf_event_open(struct perf_event_attr *a, pid_t pid, int cpu, int grp, unsigned long flags) {
    return syscall(SYS_perf_event_open, a, pid, cpu, grp, flags);
}

// read uprobe PMU dynamic type number from sysfs
static int read_uprobe_type(void) {
    int t; FILE *f = fopen("/sys/bus/event_source/devices/uprobe/type","r");
    if (!f) return -1;
    if (fscanf(f,"%d",&t)!=1) { 
        fclose(f); 
        return -1; 
    }
    fclose(f); return t;
}

// sum retired-instruction counters across all of SLAM's CPUs
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

// parse a "0,1" core list into an int array; returns count
static int parse_cpus(const char *str, int *cpus, int max) {
    int n = 0;
    char buf[64];
    strncpy(buf, str, sizeof(buf)-1); 
    buf[sizeof(buf)-1] = 0;
    char *tok = strtok(buf, ",");
    while (tok && n < max) { 
        cpus[n++] = atoi(tok); 
        tok = strtok(NULL, ","); 
    }
    return n;
}

// install one uprobe on one CPU: breakpoint at offset in lib, sample on every hit
static int setup_uprobe_on_cpu(struct probe_fd *p, int uprobe_type, const char *lib_path, unsigned long offset, enum checkpoint cp, int cpu) {
    struct perf_event_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.size = sizeof(attr);
    attr.type = uprobe_type; // uprobe PMU type from sysfs
    attr.config1 = (__u64)(unsigned long)lib_path;
    attr.config2 = offset; // byte offset within the ELF binary
    attr.sample_period = 1; // fire on every call
    attr.sample_type = PERF_SAMPLE_TID | PERF_SAMPLE_TIME; // pid/tid + timestamp
    attr.disabled = 1; // start disabled, enable as a batch later
    attr.wakeup_events = 1; // wake poll() on every event

    // pid = -1, cpu = cpu means system-wide on this core
    int fd = sys_perf_event_open(&attr, -1, cpu, -1, PERF_FLAG_FD_CLOEXEC);
    if (fd < 0) return -1;

    // mmap the ring buffer: 1 metadata page + RING_BUF_PAGES data pages
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

// drain pending events from one probe's ring buffer; returns hits matching target_pid
static int drain_probe(struct probe_fd *p, pid_t target_pid, uint64_t *time_out) {
    struct perf_event_mmap_page *meta = p->meta;
    __sync_synchronize(); // fence
    uint64_t head = meta->data_head; // kernel's write pointer
    rmb(); // read fence before consuming records
    uint64_t tail = meta->data_tail; // our consumer pointer
    if (head == tail) return 0; // nothing new

    int matched = 0;
    while (tail < head) {
        uint64_t off = tail % RING_BUF_SIZE;
        struct perf_event_header *ehdr = (struct perf_event_header *)(p->ring + off);
        if (ehdr->type == PERF_RECORD_SAMPLE) {
            char *ptr = (char *)ehdr + sizeof(*ehdr);
            uint32_t pid = *(uint32_t *)ptr; ptr += 4;
            ptr += 4; // skip tid
            uint64_t ts = *(uint64_t *)ptr;
            if ((pid_t)pid == target_pid) { 
                *time_out = ts; 
                matched++; 
            }
        }
        tail += ehdr->size; // each record carries its own size
    }
    mb(); // fence before releasing slots
    meta->data_tail = tail; // ack
    return matched;
}

#endif /* SLAM_COMMON_H */