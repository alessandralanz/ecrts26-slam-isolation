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
#include "slam_common.h"

// what we're accumulating per phase
struct phase_stats {
    double total_time_ms; // sum of elapsed times from VT 
    uint64_t total_insns; // sum of instructions retired from VT
    int count; // how many samples
};

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