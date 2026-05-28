/* dynamic controller: progress based uprobe controller
 *
 * gcc -O2 -Wall -pthread -o dyn_controller dyn_controller.c -lrt -lpthread
 * sudo ./dyn_controller -p <slam_pid> -t <rt_pid> -l /path/to/libov2slam.so -C calib.txt -c 0,1,2 -F 120
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <time.h>
#include <stdint.h>
#include <poll.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <linux/perf_event.h>
#include "slam_common.h"

#define RATE_WINDOW 32 // rolling window for instruction rate samples
#define FRAME_WINDOW 50 // rolling window for frame completion times

struct phase_calib {
    double avg_time_ms; // average frame duration
    double avg_rate; // average instructions/ms during the phase
};

// rolling window/circular buffer for insn rate samples
struct rate_window {
    double samples[RATE_WINDOW]; // holding last 32 instruction rate measurements
    int head, count; // head is the index where the next sample will be written to and count is the number of valid samples in buffer
};

// push a new sample into the instruction rate circular buffer
static void rw_push(struct rate_window *w, double val) {
    // write val at the current head position (overwritting the oldest sample if the buffer is full)
    w->samples[w->head] = val;
    // advance head by 1, wrap around to 0 when it reaches RATE_WINDOW
    w->head = (w->head + 1) % RATE_WINDOW;
    // if buffer not full then increment the count
    if (w->count < RATE_WINDOW) w->count++;
}

// compute arithmetic mean of all valid samples (skip if empty; avoid division by 0)
static double rw_avg(struct rate_window *w) {
    if (w->count == 0) return 0;
    double sum = 0;
    // iterate over all count entries in the array (could be less an RATE_WINDOW in the inital fill phase)
    for (int i = 0; i < w->count; i++) sum += w->samples[i];
    return sum / w->count;
}

// rolling window for frame times (last 50 frame durations in ms)
struct frame_window {
    double times[FRAME_WINDOW];
    int head, count;
};

// same push logic as the instruction rate push func but this is for frame times
static void fw_push(struct frame_window *w, double val) {
    w->times[w->head] = val;
    w->head = (w->head + 1) % FRAME_WINDOW;
    if (w->count < FRAME_WINDOW) w->count++;
}

// what fraction of the last frame window frames exceeded the threshold
static double fw_fail_rate(struct frame_window *w, double thresh) {
    if (w->count == 0) return 0;
    int fails = 0;
    for (int i = 0; i < w->count; i++)
        if (w->times[i] > thresh) fails++;
    return (double)fails / w->count;
}

// shared state between main thread and monitor thread
// main thread writes phase info and the monitor reads it
// monitor threads writes stopped/resume decisions and the main thread reads them
struct shared_state {
    // phase tracking (main writes, monitor reads)
    volatile int current_phase; // which pipeline stage SLAM is in right now
    volatile double phase_start_time; // when the phase started (ms)
    volatile uint64_t phase_start_insns; // instruction count when phase started

    // frame counter
    volatile int total_frames;

    // both threads read and write via compare and swap
    volatile int stopped; // 1 = RT-bench frozen

    // debounce counters (main thread increments on each CP_VT)
    volatile int frames_since_stop; // frames elapsed since last stop
    volatile int frames_since_resume; // frames elpased since last resume

    volatile int total_stops, total_resumes;
    volatile int monitor_stops, monitor_resumes, frame_stops;
    volatile int fail_count; // frames that exceeded the time threshold
    volatile int stop_cycle; // completed stop and resume cycles
    volatile int sustained_good_samples; // consecutive good monitor samples

    struct rate_window insn_rates; //instruction sample rates (written by montior)
    struct frame_window frame_times; // frame completion times (written by main)

    // config
    pid_t rt_pid;
    double frame_thresh_ms; // frame failure threshold
    int min_stop_frames; // min frames to stay stopped before we should consider resuming
    int immune_frames; // frames of immunity after a resume
    double progress_threshold; // stop if progress ratio drops below this
    double recovery_ratio; // resume if instruction rate recovers above this
    int sustained_recovery_count; // consecutive good samples needed to resume (15)
    double rolling_fail_thresh; // rolling fail rate must be below this to resume
    double sample_interval_ms; // monitor thread wakeup interval (2.0 ms)

    struct phase_calib calib[CP_COUNT]; // per phase calibration data from calibration file
    double calib_overall_rate; // overall instructions/ms from frameDone calibration

    volatile int running; // monitor thread checks this to know when to exit (set to 1 at init and 0 to tell monitor to exit its loop)
    int insn_fds[MAX_CPUS]; // per-CPU instruction counter FDs
    int n_cpus, cpus[MAX_CPUS]; // which CPUs SLAM runs on 

    FILE *log_fp; // output CSV file pointer (or stdout)
    pthread_mutex_t log_mutex; // protects it because both threads write to the same file
};

// load calibration from file
// each line (one line per phase): phase_name avg_time_ms avg_rate_insns_per_ms
// frameDone's rate becomes the overall baseline
static int load_calibration(const char *path, struct shared_state *st) {
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return -1; }

    char line[256];
    int loaded = 0;
    while (fgets(line, sizeof(line), f)) {
        // skip comments and blank lines
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\n' || *p == '\0') continue;

        char name[64];
        double time_ms, rate;
        if (sscanf(p, "%63s %lf %lf", name, &time_ms, &rate) != 3) {
            fprintf(stderr, "WARNING: bad calib line: %s", line);
            continue;
        }

        // match name to checkpoint
        int cp = -1;
        for (int i = 1; i < CP_COUNT; i++) {
            if (strcmp(name, cp_names[i]) == 0) { cp = i; break; }
        }
        if (cp < 0) {
            fprintf(stderr, "WARNING: unknown phase '%s', skipping\n", name);
            continue;
        }

        st->calib[cp].avg_time_ms = time_ms;
        st->calib[cp].avg_rate = rate;
        loaded++;

        if (cp == CP_DONE)
            st->calib_overall_rate = rate;
    }
    fclose(f);
    return loaded;
}

// monitor thread: samples instruction rate every sample_interval_ms, reads instruction counters, and decides to stop/resume
static void *monitor_thread_func(void *arg) {
    struct shared_state *st = (struct shared_state *)arg;
    struct timespec sleep_ts;
    sleep_ts.tv_sec = 0;
    sleep_ts.tv_nsec = (long)(st->sample_interval_ms * 1e6);

    uint64_t prev_insns = 0;
    double prev_time = 0;

    fprintf(stderr, "monitor thread started (%.1fms interval)\n", st->sample_interval_ms);

    while (st->running) {
        nanosleep(&sleep_ts, NULL);
        if (!st->running) continue;

        double now = now_ms();
        uint64_t cur_insns = read_total_insns(st->insn_fds, st->n_cpus);

        if (prev_time > 0) {
            double dt = now - prev_time;
            if (dt > 0.5) {
                uint64_t di = (cur_insns > prev_insns) ? (cur_insns - prev_insns) : 0;
                double rate = (double)di / dt;
                rw_push(&st->insn_rates, rate);
                double avg_rate = rw_avg(&st->insn_rates);

                // resume: check if we've recovered
                if (st->stopped && st->frames_since_stop >= st->min_stop_frames) {
                    double baseline = st->calib_overall_rate;
                    double ratio = (baseline > 0) ? avg_rate / baseline : 0;
                    double roll_fail = fw_fail_rate(&st->frame_times, st->frame_thresh_ms);

                    if (ratio >= st->recovery_ratio && roll_fail < st->rolling_fail_thresh) {
                        st->sustained_good_samples++;
                        if (st->sustained_good_samples >= st->sustained_recovery_count) {
                            if (__sync_bool_compare_and_swap(&st->stopped, 1, 0)) {
                                kill(st->rt_pid, SIGCONT);
                                st->total_resumes++;
                                st->monitor_resumes++;
                                st->frames_since_resume = 0;
                                st->sustained_good_samples = 0;
                                st->stop_cycle++;

                                pthread_mutex_lock(&st->log_mutex);
                                fprintf(st->log_fp,
                                    "%d,monitor,%.1f,0,0,%.4f,"
                                    "RESUME,monitor,"
                                    "avg_rate=%.0f,ratio=%.2f,"
                                    "roll_fail=%.3f,cycle=%d\n",
                                    st->total_frames, now,
                                    st->total_frames > 0 ? (double)st->fail_count / st->total_frames : 0,
                                    avg_rate, ratio, roll_fail,
                                    st->stop_cycle);
                                fflush(st->log_fp);
                                pthread_mutex_unlock(&st->log_mutex);
                            }
                        }
                    } else {
                        st->sustained_good_samples = 0;
                    }
                }

                // stop: check for slowdown mid-phase
                if (!st->stopped) {
                    int phase = st->current_phase;
                    if (phase > CP_VT && phase < CP_DONE && st->frames_since_resume > st->immune_frames) {
                        double phase_start = st->phase_start_time;
                        uint64_t phase_start_i = st->phase_start_insns;

                        if (phase_start > 0) {
                            double pe = now - phase_start;
                            if (pe >= 3.0) {
                                uint64_t pi = (cur_insns > phase_start_i) ? (cur_insns - phase_start_i) : 0;
                                struct phase_calib *cal = &st->calib[phase];
                                if (cal->avg_rate > 0) {
                                    double expected = cal->avg_rate * pe;
                                    double pr = (expected > 0) ? (double)pi / expected : 1.0;
                                    int overtime = (cal->avg_time_ms > 5.0 && pe > cal->avg_time_ms * 2.0);

                                    if (pr < st->progress_threshold || overtime) {
                                        if (__sync_bool_compare_and_swap(&st->stopped, 0, 1)) {
                                            kill(st->rt_pid, SIGSTOP);
                                            st->total_stops++;
                                            st->monitor_stops++;
                                            st->frames_since_stop = 0;
                                            st->sustained_good_samples = 0;

                                            pthread_mutex_lock(&st->log_mutex);
                                            fprintf(st->log_fp,
                                                "%d,%s,%.1f,%.1f,%lu,%.3f,"
                                                "STOP,monitor,"
                                                "ratio=%.2f,avg=%.0f%s\n",
                                                st->total_frames,
                                                cp_names[phase],
                                                now, pe, (unsigned long)pi,
                                                st->total_frames > 0 ? (double)st->fail_count / st->total_frames : 0,
                                                pr, avg_rate,
                                                overtime ? ",overtime":"");
                                            fflush(st->log_fp);
                                            pthread_mutex_unlock(&st->log_mutex);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        prev_insns = cur_insns;
        prev_time = now;
    }

    fprintf(stderr, "monitor thread exiting\n");
    return NULL;
}

int main(int argc, char **argv) {
    pid_t slam_pid = -1; // pid of ov2slam
    struct shared_state state;
    memset(&state, 0, sizeof(state)); // write zeros to every byte in struct when shared_state struct initialized on stack

    // set defaults for all turnable parameters
    // can be overridden by command-line options
    state.rt_pid = -1;
    state.frame_thresh_ms = 120.0;
    state.min_stop_frames = 50;
    state.immune_frames = 30;
    state.progress_threshold = 0.5;
    state.recovery_ratio = 0.70;
    state.sustained_recovery_count = 15;
    state.rolling_fail_thresh = 0.15;
    state.sample_interval_ms = 2.0;
    state.running = 1;
    pthread_mutex_init(&state.log_mutex, NULL); // NULL means use default mutex settings (give a standard lock)

    // command-line parsing
    char *lib_path = NULL, *logfile = NULL, *cpu_str = "0,1,2";
    char *calib_path = NULL;

    int c;
    while ((c = getopt(argc, argv, "p:t:l:c:C:F:S:R:I:P:V:N:W:o:h")) != -1) {
        switch (c) {
        case 'p': slam_pid = atoi(optarg); break;
        case 't': state.rt_pid = atoi(optarg); break;
        case 'l': lib_path = optarg; break;
        case 'c': cpu_str = optarg; break;
        case 'C': calib_path = optarg; break;
        case 'F': state.frame_thresh_ms = atof(optarg); break;
        case 'S': state.sample_interval_ms = atof(optarg); break;
        case 'R': state.min_stop_frames = atoi(optarg); break;
        case 'I': state.immune_frames = atoi(optarg); break;
        case 'P': state.progress_threshold = atof(optarg); break;
        case 'V': state.recovery_ratio = atof(optarg); break;
        case 'N': state.sustained_recovery_count = atoi(optarg); break;
        case 'W': state.rolling_fail_thresh = atof(optarg); break;
        case 'o': logfile = optarg; break;
        case 'h':
            printf("Usage: %s -p <slam_pid> -t <rt_pid> -l <lib> -C <calib> [opts]\n", argv[0]);
            printf("-C  calibration file (required)\n");
            printf("-c  SLAM CPU cores [0,1,2]\n");
            printf("-F  frame fail threshold ms [120]\n");
            printf("-S  sample interval ms [2.0]\n");
            printf("-R  min frames to stay stopped [50]\n");
            printf("-I  immune frames after resume [30]\n");
            printf("-P  progress ratio to STOP [0.5]\n");
            printf("-V  recovery ratio to RESUME [0.70]\n");
            printf("-N  sustained good samples [15]\n");
            printf("-W  rolling fail rate to resume [0.15]\n");
            printf("-o  csv log file [stdout]\n");
            return 0;
        }
    }

    // check that all required options were provided 
    if (slam_pid <= 0 || state.rt_pid <= 0 || !lib_path || !calib_path) {
        fprintf(stderr, "need -p <slam_pid> -t <rt_pid> -l <lib_path> -C <calib_file>\n");
        return 1;
    }
    if (kill(slam_pid, 0) != 0) {
        fprintf(stderr, "ERROR: SLAM pid %d not found\n", slam_pid); return 1;
    }
    // sends signal 0 to a process (doesn't actually send any signal to a process)
    // the kernel checks whether the process exists and whether we have permission to signal it
    // if the process exists and we have permission then kill returns 0 but if it doesn't exist kill returns -1
    // checking if the call failed 
    if (kill(state.rt_pid, 0) != 0) {
        fprintf(stderr, "ERROR: interferer pid %d not found\n", state.rt_pid);
        return 1;
    }

    // load calibration and check that we have all 4 phases
    int n_calib = load_calibration(calib_path, &state);
    if (n_calib < 4) {
        fprintf(stderr, "ERROR: need 4 phases in calib file, got %d\n", n_calib);
        return 1;
    }
    if (state.calib_overall_rate <= 0) {
        fprintf(stderr, "ERROR: frameDone entry missing or has zero rate\n");
        return 1;
    }

    state.n_cpus = parse_cpus(cpu_str, state.cpus, MAX_CPUS);

    state.log_fp = stdout;
    if (logfile) {
        state.log_fp = fopen(logfile, "w");
        if (!state.log_fp) { perror("fopen"); return 1; }
    }
    fprintf(state.log_fp, "frame,checkpoint,time_ms,since_vt_ms,insns," "fail_rate,action,source,detail\n");

    // register the signal handler
    signal(SIGINT, handle_signal); // when this process receives SIGINT (control-c), call handle_signal instead of the default action (which is to terminate)
    signal(SIGTERM, handle_signal); // handler sets g_running = 0 to shut down

    fprintf(stderr, "Dynamic Controller (pre-calibrated)\n");
    fprintf(stderr, "SLAM PID: %d\n", slam_pid);
    fprintf(stderr, "Interferer: %d\n", state.rt_pid);
    fprintf(stderr, "Library: %s\n", lib_path);
    fprintf(stderr, "Calibration: %s\n", calib_path);
    fprintf(stderr, "SLAM CPUs: %s\n", cpu_str);
    fprintf(stderr, "Frame threshold: %.0f ms\n", state.frame_thresh_ms);
    fprintf(stderr, "Sample interval: %.1f ms\n", state.sample_interval_ms);
    fprintf(stderr, "STOP ratio: %.2f\n", state.progress_threshold);
    fprintf(stderr, "RESUME ratio: %.2f\n", state.recovery_ratio);
    fprintf(stderr, "Sustained samples: %d\n", state.sustained_recovery_count);
    fprintf(stderr, "Rolling fail max: %.2f\n", state.rolling_fail_thresh);
    fprintf(stderr, "Min stop frames: %d\n", state.min_stop_frames);
    fprintf(stderr, "Immune frames: %d\n", state.immune_frames);

    fprintf(stderr, "\nloaded calibration:\n");
    for (int i = 1; i < CP_COUNT; i++) {
        struct phase_calib *cal = &state.calib[i];
        if (cal->avg_rate > 0)
            fprintf(stderr, "  %s: avg %.1fms, rate %.0f insns/ms\n",
                cp_names[i], cal->avg_time_ms, cal->avg_rate);
    }
    fprintf(stderr, "  overall baseline: %.0f insns/ms\n", state.calib_overall_rate);

    // configure hardware performance counter for retired instructions
    struct perf_event_attr hw_attr;
    memset(&hw_attr, 0, sizeof(hw_attr));
    hw_attr.size = sizeof(hw_attr);
    hw_attr.type = PERF_TYPE_HARDWARE; // want a hardware PMU event
    hw_attr.config = PERF_COUNT_HW_INSTRUCTIONS; // selects the instructions retired counter
    hw_attr.disabled = 0; // start counting immediately
    hw_attr.exclude_kernel = 1; // don't count instructions executed in kernel mode (only want userspace instructions)
    hw_attr.exclude_hv = 1; // exclude hypervisor instructions (not relevant for zcu)

    fprintf(stderr, "\ninstruction counters (system-wide per CPU):\n");
    for (int i = 0; i < state.n_cpus; i++) {
        // open one instruction counter per CPU
        // pid = -1 means system wide (count all processes' instructions on this CPU)
        // cpu = state.cpu[i] pins it to a specific CPU (system wide counters bc SLAM threads might migrate between specified CPUs)
        state.insn_fds[i] = sys_perf_event_open(&hw_attr, -1, state.cpus[i], -1, 0);
        if (state.insn_fds[i] >= 0)
            fprintf(stderr, "  cpu%d: fd=%d OK\n", state.cpus[i], state.insn_fds[i]);
        else
            fprintf(stderr, "  cpu%d: FAILED (%s)\n", state.cpus[i], strerror(errno));
    }

    // uprobe installation
    // get the PMU type number
    int uprobe_type = read_uprobe_type();
    if (uprobe_type < 0) {
        fprintf(stderr, "ERROR: uprobe PMU not available\n"); return 1;
    }

    // array to hold all probe FDs (max 5 checkpoints x 4 CPUs = 20 entries)
    struct probe_fd all_probes[CP_COUNT * MAX_CPUS];
    int n_fds = 0; // number of probes we've successfully installed so far

    fprintf(stderr, "\ninstalling uprobes:\n");
    // for each checkpoint install a probe on each CPU
    for (int cp = 0; cp < CP_COUNT; cp++) {
        int ok = 0; // count successes for this checkpoint (how many CPUs successfully succeeded for this checkpoint)
        // iterate through each CPU that we are using (passed through at the command line)
        for (int ci = 0; ci < state.n_cpus; ci++) {
            if (setup_uprobe_on_cpu(&all_probes[n_fds], uprobe_type, lib_path, default_offsets[cp], cp, state.cpus[ci]) == 0) {
                n_fds++; ok++;
            }
        }
        fprintf(stderr, "  %s (0x%lx): %d/%d CPUs\n", cp_names[cp], default_offsets[cp], ok, state.n_cpus);
    }
    // if no probe succeeded then no point continuing
    if (n_fds == 0) {
        fprintf(stderr, "ERROR: no probes installed\n"); return 1;
    }

    // enables all probes
    // tell kernel to start watching for the probe hit
    // whenever any process hits the probed instruction on the relevant CPU, the kernel will write to the ring buffer
    for (int i = 0; i < n_fds; i++)
        ioctl(all_probes[i].fd, PERF_EVENT_IOC_ENABLE, 0);

    // spawn the montior thread
    pthread_t monitor_tid;
    // pthread_create takes a pointer to store the thread ID (monitor_tid), thread attributes (NULL = defaults), the function to run (monitor_thread_func), and the arg to pass (&state)
    if (pthread_create(&monitor_tid, NULL, monitor_thread_func, &state) != 0) {
        fprintf(stderr, "ERROR: failed to create monitor thread\n");
        return 1;
    }

    fprintf(stderr, "\nmonitoring (armed from frame 1)...\n\n");
    
    // set up pollfd array for poll()
    // each entry specifies a file descriptor to watch and what events to watch for
    struct pollfd pfds[MAX_FDS]; // array of all pollfd for each checkpoint
    for (int i = 0; i < n_fds; i++) {
        pfds[i].fd = all_probes[i].fd;
        // watch for incoming data
        pfds[i].events = POLLIN; // POLLIN means there is data to read (record has been written to the ring buffer)
    }

    double frame_start_time = 0;
    state.frames_since_resume = state.immune_frames + 1; // don't want an immunity period at startup (no resume to recover from)

    // poll() blocks until one FD has data or until the timeout (500ms) expires
    // returns the number of FDs with events 
    while (g_running) {
        // returns number of FDs with events, 0 on timeout, or -1 on error
        int ret = poll(pfds, n_fds, 500);
        // if poll was interrupted by a signal (signal handler ran), we retry the poll if the error is EINTR but anything else we break
        if (ret < 0) { 
            if (errno == EINTR) continue; 
            break; 
        }
        if (ret == 0) {
            // check if the process has exited 
            // on poll timeout (no events in 500ms) check if the SLAM process is still alive
            // kill(slam_pid, 0) sends signal 0 to the SLAM process to check if it exists
            // ESRCH = error: search (kernel searched its process table for slam_pid and didn't find it meaning it exited)
            // check both conditions because we only want to exit if the process does not exist
            if (kill(slam_pid, 0) != 0 && errno == ESRCH) {
                fprintf(stderr, "slam process exited\n"); break;
            }
            continue; // go back to the top of the while loop to call poll again if the SLAM process is still alive (timeout happened bc no uprobe event within 500ms)
        }

        // snapshot current time and instruction count 
        double now = now_ms();
        uint64_t cur_insns = read_total_insns(state.insn_fds, state.n_cpus);

        // iterate through all probe FDs 
        for (int i = 0; i < n_fds; i++) {
            if (!(pfds[i].revents & POLLIN)) continue;

            // drain ring buffer for this probe
            uint64_t ts;
            int hits = drain_probe(&all_probes[i], slam_pid, &ts);
            if (hits <= 0) continue; // if no events match the PID then skip

            // find which checkpoint the probe represents 
            enum checkpoint cp = all_probes[i].cp;
            double since_vt = (frame_start_time > 0) ? (now - frame_start_time) : 0; // time elapsed since the current frame started

            // new frame started
            if (cp == CP_VT) {
                state.total_frames++; // increment the frame counter
                frame_start_time = now; // record when the frame started
                state.phase_start_time = now; // update debounce counter for whichever state we're in (stopped/running)
                state.phase_start_insns = cur_insns;
                state.current_phase = CP_VT;

                if (state.stopped) state.frames_since_stop++;
                else state.frames_since_resume++;
                continue;
            }

            // mid pipeline checkpoint
            // update the phase tracking in shared state 
            // monitor thread will read these to compute the current phase's progress ratio
            if (cp > CP_VT && cp < CP_DONE) {
                state.current_phase = cp;
                state.phase_start_time = now;
                state.phase_start_insns = cur_insns;
            }

            // fame processing done
            if (cp == CP_DONE) {
                state.current_phase = CP_DONE;
                // check if we exceeded the time threshold and record the frame time in the rolling window
                int failed = (since_vt > state.frame_thresh_ms);
                if (failed) state.fail_count++;
                fw_push(&state.frame_times, since_vt);

                // compute cumulative and rolling fail rates
                double fail_rate = state.total_frames > 0 ? (double)state.fail_count / state.total_frames : 0;
                double roll_fail = fw_fail_rate(&state.frame_times, state.frame_thresh_ms);

                // frame level STOP backup
                // if monitor thread misses it's midframe STOP, the frame finished over threshold, the rolling fail rate > 15% and past the immunity point then stop interferer
                if (!state.stopped && failed && roll_fail > 0.15 && state.frames_since_resume > state.immune_frames) {
                    // compare and swap to claim the stop action and freeze interferer
                    if (__sync_bool_compare_and_swap(&state.stopped, 0, 1)) {
                        // log the event
                        kill(state.rt_pid, SIGSTOP);
                        state.total_stops++;
                        state.frame_stops++;
                        state.frames_since_stop = 0;
                        state.sustained_good_samples = 0;

                        pthread_mutex_lock(&state.log_mutex);
                        fprintf(state.log_fp,
                            "%d,%s,%.1f,%.1f,%lu,%.4f,STOP,frame,"
                            "roll_fail=%.3f,cycle=%d\n",
                            state.total_frames, cp_names[cp],
                            now, since_vt, (unsigned long)cur_insns,
                            fail_rate, roll_fail, state.stop_cycle);
                        fflush(state.log_fp);
                        pthread_mutex_unlock(&state.log_mutex);
                    }
                }

                // every 100 frames print a status line to stderr showing frame time, fail counts, rolling fail rate, instruction rate, and current state
                if (state.total_frames % 100 == 0) {
                    double avg_rate = rw_avg(&state.insn_rates);
                    double ratio = (state.calib_overall_rate > 0) ? avg_rate / state.calib_overall_rate : 0;
                    fprintf(stderr,
                        "  frame %d: %.1fms fails=%d/%d (%.1f%%) "
                        "roll=%.1f%% rate=%.0f (%.0f%%) "
                        "[%s] stops=%d res=%d cyc=%d\n",
                        state.total_frames, since_vt,
                        state.fail_count, state.total_frames, fail_rate * 100,
                        roll_fail * 100, avg_rate, ratio * 100,
                        state.stopped ? "STOPPED" : "running",
                        state.total_stops, state.total_resumes,
                        state.stop_cycle);
                }
            }
        }
    }

    // tell the monitor thread to exit 
    state.running = 0;
    pthread_join(monitor_tid, NULL); // pthread_join blocks until the thread terminates (NULL means we don't care about the thread's return value)

    // if we're exiting while the interferer is still stopped, resume it
    // don't want to leave a process frozen because the controller exited
    if (state.stopped) {
        kill(state.rt_pid, SIGCONT);
        fprintf(stderr, "resumed interferer on exit\n");
    }

    // disable each uprobe, unmap its ring buffer, and close the FD
    for (int i = 0; i < n_fds; i++) {
        ioctl(all_probes[i].fd, PERF_EVENT_IOC_DISABLE, 0); // PERF_EVENT_IOC_DISABLE tells kernel to stop monitoring probe for hits
        munmap(all_probes[i].meta, all_probes[i].mmap_size); // munmap releasess the mapped memory region (metadata + data pages)
        close(all_probes[i].fd); // releases the kernel resources associated with the FD
    }
    for (int i = 0; i < state.n_cpus; i++)
        // close the hardware instruction counter FDs
        if (state.insn_fds[i] >= 0) close(state.insn_fds[i]);
    // close the log file (but not if it's stdout)
    if (state.log_fp != stdout) fclose(state.log_fp);
    pthread_mutex_destroy(&state.log_mutex); // destroy the mutex that frees any resources pthread allocated for it

    double fr = state.total_frames > 0 ? (double)state.fail_count / state.total_frames : 0;
    fprintf(stderr, "\n Summary \n");
    fprintf(stderr, "total frames: %d\n", state.total_frames);
    fprintf(stderr, "fails: %d (%.1f%%)\n", state.fail_count, fr * 100);
    fprintf(stderr, "total stops: %d\n", state.total_stops);
    fprintf(stderr, "  monitor stops: %d\n", state.monitor_stops);
    fprintf(stderr, "  frame stops: %d\n", state.frame_stops);
    fprintf(stderr, "total resumes: %d\n", state.total_resumes);
    fprintf(stderr, "  monitor resumes: %d\n", state.monitor_resumes);
    fprintf(stderr, "stop-resume cycles: %d\n", state.stop_cycle);
    return 0;
}