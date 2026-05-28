# Under Pressure: Adaptive Isolation for Visual SLAM on Contended ARM Hardware

**ECRTS 2026 Industrial Challenge — Arm AR HUD**

Source code and evaluation scripts accompanying our submission to the
[ECRTS 2026 Industrial Challenge (Arm Augmented Reality Head-Up Display)](https://www.ecrts.org/industrial-challenge-current-challenge-arm/).

## Overview

This repository contains a software-only feedback controller for preserving
OV²SLAM tracking accuracy under memory-bandwidth contention from co-running
workloads on a Xilinx ZCU102 (quad-core ARM Cortex-A53, 1 MB shared L2 cache).

The controller uses Linux `uprobes` and hardware performance counters to monitor
SLAM pipeline progress at sub-frame granularity. When progress falls below a
calibrated threshold, it suspends the interfering workload by sending `SIGSTOP`,
and resumes it with `SIGCONT` once progress recovers. Everything runs in user
space: no kernel modifications, no changes to the application source, and no
recompilation of the SLAM binary.

The system is two cooperating tools:

- **`calibrate`** — an offline profiler that runs OV²SLAM solo (no interferer)
  and records per-phase timing and instruction-retirement rates to a
  calibration file.
- **`dyn_controller`** — the runtime controller that loads the calibration file,
  installs uprobes at the pipeline checkpoints, and regulates the interferer.

## Repository contents

| File | Description |
|------|-------------|
| `calibrate.c` | Offline calibrator. Profiles per-phase timing/instruction rates and writes the calibration file. |
| `dyn_controller.c` | Runtime feedback controller. Loads the calibration file, installs uprobes, and issues `SIGSTOP`/`SIGCONT` to the interferer. |
| `slam_common.h` | Shared definitions used by both C tools: checkpoint list, hard-coded uprobe offsets, ring-buffer setup, and `perf_event` helpers. |
| `evaluate_all.py` | Computes Absolute Pose Error (APE) RMSE of estimated trajectories against ground truth, using Umeyama similarity alignment. |
| `plot_trajectory.py` | Plots estimated trajectories against ground truth (2D projection, aligned via Umeyama). |
| `run_paper_matrix.sh` | End-to-end experiment driver: runs the Solo / Uncontrolled / Controlled matrix and captures trajectories and interferer bandwidth logs. |

## Prerequisites

These are required to run the experiments but are **not** included in this
repository; set them up separately:

- **Hardware:** Xilinx ZCU102 development board (quad-core ARM Cortex-A53).
- **OS:** Ubuntu 22.04 LTS, Linux kernel 5.15.
- **ROS:** ROS2 Humble.
- **SLAM:** OV²SLAM, compiled against OpenCV without the `contrib` module.
- **Dataset:** EuRoC V1_01_easy stereo dataset, with its Vicon ground-truth
  trajectory in TUM format.
- **Interferer:** the `bandwidth` benchmark from the IsolBench suite, run within
  the RT-Bench framework.
- **Python:** Python 3 with `numpy` (for `evaluate_all.py`) and additionally
  `matplotlib` (for `plot_trajectory.py`).
- **Privileges:** root (`sudo`) is required to install uprobes and to signal the
  interferer process.

> **Note on uprobe offsets.** The checkpoint byte offsets are hard-coded in
> `slam_common.h` (`default_offsets[]`), obtained from `libov2slam.so` via
> `nm -D`. They are specific to a single compiled binary and **must be updated
> whenever `libov2slam.so` is recompiled.**

## Building

Both tools depend only on the standard C library, the Linux `perf_event`
headers, and POSIX threads/signals. `slam_common.h` must be in the same
directory as the `.c` files.

```bash
# Calibrator
gcc -O2 -Wall -o calibrate calibrate.c -lrt

# Dynamic controller
gcc -O2 -Wall -pthread -o dyn_controller dyn_controller.c -lrt -lpthread
```

## Calibration file format

`calibrate` writes one line per pipeline phase. The format is:

```
# OV2SLAM calibration profile
# profiled over <N> frames (skipped first <M>)
# format: phase_name  avg_time_ms  avg_insns_per_ms

preprocessImage  <avg_time_ms>  <avg_insns_per_ms>
kltTracking      <avg_time_ms>  <avg_insns_per_ms>
computePose      <avg_time_ms>  <avg_insns_per_ms>
frameDone        <avg_time_ms>  <avg_insns_per_ms>
```

`avg_time_ms` is the average elapsed time from each frame's `visualTracking`
entry to that checkpoint; `avg_insns_per_ms` is the average instruction-retirement
rate over the same interval. The controller requires all four phases to be
present and uses the `frameDone` rate as the overall baseline.

## Usage

The cores below (SLAM on `0,1`, interferer on `2,3`) and the 0.6× playback rate
match the configuration used in the paper.

### Step 1 — Calibration (solo run, no interferer)

```bash
# Start SLAM on cores 0,1
taskset -c 0,1 ros2 run ov2slam ov2slam_node <params.yaml> &
sleep 8
SLAM_PID=$(pgrep -x ov2slam_node)

# Run the calibrator against that PID (writes calib.txt when the run ends)
sudo ./calibrate -p "$SLAM_PID" \
  -l /path/to/libov2slam.so \
  -c 0,1 -s 10 -o calib.txt

# Play the dataset to drive the run
taskset -c 0,1 ros2 bag play <dataset_path> -r 0.6
```

This produces `calib.txt`, skipping the first 10 frames to avoid startup
transients.

### Step 2 — Controlled run (with interferer)

```bash
# Start the interferer on cores 2,3
taskset -c 2,3 /path/to/IsolBench/bandwidth -b "-m 1024 -a write" &
BW_PID=$(pgrep -x bandwidth)

# Start SLAM on cores 0,1
taskset -c 0,1 ros2 run ov2slam ov2slam_node <params.yaml> &
sleep 8
SLAM_PID=$(pgrep -x ov2slam_node)

# Start the controller
sudo ./dyn_controller -p "$SLAM_PID" -t "$BW_PID" \
  -l /path/to/libov2slam.so \
  -C calib.txt -c 0,1 -F 120 -o controlled.csv

# Play the dataset
taskset -c 0,1 ros2 bag play <dataset_path> -r 0.6
```

The controller writes a per-event CSV log (header:
`frame,checkpoint,time_ms,since_vt_ms,insns,fail_rate,action,source,detail`)
and prints a summary of total frames, stops, resumes, and stop/resume cycles on
exit. If the interferer is still suspended when the controller exits, it is
resumed automatically.

### Step 3 — Evaluation

```bash
# Single trajectory file, or a directory + glob pattern
python3 evaluate_all.py <ground_truth.txt> <trajectory_dir_or_file> '<pattern>'
```

`evaluate_all.py` reads TUM-format trajectories (`timestamp x y z qx qy qz qw`),
associates estimated poses to ground-truth poses by nearest timestamp (within
0.02 s), aligns with the Umeyama similarity transform, and reports per-file and
summary RMSE. Files with fewer than 10 timestamp matches are reported as `FAIL`.

### Plotting

```bash
python3 plot_trajectory.py <ground_truth.txt> \
  -t solo.txt        "Solo" \
  -t uncontrolled.txt "Uncontrolled" \
  -t controlled.txt  "Controlled" \
  -o trajectory_comparison.pdf
```

`-t PATH LABEL` is repeatable. `--axes` selects the 2D projection
(`xy` default, `xz`, or `yz`); `--title` is optional.

### Running the full experiment matrix

`run_paper_matrix.sh` runs all three conditions (Solo / Uncontrolled /
Controlled, five trials each), saves each trajectory, and captures the
interferer's per-iteration bandwidth output to `.bwlog` files for offline
analysis. **Edit the path variables at the top of the script** (`YAML`, `BAG`,
`LIB`, `CALIB`, `TRAJ`, `OUT`) to match your system before running:

```bash
sudo -v
bash run_paper_matrix.sh > paper_matrix.log 2>&1 &
tail -f paper_matrix.log
```

## Controller parameters (`dyn_controller`)

| Flag | Parameter | Default |
|------|-----------|---------|
| `-p` | SLAM process PID | *(required)* |
| `-t` | Interferer process PID | *(required)* |
| `-l` | Path to `libov2slam.so` | *(required)* |
| `-C` | Calibration file | *(required)* |
| `-c` | SLAM CPU cores | `0,1,2` |
| `-F` | Per-frame fail threshold (ms) | `120` |
| `-S` | Monitor sample interval (ms) | `2.0` |
| `-R` | Min frames to stay stopped before resume | `50` |
| `-I` | Immune frames after a resume | `30` |
| `-P` | Progress ratio at/below which to `SIGSTOP` | `0.5` |
| `-V` | Recovery ratio at/above which to `SIGCONT` | `0.70` |
| `-N` | Consecutive good samples required to resume | `15` |
| `-W` | Max rolling fail rate that still permits resume | `0.15` |
| `-o` | CSV log file | stdout |

> In the paper's experiments SLAM is pinned to cores `0,1` (passed as `-c 0,1`),
> overriding the built-in `0,1,2` default.

## Calibrator parameters (`calibrate`)

| Flag | Parameter | Default |
|------|-----------|---------|
| `-p` | SLAM process PID | *(required)* |
| `-l` | Path to `libov2slam.so` | *(required)* |
| `-c` | SLAM CPU cores | `0,1` |
| `-s` | Frames to skip at start | `10` |
| `-o` | Output calibration file | stdout |

> **Run-to-run variability.** OV²SLAM exhibits nondeterministic behavior across
> runs, and individual trials may diverge even in the Solo condition. The numbers
> above are from one set of five trials per condition; a re-run may not reproduce
> them exactly. See the evaluation section of the paper for per-trial results and
> discussion.

## License

Released under the MIT License. See [`LICENSE`](LICENSE).

## Authors

- Alessandra María Lanz (Boston University) — alanz@bu.edu
- Matias Ou (Boston University) — matiasou@bu.edu
- Mattia Nicolella (Boston University) — mnico@bu.edu
- Renato Mancuso (Boston University) — rmancuso@bu.edu
