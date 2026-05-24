# Under Pressure: Adaptive Isolation for Visual SLAM on Contended ARM Hardware

**ECRTS 2026 Industrial Challenge – Arm AR HUD**

This repository contains the source code and evaluation scripts for our submission to the [ECRTS 2026 Industrial Challenge (Arm Augmented Reality Head-Up Display)](https://www.ecrts.org/industrial-challenge-current-challenge-arm/).

## Overview

We introduce a software-only feedback controller for preserving OV²SLAM tracking performance under memory-bandwidth contention from co-running workloads on a Xilinx ZCU102 platform with a quad-core ARM Cortex-A53 processor and 1 MB shared L2 cache. The controller uses Linux uprobes and hardware performance counters to monitor SLAM pipeline progress at sub-frame granularity and dynamically throttles the interfering workload by issuing `SIGSTOP`/`SIGCONT` signals when progress begins to degrades.

## Platform Requirements

- **Hardware:** Xilinx ZCU102 development board (quad-core ARM Cortex-A53)
- **OS:** Ubuntu 22.04 LTS, Linux kernel 5.15
- **ROS:** ROS2 Humble
- **SLAM:** OV²SLAM (compiled without OpenCV contrib; ORB replaces BRIEF)
- **Dataset:** EuRoC V1_01_easy stereo dataset
- **Interferer:** RT-Bench IsolBench `bandwidth` benchmark

## Building

The tools have no dependencies beyond the standard C library, Linux perf_event headers, and POSIX threads/signals.

```bash
# Build the calibrator
gcc -O2 -Wall -o calibrate calibrate.c -lrt

# Build the dynamic controller
gcc -O2 -Wall -pthread -o dyn_controller dyn_controller.c -lrt -lpthread
```

## Usage

### Step 1: Calibration (solo run, no interferer)

Start OV²SLAM on the target cores, then run the calibrator:

```bash
# Start SLAM (example: cores 0,1 at 0.6x playback)
taskset -c 0,1 ros2 run ov2slam ov2slam_node <params.yaml> &
sleep 8
SLAM_PID=$(pgrep -x ov2slam_node)

# Run calibrator
sudo ./calibrate -p $SLAM_PID \
  -l /path/to/libov2slam.so \
  -c 0,1 -s 10 -o calib.txt

# Play the dataset
taskset -c 0,1 ros2 bag play <dataset_path> -r 0.6
```

This produces `calib.txt` with per-phase baselines (skipping the first 10 frames to avoid startup transients).

### Step 2: Controlled run (with interferer)

Start both OV²SLAM and the interferer, then run the controller:

```bash
# Start interferer on separate cores
taskset -c 2,3 ./rt-bench/IsolBench/bandwidth \
  -b "-m 1024 -a write" &
BW_PID=$(pgrep -x bandwidth)

# Start SLAM
taskset -c 0,1 ros2 run ov2slam ov2slam_node <params.yaml> &
sleep 8
SLAM_PID=$(pgrep -x ov2slam_node)

# Start controller
sudo ./dyn_controller -p $SLAM_PID -t $BW_PID \
  -l /path/to/libov2slam.so \
  -C calib.txt -c 0,1 -F 120

# Play dataset
taskset -c 0,1 ros2 bag play <dataset_path> -r 0.6
```

### Step 3: Evaluation

```bash
# Evaluate trajectory RMSE against ground truth
python3 evaluate_all.py <ground_truth.txt> <trajectory_dir> '<pattern>'
```

## Controller Parameters

| Flag | Parameter | Default | Description |
|------|-----------|---------|-------------|
| `-P` | STOP threshold | 0.50 | Progress ratio below which SIGSTOP is sent |
| `-V` | RESUME threshold | 0.70 | Progress ratio above which SIGCONT is sent |
| `-F` | Frame deadline | 120 ms | Wall-clock deadline for per-frame completion |
| `-S` | Sample interval | 2.0 ms | Monitor thread polling interval |
| `-R` | Min stop frames | 50 | Minimum frames to stay stopped before considering resume |
| `-I` | Immune frames | 30 | Frames after resume before STOP can trigger again |
| `-N` | Sustained recovery | 15 | Consecutive good samples needed for RESUME |
| `-W` | Rolling fail threshold | 0.15 | Max fraction of failed frames to allow RESUME |

## Key Results

Evaluated on ZCU102 with 2 SLAM cores at 0.6× playback, RT-Bench bandwidth interferer (`-m 1024`) on cores 2,3. Five trials per condition.

| Condition    | Avg Poses | Mean RMSE (m) | Diverged |
|-------------|-----------|---------------|----------|
| Solo         | 2,525     | 0.087         | 0/5      |
| Uncontrolled | 1,950     | 1.027         | 3/5      |
| Controlled   | 2,412     | 0.095         | 0/5      |

## Authors

- Alessandra María Lanz (Boston University) — alanz@bu.edu
- Matias Ou (Boston University) — matiasou@bu.edu
- Mattia Nicolella (Boston University) — mnico@bu.edu
- Renato Mancuso (Boston University) — rmancuso@bu.edu
