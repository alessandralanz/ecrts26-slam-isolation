#!/bin/bash
# run_paper_matrix.sh
# Runs the paper's full-res 0.6x matrix (Solo / Uncontrolled / Controlled, x5)
# with the RT-Bench interferer's stdout captured to .bwlog files
#
# Conditions are identical to the paper's existing Tables 1-3:
#   - full-res euroc_stereo.yaml (accurate mode)
#   - 2 SLAM cores (0,1) @ 0.6x playback
#   - RT-Bench bandwidth -m 1024 -a write on cores 2,3
#   - controller: -C calib.txt -F 120 (defaults everywhere else)
#
# Usage:
#   sudo -v
#   bash ~/run_paper_matrix.sh > ~/paper_matrix.log 2>&1 &
#   tail -f ~/paper_matrix.log

YAML=~/ros2_ws/src/ov2slam/parameters_files/accurate/euroc/euroc_stereo.yaml
BAG=~/datasets/V1_01_easy
LIB=~/ros2_ws/install/ov2slam/lib/libov2slam.so
CALIB=~/calib.txt
TRAJ=~/ros2_ws/ov2slam_traj.txt
OUT=~/datasets/rtbench_experiments/paper_rerun
RATE=0.6

echo "setup $(date +%H:%M:%S): out=${OUT} rate=${RATE}"
mkdir -p "$OUT"
cd ~/ros2_ws
source /opt/ros/humble/setup.bash > /dev/null 2>&1
source install/setup.bash > /dev/null 2>&1

( while true; do sudo -n true; sleep 50; done ) &
SUDO_KEEPALIVE=$!
trap 'kill $SUDO_KEEPALIVE 2>/dev/null; pkill -9 -x ov2slam_node; pkill -9 -x bandwidth; pkill -9 -x dyn_controller' EXIT

start_slam () {
  taskset -c 0,1 ros2 run ov2slam ov2slam_node "$YAML" > /dev/null 2>&1 &
  sleep 8
}

# start interferer redirect its stdout to a .bwlog file we can analyze later
start_interferer_log () { # $1 = bwlog path
  taskset -c 2,3 ~/rt-bench/IsolBench/bandwidth -b "-m 1024 -a write" > "$1" 2>&1 &
  sleep 2
}

finish_trial () { # $1 = dest trajectory file
  sleep 30
  if [ -f "$TRAJ" ]; then
    cp "$TRAJ" "$1"
    echo "  saved $(basename "$1"): $(wc -l < "$1") poses"
  else
    echo "  $(basename "$1"): FILE NOT FOUND"
  fi
  pkill -9 -x ov2slam_node; pkill -9 -x bandwidth; pkill -9 -x dyn_controller
  sleep 8
}

echo "SOLO"
for t in 1 2 3 4 5; do
  echo "[solo trial $t] $(date +%H:%M:%S)"
  pkill -9 -x ov2slam_node; pkill -9 -x bandwidth; sleep 3
  rm -f "$TRAJ"
  start_slam
  taskset -c 0,1 ros2 bag play "$BAG" -r $RATE < /dev/null > /dev/null 2>&1
  finish_trial "$OUT/solo_t${t}.txt"
done

echo "UNCONTROLLED"
for t in 1 2 3 4 5; do
  echo "[uncontrolled trial $t] $(date +%H:%M:%S)"
  pkill -9 -x ov2slam_node; pkill -9 -x bandwidth; sleep 3
  rm -f "$TRAJ"
  start_interferer_log "$OUT/uncontrolled_t${t}.bwlog"
  start_slam
  taskset -c 0,1 ros2 bag play "$BAG" -r $RATE < /dev/null > /dev/null 2>&1
  finish_trial "$OUT/uncontrolled_t${t}.txt"
done

echo "CONTROLLED"
for t in 1 2 3 4 5; do
  echo "[controlled trial $t] $(date +%H:%M:%S)"
  pkill -9 -x ov2slam_node; pkill -9 -x bandwidth; pkill -9 -x dyn_controller; sleep 3
  rm -f "$TRAJ"
  start_interferer_log "$OUT/controlled_t${t}.bwlog"
  BW_PID=$(pgrep -x bandwidth | head -1)
  start_slam
  SLAM_PID=$(pgrep -x ov2slam_node | head -1)
  echo "  SLAM_PID=$SLAM_PID  BW_PID=$BW_PID"
  if [ -n "$SLAM_PID" ] && [ -n "$BW_PID" ]; then
    sudo -n ~/dyn_controller -p "$SLAM_PID" -t "$BW_PID" \
      -l "$LIB" -C "$CALIB" -c 0,1 -F 120 \
      -o "$OUT/controlled_t${t}.csv" > "$OUT/controlled_t${t}.stderr" 2>&1 &
  else
    echo "  WARNING: missing PID, controller NOT started for trial $t"
  fi
  taskset -c 0,1 ros2 bag play "$BAG" -r $RATE < /dev/null > /dev/null 2>&1
  finish_trial "$OUT/controlled_t${t}.txt"
  grep -c STOP "$OUT/controlled_t${t}.csv" 2>/dev/null | xargs echo "  STOP events:"
done

echo "DONE $(date +%H:%M:%S)"
echo "Evaluate trajectories:"
echo "  python3 ~/evaluate_all.py ~/datasets/gt_official.txt $OUT 'solo_t*.txt'"
echo "  python3 ~/evaluate_all.py ~/datasets/gt_official.txt $OUT 'uncontrolled_t*.txt'"
echo "  python3 ~/evaluate_all.py ~/datasets/gt_official.txt $OUT 'controlled_t*.txt'"
echo "Bandwidth logs at: $OUT/*.bwlog"
