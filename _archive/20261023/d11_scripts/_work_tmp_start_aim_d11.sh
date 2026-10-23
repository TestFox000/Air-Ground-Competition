#!/bin/bash
set +u
WS=/home/duzhong/dzrobot
R3=/home/duzhong/r3_field_20261021
LOG=$R3/logs
RUN_TAG=d11_aim
source /opt/ros/humble/setup.bash
source $WS/dzros.env
source $WS/install/setup.bash
mkdir -p "$LOG"
cd "$WS"
spawn() { local name="$1"; shift; echo "=== SESSION START $(date -Is) run=$RUN_TAG ===" >> "$LOG/$name.log"; setsid "$@" >> "$LOG/$name.log" 2>&1 < /dev/null & echo $! > "$LOG/$name.pid"; echo "spawn $name pid=$!"; }
spawn 2_camera ros2 launch dzactuator camera.launch.py
sleep 3
spawn 4_det env DISPLAY=:1 ros2 launch det_node det_node.launch.py
sleep 4
spawn 5_gimbal ros2 launch dzrknn_dataprocess dzrknn_dataprocess.launch.py
sleep 3
spawn 6_judgment ros2 launch dzjudgment judgment.launch.py
echo "AIM_STARTED $(date -Is)"
sleep 14
echo "=== NODES ==="
ros2 node list
echo "=== LASER ==="
ros2 topic info /LaserShot_Command
echo "=== OFFSET ==="
ros2 topic info /offset_center
echo "=== LOG TAILS ==="
for f in 2_camera 4_det 5_gimbal 6_judgment; do echo "--- $f ---"; tail -6 "$LOG/$f.log"; done
echo "DONE"
