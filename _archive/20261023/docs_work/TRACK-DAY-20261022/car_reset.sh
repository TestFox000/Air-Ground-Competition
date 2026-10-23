#!/bin/bash
LOG=/home/duzhong/r3_field_20261021/logs
cd /home/duzhong/dzrobot
source /opt/ros/humble/setup.bash
source /home/duzhong/dzrobot/dzros.env
source /home/duzhong/dzrobot/install/setup.bash
echo "===SEND RESET==="
timeout 6 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:E11,x'}" 2>&1 | tail -2
sleep 3
echo "===LOC-RESET-LOG==="
grep -n 'reset:' "$LOG/3_localization_aim.log" | tail -4
tail -4 "$LOG/3_localization_aim.log"
echo "===BRAIN-RESET-LOG==="
grep -n '复位 belief\|reset 命令' "$LOG/7_path_aim.log" | tail -4
echo "===BRAIN-BELIEF-NOW==="
grep -n '\[T\]' "$LOG/7_path_aim.log" | tail -3
