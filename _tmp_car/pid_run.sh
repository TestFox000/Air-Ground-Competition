#!/bin/bash
# 起扫 -> 等锁定 -> 录 20s（PID 模式实测）
set +u
cd /home/duzhong/dzrobot || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source ./dzros.env
P() { ros2 topic pub --once "$1" std_msgs/msg/UInt8 "{data: $2}" >/dev/null 2>&1; }
P /nav/manual_gimbal 0
P /nav/scan_request 2
echo "等锁定 8s..."; sleep 8
echo "状态: $(timeout 3 ros2 topic echo /aim/status --once 2>/dev/null | head -1)"
echo "一帧: $(timeout 3 ros2 topic echo /offset_center --once 2>/dev/null | sed -n '/^data:/,$p' | tr '\n' ' ')"
python3 /tmp/log_aim.py /tmp/pid_run.csv 20
wc -l /tmp/pid_run.csv
P /nav/scan_request 0
