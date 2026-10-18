#!/bin/bash
# 触发扫描锁定后录制 20s（用于 D 值对比）
set +u
cd /home/duzhong/dzrobot || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source ./dzros.env
P() { ros2 topic pub --once "$1" std_msgs/msg/UInt8 "{data: $2}" >/dev/null 2>&1; }
P /nav/manual_gimbal 0
P /nav/scan_request 2
sleep 8
echo "状态(1=SCAN 2=TRACK):"; timeout 3 ros2 topic echo /aim/status --once 2>/dev/null | head -1
python3 /tmp/log_aim.py /tmp/d45.csv 20
wc -l /tmp/d45.csv
