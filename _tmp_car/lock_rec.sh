#!/bin/bash
# 触发扫描锁定后再录：判定 pid 模式是否真在跟
set +u
cd /home/duzhong/dzrobot || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source ./dzros.env
P() { ros2 topic pub --once "$1" std_msgs/msg/UInt8 "{data: $2}" >/dev/null 2>&1; }

P /nav/manual_gimbal 0; sleep 0.5
P /nav/scan_request 2; sleep 4           # 归位段 -> 扫描 -> 命中锁定
echo "锁定后状态:"; timeout 3 ros2 topic echo /aim/status --once 2>/dev/null | head -1
python3 /tmp/log_aim.py /tmp/${1:-ki24b}.csv 20
wc -l /tmp/${1:-ki24b}.csv
