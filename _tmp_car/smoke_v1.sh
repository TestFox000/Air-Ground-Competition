#!/bin/bash
# 冒烟：确认 v1 模式下 指令在发、状态机 0->1(SCAN)、云台确实动，然后停止扫描回 IDLE
set +u
cd /home/duzhong/dzrobot || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source ./dzros.env
P() { ros2 topic pub --once "$1" std_msgs/msg/UInt8 "{data: $2}" >/dev/null 2>&1; }

echo "== 指令频率 =="; timeout 5 ros2 topic hz /CameMo_Conl 2>&1 | tail -2
echo "== 视觉频率 =="; timeout 5 ros2 topic hz /offset_center 2>&1 | tail -2
echo "== 起扫前状态 =="; timeout 3 ros2 topic echo /aim/status --once 2>/dev/null | head -1
P /nav/manual_gimbal 0; sleep 0.5
P /nav/scan_request 2; sleep 4
echo "== 起扫 4s 后状态 =="; timeout 3 ros2 topic echo /aim/status --once 2>/dev/null | head -1
echo "== 反馈位置(应在扫描中变化) =="
for i in 1 2 3; do timeout 2 ros2 topic echo /camemo_feedback --once 2>/dev/null | grep -A3 'data:' | tr '\n' ' '; echo; sleep 1; done
echo "== 停扫 =="; P /nav/scan_request 0; sleep 2
timeout 3 ros2 topic echo /aim/status --once 2>/dev/null | head -1
echo done
