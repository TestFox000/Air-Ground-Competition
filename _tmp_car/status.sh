#!/bin/bash
# 车端状态快照（无引号地狱）
set +u
cd /home/duzhong/dzrobot || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source ./dzros.env

echo "== 进程 =="
ps -ef | grep '[d]zrknn' | awk '{print $2, $9, $10}' | head -3
echo "== 状态 (0 IDLE / 1 SCAN / 2 TRACK) =="
timeout 3 ros2 topic echo /aim/status --once 2>/dev/null | head -1
echo "== 视觉一帧 (dx dy cls w h ...) =="
timeout 3 ros2 topic echo /offset_center --once 2>/dev/null | sed -n '/^data:/,$p' | head -6
echo "== 云台位置 (P0 P1 S0 S1) =="
timeout 3 ros2 topic echo /camemo_feedback --once 2>/dev/null | sed -n '/^data:/,$p' | head -4
echo "== 当前增益 =="
grep -n -E 'track_mode|pid_kd|v1_kd|v1_kp|v1_inc_gain' config/robot.yaml | head -8
echo "== 横幅 =="
grep -a '跟踪控制' /home/duzhong/robot_logs/5_gimbal.log | tail -1
