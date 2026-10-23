#!/bin/bash
LOG=/home/duzhong/r3_field_20261021/logs
pkill -TERM -f 'dzbrain.main --config /home/duzhong/r3_field_20261021' 2>/dev/null
sleep 2
cd /home/duzhong/dzrobot
source /opt/ros/humble/setup.bash
source /home/duzhong/dzrobot/dzros.env
source /home/duzhong/dzrobot/install/setup.bash
export DZ_ROS_DOMAIN=118
export ROS_DOMAIN_ID=118
export ROS_LOCALHOST_ONLY=0
export DZ_SKIP_CONTRACT=1
PYTHONUNBUFFERED=1 PYTHONIOENCODING=utf-8 setsid python3 -u -m dzbrain.main --config /home/duzhong/r3_field_20261021/config/robot_r3.yaml --map /home/duzhong/dzrobot/config/map.yaml > "$LOG/7_path_aim.log" 2>&1 &
echo "brain pid=$!"
sleep 13
echo "=== PROC ==="
pgrep -af 'dzbrain.main|serial_reader_node' | head -4
echo "=== ODOM VIEW ==="
ros2 topic info /odom/localization 2>/dev/null | head -4
echo "=== BRAIN STATE ==="
grep -n '阶段\|安全\|契约\|跳过\|位姿' "$LOG/7_path_aim.log" | tail -8
echo "=== BELIEF ==="
grep -n '\[T\]' "$LOG/7_path_aim.log" | tail -3
