#!/bin/bash
LOG=/home/duzhong/r3_field_20261021/logs
cd /home/duzhong/dzrobot
source /opt/ros/humble/setup.bash
source /home/duzhong/dzrobot/dzros.env
source /home/duzhong/dzrobot/install/setup.bash
setsid ros2 launch car_localization localization.launch.py > "$LOG/3_localization_aim.log" 2>&1 &
echo $! > "$LOG/3_localization_aim.pid"
sleep 4
PYTHONUNBUFFERED=1 PYTHONIOENCODING=utf-8 setsid python3 -u -m dzbrain.main --config /home/duzhong/r3_field_20261021/config/robot_r3.yaml --map /home/duzhong/dzrobot/config/map.yaml > "$LOG/7_path_aim.log" 2>&1 &
echo $! > "$LOG/7_path_aim.pid"
echo "spawned loc=$(cat $LOG/3_localization_aim.pid) brain=$(cat $LOG/7_path_aim.pid)"
sleep 16
echo "=== PROC ==="
pgrep -af 'serial_reader_node|dzbrain.main|localization.launch|det_node|dzjudgment|dzrknn|dzactuator' | head -20
echo "=== LOC TAIL ==="
tail -8 "$LOG/3_localization_aim.log"
echo "=== BRAIN GREP ==="
grep -n '阶段\|安全\|就绪\|复位\|定位' "$LOG/7_path_aim.log" | tail -14
echo "=== BRAIN TAIL ==="
tail -8 "$LOG/7_path_aim.log"
