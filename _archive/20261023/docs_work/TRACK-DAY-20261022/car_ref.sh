#!/bin/bash
LOG=/home/duzhong/r3_field_20261021/logs
cd /home/duzhong/dzrobot
source /opt/ros/humble/setup.bash
source /home/duzhong/dzrobot/dzros.env
source /home/duzhong/dzrobot/install/setup.bash
echo "===HP-INFO==="
ros2 topic info /HpAndHitmsg -v 2>/dev/null | head -25
echo "===HP-ECHO-3==="
timeout 4 ros2 topic echo /HpAndHitmsg 2>&1 | head -30
echo "===AMMO-CHANGE-LOG==="
grep -n '弹药变化\|裁判回传\|坏帧\|malformed' "$LOG/7_path_aim.log" | tail -8
echo "===AIM-FIRE-LINES==="
grep -n '开火\|fire_ready\|fire_gate\|发0/中0' "$LOG/7_path_aim.log" | tail -6
echo "===JUDGE-SERIAL==="
grep -c . "$LOG/6_judgment.log"; tail -5 "$LOG/6_judgment.log"
