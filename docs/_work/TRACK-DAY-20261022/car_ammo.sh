#!/bin/bash
LOG=/home/duzhong/r3_field_20261021/logs
cd /home/duzhong/dzrobot
source /opt/ros/humble/setup.bash
source /home/duzhong/dzrobot/dzros.env
source /home/duzhong/dzrobot/install/setup.bash
echo "===TOPICS==="
ros2 topic list 2>/dev/null | grep -i -E 'referee|hp|hit|shot|laser|judg' | head -20
echo "===REFEREE-AMMO-ONCE==="
timeout 3 ros2 topic echo --once /referee/ammo 2>&1 | head -5
echo "===HP-ONCE==="
timeout 3 ros2 topic echo --once /HpAndHitmsg 2>&1 | head -6
echo "===JUDGMENT-LOG-TAIL==="
tail -25 "$LOG/6_judgment.log"
echo "===BRAIN-REF-LINES==="
grep -n '方位\|弹药\|referee\|HpAndHit\|发数' "$LOG/7_path_aim.log" | tail -10
