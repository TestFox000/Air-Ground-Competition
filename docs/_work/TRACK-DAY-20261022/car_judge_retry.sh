#!/bin/bash
LOG=/home/duzhong/r3_field_20261021/logs
cd /home/duzhong/dzrobot
source /opt/ros/humble/setup.bash
source /home/duzhong/dzrobot/dzros.env
source /home/duzhong/dzrobot/install/setup.bash
export ROS_DOMAIN_ID=118
echo "=== DEV ==="
ls -l /dev/dzjudgment /dev/ttyACM* 2>&1 | head -5
echo "=== KILL OLD ==="
pkill -TERM -f 'judgment.launch.py' 2>/dev/null
pkill -TERM -f 'dzjudgment' 2>/dev/null
sleep 3
echo "=== DEV AFTER ==="
ls -l /dev/dzjudgment 2>&1 | head -2
echo "=== RESPAWN ==="
setsid ros2 launch dzjudgment judgment.launch.py > "$LOG/6_judgment_retry.log" 2>&1 &
sleep 5
echo "=== LOG ==="
tail -8 "$LOG/6_judgment_retry.log"
echo "=== LASER TOPIC ==="
ros2 topic info /LaserShot_Command 2>/dev/null | head -6
echo "=== NODES ==="
ros2 node list 2>/dev/null | grep -i judg
