#!/bin/bash
LOG=/home/duzhong/r3_field_20261021/logs
cd /home/duzhong/dzrobot
source /opt/ros/humble/setup.bash
source /home/duzhong/dzrobot/dzros.env
source /home/duzhong/dzrobot/install/setup.bash
export DZ_ROS_DOMAIN=118
export ROS_DOMAIN_ID=118
export ROS_LOCALHOST_ONLY=0
echo "=== BEFORE $(date +%H:%M:%S) ==="
wc -l "$LOG/5_gimbal.log" "$LOG/6_judgment.log" "$LOG/7_path_aim.log"
echo "=== REFEREE-NOW ==="
timeout 3 ros2 topic echo --once /HpAndHitmsg 2>&1 | grep -A6 'data:' | head -8
echo "=== SEND ROUTE ==="
timeout 6 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: '多点 E11 D11*,x'}" 2>&1 | tail -1
sleep 58
echo "=== AFTER $(date +%H:%M:%S) ==="
echo "=== ROUTE + FIRE LINES ==="
grep -n '路线载入\|停站点\|任务开始\|到点\|FINISH\|完成\|安全STOP\|不开火\|开火\|发射\|命中\|弹药' "$LOG/7_path_aim.log" | tail -45
echo "=== TAIL ==="
tail -12 "$LOG/7_path_aim.log"
