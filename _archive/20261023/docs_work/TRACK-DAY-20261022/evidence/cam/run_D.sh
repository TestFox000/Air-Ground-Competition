#!/bin/bash
LOG=/home/duzhong/r3_field_20261021/logs
cd /home/duzhong/dzrobot
source /opt/ros/humble/setup.bash
source /home/duzhong/dzrobot/dzros.env
source /home/duzhong/dzrobot/install/setup.bash
export DZ_ROS_DOMAIN=118 ROS_DOMAIN_ID=118 ROS_LOCALHOST_ONLY=0
STAMP=$(date +%H%M%S)
echo "=== START $STAMP ==="
PYTHONUNBUFFERED=1 setsid python3 -u /tmp/aim_rec2.py /tmp/aim_rec_$STAMP.jsonl 70 > /tmp/aim_rec_$STAMP.out 2>&1 &
for i in 1 2 3 4 5 6; do
  timeout 8 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: '多点 E11 D11*,x'}" >/dev/null 2>&1
  sleep 1
done
echo "SENT x6 at $(date +%H:%M:%S)"
sleep 45
echo "=== KEY ==="
grep -n "路线载入\|停站点\|停稳\|ACQUIRE\|FIRE\|任务结束\|到点" "$LOG/7_path_B.log" | tail -15
echo "=== PHASE ==="
grep "\[T\]" "$LOG/7_path_B.log" | tail -2
echo "=== REASONS ==="
grep -o "不开火：[^|]*" "$LOG/7_path_B.log" | tail -150 | sed 's/不开火：//' | sort | uniq -c | sort -rn | head -6
echo "REFEREE=$(timeout 3 ros2 topic echo --once /HpAndHitmsg 2>/dev/null | tr '\n' ' ' | head -c 160)"
echo "RECFILE=/tmp/aim_rec_$STAMP.jsonl"
