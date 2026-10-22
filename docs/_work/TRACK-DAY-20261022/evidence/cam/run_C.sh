#!/bin/bash
LOG=/home/duzhong/r3_field_20261021/logs
cd /home/duzhong/dzrobot
source /opt/ros/humble/setup.bash
source /home/duzhong/dzrobot/dzros.env
source /home/duzhong/dzrobot/install/setup.bash
export DZ_ROS_DOMAIN=118 ROS_DOMAIN_ID=118 ROS_LOCALHOST_ONLY=0
STAMP=$(date +%H%M%S)
echo "=== START $STAMP ==="
echo "SUBS /nav/waypoint:"; timeout 8 ros2 topic info /nav/waypoint 2>&1 | head -4
PYTHONUNBUFFERED=1 setsid python3 -u /tmp/aim_rec2.py /tmp/aim_rec_$STAMP.jsonl 75 > /tmp/aim_rec_$STAMP.out 2>&1 &
echo REC_STARTED
for i in 1 2 3 4 5; do
  out=$(timeout 15 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: '多点 E11 D11*,x'}" 2>&1)
  echo "TRY$i: $(echo "$out" | tail -2 | tr '\n' ' ')"
  echo "$out" | grep -q "publishing" && { echo ROUTE_SENT; break; }
  sleep 4
done
sleep 50
echo "=== END $(date +%H:%M:%S) ==="
echo "REFEREE=$(timeout 3 ros2 topic echo --once /HpAndHitmsg 2>/dev/null | tr '\n' ' ' | head -c 160)"
echo "=== KEY ==="
grep -n "路线载入\|停站点\|停稳\|ACQUIRE\|FIRE\|任务结束\|到站" "$LOG/7_path_B.log" | tail -20
echo "=== REASONS(last 60s) ==="
grep -o "不开火：[^|]*" "$LOG/7_path_B.log" | tail -200 | sed 's/不开火：//' | sort | uniq -c | sort -rn | head -8
echo "RECFILE=/tmp/aim_rec_$STAMP.jsonl"; ls -l /tmp/aim_rec_$STAMP.jsonl
