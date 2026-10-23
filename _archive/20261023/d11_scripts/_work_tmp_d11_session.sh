#!/bin/bash
set +u
WS=/home/duzhong/dzrobot
R3=/home/duzhong/r3_field_20261021
LOG=$R3/logs
source /opt/ros/humble/setup.bash
source $WS/dzros.env
source $WS/install/setup.bash
cd $WS
CAP=/tmp/d11cap
rm -rf $CAP; mkdir -p $CAP
cap() { nohup timeout 52 ros2 topic echo "$1" > "$CAP/$2.txt" 2>&1 < /dev/null & }
cap /aim/fire_gate gate
cap /aim/fire_ready ready
cap /aim/lead_x lead
cap /offset_center offset
cap /LaserShot_Command laser
cap /aim/status status
echo "CAP_STARTED $(date -Is)"
sleep 3
echo "=== PRE pursuitAngle ==="
timeout 5 ros2 topic echo --once --qos-reliability best_effort /pursuitAngle
echo "=== CMD reset ==="
timeout 6 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:D11,x'}"
sleep 1
echo "=== CMD single ==="
timeout 6 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: '单点 D11*,x'}"
echo "CMD_SENT $(date -Is)"
sleep 46
echo "=== CAP COUNTS ==="
for f in gate ready laser; do printf "%s: %s lines, ones=%s\n" "$f" "$(wc -l < $CAP/$f.txt)" "$(grep -c 'data: 1' $CAP/$f.txt)"; done
echo "=== dzbrain tail (mission/aim) ==="
grep -a -E "单点|到点|停站|开火|FINISH|SETTLE|ACQUIRE|FIRE|跳过|复位|belief" "$LOG/7_dzbrain.log" | tail -30
echo "=== gimbal tail ==="
tail -12 "$LOG/5_gimbal.log"
echo "=== judgment tail ==="
tail -8 "$LOG/6_judgment.log"
echo "=== DONE $(date -Is) ==="
