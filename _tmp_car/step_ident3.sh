#!/bin/bash
# 阶跃辨识 v3（无靶）：扫描途中直接发 scan_request=0
#   扫描中(云台在侧向角) -> 回 IDLE -> 回正到 2048/2048
#   ⇒ cmd 是一个"瞬间跳到 2048"的阶跃；fb 以 idle_homing_speed 跟随 => 可辨识死区时间/上升速率/超调
set +u
cd /home/duzhong/dzrobot || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source ./dzros.env
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-118}"
P() { ros2 topic pub --once "$1" std_msgs/msg/UInt8 "{data: $2}" >/dev/null 2>&1; }

P /nav/manual_gimbal 0; sleep 0.5
echo "== 起扫 4s 甩到侧向 =="; P /nav/scan_request 2; sleep 4.0
echo "== 录制 6s；+0.5s 发 scan_request=0（回正阶跃）=="
setsid nohup python3 /tmp/log_aim.py /tmp/step3.csv 6 >/tmp/log_step3.out 2>&1 < /dev/null &
REC=$!
sleep 0.5
P /nav/scan_request 0
wait $REC 2>/dev/null
cat /tmp/log_step3.out 2>/dev/null
wc -l /tmp/step3.csv 2>/dev/null
P /nav/scan_request 0
echo done
