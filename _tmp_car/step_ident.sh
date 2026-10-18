#!/bin/bash
# 云台位置伺服阶跃辨识（不需要靶子；v2：manual 接管必须先释放，否则节点全程静默）
#   0) 释放接管 -> 1) 起扫 6s 甩到一侧 -> 2) 冻结在那一侧
#   3) 录制 5s，+1.2s 时【释放接管 + scan_request=0】=> 回 IDLE 并回正
#      => 对位置环就是一个"当前角度 -> 正前/水平"的阶跃
#   4) 用 cmd/fb 序列辨识：死区时间、上升时间、超调、稳态误差
set +u
cd /home/duzhong/dzrobot || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source ./dzros.env
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-118}"

P() { ros2 topic pub --once "$1" std_msgs/msg/UInt8 "{data: $2}" >/dev/null 2>&1; }

echo "== 0) 释放接管 =="; P /nav/manual_gimbal 0; sleep 1.0
echo "== 1) 起扫 6s 甩到一侧 =="; P /nav/scan_request 2; sleep 6.0
echo "== 2) 冻结在当前位置 =="; P /nav/manual_gimbal 1; sleep 1.2

echo "== 3) 录制 5s（+1.2s 触发回正阶跃）=="
setsid nohup python3 /tmp/log_aim.py /tmp/step.csv 5 >/tmp/log_step.out 2>&1 < /dev/null &
REC=$!
sleep 1.2
P /nav/manual_gimbal 0
sleep 0.05
P /nav/scan_request 0
wait $REC 2>/dev/null
cat /tmp/log_step.out 2>/dev/null
wc -l /tmp/step.csv 2>/dev/null

echo "== 4) 收尾：保持 IDLE 归位（不残留扫描/接管）=="
P /nav/scan_request 0
echo done
