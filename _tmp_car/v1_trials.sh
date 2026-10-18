#!/bin/bash
# v1 模式实车"静止靶偏置起跟"三组录制（靶已确认在视野内：cls=1 蓝，框 77x76）
#   每组：回正(IDLE) 让静止靶偏出画面中心 -> 录制 14s，其中 +1s 触发起扫
#         -> 归位段 -> 扫描 -> 锁定(TRACK) -> 由 v1 控制律把靶拉回中心
#   dx(t) 序列里"锁定时刻后的阶跃响应"就是偏心起跟：可算 超调/整定时间/稳态误差
set +u
cd /home/duzhong/dzrobot || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source ./dzros.env
P() { ros2 topic pub --once "$1" std_msgs/msg/UInt8 "{data: $2}" >/dev/null 2>&1; }

P /nav/manual_gimbal 0
for i in 1 2 3; do
  echo "===== 第 $i 组 ====="
  P /nav/scan_request 0            # 回 IDLE 归位（靶偏出中心）
  sleep 3.0
  setsid nohup python3 /tmp/log_aim.py /tmp/v1_$i.csv 14 >/tmp/log_v1_$i.out 2>&1 < /dev/null &
  REC=$!
  sleep 1.0
  P /nav/scan_request 2            # 起扫 -> 归位 -> 扫描 -> 命中锁定
  wait $REC 2>/dev/null
  echo "  录制行数: $(wc -l < /tmp/v1_$i.csv)"
  P /nav/scan_request 0            # 收尾回 IDLE
  sleep 1.0
done
echo "== 收尾：保持 IDLE =="
P /nav/scan_request 0
echo done
