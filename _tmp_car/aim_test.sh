#!/bin/bash
# 云台增量式 PID 实车实测驱动脚本
#   1) 先起录制（55s，写 /tmp/aim_pid.csv）
#   2) 解除手动接管 → 3) 触发全向扫描（起扫→锁定）
#   4) 每 5s 采样 /aim/status 共 50s
#   5) 结束后重新冻结，避免云台长时间空扫
set -u
cd /home/duzhong/dzrobot || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source ./dzros.env
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-118}"
mkdir -p /home/duzhong/robot_logs

echo "== 1) 起录制 /tmp/aim_pid.csv (55s) =="
setsid nohup python3 /tmp/log_aim.py /tmp/aim_pid.csv 55 >/tmp/log_aim.out 2>&1 < /dev/null &
REC=$!
sleep 1.5

echo "== 2) 解除手动接管 =="
ros2 topic pub --once /nav/manual_gimbal std_msgs/msg/UInt8 "{data: 0}" >/dev/null 2>&1
sleep 1.0

echo "== 3) 触发扫描 scan_request=2 =="
ros2 topic pub --once /nav/scan_request std_msgs/msg/UInt8 "{data: 2}" >/dev/null 2>&1

echo "== 4) 采样 aim/status =="
for i in $(seq 1 10); do
  st=$(timeout 3 ros2 topic echo /aim/status --once 2>/dev/null | tr -d '\n' | tr -s ' ')
  echo "t=$((i*5))s status=[${st}]"
  sleep 5
done

echo "== 5) 冻结云台 =="
ros2 topic pub --once /nav/manual_gimbal std_msgs/msg/UInt8 "{data: 1}" >/dev/null 2>&1
wait $REC 2>/dev/null
echo "== 录制输出 =="
cat /tmp/log_aim.out 2>/dev/null
wc -l /tmp/aim_pid.csv 2>/dev/null
echo "done"
