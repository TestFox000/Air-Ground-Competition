#!/bin/bash
# 检查无人机侧 /livox/lidar 与 /livox/imu 的实际发布速率与节点清单
# 注意：不要用 set -u —— ROS 的 setup.bash 会引用未绑定变量
set +u

source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"

echo "=== 1. livox topics (type) ==="
ros2 topic list -t 2>/dev/null | grep -i livox

echo
echo "=== 2. /livox/lidar rate (12s sample) ==="
timeout 12 stdbuf -oL ros2 topic hz /livox/lidar 2>&1 | head -4

echo
echo "=== 3. /livox/imu rate (8s sample) ==="
timeout 8 stdbuf -oL ros2 topic hz /livox/imu 2>&1 | head -3

echo
echo "=== 4. node list (what the car should discover) ==="
ros2 node list 2>/dev/null | head -10

echo
echo "=== 5. all topics ==="
ros2 topic list 2>/dev/null | head -20

echo
echo "=== 6. domain / rmw / addrs ==="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset=0>}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset=default>}"
hostname -I
