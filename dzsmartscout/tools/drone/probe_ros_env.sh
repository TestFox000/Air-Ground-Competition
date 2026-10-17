#!/usr/bin/env bash
# probe_ros_env.sh — 核实 ROS2 环境（只读探测，无人机/小车通用）
# 用法: Get-Content probe_ros_env.sh | ssh <host> "bash -s"
set -u
echo "== host: $(hostname) =="
echo "== ros distro =="
ls /opt/ros/ 2>/dev/null
echo "== env =="
bash -lc 'source /opt/ros/humble/setup.bash 2>/dev/null; echo ROS_DISTRO=$ROS_DISTRO; echo DOMAIN=${ROS_DOMAIN_ID:-empty}; echo RMW=${RMW_IMPLEMENTATION:-default}'
echo "== ros2 node list =="
bash -lc 'source /opt/ros/humble/setup.bash 2>/dev/null; timeout 10 ros2 node list 2>&1 | head -15'
echo "== ros2 daemon =="
pgrep -af "ros2 daemon" | head -3
echo "== livox/ros processes =="
pgrep -af "livox|ros2 launch|fastdds|cyclonedds" | head -8
echo "== done =="
