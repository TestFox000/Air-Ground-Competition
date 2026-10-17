#!/bin/bash
# 只读探针：车的位置与决策层状态
source /opt/ros/humble/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=118
echo "=== odom (once) ==="
timeout 8 ros2 topic echo /odom --once 2>&1 | head -40
echo "=== nodes ==="
timeout 5 ros2 node list 2>&1 | head -20
echo "=== mission logs tail ==="
ls -lt /home/duzhong/dzrobot/robot_logs/ 2>/dev/null | head -6
