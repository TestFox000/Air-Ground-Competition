#!/bin/bash
# 纸鸢夜巡：只读探针。不启动/不停止任何节点，只看相机话题。
source /opt/ros/humble/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=118
echo "=== topics(image) ==="
ros2 topic list 2>/dev/null | grep -i image || echo "no image topics"
echo "=== hz /image_raw ==="
timeout 8 ros2 topic hz /image_raw 2>&1 | head -4 || echo "hz failed"
echo "=== det node alive ==="
ros2 node list 2>/dev/null | head -10 || echo "node list failed"
