#!/bin/bash
# Lidar status check script (run on drone)
source /opt/ros/humble/setup.bash
source ~/dzpx4/install/setup.bash

echo "=== publisher QoS (/livox/lidar) ==="
ros2 topic info /livox/lidar -v 2>&1 | head -40

echo ""
echo "=== MID360 config QoS settings ==="
grep -iE "qos|reliab|durab" ~/dzpx4/src/livox_ros_driver2/config/MID360_config.json

echo ""
echo "=== livox node alive? ==="
pgrep -af livox_ros_driver2_node | head -3
