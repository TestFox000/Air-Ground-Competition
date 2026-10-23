#!/bin/bash
# Quick IMU verification on drone
source /opt/ros/humble/setup.bash
source ~/dzpx4/install/setup.bash
timeout 6 ros2 topic echo /livox/imu --once --field angular_velocity 2>&1 | head -4
echo "=== log error count ==="
grep -icE "error|fail|timeout" /tmp/livox_launch.log
