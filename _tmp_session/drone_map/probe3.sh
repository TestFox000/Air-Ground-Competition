#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/duzhong/dzpx4/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
echo '### HZ_LIVOX_IMU'
timeout 5 ros2 topic hz /livox/imu 2>&1 | head -3
echo '### HZ_LIVOX_LIDAR'
timeout 5 ros2 topic hz /livox/lidar 2>&1 | head -3
echo '### HZ_FMU_STATUS'
timeout 5 ros2 topic hz /fmu/out/vehicle_status 2>&1 | head -3
echo '### HZ_FMU_LPOS'
timeout 5 ros2 topic hz /fmu/out/vehicle_local_position 2>&1 | head -3
echo '### IMU_SAMPLE'
timeout 6 ros2 topic echo /livox/imu --once 2>&1 | sed -n '/linear_acceleration:/,/orientation:/p' | head -12
echo '### FMU_LPOS'
timeout 8 ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>&1 | grep -E 'x:|y:|z:|xy_valid|z_valid|heading' | head -8
echo '### FMU_STATUS'
timeout 8 ros2 topic echo /fmu/out/vehicle_status --qos-reliability best_effort --once 2>&1 | grep -E 'arming_state|nav_state|failsafe' | head -6
echo '### ROSLOG'
ls -lt /home/duzhong/.ros/log 2>/dev/null | head -8
echo '### SCREEN_TOOLS'
which import scrot xwd gnome-screenshot 2>/dev/null; echo '---'
echo '### END'
