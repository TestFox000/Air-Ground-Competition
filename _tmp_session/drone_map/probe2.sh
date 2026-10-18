#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/duzhong/dzpx4/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
echo '### VSTATUS_HZ'
timeout 6 ros2 topic hz /fmu/out/vehicle_status 2>&1 | head -4
echo '### ODOM_A'
timeout 5 ros2 topic echo /Odometry --once 2>&1 | sed -n '/position:/,/orientation:/p' | head -5
echo '### ODOM_B'
timeout 5 ros2 topic echo /Odometry --once 2>&1 | sed -n '/position:/,/orientation:/p' | head -5
echo '### LOC_A'
timeout 5 ros2 topic echo /localization --once 2>&1 | sed -n '/position:/,/orientation:/p' | head -5
echo '### PATHLOG'
ls -la /home/duzhong/drone_logs/ 2>&1 | tail -6
tail -n 20 /home/duzhong/drone_logs/20261017_195407_path.log 2>&1
echo '### END'
