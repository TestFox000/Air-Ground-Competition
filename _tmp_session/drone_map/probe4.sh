#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/duzhong/dzpx4/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
LOG=$(ls -t /home/duzhong/.ros/log/fastlio_mapping_*.log | head -1)
echo "### FASTLIO_LOG $LOG"
wc -l $LOG
grep -c -i -E 'warn|error|fail|degener|reset' $LOG
echo '--- tail 40'
tail -n 40 $LOG
echo '### GREP_WARN_LAST'
grep -i -E 'warn|error|fail|degener|reset|drop' $LOG | tail -n 15
echo '### IMU_GYRO'
timeout 6 ros2 topic echo /livox/imu --once 2>&1 | sed -n '/angular_velocity:/,/linear_acceleration:/p' | head -8
echo '### SCREENSHOT'
DISPLAY=:1 gnome-screenshot -f /tmp/dzmap/screen.png 2>&1 | head -3; ls -la /tmp/dzmap/screen.png 2>&1
echo '### END'
