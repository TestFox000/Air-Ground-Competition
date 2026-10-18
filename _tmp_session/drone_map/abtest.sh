#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/duzhong/dzpx4/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
echo '### STOP_RECORDER'
touch /tmp/dzmap/STOP
sleep 5
pgrep -af 'dzmap/rec.py' || echo 'recorder stopped'
ls -la /tmp/dzmap/map_final.pcd /tmp/dzmap/DONE 2>&1
echo '### SLEEP_3'
sleep 3
echo '### ODOM_AFTER_STOP'
timeout 25 python3 /tmp/dzmap/diag2.py
echo '### END'
