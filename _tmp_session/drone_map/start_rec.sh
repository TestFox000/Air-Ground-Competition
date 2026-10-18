#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/duzhong/dzpx4/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
export DZMAP_OUT=/tmp/dzmap
export DZMAP_VOX=0.05
export DZMAP_DUR=1800
mkdir -p /tmp/dzmap
rm -f /tmp/dzmap/DONE /tmp/dzmap/STOP /tmp/dzmap/map_final.pcd
pkill -f '/tmp/dzmap/rec.py' 2>/dev/null
sleep 1
setsid nohup python3 /tmp/dzmap/rec.py </dev/null >/tmp/dzmap/rec.log 2>&1 &
sleep 4
echo '### PID'
pgrep -af 'dzmap/rec.py' || echo 'NOT RUNNING'
echo '### LOG'
tail -n 10 /tmp/dzmap/rec.log 2>/dev/null
echo '### FILES'
ls -la /tmp/dzmap
echo '### END'
