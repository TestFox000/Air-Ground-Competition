#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/duzhong/dzpx4/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
echo '### KILL_OLD'
kill -9 9942 2>/dev/null; sleep 1
pgrep -af 'dzmap/rec.py' || echo 'none'
echo '### LOGTAIL'
tail -n 5 /tmp/dzmap/rec.log
echo '### STATS'
cat /tmp/dzmap/stats.json
echo ''
echo '### SRC_GREP'
grep -rn 'Propagation rates' /home/duzhong/dzpx4/src 2>/dev/null | head -5
ls -d /home/duzhong/dzpx4/src/*fast* /home/duzhong/dzpx4/src/*FAST* 2>/dev/null
echo '### END'
