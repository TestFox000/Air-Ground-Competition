#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/duzhong/dzpx4/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
echo '### PATHLOG_TAIL'
LOG=/home/duzhong/drone_logs/20261017_195407_path.log
tail -n 3 $LOG
echo '### JUMP_WARN_COUNT'
grep -c 'SLAM position jump' $LOG
tail -n 200 $LOG | grep -c 'SLAM position jump' || true
echo '### REC_PROC'
pgrep -af 'dzmap/rec.py' || echo 'no recorder'
echo '### RESTART_RECORDER'
rm -f /tmp/dzmap/STOP /tmp/dzmap/DONE
export DZMAP_DUR=3600
setsid nohup python3 /tmp/dzmap/rec.py </dev/null >>/tmp/dzmap/rec.log 2>&1 &
sleep 4
pgrep -af 'dzmap/rec.py' || echo 'NOT RUNNING'
echo '### FMU'
timeout 6 ros2 topic info /fmu/out/vehicle_status 2>&1 | head -4
pgrep -af MicroXRCEAgent | head -3
echo '### REPLAY_SRC'
grep -rn -i 'replay' /home/duzhong/dzpx4/src/fast_lio_localization/src --include='*.cpp' --include='*.hpp' 2>/dev/null | head -10
echo '### END'
