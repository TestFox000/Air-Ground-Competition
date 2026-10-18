#!/bin/bash
echo '### HOST'
hostname; date; uptime
echo '### ROS_SETUP'
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/duzhong/dzpx4/install/setup.bash 2>/dev/null
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
for D in 0 118; do
  export ROS_DOMAIN_ID=$D
  echo "### NODES domain=$D"
  timeout 10 ros2 node list 2>&1 | head -40
  echo "### TOPICS domain=$D"
  timeout 10 ros2 topic list 2>&1 | grep -E 'Odometry|cloud|livox|localization|map_to_odom|fmu|nav/|scan' | head -40
done
echo '### MAPS_DIR'
ls -la /home/duzhong/dzpx4/src/maps/ 2>&1
echo '### PCD_RECENT'
find /home/duzhong -name '*.pcd' -newermt '-3 days' -printf '%TY-%Tm-%Td %TH:%TM %10s %p\n' 2>/dev/null | sort | tail -15
echo '### CFG_HITS'
grep -rn -E 'pcd_save|save_directory|map_file_path' /home/duzhong/dzpx4/src --include='*.yaml' --include='*.py' 2>/dev/null | head -40
echo '### PROCS'
ps -eo pid,etime,pcpu,pmem,cmd --sort=-pcpu | grep -E 'fastlio|livox|global_local|transform_fusion|px4|rviz2|MicroXRCE' | grep -v grep
echo '### DISK'
df -h /home | tail -1
echo '### END'
