#!/bin/bash
# 车侧探针启动器：只订阅 /nav/waypoint，不动车、不起栈
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=118
export ROS_LOCALHOST_ONLY=0
nohup python3 /tmp/dz_probe.py > /tmp/dz_probe.log 2>&1 &
echo "probe started pid $!"
