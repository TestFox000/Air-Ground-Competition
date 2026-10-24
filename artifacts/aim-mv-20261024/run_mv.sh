#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=118
# 1) 清云台开火预算
timeout 6 ros2 topic pub --once /shoot_reset std_msgs/msg/UInt8 "{data: 1}" > /tmp/mv_reset.log 2>&1
echo "RESET_AT $(date +%H:%M:%S)" | tee /tmp/mv_reset.log
# 2) 记录开录前的日志行数
GLOG=/tmp/t_fire_det.log
JLOG=/tmp/dzjud2.log
wc -l $GLOG $JLOG > /tmp/mv_loglines.txt 2>&1
cat /tmp/mv_loglines.txt
# 3) 录 75 秒
timeout -s INT 78 python3 /tmp/mvrec.py /tmp/mvlog.csv 75
echo "ROWS=$(wc -l < /tmp/mvlog.csv)"
