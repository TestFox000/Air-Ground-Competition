#!/bin/bash
# 无人机侧跨机链路只读检查（不写任何文件）
export ROS_DOMAIN_ID=118
source /opt/ros/humble/setup.bash 2>/dev/null
echo "=== 域号 = $ROS_DOMAIN_ID ==="
echo
echo "=== 跨机可见的节点总数 ==="
ros2 node list > /tmp/_nl.txt 2>/dev/null
wc -l < /tmp/_nl.txt
echo
echo "=== 其中属于小车的节点 ==="
grep -E 'path_executor|dzactuator|dzjudgment|det_node|serial_reader|cam_viewer|usb_cam|dzrknn' /tmp/_nl.txt
echo
echo "=== /nav/waypoint 详情 ==="
ros2 topic info /nav/waypoint -v 2>&1 | head -30
echo
echo "=== /pursuitAngle 详情（车在执行时才活跃）==="
ros2 topic info /pursuitAngle -v 2>&1 | head -20
echo
echo "=== 车回传话题 ==="
ros2 topic list 2>/dev/null | grep -E 'dzlink|nav/' | head -20
