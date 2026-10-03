#!/bin/bash
# 车端综合健康探测（只读，不改任何配置）
echo '=== [1] 系统与 ROS 环境 ==='
hostname; uname -r; echo "ROS: ${ROS_DISTRO:-未加载}"
ls /opt/ros/ 2>/dev/null
command -v ros2 >/dev/null && ros2 --version 2>&1 | head -1 || echo '  (无 ros2 在 PATH)'

echo
echo '=== [2] ag_integrate 工作区状态 ==='
for ws in ~/ag_ws /home/duzhong/ag_ws ~/dev_ws; do
  [ -d "$ws" ] && echo "  $ws 存在" && ls "$ws" 2>/dev/null
done
find /home/duzhong -maxdepth 3 -name 'package.xml' 2>/dev/null | grep -i integrat

echo
echo '=== [3] 串口/数传设备 ==='
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* 2>&1 | head
ls /dev/serial/by-id/ 2>&1

echo
echo '=== [4] 关键进程 ==='
ps aux | grep -Ei 'integrat|ros2|node|actuator' | grep -v grep | head

echo
echo '=== [5] 负载与磁盘 ==='
uptime; df -h / 2>/dev/null | tail -1