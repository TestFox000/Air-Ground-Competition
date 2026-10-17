#!/bin/bash
# 只读探测：雷达 + 相机现状。不启动任何节点。
echo "=== 0. 主机与时间 ==="
hostname; uptime -p; date
echo
echo "=== 1. 相关进程 ==="
pgrep -af 'livox|fastlio|v4l2|pc2_bridge|rviz|MicroXRCE|px4_|odom_convert' || echo "(无相关进程)"
echo
echo "=== 2. eno1 载波（雷达以太网 192.168.3.x） ==="
cat /sys/class/net/eno1/carrier 2>/dev/null && echo " (1=up 0=down)" || echo "eno1 不存在"
ip -4 addr show eno1 2>/dev/null | grep inet || echo "eno1 无 IP"
echo
echo "=== 3. 雷达 ping 192.168.3.105 ==="
ping -c 2 -W 2 192.168.3.105 2>&1 | tail -3
echo
echo "=== 4. 相机 ==="
ls -l /dev/video* 2>/dev/null || echo "NO /dev/video*"
echo "--- lsusb ---"
lsusb
echo
echo "=== 5. ROS 守护进程与话题 ==="
source /opt/ros/humble/setup.bash 2>/dev/null
if pgrep -x ros2 >/dev/null 2>&1 || pgrep -f 'ros2_daemon' >/dev/null 2>&1; then
    timeout 6 ros2 topic list 2>/dev/null || echo "(话题列表超时)"
else
    echo "(ROS 守护进程未运行 — 正常，刚重启)"
fi
echo
echo "=== 6. 工作区 install 是否存在 ==="
for d in ~/dzpx4/install ~/dzlink_ws/install; do
    [ -f "$d/setup.bash" ] && echo "OK  $d/setup.bash" || echo "MISSING  $d/setup.bash"
done
echo
echo "=== 7. v4l2_camera 包是否装 ==="
dpkg -l ros-humble-v4l2-camera 2>/dev/null | grep -q '^ii' && echo "INSTALLED" || echo "NOT installed"
echo
echo "=== 8. 雷达 SN 配置 ==="
grep -o '47MCN850037605' ~/dzpx4/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py 2>/dev/null && echo "SN 配置存在" || echo "SN 未找到或文件路径不同"
