#!/bin/bash
# 诊断：为什么 Python 桥接只有 3~4 Hz？（先停桥，测干净源头速率）
source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"

echo "=== 0) 停掉 Python 桥（排除它反压驱动） ==="
pkill -f dzlink_livox_bridge.py 2>/dev/null && echo "killed" || echo "(未在运行)"
sleep 3

echo "=== 1) 源头 /livox/lidar 干净速率（12s，纯 rclpy 计数订阅者） ==="
timeout 12 stdbuf -oL ros2 topic hz /livox/lidar 2>&1 | tail -4

echo "=== 2) /livox/imu 速率（8s） ==="
timeout 8 stdbuf -oL ros2 topic hz /livox/imu 2>&1 | tail -3

echo "=== 3) 驱动可用 launch 文件 ==="
LD="$HOME/dzpx4/src/livox_ros_driver2/launch_ROS2"
[ -d "$LD" ] || LD="$HOME/dzpx4/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2"
echo "目录: $LD"
ls -1 "$LD" 2>/dev/null

echo "=== 4) 各 launch 里的 xfer_format / publish_freq 默认值 ==="
grep -rn "xfer_format\|publish_freq" "$LD" 2>/dev/null | head -30

echo "=== 5) 当前驱动进程实际参数 ==="
pgrep -af livox_ros_driver2_node | head -2
echo "--- 从驱动的 tmp 参数文件里找 xfer_format ---"
grep -h "xfer_format\|publish_freq\|multi_topic" /tmp/launch_params_* 2>/dev/null | sort -u | head

echo "=== 6) 驱动日志尾部（有无告警/丢帧） ==="
tail -8 /tmp/livox_launch.log 2>/dev/null
echo "=== DONE ==="
