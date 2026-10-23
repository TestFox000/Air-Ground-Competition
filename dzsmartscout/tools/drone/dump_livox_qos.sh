#!/bin/bash
# 导出无人机侧 livox 话题的 QoS 与类型，供小车侧选择正确的订阅参数（纯只读）
set +u

source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"

echo "=== 1. /livox/lidar info -v ==="
timeout 15 ros2 topic info /livox/lidar -v 2>&1 | head -30

echo
echo "=== 2. /livox/imu info -v ==="
timeout 15 ros2 topic info /livox/imu -v 2>&1 | head -30

echo
echo "=== 3. config QoS settings ==="
grep -iE 'qos|reliab|durab|history|depth' "$HOME/dzpx4/src/livox_ros_driver2/config/MID360_config.json" 2>/dev/null || echo "no qos keys in config"

echo
echo "=== 4. publish_freq / data_src in config ==="
grep -iE 'publish_freq|data_src|multi_topic|frame_id|timestamp' "$HOME/dzpx4/src/livox_ros_driver2/config/MID360_config.json" 2>/dev/null
