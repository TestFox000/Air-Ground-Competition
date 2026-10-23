#!/bin/bash
# 复核无人机侧进程与话题状态（纯只读）
# 注意：comm 名会被内核截断到 15 字符，pgrep -x 全名永远匹配不到 -> 必须用 pgrep -f
set +u

echo "=== 1. lidar driver (pgrep -f) ==="
pgrep -af 'livox_ros_driver2/lib' || echo "NOT RUNNING"

echo
echo "=== 2. launch log tail ==="
tail -4 /tmp/livox_launch.log 2>/dev/null

echo
echo "=== 3. topics now ==="
source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"
ros2 topic list 2>/dev/null

echo
echo "=== 4. rate check /livox/lidar (8s) ==="
timeout 8 stdbuf -oL ros2 topic hz /livox/lidar 2>&1 | head -2

echo
echo "=== 5. beacon still alive? ==="
pgrep -af 'beacon.sh' || echo "beacon gone"
