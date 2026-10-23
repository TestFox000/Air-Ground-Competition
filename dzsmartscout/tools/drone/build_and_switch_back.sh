#!/bin/bash
# 构建 C++ 桥 -> 驱动回退 CustomMsg -> 桥输出标准 PointCloud2（两格式并存）
set +u
source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"

echo "=== 1) 就位包源码 ==="
PKG="$HOME/dzlink_ws/src/dzlink_pc2_bridge"
mkdir -p "$PKG/src"
cp -f /tmp/dzlink_pkg/package.xml /tmp/dzlink_pkg/CMakeLists.txt "$PKG/"
cp -f /tmp/dzlink_pkg/src/pc2_bridge.cpp "$PKG/src/"
find "$PKG" -type f | sort
echo "--- livox 消息包前缀 ---"
ros2 pkg prefix livox_ros_driver2 || { echo "❌ 找不到 livox_ros_driver2"; exit 1; }

echo "=== 2) colcon build ==="
cd "$HOME/dzlink_ws"
colcon build --packages-select dzlink_pc2_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -20
BIN="$HOME/dzlink_ws/install/dzlink_pc2_bridge/lib/dzlink_pc2_bridge/pc2_bridge"
if [ ! -x "$BIN" ]; then
  echo "❌ 构建失败：桥未产生（驱动保持 PointCloud2 模式，车侧此刻仍可直连点云，不中断）"
  exit 1
fi
echo "OK: $BIN"

echo "=== 3) 驱动回退到原厂 CustomMsg 模式（保住无人机 FAST-LIO 契约） ==="
pkill -f livox_ros_driver2_node 2>/dev/null && echo "旧驱动已停" || echo "(无旧驱动)"
pkill -f dzlink_seq_beacon.py 2>/dev/null && echo "Python 序号信标已停（改由 C++ 桥提供）" || true
sleep 3
setsid nohup bash -lc 'source /opt/ros/humble/setup.bash && source $HOME/dzpx4/install/setup.bash && exec ros2 launch livox_ros_driver2 msg_MID360_launch.py' \
  > /tmp/livox_msg_mode.log 2>&1 < /dev/null &
echo "等待雷达初始化与模式切换(25s)..."
sleep 25
grep -E "Init lds|publish use|error|fail" /tmp/livox_msg_mode.log | tail -6

echo "=== 4) 确认 /livox/lidar 已恢复 CustomMsg 且 ~10Hz ==="
timeout 12 ros2 topic list -t 2>/dev/null | grep -i livox
timeout 14 stdbuf -oL ros2 topic hz /livox/lidar 2>&1 | tail -2
timeout 8  stdbuf -oL ros2 topic hz /livox/imu 2>&1 | tail -2

echo "=== 5) 启动 C++ 桥 ==="
setsid nohup bash -lc 'source /opt/ros/humble/setup.bash && source $HOME/dzpx4/install/setup.bash && source $HOME/dzlink_ws/install/setup.bash && exec $HOME/dzlink_ws/install/dzlink_pc2_bridge/lib/dzlink_pc2_bridge/pc2_bridge --ros-args -p seq_rate:=50.0' \
  > /tmp/dzlink_pc2_bridge.log 2>&1 < /dev/null &
sleep 12
pgrep -af pc2_bridge | head -2 || echo "(桥未运行)"
tail -4 /tmp/dzlink_pc2_bridge.log

echo "=== 6) 验证桥输出 ==="
timeout 12 ros2 topic list -t 2>/dev/null | grep -iE "livox|dzlink"
echo "--- /livox/points 频率 ---"
timeout 14 stdbuf -oL ros2 topic hz /livox/points 2>&1 | tail -3
echo "--- /dzlink/seq 频率 ---"
timeout 10 stdbuf -oL ros2 topic hz /dzlink/seq 2>&1 | tail -2
echo "--- 点云内容自检（标准 PC2 反序列化） ---"
timeout 25 python3 /tmp/pc2_check.py /livox/points 8
echo "--- 桥 STAT（subs_* = 已匹配的跨机订阅者数） ---"
grep STAT /tmp/dzlink_pc2_bridge.log | tail -3
echo "=== DONE ==="
