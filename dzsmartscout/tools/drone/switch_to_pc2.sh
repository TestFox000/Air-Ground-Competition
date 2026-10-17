#!/bin/bash
# 切换到「标准 PointCloud2」模式：/livox/lidar 由 CustomMsg 变为 sensor_msgs/PointCloud2
# 回退一条命令即可： ros2 launch livox_ros_driver2 msg_MID360_launch.py
set +u                       # ROS setup.bash 会引用未定义变量，不能开 -u
source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"

echo "=== A) 证据收集：本机谁在消费 /livox/lidar（判断改格式是否影响自身栈） ==="
for d in "$HOME/dzuav" "$HOME/dzpx4"; do
  echo "--- $d ---"
  [ -d "$d" ] || { echo "(不存在)"; continue; }
  grep -rn --include=*.yaml --include=*.yml --include=*.json --include=*.py \
           --include=*.cpp --include=*.hpp --include=*.launch \
           "livox/lidar" "$d/src" 2>/dev/null | grep -v "/livox_ros_driver2/" | head -12
done
echo "--- 当前域内节点（除驱动外还有别的吗） ---"
timeout 10 ros2 node list 2>/dev/null || true

echo "=== B) 校验 config 路径 ==="
CFG="$HOME/dzpx4/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json"
if [ -f "$CFG" ]; then echo "OK: $CFG"; else echo "❌ 缺 config: $CFG"; exit 1; fi

echo "=== C) 停旧驱动（CustomMsg 模式）与旧 Python 桥 ==="
pkill -f livox_ros_driver2_node 2>/dev/null && echo "旧驱动已停" || echo "(无旧驱动)"
pkill -f dzlink_livox_bridge.py 2>/dev/null && echo "旧桥已停" || echo "(无旧桥)"
sleep 3

echo "=== D) 以 xfer_format=0 (PointCloud2) 重启驱动 ==="
setsid nohup bash -lc 'source /opt/ros/humble/setup.bash && source $HOME/dzpx4/install/setup.bash && exec ros2 launch /tmp/dzlink_mid360_pc2_launch.py' \
  > /tmp/livox_pc2.log 2>&1 < /dev/null &
echo "等待雷达初始化与工作模式切换（25s）..."
sleep 25

echo "=== E) 驱动日志关键行 ==="
grep -E "init|success|error|fail|format|frame|publish" /tmp/livox_pc2.log | tail -12

echo "=== F) 话题类型 ==="
timeout 12 ros2 topic list -t 2>/dev/null | grep -i livox

echo "=== G) /livox/lidar 频率（应为 ~10Hz） ==="
timeout 14 stdbuf -oL ros2 topic hz /livox/lidar 2>&1 | tail -3

echo "=== H) /livox/imu 频率（应为 ~200Hz） ==="
timeout 8 stdbuf -oL ros2 topic hz /livox/imu 2>&1 | tail -2

echo "=== I) 点云内容自检（标准 PC2 反序列化） ==="
timeout 25 python3 /tmp/pc2_check.py /livox/lidar 8

echo "=== J) 启动序号信标（50Hz Int64） ==="
pkill -f dzlink_seq_beacon.py 2>/dev/null; sleep 1
setsid nohup bash -lc 'source /opt/ros/humble/setup.bash && exec python3 /tmp/dzlink_seq_beacon.py --ros-args -p seq_rate:=50.0' \
  > /tmp/dzlink_seq.log 2>&1 < /dev/null &
sleep 7
pgrep -af dzlink_seq_beacon.py || echo "(信标未起)"
tail -3 /tmp/dzlink_seq.log

echo "=== K) 最终话题清单 ==="
timeout 12 ros2 topic list -t 2>/dev/null | grep -iE "livox|dzlink"
echo "=== DONE ==="
