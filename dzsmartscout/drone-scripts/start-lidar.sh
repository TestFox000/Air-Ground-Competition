#!/usr/bin/env bash
# start-lidar.sh - 启动 Livox MID-360 激光雷达驱动（AR250 / dzsmart 无人机）
# 用法: bash start-lidar.sh
# 输出日志: /tmp/livox_launch.log

LOG=/tmp/livox_launch.log

echo "== start-lidar.sh $(date '+%F %T') ==" | tee -a "$LOG"

# 1. 检查雷达网口链路（MID360 走 eno1，主机 192.168.3.50）
CARRIER=$(cat /sys/class/net/eno1/carrier 2>/dev/null)
echo "eno1 carrier=$CARRIER" | tee -a "$LOG"
if [ "$CARRIER" != "1" ]; then
  echo "WARN: eno1 无链路（carrier=$CARRIER），雷达航插可能未插好或未上电" | tee -a "$LOG"
  # 仍尝试拉起网口，若链路恢复 NM 会自动配置 192.168.3.50
  echo duzhong | sudo -S ip link set eno1 up 2>>"$LOG"
  sleep 2
  CARRIER=$(cat /sys/class/net/eno1/carrier 2>/dev/null)
  echo "eno1 carrier(重试后)=$CARRIER" | tee -a "$LOG"
else
  echo duzhong | sudo -S ip link set eno1 up 2>>"$LOG" || true
fi

# 2. 探测雷达本体 192.168.3.105
echo "ping 192.168.3.105:" | tee -a "$LOG"
ping -c 2 -W 2 192.168.3.105 >>"$LOG" 2>&1
tail -n 2 "$LOG"

# 3. 加载 ROS2 环境
source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"

# 4. 若已有驱动实例在跑则先停
if pgrep -f "livox_ros_driver2" >/dev/null; then
  echo "已有 livox 驱动进程，先停止旧实例" | tee -a "$LOG"
  pkill -f "livox_ros_driver2" || true
  sleep 2
fi

# 5. 启动驱动（前台运行，日志实时追加；Ctrl+C 停止）
echo "启动: ros2 launch livox_ros_driver2 msg_MID360_launch.py" | tee -a "$LOG"
ros2 launch livox_ros_driver2 msg_MID360_launch.py 2>&1 | tee -a "$LOG"
