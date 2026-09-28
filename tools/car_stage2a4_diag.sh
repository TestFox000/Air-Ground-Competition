#!/usr/bin/env bash
# 第 2 段 A4：诊断 path_executor 到底怎么处理我们的命令（只读风格，不下发路线）
#
# 前面两次都看不清，原因已定位：Python 节点的 rclpy INFO 日志走 C 层 stdout，
# 非 TTY 时是全缓冲，进程退出时缓冲丢失 —— 所以只有 WARN/ERROR（走 stderr）能落盘。
# 本脚本用 `script` 造一个 pty，强制行缓冲，把 INFO 日志完整抓下来。
#
# 仍然零位移：只发 reset 和"目标=当前格"的单点命令。

set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
[ -z "${DISPLAY:-}" ] && [ -e /tmp/.X11-unix/X0 ] && \
  export DISPLAY=:0 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1     # 关键：让 rcutils 走行缓冲

PIDS=()
cleanup() {
  # 用 SIGKILL 收尾：省得它进 KeyboardInterrupt 分支刷一堆没用的 traceback
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
  pkill -9 -f "path_executor.py" 2>/dev/null
  pkill -9 -f "serial_reader" 2>/dev/null
  echo "  已停止本次启动的节点"
}
trap cleanup EXIT

hr() { echo "----------------------------------------------------"; }

echo "==================== 第 2 段 A4：诊断 ===================="
date
echo

hr; echo "## 1. 起底盘 + 定位"
timeout 900 ros2 launch dzactuator bringup.launch.py > /tmp/ag_a4_actuator.log 2>&1 &
PIDS+=($!); sleep 9
timeout 900 ros2 launch car_localization localization.launch.py > /tmp/ag_a4_loc.log 2>&1 &
PIDS+=($!); sleep 6
grep -oE "block: [A-N][0-9]+ \| yaw: [^ ]+" /tmp/ag_a4_loc.log | tail -1 | sed 's/^/  serial_reader: /'
echo

hr; echo "## 2. 起执行器（用 script 造 pty，INFO 日志才会实时落盘）"
if command -v script >/dev/null 2>&1; then
  script -q -f -c "timeout 900 python3 -u $HOME/dzrobot/src/car_localization/car_localization/path_executor.py" \
    /tmp/ag_a4_executor.log > /dev/null 2>&1 &
  echo "  （用 script + pty）"
else
  stdbuf -o0 -e0 timeout 900 python3 -u \
    "$HOME/dzrobot/src/car_localization/car_localization/path_executor.py" \
    > /tmp/ag_a4_executor.log 2>&1 &
  echo "  （没有 script，退回 stdbuf -o0）"
fi
PIDS+=($!)
sleep 8
echo "-- 启动横幅（完整前 25 行）--"
head -25 /tmp/ag_a4_executor.log | sed 's/^/  /'
echo

hr; echo "## 3. 执行器看到的定位数据（它自己的日志会打 state/pos）"
grep -E "IDLE|TURN|DRIVE|ARRIVED|pos=" /tmp/ag_a4_executor.log | tail -5 | sed 's/^/  /'
echo

hr; echo "## 4. 发 reset:N10,y（零位移）"
timeout 10 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:N10,y'}" >/dev/null 2>&1
sleep 3
grep -E "reset|Reset|Odometry" /tmp/ag_a4_executor.log | tail -5 | sed 's/^/  /'
echo

hr; echo "## 5. 发零距离单点 N10,y（目标=当前格，不会产生位移）"
timeout 10 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'N10,y'}" >/dev/null 2>&1
sleep 6
echo "-- 执行器日志尾部 30 行（不再 grep，避免又漏掉关键词）--"
tail -30 /tmp/ag_a4_executor.log | sed 's/^/  /'
echo

hr; echo "## 6. 定位是否变化（应完全不变）"
timeout 12 ros2 topic echo --once /odom/localization --field pose.pose.position 2>/dev/null | tr '\n' ' ' | sed 's/^/  /'
echo
echo

echo "==================== 结束（节点将自动停止） ===================="
