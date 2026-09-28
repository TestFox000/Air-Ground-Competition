#!/usr/bin/env bash
# 实机联调 · 第 2 段 A3：**用「零距离路线」证明 /nav/waypoint 被真机正确执行**
#
# 思路（全程不产生位移）：
#   车当前就在 N10（serial_reader 实测 block (1.50,0.50)）。
#   发一条单点路线 `N10,y` 给它：
#     - 单点语法 = 恰好一个逗号且逗号前无空格  -> 走 single-target 分支
#     - 目标距当前信念 0 m  < ARRIVAL_DISTANCE  -> 立即 ARRIVED，不驱动电机
#     - ARRIVED 分支会发 `/move_base/stop_signal = 1`（源码里写死的）
#   于是：**收到 stop_signal=1 就证明它真的解析并执行了我们的命令**，
#   而且车一步没动。这比看日志可靠（Python 节点的 rclpy 日志有 C 层缓冲，被 kill 时会丢）。
#
# 同时补上日志：export RCUTILS_LOGGING_BUFFERED_STREAM=0 + stdbuf -oL，让日志能实时落盘。

set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
[ -z "${DISPLAY:-}" ] && [ -e /tmp/.X11-unix/X0 ] && \
  export DISPLAY=:0 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export RCUTILS_LOGGING_BUFFERED_STREAM=0

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
  sleep 2
  for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && kill -9 "$pid" 2>/dev/null; done
  pkill -f "path_executor.py" 2>/dev/null
  pkill -f "serial_reader" 2>/dev/null
  echo "  已停止本次启动的节点"
}
trap cleanup EXIT

hr() { echo "----------------------------------------------------"; }
pos() { timeout 12 ros2 topic echo --once /odom/localization --field pose.pose.position 2>/dev/null | tr '\n' ' '; }

echo "==================== 第 2 段 A3：零距离路线证明 ===================="
date
echo

hr; echo "## 1. 启动底盘 + 定位 + 执行器（日志改为实时落盘）"
timeout 900 ros2 launch dzactuator bringup.launch.py > /tmp/ag_a3_actuator.log 2>&1 &
PIDS+=($!); sleep 9
timeout 900 ros2 launch car_localization localization.launch.py > /tmp/ag_a3_loc.log 2>&1 &
PIDS+=($!); sleep 5
stdbuf -oL -eL timeout 900 python3 -u \
  "$HOME/dzrobot/src/car_localization/car_localization/path_executor.py" \
  > /tmp/ag_a3_executor.log 2>&1 &
PIDS+=($!); sleep 7

echo "-- 执行器启动日志（现在应该能看到）--"
grep -E "Initialized|Command format|Example|Heading|Waiting" /tmp/ag_a3_executor.log | head -8 | sed 's/^/  /'
echo "-- 车当前信念位置 --"
grep -oE "pos: \([0-9.]+, [0-9.]+\) \| block: [A-N][0-9]+ \| yaw: [^ ]+" /tmp/ag_a3_loc.log | tail -2 | sed 's/^/  /'
P0=$(pos); echo "  /odom/localization = $P0"
echo

hr; echo "## 2. 先发起始锚定 reset:N10,y（复位里程计，零位移）"
timeout 10 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:N10,y'}" >/dev/null 2>&1
sleep 3
grep -E "Odometry reset" /tmp/ag_a3_executor.log | tail -2 | sed 's/^/  /' || echo "  （日志里暂无，继续看第 4 节的行为证据）"
echo

hr; echo "## 3. 零距离路线：发送 N10,y（目标=当前格，不会产生位移）"
echo "-- 先在后台监听 /move_base/stop_signal（ARRIVED 时会发 1）--"
( timeout 25 ros2 topic echo --once /move_base/stop_signal > /tmp/ag_a3_stopsignal.log 2>&1 ) &
sleep 2
timeout 10 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'N10,y'}" >/dev/null 2>&1
echo "  已发送: N10,y"
sleep 8
echo "-- stop_signal 收到什么 --"
cat /tmp/ag_a3_stopsignal.log | sed 's/^/  /'
if grep -q "data: 1" /tmp/ag_a3_stopsignal.log; then
  echo "  [OK] 真机执行器解析了我们的命令并走到了 ARRIVED 分支"
  echo "       （stop_signal=1 是源码里 ARRIVED 时才会发的信号）"
else
  echo "  ⚠️ 没收到 stop_signal=1，继续看执行器日志"
fi
echo "-- 执行器日志里的相关行 --"
grep -E "Navigating to|ARRIVED|Current|stop_signal" /tmp/ag_a3_executor.log | tail -8 | sed 's/^/  /'
echo

hr; echo "## 4. 反证：车一步没动"
P1=$(pos); echo "  发送前 = $P0"; echo "  发送后 = $P1"
timeout 10 ros2 topic echo --once /pursuitAngle 2>/dev/null | head -4 | sed 's/^/  /'
echo

hr; echo "## 5. 结论"
echo "  stop_signal=1 收到 => /nav/waypoint 的命令格式被真机正确解析执行（第 3 节）"
echo "  位置不变           => 全程零位移（第 4 节）"
echo "  真正让车跑起来那条路线（如 'K9 K4 D2* B2 C3 G8 N11,y'）留到有人在场的正式联调。"
echo "==================== 结束（节点将自动停止） ===================="
