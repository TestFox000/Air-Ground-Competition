#!/usr/bin/env bash
# 第 2 段 A6：强制无缓冲 + 趁进程活着读日志（零位移）
#
# 已定位的问题：rclpy 的 INFO 日志写 C 层 stdout，非 TTY 时是全缓冲；
# 进程被 kill 时缓冲整体丢失 —— 所以我们一直只看到 stderr 的 WARN/traceback。
# 解法：stdbuf -o0（LD_PRELOAD 强制 setvbuf 无缓冲）+ RCUTILS_LOGGING_BUFFERED_STREAM=1，
#       并且**在进程运行期间**就把日志 cat 出来。

source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
[ -z "${DISPLAY:-}" ] && [ -e /tmp/.X11-unix/X0 ] && \
  export DISPLAY=:0 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1

hr() { echo "----------------------------------------------------"; }
LOG=/tmp/ag_a6_executor.log
EXEC="$HOME/dzrobot/src/car_localization/car_localization/path_executor.py"

hr; echo "## 1. 起底盘 + 定位"
nohup ros2 launch dzactuator bringup.launch.py > /tmp/ag_a6_actuator.log 2>&1 &
A1=$!
sleep 9
nohup ros2 launch car_localization localization.launch.py > /tmp/ag_a6_loc.log 2>&1 &
A2=$!
sleep 6
grep -oE "block: [A-N][0-9]+ \| yaw: [^ ]+" /tmp/ag_a6_loc.log | tail -1 | sed 's/^/  serial_reader: /'
echo

hr; echo "## 2. 用 stdbuf -o0 起执行器（强制无缓冲）"
: > "$LOG"
stdbuf -o0 -e0 python3 -u "$EXEC" > "$LOG" 2>&1 &
E=$!
sleep 8
echo "-- 进程状态 --"
kill -0 $E 2>/dev/null && echo "  执行器在跑 (pid=$E)" || echo "  执行器已退出"
echo "-- 【运行中】读日志（共 $(wc -l < "$LOG") 行）--"
sed 's/^/  /' "$LOG"
echo

hr; echo "## 3. 发 reset:N10,y + 零距离单点 N10,y"
timeout 10 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:N10,y'}" >/dev/null 2>&1
echo "  已发 reset:N10,y"; sleep 3
timeout 10 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'N10,y'}" >/dev/null 2>&1
echo "  已发 N10,y"; sleep 6
echo "-- 【运行中】日志尾部 25 行 --"
tail -25 "$LOG" | sed 's/^/  /'
echo

hr; echo "## 4. 让执行器自己干净退出（SIGINT -> 它的收尾逻辑），确保缓冲落盘"
kill -INT $E 2>/dev/null
sleep 4
kill -0 $E 2>/dev/null && { kill -9 $E 2>/dev/null; echo "  （没退干净，补了一刀）"; } || echo "  [OK] 已干净退出（缓冲应已落盘）"
echo "-- 最终日志行数: $(wc -l < "$LOG") --"
grep -E "reset|Reset|reset:N10|Odometry reset|ARRIVED|Navigating" "$LOG" | tail -10 | sed 's/^/  /'
echo

hr; echo "## 5. 收尾 + 位置核对（应完全没动）"
timeout 12 ros2 topic echo --once /odom/localization --field pose.pose.position 2>/dev/null | tr '\n' ' ' | sed 's/^/  位置: /'
echo
kill -9 $A1 $A2 2>/dev/null
pkill -9 -f "path_executor.py" 2>/dev/null
pkill -9 -f "serial_reader" 2>/dev/null
echo "  已停掉底盘与定位"
echo "==================== 结束 ===================="
