#!/usr/bin/env bash
# 第 2 段 A5：**不做任何重定向**，直接看 path_executor 的输出。
#
# 前面被 Python + rclpy 的缓冲机制绕晕了（INFO 走 C stdout，非 TTY 全缓冲，kill 时丢）。
# 这次把它的 stdout 直接接到 SSH 会话上，所见即所得。
#
# 仍然零位移：只发 reset 与"目标=当前格"的单点命令。

source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
[ -z "${DISPLAY:-}" ] && [ -e /tmp/.X11-unix/X0 ] && \
  export DISPLAY=:0 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

hr() { echo "----------------------------------------------------"; }
EXEC="$HOME/dzrobot/src/car_localization/car_localization/path_executor.py"

hr; echo "## 1. 起底盘 + 定位（后台，输出丢 /tmp）"
nohup ros2 launch dzactuator bringup.launch.py > /tmp/ag_a5_actuator.log 2>&1 &
A1=$!
sleep 9
nohup ros2 launch car_localization localization.launch.py > /tmp/ag_a5_loc.log 2>&1 &
A2=$!
sleep 6
echo "  底盘 pid=$A1  定位 pid=$A2"
grep -oE "block: [A-N][0-9]+ \| yaw: [^ ]+" /tmp/ag_a5_loc.log | tail -1 | sed 's/^/  serial_reader: /'
echo

hr; echo "## 2. 发命令（在执行器还没起来之前就发，测试它能否补收）"
echo "  （Linux 下话题发布是「有订阅者才送达」，所以这步主要是确认没爆炸）"
timeout 10 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:N10,y'}" >/dev/null 2>&1
echo "  reset:N10,y 已发"
echo

hr; echo "## 3. 前台起执行器 8 秒，输出直接回传"
echo "  ⚠️ 8 秒后会被 kill；只观察启动日志与它对定位数据的反应"
( sleep 4
  timeout 10 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'N10,y'}" >/dev/null 2>&1
  echo "  [4s 时已发 N10,y]" ) &
timeout 8 python3 -u "$EXEC" 2>&1 | head -60
echo "  [执行器已停止]"
echo

hr; echo "## 4. 收尾"
kill -9 $A1 $A2 2>/dev/null
pkill -9 -f "path_executor.py" 2>/dev/null
pkill -9 -f "serial_reader" 2>/dev/null
echo "  已停掉底盘与定位"
timeout 12 ros2 topic echo --once /odom/localization --field pose.pose.position 2>/dev/null | tr '\n' ' ' | sed 's/^/  最后位置: /'
echo
echo "==================== 结束 ===================="
