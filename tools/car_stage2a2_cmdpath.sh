#!/usr/bin/env bash
# 实机联调 · 第 2 段 A2：**命令链路零位移验证**
#
# 要证明的事：我们发出去的 `/nav/waypoint` 命令格式，真机执行器能正确解析并生效。
# 做法（全程不产生位移）：
#   1. 发 `reset:K4,y`  -> path_executor 的信念位姿应被锚定到 K4
#   2. 读它的日志：应出现 "Odometry reset to K4 facing y"
#   3. 发 `reset:N10,y` -> 恢复到真实位置
#   4. 全程对比 /odom/localization，确认车没动
#
# 为什么用 reset 而不是真发一条路线：reset 只改**信念**，不动电机。
# 这是"能验证命令链路"与"绝不乱动车"之间的最佳折中。
#
# 启动：dzactuator + serial_reader + path_executor（都不产生位移）
# 不启动：dzjudgment（裁判串口，与本验证无关）、dzrknn_dataprocess（会自主开火）

set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
[ -z "${DISPLAY:-}" ] && [ -e /tmp/.X11-unix/X0 ] && \
  export DISPLAY=:0 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

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
send() {
  timeout 10 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: '$1'}" >/dev/null 2>&1
  echo "  已发送: $1"
}

echo "==================== 第 2 段 A2：命令链路零位移验证 ===================="
date
echo

hr; echo "## 1. 启动底盘 + 定位 + 执行器"
timeout 600 ros2 launch dzactuator bringup.launch.py > /tmp/ag_a2_actuator.log 2>&1 &
PIDS+=($!); sleep 9
timeout 600 ros2 launch car_localization localization.launch.py > /tmp/ag_a2_loc.log 2>&1 &
PIDS+=($!); sleep 5
# -u：不加的话 stdout 重定向到文件是全缓冲，日志要等进程结束才看得到
timeout 600 python3 -u "$HOME/dzrobot/src/car_localization/car_localization/path_executor.py" \
  > /tmp/ag_a2_executor.log 2>&1 &
PIDS+=($!); sleep 7

echo "-- 执行器启动日志 --"
grep -E "Initialized|Command format|Example|Heading:" /tmp/ag_a2_executor.log | head -8 | sed 's/^/  /'
echo "-- 当前信念位置（来自 serial_reader）--"
grep -oE "pos: \([0-9.]+, [0-9.]+\) \| block: [A-N][0-9]+ \| yaw: [^ ]+ \[[a-z]+\]" /tmp/ag_a2_loc.log | tail -2 | sed 's/^/  /'
echo

hr; echo "## 2. 基线：车当前在哪"
P0=$(pos); echo "  /odom/localization = $P0"
echo

hr; echo "## 3. 发 reset:K4,y（只改信念，不动电机）"
send "reset:K4,y"; sleep 3
echo "-- path_executor 的反应（关键证据）--"
grep -E "Odometry reset|Failed to reset" /tmp/ag_a2_executor.log | tail -3 | sed 's/^/  /'
if grep -q "Odometry reset to K4" /tmp/ag_a2_executor.log; then
  echo "  [OK] 真机执行器解析并接受了我们的命令格式"
else
  echo "  [FAIL] 没看到 reset 生效的日志"
fi
echo

hr; echo "## 4. 恢复到真实位置：reset:N10,y"
send "reset:N10,y"; sleep 3
grep -E "Odometry reset to N10" /tmp/ag_a2_executor.log | tail -2 | sed 's/^/  /'
if grep -q "Odometry reset to N10" /tmp/ag_a2_executor.log; then
  echo "  [OK] 已恢复到 N10"
fi
echo

hr; echo "## 5. 反证：全程没有位移"
P1=$(pos); echo "  发送前 = $P0"
echo "  发送后 = $P1"
echo "-- 速度指令（应恒为 0）--"
timeout 10 ros2 topic echo --once /pursuitAngle 2>/dev/null | head -4 | sed 's/^/  /'
echo

hr; echo "## 6. 路线语法解析验证（**只解析、不下发**）"
echo "  说明：path_executor 一收到路线就会开始跑，所以这里不真发；"
echo "       改用我们自己的 route_builder 打印将被下发的字符串，人工核对语法。"
python3 - <<'PY'
import sys
sys.path.insert(0, "/tmp/ag_ws/src/ag_car")
from ag_car.route_builder import build_route, reset_command, route_summary
print("  复位命令 :", reset_command("N10", "y"))
print("  演示路线 :", build_route(["K9", "K4", "D2", "B2", "C3", "G8", "N11"], ["D2"], "y"))
s = route_summary(["N9", "K14", "D2*", "N11"], shoot_at=["D2"])
print("  含非法格号时:", s["invalid"], "-> 会被整体拒绝，不会发出半个命令")
PY
echo

echo "==================== 结束（节点将自动停止） ===================="
