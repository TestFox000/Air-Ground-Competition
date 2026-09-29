#!/usr/bin/env bash
# 底盘为什么不转：只读，读它自己上报的状态
#
# 已知（2026-09-29 对比实验）：直接往 /cmd_vel 和 /pursuitAngle 灌 0.3 m/s，
# 车都是 0.000 m —— 底盘收到了速度却不下发到电机。
# 所以这里不再碰控制话题，只做一件事：**把底盘自己说出来的状态全读一遍**，
# 找出"使能 / 模式 / 急停"这一层到底处于什么状态。
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
mkdir -p /tmp/ag_cs

PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_cs/${n}.log" 2>&1 & PGRPS="$PGRPS $!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 2
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  echo "  已停止本脚本启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 底盘状态体检（只读，不发车） ===================="
date
echo

hr; echo "## 1. 起底盘 + 定位"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 10
start localization ros2 launch car_localization localization.launch.py
sleep 6
echo

hr; echo "## 2. 场上全部话题（不猜名字）"
timeout 10 ros2 topic list 2>/dev/null | sed 's/^/    /'
echo

hr; echo "## 3. 底盘自己发布的那些：逐个读一次"
TOPICS="$(timeout 10 ros2 topic list 2>/dev/null)"
for t in $(echo "$TOPICS" | grep -iE "volt|batter|state|valid|imu|odom|navi|voice|mode|pow|charge" | head -14); do
  echo "  -- $t --"
  timeout 6 ros2 topic echo --once "$t" 2>/dev/null | head -6 | sed 's/^/      /' \
    || echo "      （读不到）"
done
echo

hr; echo "## 4. 底盘节点的参数（可能藏着使能/模式开关）"
timeout 10 ros2 param list /dzactuator 2>/dev/null | sed 's/^/    /' | head -40
echo
echo "  -- 与模式/使能相关的参数取值 --"
for p in $(timeout 10 ros2 param list /dzactuator 2>/dev/null | grep -iE "mode|enable|stop|voice|switch|drive" | head -12); do
  echo "      $p = $(timeout 6 ros2 param get /dzactuator "$p" 2>/dev/null)"
done
echo

hr; echo "## 5. 底盘串口侧的打印（rx32 = 底盘上报状态字）"
grep -aE "Trigger|rx32|voice|serial|port|error|fail" /tmp/ag_cs/actuator.log 2>/dev/null | tail -12 | sed 's/^/    /'
echo
echo "==================== 结束 ===================="
