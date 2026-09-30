#!/usr/bin/env bash
# 为什么 `car.py move` 读不到定位，而 `car.py preflight` 能读到 50 Hz
#
# 现象（2026-09-30）：preflight 起「底盘+定位」→ /odom/localization 50 Hz 正常；
#                    move 起「底盘+裁判+referee+定位+执行器」→ 连试 13 次全是空。
# 差别在**多出来的那几个节点**。本脚本按 move 的顺序逐个加，每加一个测一次，
# 找出到底是谁把定位"吃"掉了。
#
# ⚠️ 只起节点，不发速度指令，车不会动。
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
[ -f /tmp/ag_ws/install/setup.bash ] && source /tmp/ag_ws/install/setup.bash 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
# 与 move 一致：优先用补丁副本
PE_OFFICIAL="$HOME/dzrobot/src/car_localization/car_localization/path_executor.py"
PE_PATCH="${AG_PATCH_DIR:-/tmp/ag_ws/patched}/car_localization/path_executor.py"
if [ -f "$PE_PATCH" ]; then
  export PYTHONPATH="$(dirname "$(dirname "$PE_PATCH")"):${PYTHONPATH}"; PE="$PE_PATCH"
else
  export PYTHONPATH="$HOME/dzrobot/src/car_localization:${PYTHONPATH}"; PE="$PE_OFFICIAL"
fi
mkdir -p /tmp/ag_lw

PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_lw/${n}.log" 2>&1 & PGRPS="$PGRPS $!"; echo "  起 $n ($!)"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 2
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  echo "  已停止本脚本启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

hz() {  # hz <说明>
  local r
  r="$(timeout 10 ros2 topic hz /odom/localization 2>/dev/null | grep -m1 'average rate')"
  if [ -n "$r" ]; then echo "  [OK] $1 → $r"; else echo "  [--] $1 → 收不到数据"; fi
}

echo "==================== 定位被谁吃掉了 ===================="
date
echo "  执行器: $PE"
echo

hr; echo "## 1. 只起底盘（基线）"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 12
echo "  -- 此时 /odom（底盘里程计）--"
timeout 8 ros2 topic hz /odom 2>/dev/null | grep -m1 'average rate' | sed 's/^/      /'
echo

hr; echo "## 2. + 定位"
start localization ros2 launch car_localization localization.launch.py
sleep 10
hz "只起到底盘+定位"
echo "  -- localization 日志 --"
tail -4 /tmp/ag_lw/localization.log 2>/dev/null | sed 's/^/      /'
echo

hr; echo "## 3. + 裁判串口（dzjudgment）"
start judgment ros2 launch dzactuator judgment.launch.py
sleep 8
hz "加了 judgment"
echo

hr; echo "## 4. + 我们的 referee_node"
start referee ros2 run ag_referee referee_node --ros-args -p source:=dz -p hp_topic:=/HpAndHitmsg -p log_dir:=/tmp/ag_logs
sleep 6
hz "加了 referee_node"
echo

hr; echo "## 5. + 路线执行器（path_executor）"
start executor python3 -u "$PE"
sleep 8
hz "加了 path_executor ← 关键，看是不是它"
echo "  -- 执行器日志 --"
tail -5 /tmp/ag_lw/executor.log 2>/dev/null | sed 's/^/      /'
echo

hr; echo "## 6. 场上与定位相关的话题"
timeout 10 ros2 topic list 2>/dev/null | grep -iE "odom|localiz" | sed 's/^/    /'
echo
echo "  -- 节点列表 --"
timeout 10 ros2 node list 2>/dev/null | sed 's/^/    /'
echo
echo "  判读：看第几步开始 [--]，那一步起的节点就是元凶。"
echo "==================== 结束（没发速度，车没动） ===================="
