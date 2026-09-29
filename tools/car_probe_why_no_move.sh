#!/usr/bin/env bash
# 查"命令发了但车不动"的下一环（只读 + 短暂起底盘/执行器，不发控制命令）
#
# 已确认的：car_node 发了 /nav/waypoint（reset + 13 段路线，都录到了）。
# 现在要查：① 有没有人订阅 /nav/waypoint  ② 执行器起没起来、订阅/发布什么
#          ③ 它转发给谁（话题名）          ④ 底盘那边有没有收到东西
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
# path_executor 需要源码目录（map_utils 只存在于源码）：见 car_move_dryrun.sh 里的说明
export PYTHONPATH="$HOME/dzrobot/src/car_localization:${PYTHONPATH}"

PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_nm_${n}.log" 2>&1 & PGRPS="$PGRPS $!"; echo "  $n 进程组=$!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 1
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  echo "  已停止本脚本启动的进程组"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 为什么车不动（下一环排查） ===================="
date
echo

hr; echo "## 1. path_executor 源码：订阅什么 / 发布什么"
PE="$HOME/dzrobot/src/car_localization/car_localization/path_executor.py"
grep -nE "create_subscription|create_publisher|subscribe|publish|/nav|/cmd|/move|topic" "$PE" 2>/dev/null \
  | head -25 | sed 's/^/    /'
echo

hr; echo "## 2. 起底盘 + 执行器（不给任何控制命令）"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 12
start executor python3 "$PE"
sleep 8
echo "  -- 执行器日志（全部）--"
cat /tmp/ag_nm_executor.log 2>/dev/null | tail -20 | sed 's/^/    /'
echo

hr; echo "## 3. 现在 /nav/waypoint 有没有订阅者（关键）"
timeout 10 ros2 topic info /nav/waypoint 2>/dev/null | sed 's/^/    /' \
  || echo "    （话题不存在：说明执行器没建出来）"
echo

hr; echo "## 4. 场上所有跟底盘/控制有关的话题"
timeout 10 ros2 topic list 2>/dev/null | grep -iE "nav|cmd|move|actuator|wheel|serial|path|goal" | sed 's/^/    /'
echo

hr; echo "## 5. 执行器进程还活着吗"
pgrep -af "path_executor" 2>/dev/null | sed 's/^/    /' || echo "    [FAIL] 执行器进程没在跑"
pgrep -af "dzactuator" 2>/dev/null | head -2 | sed 's/^/    /'
echo

hr; echo "## 6. 提示"
echo "  - 若第 3 节订阅者数为 0：执行器没建成订阅 → 发的命令没人接，车不会动"
echo "  - 若订阅者 ≥1 但仍不动：看执行器往哪发（第 1 节），确认底盘收的是不是那个话题"
echo "  - 若第 5 节执行器没在跑：看第 2 节点的日志报错"
echo "==================== 结束 ===================="
