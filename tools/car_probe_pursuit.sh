#!/usr/bin/env bash
# 真·决定性验证：定位有没有进执行器，执行器有没有出 pursuitAngle
#
# 两次误判的教训（都记在这儿，别再重犯）：
#   1) 执行器发的是 **'pursuitAngle'**（不是 /cmd_vel）。/cmd_vel 上"发布者 0"是**正常的**，
#      拿它当判据会得出"执行器根本没发"的错误结论（2026-09-29 踩过）。
#   2) 执行器的 control_loop 第一行就是 `if not self.has_odom: return` ——
#      **收不到定位时它一帧都不发**（连零 Twist 保活都不发）。
#      所以"pursuitAngle 上有没有数据"正好能区分这两种情况：
#          有数据（哪怕全是 0）→ 定位进来了，问题在路线/命令
#          一条都没有        → 定位没进执行器（QoS 或话题名），车当然不动
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
[ -f /tmp/ag_ws/install/setup.bash ] && source /tmp/ag_ws/install/setup.bash 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export PYTHONPATH="$HOME/dzrobot/src/car_localization:${PYTHONPATH}"

TARGET="${AG_TARGET:-N9}"
PE="$HOME/dzrobot/src/car_localization/car_localization/path_executor.py"
mkdir -p /tmp/ag_pu

PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_pu/${n}.log" 2>&1 & PGRPS="$PGRPS $!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 2
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  pkill -f "path_executor.py" 2>/dev/null
  echo "  已停止本脚本启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 定位 → 执行器 → pursuitAngle ===================="
date
echo

hr; echo "## 1. 起底盘 + 定位 + 执行器"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 10
start localization ros2 launch car_localization localization.launch.py
sleep 8
start executor python3 -u "$PE"
sleep 10
echo "  -- 执行器 stdout/stderr --"
tail -8 /tmp/ag_pu/executor.log 2>/dev/null | sed 's/^/    /'
echo

hr; echo "## 2. 关键话题：谁发谁收（含 QoS）"
echo "  -- /pursuitAngle（执行器 → 底盘，真正驱动车的话题）--"
timeout 8 ros2 topic info /pursuitAngle -v 2>/dev/null | sed 's/^/      /' || echo "      （不存在 → 执行器没起来）"
echo "  -- /odom/localization（定位 → 执行器）--"
timeout 8 ros2 topic info /odom/localization -v 2>/dev/null | sed 's/^/      /' || echo "      （不存在）"
echo

hr; echo "## 3. 空跑：不发命令，只看 pursuitAngle 有没有在发"
( timeout 6 ros2 topic echo /pursuitAngle > /tmp/ag_pu/idle.txt 2>&1 ) &
sleep 7
N_IDLE="$(grep -c 'linear' /tmp/ag_pu/idle.txt 2>/dev/null)"
echo "  空跑 6 s 抓到 ${N_IDLE:-0} 条"
if [ "${N_IDLE:-0}" -gt 0 ]; then
  echo "  [OK] 执行器在发 → 定位已经进来了（has_odom=True）"
  grep -E '^\s+x:' /tmp/ag_pu/idle.txt | head -3 | sed 's/^/      linear.x: /'
else
  echo "  [FAIL] 一条都没有 → control_loop 在 `if not self.has_odom: return` 处直接返回"
  echo "         = **定位没进执行器**。下一步查上面的 QoS 是否匹配"
fi
echo

hr; echo "## 4. 发命令（$TARGET，一格）看速度有没有变非零"
( timeout 14 ros2 topic echo /pursuitAngle > /tmp/ag_pu/run.txt 2>&1 ) &
CV_PID=$!
( timeout 14 ros2 topic echo /odom/localization --field pose.pose.position > /tmp/ag_pu/traj.txt 2>&1 ) &
TR_PID=$!
sleep 2
ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:N10,y'}" >/dev/null 2>&1
sleep 2
ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: '$TARGET,y'}" >/dev/null 2>&1
echo "  已发 reset:N10,y 与 $TARGET,y"
wait "$CV_PID" 2>/dev/null
wait "$TR_PID" 2>/dev/null
N_RUN="$(grep -c 'linear' /tmp/ag_pu/run.txt 2>/dev/null)"
echo "  发车 14 s 抓到 ${N_RUN:-0} 条"
NZ="$(grep -A1 'linear' /tmp/ag_pu/run.txt 2>/dev/null | grep -E '^\s+x:' | grep -vE '^\s+x: 0\.0*$' | head -5)"
if [ -n "$NZ" ]; then echo "  [OK] 出现非零线速度："; echo "$NZ" | sed 's/^/      /'
else echo "  [FAIL] 线速度全是 0.0 —— 定位有了、命令发了，但纯跟踪没算出速度"; fi

echo "  -- 位移 --"
python3 - /tmp/ag_pu/traj.txt <<'PY' 2>/dev/null
import sys, math, re
try:
    pts = re.findall(r"^\s*x:\s*(-?[\d.]+)\s*\n\s*y:\s*(-?[\d.]+)",
                     open(sys.argv[1], encoding="utf-8", errors="replace").read(), re.M)
    if len(pts) < 2: print("      （轨迹样本不足）")
    else:
        d = math.dist((float(pts[0][0]), float(pts[0][1])), (float(pts[-1][0]), float(pts[-1][1])))
        print(f"      样本 {len(pts)}，位移 {d:.3f} m → {'[OK] 车动了' if d > 0.15 else '[FAIL] 基本没动'}")
except Exception: print("      （算不出位移）")
PY
echo
echo "==================== 结束 ===================="
