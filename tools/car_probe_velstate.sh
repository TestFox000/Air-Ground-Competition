#!/usr/bin/env bash
# 速度到底有没有被下发：读底盘自己回吐的 /current_vel_state 与原始帧 /raw
#
# 判据（读源码得出）：callback_movebase_angle 在真正下发速度时会
#   pub_current_vel->publish(current_vel_msg)  其中
#     linear.y = 档位（>0.05 → 9.0=D 档；否则 3.0=N）
#     linear.z = 驾驶模式（固定 1.0）
# 所以：
#   有 /current_vel_state 数据 → 回调执行了、速度算出来了 → 断点在串口/STM32 那一层
#   完全没有            → 回调在模式闸处就 return 了 → 断点还在 ROS 层
# /raw 是底盘上报的原始帧，里面可能带着"当前控制源/模式"状态字。
#
# ⚠️ 会让车动（0.3 m/s × 2 s，最多约 0.6 m）；需人在场。
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
mkdir -p /tmp/ag_vs

V="${AG_V:-0.3}"
DUR="${AG_DUR:-2}"
PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_vs/${n}.log" 2>&1 & PGRPS="$PGRPS $!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 2
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  echo "  已停止本脚本启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 速度有没有真的下发 ===================="
date
echo "  ${V} m/s × ${DUR}s"
echo

hr; echo "## 1. 起底盘 + 定位"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 10
start localization ros2 launch car_localization localization.launch.py
sleep 6
echo

hr; echo "## 2. 只读：底盘原始帧 /raw（找控制源/模式状态字）"
timeout 8 ros2 topic echo --once /raw 2>/dev/null | head -20 | sed 's/^/    /' || echo "    （读不到 /raw）"
echo

hr; echo "## 3. 发 pursuitAngle ${V} m/s，同时听 /current_vel_state"
( timeout "$((DUR + 4))" ros2 topic echo /current_vel_state > /tmp/ag_vs/vel_state.txt 2>&1 ) &
VS_PID=$!
( timeout "$((DUR + 4))" ros2 topic echo /odom/localization --field pose.pose.position > /tmp/ag_vs/traj.txt 2>&1 ) &
TR_PID=$!
sleep 2
timeout "$DUR" ros2 topic pub -r 20 /pursuitAngle geometry_msgs/msg/Twist \
  "{linear: {x: $V, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
wait "$VS_PID" 2>/dev/null
wait "$TR_PID" 2>/dev/null

echo "  -- /current_vel_state（底盘回吐的档位与驾驶模式）--"
if [ -s /tmp/ag_vs/vel_state.txt ] && grep -q 'linear' /tmp/ag_vs/vel_state.txt; then
  echo "      [OK] 有数据 → **速度确实算出来并进了下发分支**"
  grep -E '^\s+[xyz]:' /tmp/ag_vs/vel_state.txt | head -6 | sed 's/^/      /'
  echo "      （linear.y=9.0 表示 D 档前进，linear.z=1.0 为驾驶模式）"
else
  echo "      [FAIL] 一条都没有 → 回调在 `current_mode_ != 2` 或 `control_mode != 1` 处就 return 了"
  echo "             = 速度根本没走到下发那一步，断点仍在 ROS 层"
  head -5 /tmp/ag_vs/vel_state.txt 2>/dev/null | sed 's/^/      /'
fi
echo

echo "  -- 位移 --"
python3 - /tmp/ag_vs/traj.txt <<'PY' 2>/dev/null
import sys, math, re
try:
    pts = re.findall(r"^\s*x:\s*(-?[\d.]+)\s*\n\s*y:\s*(-?[\d.]+)",
                     open(sys.argv[1], encoding="utf-8", errors="replace").read(), re.M)
    if len(pts) < 2: print("      （样本不足）")
    else:
        d = math.dist((float(pts[0][0]), float(pts[0][1])), (float(pts[-1][0]), float(pts[-1][1])))
        print(f"      样本 {len(pts)}，位移 {d:.3f} m → {'[OK] 动了' if d > 0.10 else '[FAIL] 没动'}")
except Exception: print("      （算不出）")
PY
echo
echo "==================== 结束 ===================="
