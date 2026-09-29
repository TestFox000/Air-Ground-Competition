#!/usr/bin/env bash
# 车为什么转圈：抓执行器实际输出的 linear.x / angular.z
#
# 怀疑（2026-09-29 实测车在转圈）：dzactuator 里
#     moveBaseControl.TargetAngle = msg->angular.z + 60;
#     if (<25) 25;  if (>95) 95;
#   而 buf[3..4] = (int16)(TargetAngle * 10)，60 是中位、25~95 是两端极限。
#   若 path_executor 输出的 angular.z 用的是**度**（ROS 标准是弧度），
#   那么几十度的角速度会被直接顶到 95 → 方向盘打死 → 车原地转圈。
#
# 本脚本抓真实数值，用**量级**判断：
#   angular.z 在 ±0.1~3 之间   → 弧度，单位没问题，转圈另有原因
#   angular.z 动辄 ±10~±95     → 度，单位错配，这就是转圈的原因
#
# ⚠️ 会让车动（发一格命令）。需人在场、留出空间。
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
[ -f /tmp/ag_ws/install/setup.bash ] && source /tmp/ag_ws/install/setup.bash 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export PYTHONPATH="$HOME/dzrobot/src/car_localization:${PYTHONPATH}"

TARGET="${AG_TARGET:-N9}"
DUR="${AG_DUR:-6}"
PE="$HOME/dzrobot/src/car_localization/car_localization/path_executor.py"
mkdir -p /tmp/ag_tn

PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_tn/${n}.log" 2>&1 & PGRPS="$PGRPS $!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 2
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  pkill -f "path_executor.py" 2>/dev/null
  echo "  已停止本脚本启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 转圈根因：抓 angular.z 量级 ===================="
date
echo "  目标 ${TARGET}（一格），发命令后抓 ${DUR}s"
echo

hr; echo "## 1. 起底盘 + 定位 + 执行器"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 10
start localization ros2 launch car_localization localization.launch.py
sleep 8
start executor python3 -u "$PE"
sleep 8
echo

hr; echo "## 2. 发命令并抓 /pursuitAngle"
( timeout "$DUR" ros2 topic echo /pursuitAngle > /tmp/ag_tn/pur.txt 2>&1 ) &
P_PID=$!
sleep 2
ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:N10,y'}" >/dev/null 2>&1
sleep 2
ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: '$TARGET,y'}" >/dev/null 2>&1
wait "$P_PID" 2>/dev/null
echo

hr; echo "## 3. 数值与判读"
python3 - /tmp/ag_tn/pur.txt <<'PY' 2>/dev/null
import sys, re
try:
    t = open(sys.argv[1], encoding="utf-8", errors="replace").read()
    lin = [float(x) for x in re.findall(r"linear:\s*\n\s*x:\s*(-?[\d.eE+-]+)", t)]
    ang = [float(x) for x in re.findall(r"angular:\s*\n\s*z:\s*(-?[\d.eE+-]+)", t)]
    def stat(v, name):
        if not v:
            print(f"      {name}: 无样本"); return
        mx = max(abs(min(v)), abs(max(v)))
        print(f"      {name}: 样本 {len(v)}，范围 [{min(v):.4f}, {max(v):.4f}]，绝对峰值 {mx:.4f}")
    stat(lin, "linear.x (线速度)")
    stat(ang, "angular.z (角速度)")
    if ang:
        mx = max(abs(x) for x in ang)
        print()
        if mx > 8:
            print("      >>> [FAIL] angular.z 量级是**度**（ROS 标准是弧度）。")
            print("          底盘 TargetAngle = angular.z + 60 会被顶到 95 上限 → 方向盘打死 → 转圈。")
        else:
            print("      >>> [OK] angular.z 在弧度量级（|z| < 8），单位没问题。")
            print("          转圈要另找原因：多半是线速度太小而转向仍有输出。")
    if lin:
        print(f"      线速度峰值 {max(abs(x) for x in lin):.4f} m/s"
              f" → 底盘字节约 {max(abs(x) for x in lin) * 23 / 0.33:.0f}/255")
except Exception as e:
    print("      解析失败", e)
PY
echo
echo "==================== 结束 ===================="
