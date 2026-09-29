#!/usr/bin/env bash
# ⚠️ 位移判据可能是假的：/odom/localization 可能处于 dr-reset 冻结状态
#
# 疑点（2026-09-29）：serial_reader 的日志一直打
#     pos: (1.50, 0.50) | block: N10 | yaw: +90deg [imu] [v4-dr-reset]
# 位置死死钉在起点。如果 /odom/localization 是"重置后锁死"的，
# 那么之前所有"位移 0.000 m"的结论**全都不可信** —— 车可能真的动了。
#
# 本脚本改用底盘自己的原始里程计 /odom（编码器直接上来，不走 serial_reader）：
#   - 读 twist.linear.x（轮子有没有转，比位置更快出结果）
#   - 读 pose（编码器累计位移）
#   - 同时读 /odom/localization 做对照
# 三者的差异本身就是结论：
#   /odom 变了、/odom/localization 没变 → 判据一直错了，车其实动过
#   /odom 也没变                        → 确实没转，底盘固件那层的问题坐实
#
# ⚠️ 会让车动：0.5 m/s × 3 s（最多约 1.5 m）。需人在场、场地有空间。
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
mkdir -p /tmp/ag_od

V="${AG_V:-0.5}"
DUR="${AG_DUR:-3}"
PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_od/${n}.log" 2>&1 & PGRPS="$PGRPS $!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 2
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  echo "  已停止本脚本启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

show() {  # show <文件> <标签>
python3 - "$1" "$2" <<'PY' 2>/dev/null
import sys, math, re
try:
    t = open(sys.argv[1], encoding="utf-8", errors="replace").read()
    pts = re.findall(r"^\s*x:\s*(-?[\d.]+)\s*\n\s*y:\s*(-?[\d.]+)", t, re.M)
    vel = re.findall(r"linear:\s*\n\s*x:\s*(-?[\d.]+)", t)
    if pts and len(pts) >= 2:
        d = math.dist((float(pts[0][0]), float(pts[0][1])), (float(pts[-1][0]), float(pts[-1][1])))
        p = f"位姿位移 {d:.3f} m（{len(pts)} 样本）"
    else:
        p = "位姿样本不足"
    if vel:
        mx = max(abs(float(v)) for v in vel)
        p += f" ｜ 速度峰值 {mx:.3f} m/s（{len(vel)} 样本）"
    print(f"      {sys.argv[2]}: {p}")
except Exception:
    print(f"      {sys.argv[2]}: 读不出")
PY
}

echo "==================== 位移判据校验：/odom vs /odom/localization ===================="
date
echo "  ${V} m/s × ${DUR}s"
echo

hr; echo "## 1. 起底盘 + 定位"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 10
start localization ros2 launch car_localization localization.launch.py
sleep 6
echo

hr; echo "## 2. 发 pursuitAngle ${V} m/s，同时读两个里程计"
# ⚠️ 只取 pose.pose.position 字段，不要 echo 整条消息：
#    整条 /odom 里 pose.position 和 twist.linear 都有 x/y，用正则一起抓会把
#    twist 的坐标当成位置 —— 实测首尾两点正好差 √(1.5²+0.5²)=1.581 m，
#    凭空"看出"车走了 1.58 米（2026-09-29 踩过）。
( timeout "$((DUR + 5))" ros2 topic echo /odom --field pose.pose.position > /tmp/ag_od/odom.txt 2>&1 ) &
O_PID=$!
( timeout "$((DUR + 5))" ros2 topic echo /odom/localization --field pose.pose.position > /tmp/ag_od/loc.txt 2>&1 ) &
L_PID=$!
( timeout "$((DUR + 5))" ros2 topic echo /odom --field twist.twist.linear.x > /tmp/ag_od/vel.txt 2>&1 ) &
V_PID=$!
sleep 2
timeout "$DUR" ros2 topic pub -r 20 /pursuitAngle geometry_msgs/msg/Twist \
  "{linear: {x: $V, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
wait "$O_PID" 2>/dev/null
wait "$L_PID" 2>/dev/null
wait "$V_PID" 2>/dev/null
echo "  -- /odom 编码器速度（twist.twist.linear.x）峰值 --"
awk '{ if ($0+0 > m) m = $0+0 } END { printf "      %.3f m/s\n", m+0 }' /tmp/ag_od/vel.txt 2>/dev/null
echo

hr; echo "## 3. 对照结果"
show /tmp/ag_od/odom.txt "/odom（底盘编码器，未加工）"
show /tmp/ag_od/loc.txt "/odom/localization（serial_reader 输出）"
echo
echo "  判读："
echo "    /odom 动了、/odom/localization 没动 → **位移判据一直是错的**，车其实动过"
echo "    /odom 也没动                        → 电机确实没转，底盘固件层问题坐实"
echo

hr; echo "## 4. serial_reader 的状态字（看是不是 dr-reset 锁死）"
grep -aE "dr-reset|pos:|block" /tmp/ag_od/localization.log 2>/dev/null | tail -6 | sed 's/^/    /'
echo
echo "==================== 结束 ===================="
