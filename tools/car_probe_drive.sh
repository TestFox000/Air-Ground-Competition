#!/usr/bin/env bash
# 对比实验：分别用 /cmd_vel 和 /pursuitAngle 直接驱底盘，看哪个能把车推动
#
# 读到的事实（dzactuator 源码，2026-09-29）：
#   - 底盘同时订阅 cmd_vel（callback_cmd_vel_angle）和 pursuitAngle（callback_movebase_angle）
#   - modeTimerCallback 做模式仲裁：只有 cmd_vel 活跃 → current_mode_=1(DWA)
#                                    只有 pursuitAngle 活跃 → current_mode_=2(Pursuit)
#   - pursuitAngle 回调里：`if (current_mode_ != 2) return;` 然后
#     `if (control_mode == 1) { ... 真正下发速度 ... }`
# 也就是说 pursuit 这条路要连过两道闸。本脚本不再猜，直接两边各推一次。
#
# ⚠️ 会让车动：两轮各 5 秒、速度 0.3 m/s。必须有人在车边、遥控器在手。
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1

V="${AG_V:-0.3}"        # 目标速度 m/s
DUR="${AG_DUR:-5}"      # 每轮持续秒数
mkdir -p /tmp/ag_dr

PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_dr/${n}.log" 2>&1 & PGRPS="$PGRPS $!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 2
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  echo "  已停止本脚本启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

pos() { timeout 8 ros2 topic echo --once /odom/localization --field pose.pose.position 2>/dev/null | tr '\n' ' '; }
disp() {
python3 - "$1" <<'PY' 2>/dev/null
import sys, math, re
try:
    pts = re.findall(r"^\s*x:\s*(-?[\d.]+)\s*\n\s*y:\s*(-?[\d.]+)",
                     open(sys.argv[1], encoding="utf-8", errors="replace").read(), re.M)
    if len(pts) < 2: print("样本不足")
    else:
        d = math.dist((float(pts[0][0]), float(pts[0][1])), (float(pts[-1][0]), float(pts[-1][1])))
        print(f"{d:.3f} m → {'[OK] 动了' if d > 0.10 else '[FAIL] 没动'}")
except Exception: print("算不出")
PY
}

echo "==================== 底盘对比实验：cmd_vel vs pursuitAngle ===================="
date
echo "  速度 ${V} m/s，每轮 ${DUR}s；⚠️ 车会动，需人在场"
echo

hr; echo "## 1. 只起底盘 + 定位（不起我们的执行器，避免干扰）"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 10
start localization ros2 launch car_localization localization.launch.py
sleep 8
echo "  起始位置: $(pos)"
echo

hr; echo "## 2. 第一轮：往 /cmd_vel 持续发 ${V} m/s"
( timeout "$DUR" ros2 topic echo /odom/localization --field pose.pose.position > /tmp/ag_dr/traj_cmd.txt 2>&1 ) &
T1=$!
timeout "$DUR" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: $V, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
wait "$T1" 2>/dev/null
echo "  位移: $(disp /tmp/ag_dr/traj_cmd.txt)"
sleep 2
echo "  本轮后位置: $(pos)"
echo

hr; echo "## 3. 第二轮：往 /pursuitAngle 持续发 ${V} m/s"
( timeout "$DUR" ros2 topic echo /odom/localization --field pose.pose.position > /tmp/ag_dr/traj_pur.txt 2>&1 ) &
T2=$!
timeout "$DUR" ros2 topic pub -r 20 /pursuitAngle geometry_msgs/msg/Twist \
  "{linear: {x: $V, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
wait "$T2" 2>/dev/null
echo "  位移: $(disp /tmp/ag_dr/traj_pur.txt)"
sleep 2
echo "  本轮后位置: $(pos)"
echo

hr; echo "## 4. 底盘侧看到了什么（dzactuator 自己的打印）"
grep -aE "Trigger|rx32|mode|voice|serial|Speed" /tmp/ag_dr/actuator.log 2>/dev/null | tail -12 | sed 's/^/    /'
echo
echo "==================== 结束 ===================="
echo
echo "判读："
echo "  两轮都不动 → 底盘层面被禁用（遥控器/使能/急停），与我们的代码无关"
echo "  只有 cmd_vel 动 → pursuit 这条路还差一道闸（current_mode_/control_mode）"
echo "  两轮都动     → 底盘没问题，问题在执行器给的速度(0.1225)太小或没持续发"
