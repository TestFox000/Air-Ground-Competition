#!/usr/bin/env bash
# 定位遥控器上到底哪个开关是「车辆 遥控/串口程序驱动」模式切换
#
# 为什么需要这个脚本：`docs/raw_txt/04_遥控器控制.txt` 只写了
#   「左半边为云台控制，右半边为车辆控制……车辆遥控/串口程序驱动模式切换」
# 但**没给图**，所以不知道对应 SWA / SWB / SWC / SWD 里的哪一个。
# 猜错一次就要重跑一遍全套，不如一次扫出来。
#
# 做法：以固定节奏发一串**很轻的速度脉冲**（0.25 m/s × 0.6 s，每 2 s 一次，共 45 次 ≈ 90 s）。
#   底盘在「遥控」档 → 每个脉冲的速度峰值都是 0
#   一旦拨到「串口程序驱动」档 → 之后的脉冲立刻出现非零速度
# 你只要记下自己**什么时候拨了哪个开关**，看输出里从第几个脉冲开始非零即可对上。
#
# 建议的拨动节奏（脚本一跑起来就开始）：
#   0 s 拨 SWA → 20 s 拨 SWB → 40 s 拨 SWC → 60 s 拨 SWD
#   （每个拨到另一端就保持，不要拨回来）
#
# ⚠️ 拨对之后车会随脉冲一点一点往前挪，人在场、前方留出空间。
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
mkdir -p /tmp/ag_sw

V="${AG_V:-0.25}"
N="${AG_N:-45}"
PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_sw/${n}.log" 2>&1 & PGRPS="$PGRPS $!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 2
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  echo "  已停止本脚本启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 遥控器模式开关扫描 ===================="
date
echo "  ${N} 个脉冲 × ${V} m/s × 0.6 s（每 2 s 一次，约 $((N * 2)) s）"
echo "  节奏建议：0s 拨 SWA ／ 20s 拨 SWB ／ 40s 拨 SWC ／ 60s 拨 SWD"
echo

hr; echo "## 1. 起底盘 + 定位"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 10
start localization ros2 launch car_localization localization.launch.py
sleep 6
echo "  就绪，$(date +%T) 开始发脉冲 —— 现在开始拨开关"
echo

hr; echo "## 2. 脉冲与编码器速度（底盘 /odom 原始，未加工）"
MOVE_FROM=""
for i in $(seq 1 "$N"); do
  timeout 0.6 ros2 topic pub -r 20 /pursuitAngle geometry_msgs/msg/Twist \
    "{linear: {x: $V, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
  S="$(timeout 1.2 ros2 topic echo --once /odom --field twist.twist.linear.x 2>/dev/null | head -1)"
  S="${S:-0}"
  NZ="$(awk -v s="$S" 'BEGIN{print (s+0 > 0.02) ? 1 : 0}')"
  if [ "$NZ" = "1" ] && [ -z "$MOVE_FROM" ]; then
    MOVE_FROM="$i"
    echo "  >>> 第 $i 个脉冲（约 $((i * 2)) s）首次出现非零速度：${S} m/s  ← **就是这个开关**"
  fi
  printf "    第 %2d 个（%3d s）：%s\n" "$i" "$((i * 2))" "$S"
  sleep 0.8
done
echo

hr; echo "## 3. 结论"
if [ -n "$MOVE_FROM" ]; then
  echo "  [OK] 从第 ${MOVE_FROM} 个脉冲（约 $((MOVE_FROM * 2)) s）起车开始响应速度指令。"
  echo "       对照你自己拨开关的时间，那一下拨的就是「车辆 遥控/串口程序驱动」开关。"
  echo "       ⚠️ 请把它写进 CHECKLIST.md，别让下一个人再扫一遍。"
else
  echo "  [FAIL] 全程零速度 —— 四个开关都没切到串口档。可能："
  echo "         1) 开关是三段式的，两端都不是、要拨到中间档"
  echo "         2) 需要先开机对频 / 该开关对应的是云台那一路"
  echo "         3) 底盘侧还需别的使能（拿 /current_vel_state 的 D 档回吐去问渡众）"
fi
echo
echo "==================== 结束 ===================="
