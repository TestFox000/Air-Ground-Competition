#!/usr/bin/env bash
# 对齐官方定位链路（**无雷达**版本）并对比我们现在的定位
#
# 官方发布包给出的两种定位：
#   有雷达：rplidar_s3e_launch.py + amcl.launch.py            （本车没雷达，走不了）
#   无雷达：ekf.launch.py（robot_localization 融合 IMU+里程计 → /odometry/filtered）
#           + fake_amcl.launch.py（→ /amcl_pose，并发布 map→odom 静态 TF）
# 我们目前用的是 car_localization/serial_reader 的航位推算（日志打 v4-dr-reset），
# 输出 /odom/localization，坐标是 **block 单位**（不是米）。
#
# 本脚本把官方那套也起起来，与我们的**并列跑**，对比三路定位是否都活着、
# 朝向是否一致 —— 用来判断"车转圈"是不是航位推算的 yaw 漂移造成的。
#
# ⚠️ 只起节点、不发任何速度指令，**车不会动**。
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
[ -f /tmp/ag_ws/install/setup.bash ] && source /tmp/ag_ws/install/setup.bash 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
mkdir -p /tmp/ag_al

PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_al/${n}.log" 2>&1 & PGRPS="$PGRPS $!"; echo "  $n 进程组=$!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 2
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  echo "  已停止本脚本启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 对齐官方定位链路（无雷达） ===================="
date
echo "  ⚠️ 只起节点，不发速度指令，车不会动"
echo

hr; echo "## 0. 官方那套在不在车上"
PKG="$(ros2 pkg list 2>/dev/null)"
for p in robot_localization fake_amcl_node dztrack car_localization dzactuator; do
  echo "$PKG" | grep -qx "$p" && echo "  [OK] $p" || echo "  [缺] $p"
done
echo "  -- dzactuator 的 launch --"
ls "$HOME/dzrobot/src/dzactuator/launch" 2>/dev/null | grep -E "ekf|amcl|bringup" | sed 's/^/      /'
echo

hr; echo "## 1. 起底盘（官方第一步）"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 12
grep -aE "serial port opened|can not open" /tmp/ag_al/actuator.log 2>/dev/null | tail -2 | sed 's/^/    /'
echo

hr; echo "## 2. 起我们现在的定位（serial_reader，block 坐标）"
if echo "$PKG" | grep -qx car_localization; then
  start loc_ours ros2 launch car_localization localization.launch.py
  sleep 8
else
  echo "  （car_localization 不在，跳过）"
fi
echo

hr; echo "## 3. 起官方 EKF（robot_localization，融合 IMU + 里程计）"
if echo "$PKG" | grep -qx robot_localization; then
  start ekf ros2 launch dzactuator ekf.launch.py
  sleep 8
  tail -4 /tmp/ag_al/ekf.log 2>/dev/null | sed 's/^/    /'
else
  echo "  [缺] robot_localization —— 先跑 python car.py restore <车IP>"
fi
echo

hr; echo "## 4. 起官方 fake_amcl（/odometry/filtered → /amcl_pose）"
if echo "$PKG" | grep -qx fake_amcl_node; then
  start amcl ros2 launch fake_amcl_node fake_amcl.launch.py
  sleep 6
  tail -3 /tmp/ag_al/amcl.log 2>/dev/null | sed 's/^/    /'
else
  echo "  [缺] fake_amcl_node —— 官方发布包里有，可单独拷过来编译"
fi
echo

hr; echo "## 5. 三路定位逐个验活"
for t in /odom /odom/localization /odometry/filtered /amcl_pose /imu_data; do
  R="$(timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate')"
  if [ -n "$R" ]; then echo "  [OK] $t  $R"
  else echo "  [--] $t  没有数据或不存在"; fi
done
echo

hr; echo "## 6. 朝向对比（判断是不是 yaw 漂移导致转圈）"
echo "  -- 我们的 /odom/localization（block 单位）--"
timeout 8 ros2 topic echo --once /odom/localization --field pose.pose.orientation 2>/dev/null | sed 's/^/    /'
echo "  -- 官方 EKF /odometry/filtered（米制）--"
timeout 8 ros2 topic echo --once /odometry/filtered --field pose.pose.orientation 2>/dev/null | sed 's/^/    /'
echo

echo "  判读："
echo "    /odometry/filtered 有数据 → 官方无雷达定位链路可用，可以考虑作为定位增强"
echo "    两者朝向差很多        → 航位推算的 yaw 在漂，这就是转圈的嫌疑根因"
echo
echo "==================== 结束（不发速度，车没动） ===================="
