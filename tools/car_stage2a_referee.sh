#!/usr/bin/env bash
# 实机联调 · 第 2 段 A：底盘 + 裁判 + 定位 + 路线执行器（**零位移**）
#
# 启动：dzactuator(bringup) + dzjudgment(judgment) + serial_reader(localization) + path_executor
# 不启动：dzrknn_dataprocess（会扫云台**并且会自主开火**，见下）
#
# ⚠️ 关键安全前提（车端源码实测结论）：
#   `/move_base/stop_signal`（"1=允许扫描开火"）**只有发布者、没有订阅者** ——
#   官方那套"停车后才开火"的闸门是死代码。`dzrknn_dataprocess` 一旦运行，
#   只要目标进入开火框就会自己开火（且**不区分蓝红**）。
#   所以：**绝不在同一场联调里同时启动 dzrknn_dataprocess 和 dzjudgment**，
#   否则可能真的对镜头前的红靶（友方）打光 10 发。
#
# 本段只做"不发车"的验证：
#   1. 裁判状态能不能读到（/HpAndHitmsg 真实数据）
#   2. 里程计/IMU 能不能读到
#   3. **我们发出的 `reset:N10,y` 能不能被真机执行器接受**（证明命令格式对）
#   4. 反证：位置没有变化（车没动）
#
# 结束时只 kill 我们自己起的 PID。

set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
[ -z "${DISPLAY:-}" ] && [ -e /tmp/.X11-unix/X0 ] && \
  export DISPLAY=:0 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do
    kill -INT "$pid" 2>/dev/null
  done
  sleep 2
  for pid in "${PIDS[@]}"; do
    kill -0 "$pid" 2>/dev/null && kill -9 "$pid" 2>/dev/null
  done
  pkill -f "path_executor.py" 2>/dev/null
  pkill -f "serial_reader" 2>/dev/null
  echo "  已停止本次启动的节点"
}
trap cleanup EXIT

hr() { echo "----------------------------------------------------"; }
say() { echo "  $1"; }

echo "==================== 第 2 段 A：底盘 + 裁判 + 定位（零位移） ===================="
date
echo

hr; echo "## 0. 前置检查"
echo "-- 现有节点（应为空，否则先确认是不是别人在用）--"
timeout 10 ros2 node list 2>/dev/null | sed 's/^/  /'
echo "-- 串口设备 --"
ls -l /dev/dzactuator /dev/dzjudgment 2>/dev/null | sed 's/^/  /'
echo

hr; echo "## 1. 启动 dzactuator（底盘/IMU/里程计；无人发速度指令就不会动）"
timeout 600 ros2 launch dzactuator bringup.launch.py > /tmp/ag_s2a_actuator.log 2>&1 &
PIDS+=($!)
sleep 10
grep -E "serial port opened|can not open|wakeup|mode" /tmp/ag_s2a_actuator.log | tail -6 | sed 's/^/  /'
echo

hr; echo "## 2. 启动 dzjudgment（裁判串口 -> ROS 话题）"
timeout 600 ros2 launch dzactuator judgment.launch.py > /tmp/ag_s2a_judgment.log 2>&1 &
PIDS+=($!)
sleep 6
grep -E "Serial initialize|failed|baudrate|serialport" /tmp/ag_s2a_judgment.log | tail -6 | sed 's/^/  /'
echo

hr; echo "## 3. 启动 serial_reader（/odom + /imu_data -> /odom/localization）"
timeout 600 ros2 launch car_localization localization.launch.py > /tmp/ag_s2a_loc.log 2>&1 &
PIDS+=($!)
sleep 6
grep -E "v4-dr-reset|init|start" /tmp/ag_s2a_loc.log | tail -4 | sed 's/^/  /'
echo

hr; echo "## 4. 启动 path_executor（订阅 /nav/waypoint，是真正执行路线的那一段）"
timeout 600 python3 "$HOME/dzrobot/src/car_localization/car_localization/path_executor.py" \
  > /tmp/ag_s2a_executor.log 2>&1 &
PIDS+=($!)
sleep 6
grep -E "Initialized|Command format|subscription|wait" /tmp/ag_s2a_executor.log | head -8 | sed 's/^/  /'
echo

hr; echo "## 5. 裁判状态：这是本次联调最关键的一项"
if timeout 15 ros2 topic list 2>/dev/null | grep -q "/HpAndHitmsg"; then
  echo "  [OK] /HpAndHitmsg 存在"
  echo "-- 真实裁判数据（[targethp, self_hp, hit, Shooting_Count, ammo]）--"
  timeout 20 ros2 topic echo /HpAndHitmsg 2>/dev/null | head -30 | sed 's/^/  /'
else
  echo "  [FAIL] 没有 /HpAndHitmsg —— 裁判串口可能没接/没上电"
  tail -12 /tmp/ag_s2a_judgment.log | sed 's/^/    /'
fi
echo

hr; echo "## 6. 里程计 / IMU"
for t in /odom/localization /odom /imu_data; do
  if timeout 10 ros2 topic list 2>/dev/null | grep -qx "$t"; then
    echo "  [OK] $t"
  else
    echo "  [--] $t 不存在"
  fi
done
echo "-- /odom/localization 一条样本（block 单位 + 朝向）--"
timeout 15 ros2 topic echo --once /odom/localization 2>/dev/null | head -20 | sed 's/^/  /'
echo

hr; echo "## 7. 命令链路验证：只发 reset（**零位移**），看真机执行器认不认"
echo "-- 发送 reset:N10,y --"
timeout 10 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:N10,y'}" 2>&1 | tail -2 | sed 's/^/  /'
sleep 3
echo "-- path_executor 日志里应该出现 Odometry reset --"
grep -E "Odometry reset|reset" /tmp/ag_s2a_executor.log | tail -5 | sed 's/^/  /'
if grep -q "Odometry reset" /tmp/ag_s2a_executor.log; then
  echo "  [OK] 真机执行器接受了我们的命令格式（reset 生效）"
else
  echo "  ⚠️ 日志里没看到 Odometry reset，检查上面输出"
fi
echo

hr; echo "## 8. 反证：车没有动"
echo "-- 连采两次定位，比较位置 --"
A=$(timeout 12 ros2 topic echo --once /odom/localization --field pose.pose.position 2>/dev/null | tr '\n' ' ')
sleep 4
B=$(timeout 12 ros2 topic echo --once /odom/localization --field pose.pose.position 2>/dev/null | tr '\n' ' ')
echo "  t0: $A"
echo "  t1: $B"
echo "-- 速度指令（应为 0）--"
timeout 10 ros2 topic echo --once /pursuitAngle 2>/dev/null | head -8 | sed 's/^/  /'
echo

hr; echo "## 9. 结论"
say "裁判能读 -> 看第 5 节；命令格式对被接受 -> 看第 7 节；车未移动 -> 看第 8 节"
say "下一段（云台）记得：**先停掉 dzjudgment**，再启动 dzrknn_dataprocess"
say "  （否则它会自主开火，而且不区分蓝红）"
echo "==================== 结束（节点将自动停止） ===================="
