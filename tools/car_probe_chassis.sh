#!/usr/bin/env bash
# 底盘/IMU/定位 链路诊断（只读，不发控制命令）—— 专查"定位为什么读不到"。
#
# 已知：/odom/localization 话题存在但没有数据；定位节点（serial_reader_node）日志说
#       "IMU: /imu_data, fallback after 2.0s"。所以往下游查：底盘有没有起来、
#       串口开没开、/imu_data 有没有在发。
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
export RCUITLS_LOGGING_BUFFERED_STREAM=1 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1

PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_chk_${n}.log" 2>&1 & PGRPS="$PGRPS $!"; echo "  $n 进程组=$!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 1
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  echo "  已停止本脚本启动的进程组"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 底盘 / IMU / 定位 链路诊断 ===================="
date
echo

hr; echo "## 1. 串口设备与占用情况"
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sed 's/^/    /' || echo "    （没有 ttyUSB/ttyACM 设备！底盘串口不在）"
for d in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0; do
  [ -e "$d" ] && { echo "  -- $d 被谁占着："; (fuser -v "$d" 2>&1 | sed 's/^/      /') || echo "      （没查到占用者）"; }
done
echo

hr; echo "## 2. 起底盘（dzactuator bringup）"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 15
echo "  -- 底盘日志（尾部 25 行）--"
tail -25 /tmp/ag_chk_actuator.log 2>/dev/null | sed 's/^/    /'
echo

hr; echo "## 3. 底盘相关的节点与话题"
echo "  -- 节点 --"
timeout 10 ros2 node list 2>/dev/null | sed 's/^/    /'
echo "  -- 话题（imu/odom/wheel/chassis 相关）--"
timeout 10 ros2 topic list 2>/dev/null | grep -iE "imu|odom|wheel|chassis|serial" | sed 's/^/    /' || echo "    （一个都没有）"
echo

hr; echo "## 4. /imu_data 有没有在发（定位的前提）"
if timeout 10 ros2 topic list 2>/dev/null | grep -qx "/imu_data"; then
  timeout 10 ros2 topic hz /imu_data 2>/dev/null | grep -m1 "average rate" | sed 's/^/    /' \
    || echo "    [FAIL] /imu_data 没有数据 —— 底盘没在读 IMU（串口没开 / 串口被占 / 底盘没上电）"
  timeout 8 ros2 topic echo --once /imu_data 2>/dev/null | head -10 | sed 's/^/    /'
else
  echo "    [FAIL] 没有 /imu_data 话题 —— 底盘节点没起来，或话题名不同"
fi
echo

hr; echo "## 5. 再起定位，给它足够时间"
start localization ros2 launch car_localization localization.launch.py
sleep 8
echo "  -- 定位节点日志尾部 --"
tail -12 /tmp/ag_chk_localization.log 2>/dev/null | sed 's/^/    /'
echo "  -- /odom/localization 15 秒内有没有数据 --"
timeout 15 ros2 topic hz /odom/localization 2>/dev/null | grep -m1 "average rate" | sed 's/^/    /' \
  || echo "    [FAIL] 仍然没有数据"
echo

hr; echo "## 6. 结论提示"
echo "  看上面第 2 节：底盘日志里有没有 'serial port opened'。"
echo "  - 没有 → 底盘串口没开（设备不在 / 被占 / 底盘没上电），IMU 就没有数据，定位自然读不到"
echo "  - 有但 /imu_data 仍无数据 → 看底盘节点是否真的在发（话题名可能不同）"
echo "  - 都正常但仍无定位 → 定位节点需要时间收敛，或起点参数不对（它日志里会写 Start）"
echo "==================== 结束 ===================="
