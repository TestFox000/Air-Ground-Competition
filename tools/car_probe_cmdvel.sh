#!/usr/bin/env bash
# 决定性验证：命令发进去了，执行器到底有没有把它变成"速度"？
#
# 背景（2026-09-29 真发车）：
#   car_node 确实发出了 `reset:N10,y` 和 `K9 I9 ... N11,y`，执行器进程活着、
#   /nav/waypoint 有订阅者、定位 50 Hz —— 但车位移 0.000 m。
#   所以要回答的就一个问题：**速度指令到底发没发出来、发到了哪个话题、底盘收不收。**
#
# 本脚本：起底盘 + 定位 + 执行器 → 发**一格**命令 → 同时监听所有候选控制话题 → 看位移。
# 只走一格（相邻格），不是整条路线：足以证明链路通不通，动作最小。
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
[ -f /tmp/ag_ws/install/setup.bash ] && source /tmp/ag_ws/install/setup.bash 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
# 2026-09-29 真因：map_utils 只在源码里，不带上就启动即崩
export PYTHONPATH="$HOME/dzrobot/src/car_localization:${PYTHONPATH}"

TARGET="${AG_TARGET:-N9}"      # 只走一格
RUN_S="${AG_RUN_S:-15}"
PE="$HOME/dzrobot/src/car_localization/car_localization/path_executor.py"

PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_cv_${n}.log" 2>&1 & PGRPS="$PGRPS $!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 2
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  pkill -f "path_executor.py" 2>/dev/null
  echo "  已停止本脚本启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 命令 → 速度：决定性验证 ===================="
date
echo "  目标：$TARGET（只走一格）   时长：${RUN_S}s"
echo

hr; echo "## 1. 执行器往哪个话题发速度（静态）"
grep -nE "create_publisher" "$PE" 2>/dev/null | head -10 | sed 's/^/    /'
echo

hr; echo "## 2. 起底盘 + 定位 + 执行器"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 10
start localization ros2 launch car_localization localization.launch.py
sleep 8
start executor python3 -u "$PE"
sleep 8
echo "  -- 执行器 stdout/stderr --"
cat /tmp/ag_cv_executor.log 2>/dev/null | tail -10 | sed 's/^/    /'
echo

hr; echo "## 3. 控制类话题：谁在发、谁在收（关键）"
for t in /cmd_vel /navi_cmd /StopCmd /move_base/stop_signal; do
  echo "  -- $t --"
  timeout 8 ros2 topic info "$t" 2>/dev/null | sed 's/^/      /' || echo "      （不存在）"
done
echo

hr; echo "## 4. 发命令 + 同时监听速度"
mkdir -p /tmp/ag_cv
( timeout "$RUN_S" ros2 topic echo /cmd_vel > /tmp/ag_cv/cmd_vel.txt 2>&1 ) &
CV_PID=$!
( timeout "$RUN_S" ros2 topic echo /odom/localization --field pose.pose.position > /tmp/ag_cv/traj.txt 2>&1 ) &
TR_PID=$!
sleep 2

echo "  发布 reset:N10,y"
ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:N10,y'}" >/dev/null 2>&1
sleep 2
echo "  发布 $TARGET,y"
ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: '$TARGET,y'}" 2>&1 | tail -2 | sed 's/^/    /'
sleep 3
echo "  -- /nav/waypoint 订阅者（发命令时）--"
timeout 8 ros2 topic info /nav/waypoint 2>/dev/null | sed 's/^/      /'

wait "$CV_PID" 2>/dev/null
wait "$TR_PID" 2>/dev/null
echo

hr; echo "## 5. 结果：速度指令有没有发出来"
echo "  -- /cmd_vel 抓取 --"
if [ -s /tmp/ag_cv/cmd_vel.txt ]; then
  echo "      共 $(grep -c 'linear' /tmp/ag_cv/cmd_vel.txt) 条"
  grep -E "linear|angular" -A1 /tmp/ag_cv/cmd_vel.txt | grep -E "x:|z:" | head -12 | sed 's/^/      /'
  NONZERO="$(grep -A1 'linear' /tmp/ag_cv/cmd_vel.txt | grep -E '^\s+x:' | grep -vE 'x: 0\.0*$' | head -3)"
  [ -n "$NONZERO" ] && echo "      [OK] 有非零线速度：$NONZERO" \
                    || echo "      [FAIL] 抓到的线速度全是 0.0 —— 执行器没算出速度"
else
  echo "      [FAIL] /cmd_vel 一条都没抓到 —— 执行器根本没往这个话题发"
fi

echo "  -- 位移 --"
python3 - /tmp/ag_cv/traj.txt <<'PY' 2>/dev/null
import sys, math, re
try:
    pts = re.findall(r"^\s*x:\s*(-?[\d.]+)\s*\n\s*y:\s*(-?[\d.]+)",
                     open(sys.argv[1], encoding="utf-8", errors="replace").read(), re.M)
    if len(pts) < 2:
        print("      （轨迹样本不足）")
    else:
        d = math.dist((float(pts[0][0]), float(pts[0][1])), (float(pts[-1][0]), float(pts[-1][1])))
        print(f"      样本 {len(pts)}，位移 {d:.3f} m  → {'[OK] 车动了' if d > 0.15 else '[FAIL] 基本没动'}")
except Exception as e:
    print("      （算不出位移）")
PY

echo "  -- 执行器 ROS 日志（~/.ros/log 最新一份，含 INFO/WARN）--"
LATEST="$(ls -t ~/.ros/log/*.log 2>/dev/null | head -1)"
echo "      $LATEST"
grep -aiE "path|route|waypoint|odom|publisher|wait|error|fail" "$LATEST" 2>/dev/null | tail -12 | sed 's/^/      /'
echo
echo "==================== 结束 ===================="
