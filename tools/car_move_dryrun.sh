#!/usr/bin/env bash
# 让车「能动」：起最小发车集合 + 我们的决策层。**默认不动车**。
#
# 最小发车集合（2026-09-27 实测通过；与官方 bringup_all 的区别是**不碰云台**，
# 相机/det_node 只在 AG_WITH_VISION=1 时才起 —— 官方栈 2026-09-28 已修好，视觉链可用）：
#   dzactuator bringup.launch.py      底盘 / IMU / 里程计（没人发速度指令就不会动）
#   dzactuator judgment.launch.py     裁判串口（读 /HpAndHitmsg）
#   car_localization localization.launch.py   serial_reader（定位）
#   car_localization/path_executor.py  真正执行 /nav/waypoint 的那一段
#   ag_car/car_node                    我们的决策层（发 /nav/waypoint）
#
# ⚠️ 安全设计（默认不动车）：
#   不加 AG_CONFIRM_MOVE=1 时，car_node 以 dry_run=true 运行 —— 底盘是活的、路线算得出来，
#   但**一条 /nav/waypoint 都不会发**，车不会动。适合先验证整条链。
#   真要发车必须同时给 AG_CONFIRM_MOVE=1，并且**人在车边、遥控器在手**。
#
# ⚠️ 绝不同时启动 dzrknn_dataprocess：它一跑就会扫云台并**自主开火**，且不区分蓝红
#   （官方"停车才开火"的 /move_base/stop_signal 是死代码，无订阅者）。本脚本不启动它。
#
# 用法：
#   # 先只验链路，车不动（推荐第一次这样跑）
#   python tools/remote_run.py --host <IP> --user duzhong --script tools/car_move_dryrun.sh
#   # 真发车（有人在场！）
#   python tools/remote_run.py --host <IP> --user duzhong --script tools/car_move_dryrun.sh \
#     --env AG_CONFIRM_MOVE=1 --env AG_RUN_S=60
#
# 环境变量：
#   AG_CONFIRM_MOVE=1   真的发车（不给就只是 dry_run）
#   AG_RUN_S=60         跑多久（秒）
#   AG_USE_AIR=1        情报来自机载端（默认用 fake_intel 直连车端，供练车）
#   AG_NO_JUDGMENT=1    不启动裁判串口
#   AG_WITH_VISION=1    连相机 + det_node(RKNN 检测) 一起起（默认只起底盘那套）。
#                       ⚠️ 仍然**不启动 dzrknn_dataprocess**（瞄准/开火链）——
#                          它一跑就会扫云台并**自主开火**，而且不区分蓝红。

# ⚠️ 不要用 set -u（ROS setup.bash 会引用未定义变量，非交互 shell 直接退出）
set -o pipefail

RUN_S="${AG_RUN_S:-45}"
CONFIRM="${AG_CONFIRM_MOVE:-0}"
USE_AIR="${AG_USE_AIR:-0}"
NO_JUDGMENT="${AG_NO_JUDGMENT:-0}"
WITH_VISION="${AG_WITH_VISION:-0}"

PIDS=""
start() {   # start <名字> <命令行...>
  local name="$1"; shift
  setsid "$@" > "/tmp/ag_move_${name}.log" 2>&1 &
  local pg=$!
  PIDS="$PIDS $pg"
  echo "  $name 进程组=$pg -> /tmp/ag_move_${name}.log"
}

# ⚠️ kill 进程组：`ros2 launch` 自己是壳，子节点跑在它的进程组里。
#    只 kill 壳会留下孤儿节点继续在车上跑（这个坑真的踩过）。
cleanup() {
  for pg in $PIDS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 3
  for pg in $PIDS; do kill -9 -"$pg" 2>/dev/null; done
  pkill -f "path_executor.py" 2>/dev/null
  pkill -f "car_localization/serial_reader" 2>/dev/null
  pkill -f "ag_car/car_node" 2>/dev/null
  pkill -f "ag_bridge/fake_intel_node" 2>/dev/null
  echo "  已停止本次启动的进程组与节点"
}
trap cleanup EXIT

hr() { echo "----------------------------------------------------"; }

# 官方栈环境：官方 install 已由 car_restore_official.sh 补全（2026-09-28）
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
[ -f "${AG_WS:-/tmp/ag_ws}/install/setup.bash" ] && source "${AG_WS:-/tmp/ag_ws}/install/setup.bash" 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export AG_LOG_DIR="${AG_LOG_DIR:-/tmp/ag_logs}"
export AG_LOG_SESSION="${AG_LOG_SESSION:-move}"

echo "==================== 让车能动（默认 dry_run，车不动） ===================="
date
echo "  模式: $([ "$CONFIRM" = "1" ] && echo '★ 真发车（AG_CONFIRM_MOVE=1）★' || echo '只验链路（dry_run，不发命令）')"
echo "  情报: $([ "$USE_AIR" = "1" ] && echo '机载端（需 air_link 在跑）' || echo 'fake_intel 直连车端（练车用）')"
echo "  视觉: $([ "$WITH_VISION" = "1" ] && echo '相机 + det_node(RKNN) 一起起；瞄准/开火链仍不开' || echo '不起（只要视觉加 --with-vision）')"
echo "  时长: ${RUN_S}s    日志: $AG_LOG_DIR/$AG_LOG_SESSION"
echo

if [ "$CONFIRM" = "1" ]; then
  hr; echo "########## 安全确认（真发车） ##########"
  echo "  车会按路线真的动起来，确认以下几件事："
  echo "    1) 有人站在车边，遥控器在手（可紧急接管）"
  echo "    2) 车周围 2 m 内没有人和障碍；或车已被架起"
  echo "    3) 场地格号/起点朝向与 params.yaml 一致（start_cell）"
  echo "    4) 视觉与瞄准：相机/det_node 可用（AG_WITH_VISION=1 时启动）；但**开火链默认不开**"
  echo "  5 秒后开始，Ctrl-C 可立即中止..."
  sleep 5
fi
echo

hr; echo "## 0. 前置检查"
NODES="$(timeout 10 ros2 node list 2>/dev/null)"
[ -n "$NODES" ] && { echo "  ⚠️ 车上已有节点在跑，先确认不是别人在用："; echo "$NODES" | sed 's/^/    /'; } \
                 || echo "  [OK] ROS 图是空的"
# ⚠️ 只查一次 `ros2 pkg list` 再复用：实测连着调两次会得到不一致的结果
#    （第一次有 dzactuator、第二次没有），于是"明明装好了却报缺包"直接退出。
PKG_LIST="$(ros2 pkg list 2>/dev/null)"
MISSING=""
for p in car_localization dzactuator ag_car ag_bridge; do
  if echo "$PKG_LIST" | grep -qx "$p"; then
    echo "  [OK] $p"
  else
    echo "  [缺] $p"
    MISSING="$MISSING $p"
  fi
done
if [ -n "$MISSING" ]; then
  case "$MISSING" in
    *ag_car*|*ag_bridge*)
      echo "  [FAIL] 我们自己的包还没在这台车上编译（/tmp/ag_ws 可能被 cleanup 清过）—— 先："
      echo "    python tools/remote_run.py --host <IP> --user duzhong --put src --dest /tmp/ag_ws/src"
      echo "    python tools/remote_run.py --host <IP> --user duzhong --timeout 900 --script tools/car_build.sh" ;;
    *)
      echo "  [FAIL] 缺官方包（${MISSING}）—— 先跑：python car.py restore <IP>" ;;
  esac
  echo "  （本次 ros2 pkg list 共 $(echo "$PKG_LIST" | wc -l) 个包）"
  exit 1
fi
[ -f "$HOME/dzrobot/src/car_localization/car_localization/path_executor.py" ] \
  && echo "  [OK] path_executor.py 源码在" \
  || { echo "  [FAIL] 找不到 path_executor.py"; exit 1; }
echo

hr; echo "## 1. 起底盘 / 裁判 / 定位 / 路线执行器"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 10
grep -E "serial port opened|can not open|wakeup|mode" /tmp/ag_move_actuator.log 2>/dev/null | tail -4 | sed 's/^/    /'

if [ "$NO_JUDGMENT" != "1" ]; then
  start judgment ros2 launch dzactuator judgment.launch.py
  sleep 6
  grep -E "Serial initialize|failed|baudrate" /tmp/ag_move_judgment.log 2>/dev/null | tail -3 | sed 's/^/    /'
  # 我们自己的 referee_node：订阅官方 /HpAndHitmsg、/self_Material_Number，把
  # 血量 / 弹药 / 受击计数**记进黑匣子**。没有它，复盘只能说"车跑了"，
  # 说不出"到底打中没打中、还剩几条命" —— 而实战里这才是关键结论。
  start referee ros2 run ag_referee referee_node --ros-args \
    -p source:=dz -p hp_topic:=/HpAndHitmsg -p log_dir:="$AG_LOG_DIR"
  sleep 4
  tail -2 /tmp/ag_move_referee.log 2>/dev/null | sed 's/^/    /'
else
  echo "  （按要求跳过裁判串口 —— 本次不会有血量/弹药数据，复盘会缺一块）"
fi

start localization ros2 launch car_localization localization.launch.py
sleep 6

start executor python3 "$HOME/dzrobot/src/car_localization/car_localization/path_executor.py"
sleep 6
grep -E "Initialized|Command format|wait" /tmp/ag_move_executor.log 2>/dev/null | head -4 | sed 's/^/    /'
echo

if [ "$WITH_VISION" = "1" ]; then
  hr; echo "## 1b. 视觉链（相机 + RKNN 检测）"
  # det_node 内部会 cv::imshow 画框：没有图形后端**一帧就崩**，而且崩得很安静
  # （外部只看到 /image_raw 没有发布者，或 det_node 悄悄退出）。车上有 VNC 桌面(X0)就借它的 DISPLAY。
  if [ -z "${DISPLAY:-}" ] && [ -e /tmp/.X11-unix/X0 ]; then
    export DISPLAY=:0 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
    echo "  借用车上的 X 会话：DISPLAY=$DISPLAY"
  fi
  if [ -z "${DISPLAY:-}" ]; then
    echo "  ⚠️ 没有可用的 X 会话 —— det_node 的 cv::imshow 很可能启动后即崩。"
    echo "     解法：先在车上开一下 VNC 桌面（或 SSH 里 export DISPLAY=:0）再跑。"
  fi

  start camera ros2 launch dzactuator camera.launch.py
  sleep 8
  tail -3 /tmp/ag_camera.log 2>/dev/null | sed 's/^/    /'
  if grep -q "process has died" /tmp/ag_camera.log 2>/dev/null; then
    echo "  [FAIL] usb_cam 崩了（真机踩过 SIGABRT）：设备被占 / USB 供电或带宽不足"
  fi

  start det ros2 launch dzactuator det_node.launch.py
  sleep 12
  if grep -qE "process has died|GTK|Cannot open display" /tmp/ag_det.log 2>/dev/null; then
    echo "  [FAIL] det_node 崩了 —— 多半是没有图形后端（见上面的告警）"
  fi
  echo "  -- /offset_center（检测输出；cls 1=蓝(友) 2=红）--"
  RAW="$(timeout 20 ros2 topic echo --once /offset_center --field data 2>/dev/null | tr '\n' ' ')"
  echo "    ${RAW:-（无数据：相机没出图 / 模型没加载 / 话题名不同）}"
  echo "  ⚠️ 瞄准与开火链（dzrknn_dataprocess）**本次仍未启动** ——"
  echo "     它一跑就会扫云台并自主开火，且不区分蓝红（官方\"停车才开火\"的信号是死代码）。"
fi
echo

hr; echo "## 2. 定位是否出来（发车前提）"
# 字段路径读不到时退回整条消息：不同版本的 /odom/localization 字段布局略有差异，
# 只认一种写法会误报"定位没起来"（其实节点在跑）。
read_pos() {
  local v
  v=$(timeout 12 ros2 topic echo --once /odom/localization --field pose.pose.position 2>/dev/null | tr '\n' ' ')
  if [ -z "$v" ]; then
    v=$(timeout 12 ros2 topic echo --once /odom/localization 2>/dev/null \
        | grep -E "^ *[xy]:|block" | head -3 | tr '\n' ' ')
  fi
  if [ -z "$v" ]; then
    # 有些发布者是 BEST_EFFORT，默认 QoS 的 echo 收不到任何东西
    v=$(timeout 12 ros2 topic echo --once /odom/localization --qos-reliability best_effort 2>/dev/null \
        | grep -E "^ *[xy]:|block" | head -3 | tr '\n' ' ')
  fi
  echo "$v"
}
POS0=$(read_pos)
echo "  当前位置: ${POS0:-（读不到）}"
if [ -z "$POS0" ]; then
  echo "  ⚠️ 读不到定位：底盘/定位没起来，或话题名不同。看 /tmp/ag_move_localization.log"
  # ---- 硬门禁：真发车必须有定位 ----
  # 车是照着 `/nav/waypoint` 跑的，而执行那一段（path_executor）靠定位位姿算转向；
  # 定位读不到 = 车不知道自己在哪 = 发车就是盲跑（撞东西或跑丢）。
  # 所以这里**拒绝发车**，而不是"提醒一下继续跑"。
  if [ "$CONFIRM" = "1" ] && [ "${AG_ALLOW_NO_LOC:-0}" != "1" ]; then
    echo
    echo "  [FAIL] 要真发车（AG_CONFIRM_MOVE=1），但读不到定位 —— **已拒绝发车**。"
    echo "         先解决定位（脚本会把已启动的节点都停掉，不会留下半套东西）："
    echo "           tools/car_preflight.sh        # 一键体检：会列出真实话题名并逐个验活"
    echo "           ros2 topic list | grep -i odom"
    echo "           ros2 topic hz /odom/localization"
    echo "         确实不需要定位（例如只想原地试转向）时，才加 AG_ALLOW_NO_LOC=1 强行放行。"
    echo "  本次启动的节点将自动停止。"
    exit 1
  fi
  echo "  （dry_run：只算不发，定位读不到不影响本次验证；但真发车前必须解决）"
fi
echo

hr; echo "## 3. 情报源"
if [ "$USE_AIR" = "1" ]; then
  echo "  等机载端 air_link 往 /link/field_intel 发（不启动 fake_intel）"
else
  start fake_intel ros2 run ag_bridge fake_intel_node --ros-args \
    -p intel_topic:=/link/field_intel -p mode:=full -p period_s:=0.5
  echo "  已启动 fake_intel -> /link/field_intel（full 模式：8 雷 / K4 K9 桥洞 / D2 蓝靶）"
fi
sleep 3
echo

hr; echo "## 4. 决策层 car_node（$([ "$CONFIRM" = "1" ] && echo 'dry_run=false ★会发车★' || echo 'dry_run=true 只算不发')）"
if [ "$CONFIRM" = "1" ]; then
  start car_node ros2 run ag_car car_node --ros-args \
    -p dry_run:=false -p start_cell:=N10 -p goal_cell:=N11 -p log_dir:="$AG_LOG_DIR"
else
  start car_node ros2 run ag_car car_node --ros-args \
    -p dry_run:=true -p start_cell:=N10 -p goal_cell:=N11 -p log_dir:="$AG_LOG_DIR"
fi
sleep 8
echo "  -- /car/route_status --"
timeout 12 ros2 topic echo --once /car/route_status --field data 2>/dev/null | sed 's/^/    /'
echo

hr; echo "## 5. 跑 ${RUN_S}s（同时记录：/nav/waypoint + 轨迹 + 裁判状态）"
# 这些证据**必须落进日志目录**，否则取回日志时它们不会跟着回来，
# 复盘就只能看个黑匣子、看不到"车到底走到哪、还剩几条命"。
RUN_DIR="$AG_LOG_DIR/run"
mkdir -p "$RUN_DIR" 2>/dev/null

# 轨迹采样（只读话题，不影响控制）：跑完要能回答"它到底走到哪、有没有跑偏"。
timeout "$RUN_S" ros2 topic echo /odom/localization --field pose.pose.position \
  > "$RUN_DIR/traj.txt" 2>&1 &
TRAJ_PID=$!
# 裁判状态采样：血量/弹药/受击随时间的变化（我们 referee_node 发的 JSON）。
timeout "$RUN_S" ros2 topic echo /referee/state_json --field data \
  > "$RUN_DIR/referee.txt" 2>&1 &
REF_PID=$!

timeout "$RUN_S" ros2 topic echo /nav/waypoint --field data > "$RUN_DIR/waypoint.txt" 2>/dev/null

wait "$TRAJ_PID" 2>/dev/null
wait "$REF_PID" 2>/dev/null

echo "  -- 发出的 /nav/waypoint 命令 --"
if [ -s "$RUN_DIR/waypoint.txt" ]; then
  grep -v '^---$' "$RUN_DIR/waypoint.txt" | awk '!seen[$0]++' | head -10 | sed 's/^/    /'
else
  echo "    （没有发出任何命令）"
fi

echo "  -- 轨迹（定位采样） --"
SAMPLES="$(grep -cE '^[[:space:]]*x:' "$RUN_DIR/traj.txt" 2>/dev/null)"
if [ "${SAMPLES:-0}" -gt 0 ]; then
  echo "      样本 $SAMPLES 个，落在 $RUN_DIR/traj.txt"
  grep -E '^[[:space:]]*[xyz]:' "$RUN_DIR/traj.txt" | head -3 | sed 's/^/      起 /'
  echo "      ..."
  grep -E '^[[:space:]]*[xyz]:' "$RUN_DIR/traj.txt" | tail -3 | sed 's/^/      终 /'
else
  echo "      （没有轨迹数据：定位读不到，或话题名不同 —— 这正是要修的东西）"
fi

echo "  -- 裁判状态（最后一条） --"
tail -1 "$RUN_DIR/referee.txt" 2>/dev/null | sed 's/^/      /' \
  || echo "      （没有裁判数据：referee_node 没起 / 官方 dzjudgment 没出 /HpAndHitmsg）"
echo

hr; echo "## 6. 结果"
POS1=$(read_pos)
echo "  起步位置: ${POS0:-?}"
echo "  结束位置: ${POS1:-?}"
if [ "$CONFIRM" = "1" ]; then
  [ "$POS0" != "$POS1" ] && echo "  [OK] 位置发生变化 —— 车真的动了" \
                         || echo "  ⚠️ 位置没变：可能已到点、路线为空、或底盘没执行（看 $RUN_DIR 里的 executor 日志）"
else
  echo "  （dry_run 模式：车不应移动；位置变化说明有别人在控制，立即检查）"
fi

# 把所有证据（各节点原始日志 + 轨迹/裁判/路点采样）统一收进日志目录，
# 这样 `python car.py logs <IP>` 一次就能全部取回 —— 复盘缺的就是这些原始材料。
cp -f /tmp/ag_move_*.log "$RUN_DIR/" 2>/dev/null
echo "  原始材料已归档到 $RUN_DIR ："
ls "$RUN_DIR" 2>/dev/null | sed 's/^/      /'
echo
echo "取回日志复盘："
echo "  python tools/remote_run.py --host <IP> --user duzhong --get $AG_LOG_DIR --to logs/car"
echo "  python tools/log_report.py --dir logs/car"
echo "==================== 结束（节点将自动停止） ===================="
