#!/usr/bin/env bash
# =============================================================================
#  单节点重启工具   restart_node.sh
# -----------------------------------------------------------------------------
#  改了 config/robot.yaml 里的参数后，不需要整车 restart —— 只把受影响的那个
#  节点重起即可，省时且不会打断正在跑的其他环节。
#
#  用法:
#      ./restart_node.sh gimbal       # 只重启云台自瞄（改扫描/开火参数后）
#      ./restart_node.sh camera       # 只重启相机（改曝光/分辨率后）
#      ./restart_node.sh vision       # 只重启视觉检测（换模型后）
#      ./restart_node.sh path         # 只重启循迹（改路线/速度后）
#      ./restart_node.sh viewer       # 只重启网页弹窗
#      ./restart_node.sh --list       # 看看有哪些可重启
#
#  ★ 为什么需要它：以前换模型要整车 restart，车会重新跑一遍路线；改一个开火
#    阈值也要等 20 多秒全链路重来。现在只动一个节点，1~2 秒的事。
# =============================================================================
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGDIR="$HOME/robot_logs"
PIDDIR="$LOGDIR/pids"
CONFIG="${ROBOT_CONFIG:-$WS/config/robot.yaml}"
mkdir -p "$LOGDIR" "$PIDDIR"

# 名称 -> "日志名|启动命令"
node_spec() {
  case "$1" in
    gimbal) echo "5_gimbal|ros2 launch dzrknn_dataprocess dzrknn_dataprocess.launch.py" ;;
    camera) echo "2_camera|ros2 launch dzactuator camera.launch.py" ;;
    vision) echo "4_det|ros2 launch det_node det_node.launch.py" ;;
    path)   echo "7_path|python3 $WS/src/car_localization/car_localization/path_executor.py" ;;
    viewer) echo "8_camview|python3 $WS/tools/cam_viewer.py --port 8080" ;;
    judgment) echo "6_judgment|ros2 launch dzjudgment judgment.launch.py" ;;
    chassis) echo "1_chassis|ros2 launch dzactuator bringup.launch.py" ;;
    loc)    echo "3_localization|ros2 launch car_localization localization.launch.py" ;;
    *) return 1 ;;
  esac
}

if [ "${1:-}" = "--list" ] || [ $# -eq 0 ]; then
  cat <<'EOF'

可重启的节点:
    gimbal    云台自瞄   改扫描角度/速度/开火阈值/只打蓝靶  ← 最常用
    judgment  射击驱动   开火没反应时先重启它（激光模块通讯）
    camera    相机       改曝光/分辨率/帧率
    vision    视觉检测   换模型文件后
    path      循迹       改路线/巡航速度/急停余量
    viewer    网页弹窗
    chassis   底盘串口
    loc       定位

例:  ./restart_node.sh gimbal
EOF
  exit 0
fi

SPEC=$(node_spec "$1") || { echo "❌ 未知节点: $1  （用 --list 查看）"; exit 1; }
LOGNAME="${SPEC%%|*}"
CMD="${SPEC##*|}"

echo "重启节点: $1   (日志 $LOGNAME.log)"
echo "配置中心: $CONFIG"

# ---- 1) 按 PID 精确停掉旧实例（进程组回收，连带子进程）----
PIDFILE="$PIDDIR/${LOGNAME}.pid"
if [ -f "$PIDFILE" ]; then
  OLD=$(cat "$PIDFILE" 2>/dev/null)
  if [ -n "$OLD" ] && kill -0 "$OLD" 2>/dev/null; then
    kill -TERM -- "-$OLD" 2>/dev/null || kill -TERM "$OLD" 2>/dev/null
    for _ in 1 2 3 4 5 6; do kill -0 "$OLD" 2>/dev/null || break; sleep 0.5; done
    kill -KILL -- "-$OLD" 2>/dev/null || kill -KILL "$OLD" 2>/dev/null
    echo "  已停止旧实例 pid=$OLD"
  fi
  rm -f "$PIDFILE"
else
  echo "  没有 PID 记录（可能本来就没在跑）"
fi

# 网页弹窗另按端口兜底
if [ "$1" = "viewer" ]; then
  for _p in $(ss -lntp 2>/dev/null | grep ':8080' | grep -oP '(?<=pid=)[0-9]+'); do
    kill -9 "$_p" 2>/dev/null
  done
fi

# ---- 2) 起新实例 ----
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

EXTRA=()
[ "$1" = "vision" ] && EXTRA=(env "DISPLAY=${GUI_DISPLAY:-:1}")

# shellcheck disable=SC2086
setsid "${EXTRA[@]}" $CMD > "$LOGDIR/${LOGNAME}.log" 2>&1 &
NEW=$!
echo "$NEW" > "$PIDFILE"
echo "  已启动新实例 pid=$NEW"
sleep 3

if kill -0 "$NEW" 2>/dev/null; then
  echo "  ✅ 运行中。看日志:  tail -f $LOGDIR/${LOGNAME}.log"
  # 云台节点顺手回显关键参数，避免"改了没生效"还要自己去查
  if [ "$1" = "gimbal" ]; then
    sleep 3
    grep -hE "瞄准:|射击:" "$LOGDIR/${LOGNAME}.log" 2>/dev/null | sed 's/^/     /'
  fi
else
  echo "  ❌ 启动即退出！日志尾部:"
  tail -12 "$LOGDIR/${LOGNAME}.log" 2>/dev/null | sed 's/^/     /'
  exit 1
fi
