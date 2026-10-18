#!/usr/bin/env bash
# =============================================================================
# offline_test.sh —— 离线端到端测试一键启动
# =============================================================================
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/offline_test.sh" [地图]
#
# 它做三件事（车端请**另开终端**自己启动）：
#   1. 检查环境（ROS / 工作空间 / 参数）
#   2. 后台起 fake_sensors.py（假 IMU/odom/视觉 -> 让定位健康）
#   3. 等 2 秒后起 drone_sim.py（发地图侦察数据 + 任务指令）
#   4. Ctrl-C 时**一起**收掉两个子进程
#
# ★ 为什么不在这里起车端：
#   车端日志是你观察的主要输出，放在你自己的终端里更清楚。
#   本脚本只负责"喂数据"。
#
# 用法：
#   bash offline_test.sh                                  # 用标准局地图
#   bash offline_test.sh ~/ws/src/dzmission/tools/maps/example_redblue.yaml
#   FACE=1 bash offline_test.sh                           # 红面（测禁火）
# =============================================================================

set +e
BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %s\n" "$1"; }
bad()  { printf "  ${RED}✗${RST} %s\n" "$1"; }
warn() { printf "  ${YEL}!${RST} %s\n" "$1"; }
info() { printf "  ${CYN}·${RST} %s\n" "$1"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

# 允许用环境变量覆盖
FACE="${FACE:-2}"                 # 靶标面：0未知 1红/友 2蓝/敌
PERIOD="${PERIOD:-4.0}"           # 靶标往复周期（秒）
DRIVE="${DRIVE:-0}"               # 1 = 让"车"按 /cmd_vel 真的动
MAP="${1:-}"

printf "${BOLD}离线端到端测试${RST}\n"

# ---------------------------------------------------------------- 1. 环境
sec "1. 环境检查"

if [ -z "${ROS_DISTRO:-}" ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash 2>/dev/null
fi
if [ -z "${ROS_DISTRO:-}" ]; then
  bad "ROS 2 未就绪 —— 先 source /opt/ros/humble/setup.bash"
  exit 1
fi
ok "ROS_DISTRO=$ROS_DISTRO"

if ! command -v ros2 >/dev/null 2>&1; then
  bad "ros2 不可用"
  exit 1
fi

# 工作空间
WS="$HOME/ws"
SRC="$WS/src/dzmission"
if [ ! -d "$SRC/tools" ]; then
  # 退回已安装位置
  SRC="$(ros2 pkg prefix dzmission 2>/dev/null)/share/dzmission"
fi
if [ ! -d "$SRC/tools" ]; then
  bad "找不到 dzmission/tools —— 先编译：bash build_ws.sh"
  exit 1
fi
ok "工具目录: $SRC/tools"

# 地图
if [ -z "$MAP" ]; then
  MAP="$SRC/tools/maps/example_standard.yaml"
fi
if [ ! -f "$MAP" ]; then
  bad "地图不存在: $MAP"
  printf "  可用的：\n"
  ls "$SRC/tools/maps/" 2>/dev/null | sed 's/^/      /'
  exit 1
fi
ok "地图: $(basename "$MAP")"

# ★ 关键：车端必须是 udp 模式
YAML="$WS/src/dzmission/params/dzmission.yaml"
if [ -f "$YAML" ]; then
  # ★★ 解析要点（这里踩过两个坑）★★
  #   坑 1：Windows 侧写出的 yaml 是 **CRLF** 行尾，行尾会带一个 \r。
  #         若只 tr -d '"'，变量会变成 "udp\r"，和 "udp" 比较失败。
  #         => 必须 tr -d '"\r' 一并去掉回车。
  #   坑 2：`grep -E '^\s*transport:'` 会**先命中注释行**
  #         （文件里有 `# transport: serial | udp | loopback`）。
  #         => 用 grep -v '^[[:space:]]*#' 先排除注释。
  #   坑 3：grep 的 `\s` 不是标准 ERE，用 [[:space:]] 更可靠。
  TR=$(grep -E '^[[:space:]]*transport:' "$YAML" \
        | grep -v '^[[:space:]]*#' \
        | head -1 \
        | sed 's/.*transport:[[:space:]]*//' \
        | tr -d '"' | tr -d '\r' | tr -d ' ')
  if [ "$TR" = "udp" ]; then
    ok "cmd.transport = udp"
  else
    bad "cmd.transport = ${TR:-未设置} —— 必须是 udp 才能收本工具的数据"
    printf "  修：把 %s 里 cmd.transport 改成 \"udp\"（Windows 侧改才持久）\n" "$YAML"
    exit 1
  fi
else
  warn "找不到 $YAML，无法确认 transport（继续）"
fi

# 车端是否在跑
if ros2 node list 2>/dev/null | grep -q dzmission; then
  ok "车端 dzmission 在运行"
else
  warn "没看到 dzmission 节点 —— 请**另开终端**启动："
  printf "      ros2 launch dzmission bringup_mission.launch.py\n"
  printf "  （本脚本只负责喂数据，可以继续）\n"
fi

# ---------------------------------------------------------------- 2. 启动子进程
sec "2. 启动假传感器 + 地图模拟器"

PIDS=()
cleanup() {
  printf "\n${BOLD}收尾：停止子进程…${RST}\n"
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null && info "已停止 pid $pid"
  done
  wait 2>/dev/null
  printf "  ${GRN}已全部停止${RST}\n"
}
trap cleanup EXIT INT TERM

DRIVE_ARG=""
[ "$DRIVE" = "1" ] && DRIVE_ARG="--drive-from-cmd"

printf "  启动 fake_sensors（face=%s period=%ss drive=%s）…\n" "$FACE" "$PERIOD" "$DRIVE"
python3 "$SRC/tools/fake_sensors.py" \
    --face "$FACE" --period-s "$PERIOD" $DRIVE_ARG \
    > /tmp/fake_sensors.log 2>&1 &
PIDS+=($!)
info "fake_sensors pid=${PIDS[-1]}  日志: /tmp/fake_sensors.log"

sleep 2

printf "  启动 drone_sim（地图=%s）…\n" "$(basename "$MAP")"
python3 "$SRC/tools/drone_sim.py" --map "$MAP" \
    > /tmp/drone_sim.log 2>&1 &
PIDS+=($!)
info "drone_sim pid=${PIDS[-1]}  日志: /tmp/drone_sim.log"

# ---------------------------------------------------------------- 3. 观察
sec "3. 观察（另开终端）"
cat <<'EOF'
    车端日志      : 你启动 dzmission 的那个终端

    收到的原始帧  : ros2 topic echo /mission_cmd_raw
    解析后的指令  : ros2 topic echo /mission_cmd
    定位健康      : ros2 topic echo /localization/healthy
    过滤后里程    : ros2 topic echo /odometry/filtered
    假传感器是否在发:
                    ros2 topic hz /imu_data
                    ros2 topic hz /odom
                    ros2 topic hz /offset_center

    ★ 期望在车端日志里看到：
        [Mission] 地雷清单 8 颗            <- 排雷数据通了（P0 生效）
        [Mission] 门位 ...                 <- 桥洞数据通了
        [Mission] 排雷计划 ...             <- 40 分功能在跑
        [Mission] NAVIGATING -> ...        <- 定位健康后能导航了
EOF

cat <<EOF

  ${BOLD}本脚本前台运行中（Ctrl-C 一起停止）${RST}
    fake_sensors 日志: tail -f /tmp/fake_sensors.log
    drone_sim    日志: tail -f /tmp/drone_sim.log

  若 15 秒内没动静，看上面两个日志排错。
  常见原因：
    · 车端 transport 还是 loopback
    · 车端没用 BEST_EFFORT 订阅（本脚本已匹配，应无问题）
    · 端口不对（车端 udp_bind_port 与 drone_sim --port 默认都是 9100）

EOF

# ---------------------------------------------------------------- 4. 心跳打印
n=0
while true; do
  sleep 10
  n=$((n + 10))
  printf "  ${CYN}[%3ds]${RST} 仍在运行… " "$n"
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      printf "pid%s=ok " "$pid"
    else
      printf "${RED}pid%s=已退出${RST} " "$pid"
    fi
  done
  printf "\n"
  if [ -s /tmp/drone_sim.log ]; then
    printf "        drone_sim 最近输出: %s\n" "$(tail -1 /tmp/drone_sim.log | cut -c1-70)"
  fi
done
