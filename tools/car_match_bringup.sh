#!/bin/bash
# 比赛两阶段启动（循迹对齐版）——在车上执行，或经 remote_run.py --script 上传执行。
#
# 为什么必须两阶段（dzactuator.cpp 源码实测契约）：
#   dzactuator 有模式仲裁：cmd_vel 只在模式1(DWA)被执行；path_executor 一旦
#   启动就以 20Hz 在 pursuitAngle 上发零 Twist 保活，把模式钉死在 2(Pursuit)，
#   之后 /cmd_vel 回调首行直接 return——朝向对齐的转向命令永远到不了电机。
#   所以 align 必须在 path_executor 启动之前完成：
#
#   阶段1: dzactuator(底盘) + serial_reader(定位) + integrate_node(align)
#          此时只 cmd_vel 活跃 → 模式1 → 对齐转向直通底盘（带爬行速度，
#          Ackermann 车 v==0 时舵机回中、原地不可能转）
#   等待: integrate_node 日志出现"朝向对齐完成"或"朝向对齐超时"
#   阶段2: 启动 path_executor；integrate_node 的 wait_waypoint_consumer 闸门
#          检测到 /nav/waypoint 订阅者后才发 reset + 路线（话题无滞留，
#          发早了整条丢失）
#
# 用法：
#   ./car_match_bringup.sh                     # dry_run 排练（默认，不动车）
#   AG_DRY_RUN=0 ./car_match_bringup.sh        # 实发（车会真的动！）
#   AG_WS=/tmp/ag_ws ./car_match_bringup.sh    # 指定我们的工作空间
#
# 收工：Ctrl-C 一次即可整组回收全部节点（setsid 进程组，无孤儿残留）。
set -euo pipefail

AG_WS="${AG_WS:-/tmp/ag_ws}"
AG_DRY_RUN="${AG_DRY_RUN:-1}"
ALIGN_WAIT_S="${ALIGN_WAIT_S:-25}"   # > heading_align_timeout(15s)，留出余量
LOG_DIR="/tmp/ag_logs/match_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

step() { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[警告]\033[0m %s\n' "$*"; }

# 非交互 shell 不会 source ~/.bashrc，ROS 环境必须显式加载
source /opt/ros/humble/setup.bash
source ~/dzrobot/install/setup.bash
# shellcheck source=/dev/null
source "$AG_WS/install/setup.bash"

PIDS=()
cleanup() {
    step "收工：整组回收 ${#PIDS[@]} 个进程组"
    for pid in "${PIDS[@]}"; do kill -- -"$pid" 2>/dev/null || true; done
    sleep 1
    for pid in "${PIDS[@]}"; do kill -9 -- -"$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

# ---------------- 阶段 1：底盘 + 定位 + integrate_node（align） ----------------
step "阶段1: 启动 dzactuator（底盘/串口）"
setsid ros2 launch dzactuator bringup.launch.py >"$LOG_DIR/dzactuator.log" 2>&1 &
PIDS+=($!)
sleep 3   # 等串口/模式定时器就绪

step "阶段1: 启动 serial_reader（/odom/localization）"
setsid ros2 launch car_localization localization.launch.py >"$LOG_DIR/serial_reader.log" 2>&1 &
PIDS+=($!)

step "阶段1: 启动 integrate_node（heading_bias_source=align）"
DRY_RUN_FLAG=$( [ "$AG_DRY_RUN" = "0" ] && echo false || echo true )
setsid ros2 run ag_integrate integrate_node --ros-args -p dry_run:="$DRY_RUN_FLAG" \
    >"$LOG_DIR/integrate_node.log" 2>&1 &
PIDS+=($!)

step "等待朝向对齐（最多 ${ALIGN_WAIT_S}s）……"
ALIGNED=0
for _ in $(seq "$ALIGN_WAIT_S"); do
    if grep -q "朝向对齐完成" "$LOG_DIR/integrate_node.log" 2>/dev/null; then
        ALIGNED=1
        break
    fi
    if grep -q "朝向对齐超时" "$LOG_DIR/integrate_node.log" 2>/dev/null; then
        warn "朝向对齐超时，节点将退回最近轴断言（画圆风险残留，建议检查 0x4002 链路）"
        break
    fi
    sleep 1
done
[ "$ALIGNED" = "1" ] && step "朝向对齐完成" || warn "未确认对齐完成，仍继续（排练模式可忽略）"

# ---------------- 阶段 2：path_executor 上线，路线随后下发 ----------------
step "阶段2: 启动 path_executor（pursuitAngle 通道接管底盘）"
EXEC_PY=$(find ~/dzrobot/src -name path_executor.py 2>/dev/null | head -1)
[ -z "$EXEC_PY" ] && { warn "未找到 path_executor.py"; exit 1; }
setsid python3 "$EXEC_PY" >"$LOG_DIR/path_executor.log" 2>&1 &
PIDS+=($!)

step "全部就绪。日志目录: $LOG_DIR"
step "integrate_node 检测到 /nav/waypoint 订阅者后会自动发 reset+路线"
echo "---- 实时日志（Ctrl-C 结束并回收全部节点）----"
tail -F "$LOG_DIR/integrate_node.log" "$LOG_DIR/path_executor.log"
