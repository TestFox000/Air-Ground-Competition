#!/usr/bin/env bash
# =============================================================================
# Orin 全栈一键启动：把 dzpx4_commands.txt 的 9 条手动命令收进一个 tmux 会话。
#
# 窗口顺序（依赖顺序，就绪一个起一个）:
#   dzuav    侦察感知栈（相机 -> TRT 推理 -> PnP -> UDP:50000 广播，独立于 ROS）
#   plan     机载规划（消费 dzuav 本地 UDP:50000 -> route_plan.json -> 广播 0x4003 车命令）
#   livox    激光雷达驱动
#   fastlio  FAST-LIO 定位（依赖 livox）
#   px4      MicroXRCEAgent（飞控串口桥，依赖 /dev/ttyACM0）
#   odom     px4_odom_convert（SLAM -> NED）
#   imu      px4_imu_bridge（飞控 IMU -> ROS）
#   control  px4_control 状态机（Offboard 起飞/返航）
#   path     px4_path 航点跟随
#   teleop   键盘遥控（人工接管用，可选 --no-teleop 跳过）
#
# 用法:
#   tools/orin_bringup.sh up              # 全栈启动（默认相机 /dev/video0 1920x1080@30）
#   tools/orin_bringup.sh up --no-teleop  # 不要键盘遥控窗口
#   tools/orin_bringup.sh up --no-plan    # 不起机载规划窗口（调试感知栈时用）
#   tools/orin_bringup.sh restart [flags] # down + up，flags 透传给 up
#                                         # （deploy_one_click 阶段④入口；2026-10-25 之前
#                                         #   阶段④调 restart 会落进默认分支只打用法——编排层
#                                         #   看到非零退出码，但栈从未真正重启）
#   PLAN_SEND=1 tools/orin_bringup.sh up  # 机载规划真广播 0x4003 车命令（默认只落盘不发）
#   DZUAV_CAM=/dev/video1 tools/orin_bringup.sh up
#   tools/orin_bringup.sh status          # 各窗口存活状态（含已退出窗口的 rc）
#   tools/orin_bringup.sh down            # 全部停止
#   tmux attach -t agup                   # 接入观察（Ctrl+B 数字切窗口）
#
# up 的两道门（2026-10-20 加，之前是「起了就算通过」）:
#   1. 启动前查会话外残留进程：双 MicroXRCEAgent 抢 /dev/ttyACM0、双
#      px4_odom_convert 同发 vehicle_visual_odometry 会触发 EKF reset 风暴。
#      确认要共存用 ALLOW_STRAY=1。
#   2. 沉降 SETTLE_SECONDS（默认 12s）后逐窗口验活，任一窗口的命令已退出即
#      判「全栈未就绪」并非零退出——不再打印靠不住的「全栈已启动」。
#      判活看 $STATE_DIR/<窗口>.rc（命令退出时由 win() 写入），不是 pane_dead：
#      窗口命令尾巴挂了 read 好让操作手看清报错，进程死了 pane 依然"活着"。
# =============================================================================
set -uo pipefail

SESSION="agup"
DZUAV_CAM="${DZUAV_CAM:-/dev/video0}"
DZUAV_SPEC="${DZUAV_SPEC:-1920x1080@30}"   # 内参基准 1920x1080；USB 相机 1080p 只 MJPEG 有 30fps
WS="${WS:-$HOME/dzpx4}"                      # dzpx4 colcon 工作空间
REPO="$(cd "$(dirname "$0")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/$(ls /opt/ros 2>/dev/null | head -1)/setup.bash}"
# 各窗口的退出状态落在这里。不能靠 pane_dead 判活：win() 的命令尾巴上挂了
# `read`（好让操作手看清报错），窗口内的进程死了 pane 依然"活着"。
STATE_DIR="${STATE_DIR:-/tmp/agup_$SESSION}"
# 起完到判活之间的沉降时间：ROS launch 从进程存在到建好订阅要几秒，
# 太早判会把慢启动的节点误报成死亡（误报会让人对告警脱敏，同样是事故）。
SETTLE_SECONDS="${SETTLE_SECONDS:-12}"
# 飞控串口按 USB by-id 认 NxtPX4（ttyACM 编号会随插拔/其它 USB 串口漂移——
# 2026-10-22 实测机上 /dev/ttyACM0=NxtPX4、/dev/ttyACM1=CH340，若按编号写死
# 很容易把 Agent 挂到错的设备上，看到的现象是"遥测全无"而不是报错）。
# 显式覆盖：PX4_DEV=/dev/ttyACMx tools/orin_bringup.sh up
detect_px4_dev() {
    local byid
    byid=$(ls /dev/serial/by-id/usb-*NxtPX4*-if00 2>/dev/null | head -1)
    [[ -n $byid ]] && readlink -f "$byid" || echo /dev/ttyACM0
}
PX4_DEV="${PX4_DEV:-$(detect_px4_dev)}"

say()  { printf '\e[1;36m==>\e[0m %s\n' "$*"; }
ok()   { printf '\e[1;32m[通过]\e[0m %s\n' "$*"; }
warn() { printf '\e[1;33m[警告]\e[0m %s\n' "$*"; }
fail() { printf '\e[1;31m[失败]\e[0m %s\n' "$*" >&2; exit 1; }

# tmux 窗口: 名字 + 启动命令（bash -lc 保留环境）
# 命令退出后把 rc 写进 $STATE_DIR/<名字>.rc，再由 verify_up 汇总判定。
win() {  # win <名字> <命令...>
    local name="$1"; shift
    tmux new-window -d -t "$SESSION" -n "$name" "bash -lc 'source $ROS_SETUP 2>/dev/null; \
source $WS/install/setup.bash 2>/dev/null; $*'; rc=\$?; echo \$rc > $STATE_DIR/$name.rc; \
echo \"== 窗口已退出（rc=\$rc），按回车关闭\"; read"
}

# 会话外残留进程：两个 MicroXRCEAgent 会抢 /dev/ttyACM0，两个 px4_odom_convert
# 会同发 /fmu/in/vehicle_visual_odometry 触发 EKF reset 风暴（2026-10-20 实机
# 事故）；双 global_localization/transform_fusion 会同发 /localization 撕扯
# EKF（2026-10-17 实测 26~585m 跳变）。kill-session 只清得掉自己这个会话，
# 清不掉 autostart/手工（含 qoder 等编排工具）起的实例——2026-10-26 事故夜
# 即手工栈与 bringup 栈叠加。名单必须覆盖全栈所有发布关键话题的节点。
STRAY_PROCS=(MicroXRCEAgent px4_odom_convert px4_imu_bridge fastlio_mapping
             livox_ros_driver2_node dzuav px4_control_node px4_teleop_node
             path_follow_node sensor_combined_to_imu)
# python 脚本节点 comm=python3，按命令行匹配；[.] 写法防 pgrep 命令自身回环匹配
STRAY_PATTERNS=(global_localization[.]py transform_fusion[.]py
                global_map_publisher[.]py orin_plan_sidecar[.]py)

stray_pids() {
    # 输出"<名> <pid> 列表"（bash 包装行除外——那不是节点本体）
    local name pids p
    for name in "${STRAY_PROCS[@]}"; do
        pids=$(pgrep -x "$name" 2>/dev/null)
        [[ -z ${pids// /} ]] || for p in $pids; do echo "$name $p"; done
    done
    for name in "${STRAY_PATTERNS[@]}"; do
        pids=$(pgrep -af "$name" 2>/dev/null | grep -v 'bash' | awk '{print $1}')
        [[ -z ${pids// /} ]] || for p in $pids; do echo "$name $p"; done
    done
}

check_strays() {
    local found=0 line name p
    while read -r name p; do
        [[ -z $name ]] && continue
        warn "会话外已在跑 $name（pid: $p）—— 再启会双实例"
        # 把命令行打出来：挂错串口（如 Agent 指着 CH340 而不是 NxtPX4）时，
        # 光看进程名会误以为"飞控链路已有"，实际遥测是空的。
        warn "    pid $p: $(ps -o args= -p "$p" 2>/dev/null | cut -c1-120)"
        found=1
    done < <(stray_pids)
    if [[ $found == 1 ]]; then
        [[ "${ALLOW_STRAY:-0}" == "1" ]] && { warn "ALLOW_STRAY=1，继续（后果自负）"; return 0; }
        fail "请先停掉上述进程（或运行 tools/orin_bringup.sh down 全停清扫），
       确认要共存则用 ALLOW_STRAY=1 tools/orin_bringup.sh up"
    fi
}

# 双实例断言：起栈后任何关键节点 >1 实例 = 未就绪（宁可拦下也不带病起飞）
assert_single_instance() {
    local bad=0 name p count last=""
    while read -r name p; do
        [[ -z $name ]] && continue
        if [[ ${last:-} == "$name" ]]; then continue; fi
        if [[ $name == *'['*'.'*']'* ]]; then
            count=$(pgrep -af "$name" 2>/dev/null | grep -cv 'bash')
        else
            count=$(pgrep -xc "$name" 2>/dev/null || true)
        fi
        if (( count > 1 )); then
            printf '\e[1;31m[双实例]\e[0m %-24s %s 个实例——两路同发关键话题会撕扯 EKF\n' "$name" "$count"
            bad=1
        fi
        last=$name
    done < <(stray_pids)
    if (( bad == 1 )); then
        fail "存在双实例（tools/check_stack_health.py --kill-dup 可自动清多余），全栈**未**就绪，不要起飞"
    fi
}

# 起完之后逐个窗口验活：命令已退出的（有 .rc 文件）一律判失败并带上 rc。
# 这是「全栈已启动」这句话唯一的凭据——之前每个 win 后面跟一句无条件 ok，
# 相机节点秒退也照打通过，操作手据此起飞。
verify_up() {
    local -a expected=("$@")
    local name rc bad=0
    echo
    say "校验窗口存活（$SETTLE_SECONDS 秒沉降后）..."
    sleep "$SETTLE_SECONDS"
    for name in "${expected[@]}"; do
        if [[ -f $STATE_DIR/$name.rc ]]; then
            rc=$(cat "$STATE_DIR/$name.rc")
            printf '\e[1;31m[死亡]\e[0m %-8s 命令已退出 rc=%s（tmux attach -t %s:%s 看报错）\n' \
                "$name" "$rc" "$SESSION" "$name"
            bad=1
        elif [[ $(tmux list-panes -t "$SESSION:$name" -F '#{pane_dead}' 2>/dev/null) == 1 ]]; then
            printf '\e[1;31m[死亡]\e[0m %-8s pane 已死\n' "$name"
            bad=1
        else
            printf '\e[1;32m[存活]\e[0m %-8s %s\n' "$name" \
                "$(tmux list-panes -t "$SESSION:$name" -F '#{pane_current_command}' 2>/dev/null)"
        fi
    done
    # 双实例断言：窗口活着≠只有一份——别家栈/手工实例可能正在同发关键话题
    assert_single_instance
    if [[ $bad == 1 ]]; then
        echo
        fail "有窗口没起来，全栈**未**就绪。排查后重跑 up；不要据此起飞。"
    fi
}

up() {
    command -v tmux >/dev/null || fail "缺少 tmux: sudo apt install tmux"
    [[ -f $ROS_SETUP ]] || fail "未找到 $ROS_SETUP（用 ROS_SETUP=... 指定）"
    [[ -d $WS/install ]] || warn "工作空间未编译: $WS/install 不存在（先 colcon build）"
    [[ -f $REPO/onboard/dzuav/build/dzuav ]] || warn "dzuav 未编译（orin_build_verify.sh），dzuav 窗口会起不来"
    [[ -e $DZUAV_CAM ]] || warn "相机设备 $DZUAV_CAM 不存在（DZUAV_CAM=... 指定）"
    if [[ -e $PX4_DEV ]]; then
        say "飞控串口: $PX4_DEV（按 USB by-id 认 NxtPX4；可用 PX4_DEV=... 覆盖）"
    else
        warn "飞控串口 $PX4_DEV 不存在：by-id 里没找到 NxtPX4，px4 窗口会起不来"
    fi
    # 墙钟健全性：Orin 无 RTC 电池，每次上电回到 1970 纪元（2026-10-25 部署
    # 实测，当时人工 sudo date -s 对回）。闭环各节点走单调钟不受影响，但日志/
    # 录像/KML 时间轴与地面端交叉比对全数失真——赛场上表现为"飞完对不上录像
    # 段"。只强提示不自动改（改时间要 sudo + 时区确认）。
    local now_epoch
    now_epoch=$(date +%s 2>/dev/null || echo 0)
    if (( now_epoch > 0 && now_epoch < 1735689600 )); then   # 2025-01-01 之前＝明显没对过
        warn "墙钟 $(date '+%F %T' 2>/dev/null) 未对（Orin 无 RTC，上电即 1970）——"
        warn "    日志/录像时间轴会失真。起飞前先对时: sudo date -s 'YYYY-MM-DD HH:MM:SS'"
        warn "    （或 deploy_one_click.py --fix-clock，免密 sudo 时自动对）"
    fi
    check_strays

    # 参数逐个解析（旧版只看 $1：up --no-teleop --no-plan 会静默丢掉第二个）
    local want_plan=1 want_teleop=1 a
    for a in "$@"; do
        case "$a" in
            --no-plan)   want_plan=0 ;;
            --no-teleop) want_teleop=0 ;;
            *)  warn "未知参数 $a（忽略；支持 --no-plan / --no-teleop）" ;;
        esac
    done

    tmux kill-session -t "$SESSION" 2>/dev/null || true
    rm -rf "$STATE_DIR"; mkdir -p "$STATE_DIR"
    tmux new-session -d -s "$SESSION" -n hold "echo '== agup 会话已建立 =='; read"
    say "启动窗口（依赖顺序）..."

    local -a started=(hold)

    # 1. 感知栈：独立于 ROS，车端只依赖它的 UDP 广播，最先起
    win dzuav "bash $REPO/tools/orin_run_dzuav.sh camera $DZUAV_CAM $DZUAV_SPEC"
    started+=(dzuav)

    # 1.5 机载规划：官方控制要求「规划须由机载平台自主完成」——规划层在机上，
    # 消费 dzuav 的本地 UDP:50000 广播，落 route_plan.json/cell_matrix.json，
    # 并把车命令以 0x4003 广播给车端（车端薄接收 -> /nav/waypoint）。
    # ⚠️ 默认 **dry-run（只落盘不发）**：首次上机先确认产物正确，再显式
    #    PLAN_SEND=1 开真广播——坏命令直接上底盘是不可逆的。
    local plan_send_flag=""
    if [[ $want_plan == 1 ]]; then
        [[ "${PLAN_SEND:-0}" == "1" ]] && plan_send_flag="--send"
        win plan "python3 $REPO/tools/orin_plan_sidecar.py --params $REPO/src/ag_integrate/config/match_params.yaml --out /tmp/ag_plan_live $plan_send_flag"
        started+=(plan)
    fi

    # 2. 雷达 -> SLAM
    # 2026-10-26 换型 Mid-360S：launch 切 S 版（驱动 1.2.6/S 配置 install 树 10-21 已就位，
    # 机上同步提交 e2218bf）。雷达 .105 ↔ eno1 192.168.3.189。
    win livox "ros2 launch livox_ros_driver2 msg_MID360s_launch.py"
    started+=(livox); sleep 3
    win fastlio "ros2 launch fast_lio_localization velodyne_localization.launch.py"
    started+=(fastlio); sleep 3

    # 3. 飞控桥 -> 里程计/IMU 桥
    win px4 "MicroXRCEAgent serial --dev $PX4_DEV -b 921600"
    started+=(px4); sleep 2
    win odom "ros2 launch px4_odom_convert px4_odom_convert.launch.py"
    started+=(odom)
    win imu "ros2 launch px4_imu_bridge sensor_combined_to_imu.launch.py"
    started+=(imu); sleep 2

    # 4. 控制栈
    win control "ros2 launch px4_control px4_control.launch.py"
    started+=(control)
    win path "ros2 launch px4_path px4_path.launch.py"
    started+=(path)

    # 5. 键盘遥控（可选）
    if [[ $want_teleop == 1 ]]; then
        win teleop "ros2 run px4_control px4_teleop_node --params-file $WS/install/px4_control/share/px4_control/config/params.yaml"
        started+=(teleop)
    fi

    # ROS 节点从 launch 到真正建好订阅还要几秒，太早判活会误报死亡。
    verify_up "${started[@]}"

    echo
    echo "==================== 全栈已启动 ===================="
    echo "窗口:  ${started[*]}"
    [[ -n $plan_send_flag ]] && echo "规划:  实发 0x4003 车命令" \
                             || echo "规划:  dry-run（只落盘 /tmp/ag_plan_live，不发命令）"
    echo "观察:  tmux attach -t $SESSION   （Ctrl+B 再按窗口名切页）"
    echo "停止:  tools/orin_bringup.sh down"
    echo "自检:  ros2 topic hz /fmu/out/vehicle_local_position  # 飞控链路活没活"
    echo "       tools/check_stack_health.py                     # 双实例/进程数"
    echo "====================================================="
}

status() {
    tmux has-session -t "$SESSION" 2>/dev/null || { echo "会话 $SESSION 不存在"; exit 1; }
    tmux list-windows -t "$SESSION" -F "#{window_name}: #{session_windows}窗 #{?window_active,ACTIVE,} #{pane_current_command}"
    for w in dzuav plan livox fastlio px4 odom imu control path teleop; do
        alive=$(tmux list-panes -t "$SESSION:$w" -F "#{pane_dead}" 2>/dev/null)
        [[ -z $alive ]] && continue
        if [[ -f $STATE_DIR/$w.rc ]]; then
            echo "  [死] $w（命令已退出 rc=$(cat "$STATE_DIR/$w.rc")）"
        elif [[ $alive == 0 ]]; then
            echo "  [活] $w"
        else
            echo "  [死] $w"
        fi
    done
}

down() {
    tmux kill-session -t "$SESSION" 2>/dev/null && ok "已停止 $SESSION" || warn "会话不存在"
    rm -rf "$STATE_DIR"
    # 全停清扫：kill-session 清不掉会话外起的实例（手工/qoder/autostart）。
    # down 的语义 = "全停"——不留任何双实例口子（2026-10-26 事故夜教训）。
    local line name p still
    while read -r name p; do
        [[ -z $name ]] && continue
        kill "$p" 2>/dev/null && warn "已清扫会话外实例 $name (pid $p)"
    done < <(stray_pids)
    sleep 1
    # 还活着的（忽略 TERM）补 SIGKILL
    while read -r name p; do
        [[ -z $name ]] && continue
        kill -9 "$p" 2>/dev/null && warn "强杀 $name (pid $p)"
    done < <(stray_pids)
}

case "${1:-}" in
    up)      shift; up "$@" ;;
    restart) shift; say "restart: down + up（$*）"; down; up "$@" ;;
    down)    down ;;
    status)  status ;;
    # 只打文件头那段连续注释块（到第一行非注释为止），不用写死行数——
    # 之前 head -32 会在头部变长后把用法说明截掉一半。
    *) awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"; exit 2 ;;
esac
