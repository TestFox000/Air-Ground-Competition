#!/usr/bin/env bash
# =============================================================================
# Orin 全栈一键启动：把 dzpx4_commands.txt 的 9 条手动命令收进一个 tmux 会话。
#
# 窗口顺序（依赖顺序，就绪一个起一个）:
#   dzuav    侦察感知栈（相机 -> TRT 推理 -> PnP -> UDP:50000 广播，独立于 ROS）
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
#   tools/orin_bringup.sh up              # 全栈启动（默认相机 /dev/video0 1920x1200@30）
#   tools/orin_bringup.sh up --no-teleop  # 不要键盘遥控窗口
#   DZUAV_CAM=/dev/video1 tools/orin_bringup.sh up
#   tools/orin_bringup.sh status          # 各窗口存活状态
#   tools/orin_bringup.sh down            # 全部停止
#   tmux attach -t agup                   # 接入观察（Ctrl+B 数字切窗口）
# =============================================================================
set -uo pipefail

SESSION="agup"
DZUAV_CAM="${DZUAV_CAM:-/dev/video0}"
DZUAV_SPEC="${DZUAV_SPEC:-1920x1200@30}"   # 标定分辨率（tools/calibrate_camera.py 注入的 K 基准）
WS="${WS:-$HOME/dzpx4}"                      # dzpx4 colcon 工作空间
REPO="$(cd "$(dirname "$0")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/$(ls /opt/ros 2>/dev/null | head -1)/setup.bash}"

say()  { printf '\e[1;36m==>\e[0m %s\n' "$*"; }
ok()   { printf '\e[1;32m[通过]\e[0m %s\n' "$*"; }
warn() { printf '\e[1;33m[警告]\e[0m %s\n' "$*"; }
fail() { printf '\e[1;31m[失败]\e[0m %s\n' "$*" >&2; exit 1; }

# tmux 窗口: 名字 + 启动命令（bash -lc 保留环境）
win() {  # win <名字> <命令>
    tmux new-window -d -t "$SESSION" -n "$1" "bash -lc 'source $ROS_SETUP 2>/dev/null; \
source $WS/install/setup.bash 2>/dev/null; $2'; echo '== 窗口已退出，按回车关闭'; read"
}

up() {
    command -v tmux >/dev/null || fail "缺少 tmux: sudo apt install tmux"
    [[ -f $ROS_SETUP ]] || fail "未找到 $ROS_SETUP（用 ROS_SETUP=... 指定）"
    [[ -d $WS/install ]] || warn "工作空间未编译: $WS/install 不存在（先 colcon build）"
    [[ -f $REPO/onboard/dzuav/build/dzuav ]] || warn "dzuav 未编译（orin_build_verify.sh），dzuav 窗口会起不来"
    [[ -e $DZUAV_CAM ]] || warn "相机设备 $DZUAV_CAM 不存在（DZUAV_CAM=... 指定）"

    tmux kill-session -t "$SESSION" 2>/dev/null || true
    tmux new-session -d -s "$SESSION" -n hold "echo '== agup 会话已建立 =='; read"
    say "启动窗口（依赖顺序）..."

    # 1. 感知栈：独立于 ROS，车端只依赖它的 UDP 广播，最先起
    win dzuav "bash $REPO/tools/orin_run_dzuav.sh camera $DZUAV_CAM $DZUAV_SPEC"
    ok "dzuav（UDP:50000 广播）"

    # 2. 雷达 -> SLAM
    win livox "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
    ok "livox"; sleep 3
    win fastlio "ros2 launch fast_lio_localization velodyne_localization.launch.py"
    ok "fastlio"; sleep 3

    # 3. 飞控桥 -> 里程计/IMU 桥
    win px4 "MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600"
    ok "px4(MicroXRCE)"; sleep 2
    win odom "ros2 launch px4_odom_convert px4_odom_convert.launch.py"
    ok "odom"
    win imu "ros2 launch px4_imu_bridge sensor_combined_to_imu.launch.py"
    ok "imu"; sleep 2

    # 4. 控制栈
    win control "ros2 launch px4_control px4_control.launch.py"
    ok "control"
    win path "ros2 launch px4_path px4_path.launch.py"
    ok "path"

    # 5. 键盘遥控（可选）
    if [[ "${1:-}" != "--no-teleop" ]]; then
        win teleop "ros2 run px4_control px4_teleop_node --params-file $WS/install/px4_control/share/px4_control/config/params.yaml"
        ok "teleop"
    fi

    echo
    echo "==================== 全栈已启动 ===================="
    echo "观察:  tmux attach -t $SESSION   （Ctrl+B 再按窗口名切页）"
    echo "停止:  tools/orin_bringup.sh down"
    echo "自检:  ros2 topic hz /fmu/out/vehicle_local_position  # 飞控链路活没活"
    echo "====================================================="
}

status() {
    tmux has-session -t "$SESSION" 2>/dev/null || { echo "会话 $SESSION 不存在"; exit 1; }
    tmux list-windows -t "$SESSION" -F "#{window_name}: #{session_windows}窗 #{?window_active,ACTIVE,} #{pane_current_command}"
    for w in dzuav livox fastlio px4 odom imu control path teleop; do
        alive=$(tmux list-panes -t "$SESSION:$w" -F "#{pane_dead}" 2>/dev/null)
        [[ -z $alive ]] && continue
        [[ $alive == 0 ]] && echo "  [活] $w" || echo "  [死] $w"
    done
}

down() {
    tmux kill-session -t "$SESSION" 2>/dev/null && ok "已停止 $SESSION" || warn "会话不存在"
}

case "${1:-}" in
    up)   up "${2:-}" ;;
    down) down ;;
    status) status ;;
    *) grep '^#' "$0" | sed 's/^# \{0,1\}//' | head -32; exit 2 ;;
esac
