#!/usr/bin/env bash
# ==============================================================================
# start_drone.sh v4 —— 无人机手动全栈（上层）一键启动
# 卡号: DRONE-AUTOSTART-20261024 · 2026-10-24 · 安装时替换 /home/duzhong/start_drone.sh
#
# 与 v3 的差异（10-17「自启+手动双栈导致定位发散」事故对策，逐条对应）：
#   ① 三件套（dz-agent 飞控链路 / dz-livox 雷达 / dz-link 心跳）由 systemd 常保。
#      本脚本【不 pkill、不手动 spawn 三件套】，开头只做：
#        systemctl stop  →  systemctl start   （经 systemd 复位到干净单实例）
#      v3 的 pkill 全列表 + 重复 spawn 是双栈的直接成因，v4 从机制上消掉。
#   ② 本脚本只起上层 5 件：FAST-LIO / px4_odom_convert / px4_imu_bridge /
#      px4_control / (px4_path + teleop)。
#   ③ px4_odom_convert 显式 mode:=odom：launch.py 默认覆盖成 localization，
#      会订阅不存在的 /localization 死话题（transform_fusion.py 已死于
#      ImportError），永远不给 PX4 发外部视觉 —— 10-23 审计实锤，v4 必须带参。
#   ④ FAST-LIO rviz:=false：RViz 归 dz-rviz.service（VNC :1），不再混在定位栈里。
#   ⑤ px4_control / teleop / px4_path 永不进自启（ARM 能力必须人在场）。
#      本脚本是【手动入口】，由人在现场执行，符合禁令；别把它挂进任何自启。
#
# 前置硬检查：dz-agent/dz-livox/dz-link 三个 unit 必须已安装
#   （按卡 DRONE-AUTOSTART-20261024 安装手册执行过）。缺了就不继续——
#   防止有人拿 v4 当 v3 用、又手动起三件套造出双栈。
#
# 用法:
#   bash start_drone.sh         复位三件套 + 启动上层栈
#   bash start_drone.sh stop    只停上层栈（三件套不停，仍归 systemd 常保）
#
# sudo：仅开头接管三件套需要（systemctl 操作系统级 unit）。
#   脚本内【不含】任何密码，sudo -v 现场交互输入。
# ==============================================================================
set -u

WS=/home/duzhong/dzpx4
LOG_DIR=/home/duzhong/drone_logs
UNITS="dz-agent dz-livox dz-link"
mkdir -p "$LOG_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)

# 上层栈（本脚本私有）进程模式 —— 绝不包含三件套模式
# （MicroXRCEAgent|orin_agent_watchdog|livox_ros_driver2|dzlink_supervisor 严禁进此列表）
UPPER_PAT='velodyne_localization\.launch\.py|fastlio_mapping|px4_odom_convert|sensor_combined_to_imu|px4_control\.launch\.py|px4_control_node|px4_teleop_node|path_record_node|path_follow_node'

stop_upper() {
    echo "== 停止上层栈（FAST-LIO/odom/imu/control/path/teleop）=="
    pkill -f "$UPPER_PAT" 2>/dev/null
    sleep 2
    if pgrep -f "$UPPER_PAT" >/dev/null 2>&1; then
        echo "   残留进程（再过一遍 SIGKILL）："
        pgrep -af "$UPPER_PAT" | head -10
        pkill -9 -f "$UPPER_PAT" 2>/dev/null
        sleep 1
    fi
    pgrep -af "$UPPER_PAT" | head -10 || echo "   （上层已干净）"
}

if [ "${1:-start}" = "stop" ]; then
    stop_upper
    echo "== 三件套状态（归 systemd 管，本脚本不停；若 inactive 请 systemctl start 找回）=="
    systemctl --no-pager --plain is-active $UNITS 2>/dev/null || true
    exit 0
fi

# ---- 0/6 前置：三件套必须已安装（防 v3 式手动三件套双栈）----
for u in $UNITS; do
    if ! systemctl cat "$u.service" >/dev/null 2>&1; then
        echo "!! 未安装 $u.service —— 先按卡 DRONE-AUTOSTART-20261024 安装手册装好三件套"
        echo "   （v4 不再自己起 Agent/雷达/链路；缺 systemd 三件套无法继续）"
        exit 1
    fi
done

# ---- 0/6 接管三件套（systemd 单写者；无 pkill）----
echo "== 0/6 三件套接管：systemctl 复位 dz-agent/dz-livox/dz-link =="
echo "   （需要 sudo：systemctl 操作系统级 unit；密码不落盘，现场输入）"
sudo -v || { echo "!! sudo 认证失败，退出"; exit 1; }
sudo systemctl stop $UNITS
sleep 1
sudo systemctl start $UNITS
sleep 2
echo "   三件套状态："
systemctl --no-pager --plain is-active $UNITS || true

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# ---- 1/6 等雷达点云（≤25s；超时才警告继续，不阻断）----
echo "== 1/6 等 Livox 点云就绪（≤25s）=="
if timeout 25 ros2 topic echo /livox/lidar --once >/dev/null 2>&1; then
    echo "   点云 OK"
else
    echo "!! 警告: 25s 内未见 /livox/lidar（雷达航插/供电？），继续启动但 FAST-LIO 输出可能无效"
fi

# ---- 2/6 FAST-LIO（rviz:=false；RViz 归 dz-rviz.service）----
echo "== 2/6 FAST-LIO 定位（rviz:=false）=="
nohup ros2 launch fast_lio_localization velodyne_localization.launch.py rviz:=false \
    > "$LOG_DIR/${STAMP}_fastlio.log" 2>&1 &
sleep 5

# ---- 3/6 视觉里程计转换（mode:=odom；默认 localization 订阅死话题，禁用）----
echo "== 3/6 视觉里程计转换（mode:=odom）=="
nohup ros2 launch px4_odom_convert px4_odom_convert.launch.py mode:=odom \
    > "$LOG_DIR/${STAMP}_odom.log" 2>&1 &
sleep 1

# ---- 4/6 IMU 桥 ----
echo "== 4/6 IMU 桥 =="
nohup ros2 launch px4_imu_bridge sensor_combined_to_imu.launch.py \
    > "$LOG_DIR/${STAMP}_imu.log" 2>&1 &
sleep 1

# ---- 5/6 控制节点（人在场手动入口；ARM 前仍会等 teleop 按键）----
echo "== 5/6 控制节点 px4_control =="
nohup ros2 launch px4_control px4_control.launch.py \
    > "$LOG_DIR/${STAMP}_control.log" 2>&1 &
sleep 1

# ---- 6/6 路径录制 + 键盘控制（两个独立终端；teleop 必须真 TTY，不能 setsid 后台）----
echo "== 6/6 路径录制 + 键盘控制 =="
REC_CMD="source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; trap 'echo; echo === Ctrl+C 已触发，等待保存完成 ===' INT; ros2 launch px4_path px4_path.launch.py 2>&1 | tee -a $LOG_DIR/${STAMP}_path.log; echo; echo '=== 录制进程已结束（保存结果见上方输出） ==='; exec bash -i"
KEY_CMD="source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; trap 'echo' INT; ros2 run px4_control px4_teleop_node --ros-args --params-file $WS/install/px4_control/share/px4_control/config/params.yaml; echo; echo '=== 键盘控制已退出 ==='; exec bash -i"

export DISPLAY=${DISPLAY:-:0}
if command -v gnome-terminal >/dev/null 2>&1; then
    setsid nohup gnome-terminal --title "路径录制(Ctrl+C保存)" -- bash -c "$REC_CMD" >/dev/null 2>&1 &
    sleep 2
    setsid nohup gnome-terminal --title "键盘控制" -- bash -c "$KEY_CMD" >/dev/null 2>&1 &
    echo "== 已弹出两个终端: [路径录制] 和 [键盘控制] =="
elif command -v xterm >/dev/null 2>&1; then
    setsid nohup xterm -T "路径录制(Ctrl+C保存)" -e bash -c "$REC_CMD" >/dev/null 2>&1 &
    sleep 2
    setsid nohup xterm -T "键盘控制" -e bash -c "$KEY_CMD" >/dev/null 2>&1 &
    echo "== 已弹出两个终端(xterm): [路径录制] 和 [键盘控制] =="
else
    echo "!! 未找到终端模拟器，退回旧模式: 录制在后台、键盘在本终端前台"
    nohup ros2 launch px4_path px4_path.launch.py > "$LOG_DIR/${STAMP}_path.log" 2>&1 &
    sleep 1
    exec ros2 run px4_control px4_teleop_node --ros-args \
        --params-file "$WS/install/px4_control/share/px4_control/config/params.yaml"
fi

echo ""
echo "== v4 上层栈启动完毕 =="
echo "   三件套: systemctl status dz-agent dz-livox dz-link"
echo "   验证:   ros2 topic hz /fmu/in/vehicle_visual_odometry   # 期望 ~42Hz"
echo "   录制结束请在录制终端按 Ctrl+C（自动补末点保存）；本终端可关闭"
