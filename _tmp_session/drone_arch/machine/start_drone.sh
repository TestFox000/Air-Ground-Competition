#!/usr/bin/env bash
# 无人机全系统一键启动脚本（依据 dzpx4/1.txt）
#
# 用法:
#   ./start_drone.sh          启动全部节点，并弹出两个独立终端:
#                                [路径录制]  可见 "path recording START"，
#                                            Ctrl+C 结束自动保存路径
#                                [键盘控制]  WASDQE 等按键操作
#   ./start_drone.sh stop     停止全部无人机节点

WS=/home/duzhong/dzpx4
AGENT=/home/duzhong/Micro-XRCE-DDS-Agent-2.4.3/build-agent/MicroXRCEAgent
SERIAL_DEV=/dev/ttyACM0
BAUD=921600
LOG_DIR=/home/duzhong/drone_logs

mkdir -p "$LOG_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

stop_all() {
    echo "== 停止所有无人机节点 =="
    # 必须先停看门狗，否则它会在 Agent 被杀后 15s 内自动重启一个抢串口
    pkill -f orin_agent_watchdog
    pkill -f livox_ros_driver2_node
    pkill -f velodyne_localization.launch.py
    pkill -f fastlio_mapping
    pkill -f transform_fusion
    pkill -f global_localization
    pkill -f global_map_publisher
    pkill -f MicroXRCEAgent
    pkill -f px4_odom_convert
    pkill -f sensor_combined_to_imu
    pkill -f px4_control.launch.py
    pkill -f px4_control_node
    pkill -f px4_teleop_node
    pkill -f path_record_node
    pkill -f path_follow_node
    pkill -f rviz2
    sleep 2
    echo "== 已停止 =="
}

if [ "$1" = "stop" ]; then
    stop_all
    exit 0
fi

# 启动前先清理残留节点，避免重名冲突
stop_all

if [ ! -e "$SERIAL_DEV" ]; then
    echo "!! 警告: 未找到 $SERIAL_DEV，请检查飞控连接后重试"
fi

echo "== 1/7 激光雷达驱动 =="
nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
    > "$LOG_DIR/${STAMP}_livox.log" 2>&1 &
sleep 3

echo "== 2/7 FAST-LIO 定位（含 rviz）=="
nohup ros2 launch fast_lio_localization velodyne_localization.launch.py \
    > "$LOG_DIR/${STAMP}_fastlio.log" 2>&1 &
sleep 5

echo "== 3/7 DDS 桥接（飞控）=="
nohup "$AGENT" serial -D "$SERIAL_DEV" -b "$BAUD" \
    > "$LOG_DIR/${STAMP}_agent.log" 2>&1 &
sleep 2

echo "== 4/7 视觉里程计转换 =="
nohup ros2 launch px4_odom_convert px4_odom_convert.launch.py \
    > "$LOG_DIR/${STAMP}_odom.log" 2>&1 &
sleep 1

echo "== 5/7 IMU 桥 =="
nohup ros2 launch px4_imu_bridge sensor_combined_to_imu.launch.py \
    > "$LOG_DIR/${STAMP}_imu.log" 2>&1 &
sleep 1

echo "== 6/7 控制节点 =="
nohup ros2 launch px4_control px4_control.launch.py \
    > "$LOG_DIR/${STAMP}_control.log" 2>&1 &
sleep 1

echo "== 7/7 路径录制 + 键盘控制（两个独立终端）=="

REC_CMD="source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; trap 'echo; echo === Ctrl+C 已触发，等待保存完成 ===' INT; ros2 launch px4_path px4_path.launch.py 2>&1 | tee -a $LOG_DIR/${STAMP}_path.log; echo; echo '=== 录制进程已结束（保存结果见上方输出） ==='; exec bash -i"
KEY_CMD="source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; trap 'echo' INT; ros2 run px4_control px4_teleop_node --ros-args --params-file $WS/install/px4_control/share/px4_control/config/params.yaml; echo; echo '=== 键盘控制已退出 ==='; exec bash -i"

export DISPLAY=${DISPLAY:-:0}
if command -v gnome-terminal >/dev/null 2>&1; then
    setsid nohup gnome-terminal --title "路径录制(Ctrl+C保存)" -- bash -c "$REC_CMD" >/dev/null 2>&1 &
    sleep 2
    setsid nohup gnome-terminal --title "键盘控制" -- bash -c "$KEY_CMD" >/dev/null 2>&1 &
    echo ""
    echo "== 已弹出两个终端: [路径录制] 和 [键盘控制] =="
    echo "== 录制结束请在录制终端按 Ctrl+C，自动补末点保存 =="
    echo "== 本终端可以关闭，不影响运行 =="
elif command -v xterm >/dev/null 2>&1; then
    setsid nohup xterm -T "路径录制(Ctrl+C保存)" -e bash -c "$REC_CMD" >/dev/null 2>&1 &
    sleep 2
    setsid nohup xterm -T "键盘控制" -e bash -c "$KEY_CMD" >/dev/null 2>&1 &
    echo ""
    echo "== 已弹出两个终端: [路径录制] 和 [键盘控制] =="
    echo "== 录制结束请在录制终端按 Ctrl+C，自动补末点保存 =="
    echo "== 本终端可以关闭，不影响运行 =="
else
    echo "!! 未找到终端模拟器，退回旧模式: 录制在后台、键盘在本终端前台"
    nohup ros2 launch px4_path px4_path.launch.py \
        > "$LOG_DIR/${STAMP}_path.log" 2>&1 &
    sleep 1
    exec ros2 run px4_control px4_teleop_node --ros-args \
        --params-file "$WS/install/px4_control/share/px4_control/config/params.yaml"
fi
