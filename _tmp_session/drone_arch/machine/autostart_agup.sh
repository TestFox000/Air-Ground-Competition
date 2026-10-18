#!/usr/bin/env bash
# 开机自启 v3（2026-10-17）：只保飞控链路（MicroXRCEAgent + 看门狗）。
# 雷达/定位/桥接不再自启——建模与飞行前准备由人工命令起，起飞和路径
# 录制用 ~/start_drone.sh 起（2026-10-17 曾因自启+手动双栈导致定位发散）。
#
# 用法: autostart_agup.sh start|stop|status   （无参数=start，cron 用）
# 幂等：start 前先 stop 清残留。stop 只停飞控链路，不碰手动起的雷达/定位栈。
LOGD=/home/duzhong/drone_logs
LOG=$LOGD/autostart.log
REPO=/home/duzhong/dzuav_deploy/air_ground

PAT='MicroXRCEAgent|orin_agent_watchdog'

stop_all() {
    pkill -f "$PAT" 2>/dev/null
    sleep 2
}

start_all() {
    # 幂等：清掉可能存在的旧 Agent/看门狗实例（无论谁起的）
    stop_all
    echo "[$(date '+%F %T')] autostart v3 begin (flight-link only)" >> $LOG
    # 看门狗依赖 rclpy + px4_msgs，需 source ROS 环境
    source /opt/ros/humble/setup.bash >> $LOG 2>&1
    source /home/duzhong/dzpx4/install/setup.bash >> $LOG 2>&1
    cd $REPO

    setsid nohup MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600 \
        >> $LOGD/node_agent.log 2>&1 &
    # 看门狗：飞控晚于 Orin 上电/中途断电重连时，自动重启 Agent 重建会话
    setsid nohup python3 $REPO/tools/orin_agent_watchdog.py \
        >> $LOGD/node_wdog.log 2>&1 &
    echo "[$(date '+%F %T')] autostart v3 launched (agent/wdog only)" >> $LOG
}

case "${1:-start}" in
    start)  start_all ;;
    stop)   stop_all; echo "[$(date '+%F %T')] stopped" >> $LOG ;;
    status) pgrep -fa "$PAT" | head -12 ;;
    *) echo "usage: $0 start|stop|status" >&2; exit 2 ;;
esac
