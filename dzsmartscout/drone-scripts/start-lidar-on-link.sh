#!/usr/bin/env bash
# start-lidar-on-link.sh - 等待雷达链路(eno1 carrier=1)后自动启动 MID360 驱动
# 用法: nohup bash start-lidar-on-link.sh > /tmp/lidar_watch.log 2>&1 &
LOG=/tmp/livox_launch.log

echo "== lidar-watch $(date '+%F %T') ==" | tee -a /tmp/lidar_watch.log
echo "等待 eno1 链路恢复（雷达航插插好并上电后 carrier 会变 1）..." | tee -a /tmp/lidar_watch.log

# 轮询 carrier，最多等 30 分钟
for i in $(seq 1 180); do
  CARRIER=$(cat /sys/class/net/eno1/carrier 2>/dev/null)
  if [ "$CARRIER" = "1" ]; then
    echo "$(date '+%T') eno1 链路已恢复，拉起网口并配置 192.168.3.50" | tee -a /tmp/lidar_watch.log
    echo duzhong | sudo -S ip link set eno1 up 2>>/tmp/lidar_watch.log
    sleep 3
    IP=$(ip -4 -o addr show eno1 | awk '{print $4}' | cut -d/ -f1)
    echo "eno1 IP=$IP" | tee -a /tmp/lidar_watch.log
    ping -c 2 -W 2 192.168.3.105 >>/tmp/lidar_watch.log 2>&1
    tail -n 2 /tmp/lidar_watch.log
    # 启动驱动
    if pgrep -f "livox_ros_driver2" >/dev/null; then
      echo "已有驱动实例运行，不重复启动" | tee -a /tmp/lidar_watch.log
    else
      source /opt/ros/humble/setup.bash
      source "$HOME/dzpx4/install/setup.bash"
      echo "$(date '+%T') 启动 livox 驱动" | tee -a /tmp/lidar_watch.log
      nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py >>"$LOG" 2>&1 &
      echo "驱动已后台启动 PID=$!" | tee -a /tmp/lidar_watch.log
      sleep 20
      echo "--- 驱动日志尾部 ---" | tee -a /tmp/lidar_watch.log
      tail -n 15 "$LOG" | tee -a /tmp/lidar_watch.log
    fi
    exit 0
  fi
  sleep 10
done

echo "$(date '+%T') 30 分钟内链路未恢复，退出。请检查雷达航插与供电。" | tee -a /tmp/lidar_watch.log
