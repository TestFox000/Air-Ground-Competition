#!/bin/bash
# 重启云台瞄准节点（只重启 dzrknn_dataprocess，不碰底盘/循迹/裁判）
set +u
cd /home/duzhong/dzrobot || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source ./dzros.env
export DISPLAY=:1
mkdir -p /home/duzhong/robot_logs
LOG=/home/duzhong/robot_logs/5_gimbal.log

echo "== 停旧进程 =="
pkill -f 'dzrknn_dataprocess.launch.py' 2>/dev/null
sleep 1.5
pkill -f 'dzrknn_dataprocess/lib/dzrknn_dataprocess/dzactuator_node' 2>/dev/null
sleep 1.5
n=$(ps -ef | grep -c '[d]zrknn')
echo "残留进程数: $n"
if [ "$n" != "0" ]; then
  echo "!! 仍有残留，强制 kill"; ps -ef | grep '[d]zrknn' | awk '{print $2}' | xargs -r kill -9; sleep 1
fi

echo "== 启动 =="
setsid nohup ros2 launch dzrknn_dataprocess dzrknn_dataprocess.launch.py >> "$LOG" 2>&1 < /dev/null &
sleep 8
echo "== 进程 =="
ps -ef | grep '[d]zrknn' | awk '{print $2, $8, $9}' | head -5
echo "== 横幅（本次启动）=="
tail -60 "$LOG" | grep -E '跟踪控制|自瞄: 状态机|创建节点成功' | tail -5
echo done
