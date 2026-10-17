#!/usr/bin/env bash
# 小车侧只读侦察（由本地 scp 上传后执行）
set +e
echo "--- dzbrain 相关进程 ---"
pgrep -af "dzbrain|path_executor|mission" 2>/dev/null | head -20
echo "--- ~/dzrobot 目录 ---"
ls -la ~/dzrobot/ 2>/dev/null | head -30
echo "--- robot.yaml 关键键 ---"
grep -nE "auto_start|auto_plan_enabled|engine:" ~/dzrobot/config/robot.yaml 2>/dev/null | head -30
echo "--- 当前 ROS_DOMAIN_ID 相关 ---"
cat ~/dzrobot/dzros.env 2>/dev/null
echo "--- ros2 节点（域118） ---"
source /opt/ros/humble/setup.bash 2>/dev/null
ROS_DOMAIN_ID=118 timeout 8 ros2 node list 2>&1 | head -20
echo "--- 结束 ---"
