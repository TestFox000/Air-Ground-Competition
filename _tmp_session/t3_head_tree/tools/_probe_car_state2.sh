#!/usr/bin/env bash
# 只读探针 2：进程/日志/源码布局/构建环境
set -u
echo "########## 8. 相关进程 ##########"
ps -eo pid,stat,pcpu,etime,cmd --sort=-pcpu 2>/dev/null | grep -E 'dzactuator|dzjudgment|dzrknn|path_exec|car_local|rknn_pt|cam_viewer|ros2|launch' | grep -v grep | head -15 || echo "(无匹配进程)"

echo
echo "########## 9. robot_logs ##########"
ls -lt ~/robot_logs 2>/dev/null | head -14 || echo "(无 robot_logs)"

echo
echo "########## 10. ~/dzrobot 布局 ##########"
ls -la ~/dzrobot 2>/dev/null | head -25
echo "--- src 包 ---"
ls -l ~/dzrobot/src 2>/dev/null
echo "--- config ---"
ls -l ~/dzrobot/config 2>/dev/null

echo
echo "########## 11. install 已构建包 ##########"
ls ~/dzrobot/install 2>/dev/null | head -20

echo
echo "########## 12. _backups 内容 ##########"
ls -lt ~/_backups 2>/dev/null | head -15

echo
echo "########## 13. 系统/RKNN/ROS 版本 ##########"
echo "--- uname ---"; uname -a
echo "--- os ---"; head -3 /etc/os-release 2>/dev/null
echo "--- python ---"; python3 -V
echo "--- ros ---"; ls /opt/ros 2>/dev/null
echo "--- colcon ---"; which colcon 2>/dev/null && colcon version-check 2>/dev/null | head -3
echo "--- gcc/cmake ---"; gcc --version 2>/dev/null | head -1; cmake --version 2>/dev/null | head -1

echo
echo "########## 14. 硬件设备节点 ##########"
ls -l /dev/dz* /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -12 || echo "(无 dz/tty 设备)"

echo
echo "########## 15. 网络地址 ##########"
hostname -I 2>/dev/null; ip -4 addr show 2>/dev/null | grep -E 'inet ' | head -6

echo
echo "########## 16. dzrobot 源码 git 状态 ##########"
if [ -d ~/dzrobot/.git ]; then git -C ~/dzrobot log --oneline -5 2>&1 | head -8; git -C ~/dzrobot status --short 2>&1 | head -10; else echo "(~/dzrobot 不是 git 仓库)"; fi

echo
echo "########## 探针2结束（全程只读） ##########"
