#!/usr/bin/env bash
# 车端环境探测（在 SmartMaster 的 OrangePi / 机载电脑上跑）
#
# 用法 A（我 ssh 过去跑）：
#     ssh user@<车IP> 'bash -s' < tools/probe_car.sh > probe_report.txt
#
# 用法 B（你在车上 Vscode 终端里跑，把输出贴回来）：
#     bash tools/probe_car.sh 2>&1 | tee probe_report.txt
#
# 全部是只读操作，不会让车动、不会改任何配置。

echo "==================== 车端环境探测 ===================="
date
echo

hr() { echo "----------------------------------------------------"; }

hr; echo "## 1. 系统与 ROS"
echo "host: $(hostname)"
[ -f /etc/os-release ] && grep -E "^(NAME|VERSION)=" /etc/os-release
echo "arch: $(uname -m)"
echo "ROS_DISTRO=${ROS_DISTRO:-<未设置>}"
timeout 10 ros2 --version 2>/dev/null || echo "ros2: 未找到"
timeout 10 which roslaunch 2>/dev/null && echo "（存在 roslaunch => 可能是 ROS1 环境）"
echo

hr; echo "## 2. 关键 ROS 包（决定我怎么对接）"
if command -v ros2 >/dev/null 2>&1; then
  timeout 30 ros2 pkg list 2>/dev/null > /tmp/_pkgs.txt
  echo "-- 导航/定位 --";     grep -iE "nav2|amcl|slam|robot_localization" /tmp/_pkgs.txt || echo "  无"
  echo "-- 渡众官方包 --";    grep -iE "dz|judgment|actuator|smartmaster"  /tmp/_pkgs.txt || echo "  无"
  echo "-- 相机 --";          grep -iE "camera|usb_cam|v4l2|image"        /tmp/_pkgs.txt || echo "  无"
  echo "-- 串口 --";          grep -iE "serial"                            /tmp/_pkgs.txt || echo "  无"
  echo "-- 底盘 --";          grep -iE "chassis|base|driver|can|motor"     /tmp/_pkgs.txt || echo "  无"
  echo "包总数: $(wc -l < /tmp/_pkgs.txt)"
else
  echo "ros2 不可用，跳过"
fi
echo

hr; echo "## 3. 当前活跃话题（车静止时也可能为空，最好先启动官方 launch 再跑一次）"
timeout 20 ros2 topic list 2>/dev/null | head -60 || echo "  无/超时"
echo
echo "-- 与本项目相关的候选话题 --"
timeout 20 ros2 topic list 2>/dev/null | grep -iE "image|camera|goal_pose|cmd_vel|odom|amcl|judge|referee|target|ammo|scan|imu" || echo "  无"
echo

hr; echo "## 4. 串口设备（裁判系统 / 底盘）"
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyS* 2>/dev/null || echo "  未发现串口设备"
echo "-- usb 设备 --"
lsusb 2>/dev/null | head -20 || echo "  lsusb 不可用"
echo

hr; echo "## 5. 相机"
command -v v4l2-ctl >/dev/null 2>&1 && timeout 10 v4l2-ctl --list-devices 2>/dev/null || echo "  v4l2-ctl 不可用"
ls -l /dev/video* 2>/dev/null || echo "  未发现 /dev/video*"
echo

hr; echo "## 6. NPU / RKNN（板端推理能力）"
python3 -c "from rknnlite.api import RKNNLite; print('rknnlite: 可用')" 2>/dev/null || echo "rknnlite: 不可用"
ls /usr/lib/librknnrt.so /usr/lib/aarch64-linux-gnu/librknnrt.so 2>/dev/null || echo "  librknnrt 未找到"
ls -l /dev/rknpu* 2>/dev/null || echo "  /dev/rknpu* 未找到"
echo

hr; echo "## 7. 已有模型文件"
find /home /root /opt -maxdepth 4 \( -name "*.rknn" -o -name "*.onnx" \) 2>/dev/null | head -20 || true
echo

hr; echo "## 8. Python 环境"
echo "python3: $(python3 --version 2>&1)"
python3 -c "import numpy; print('numpy', numpy.__version__)" 2>/dev/null || echo "numpy: 无"
python3 -c "import serial; print('pyserial: 可用')" 2>/dev/null || echo "pyserial: 无"
echo

hr; echo "## 9. 网络"
ip -4 addr show 2>/dev/null | grep -E "inet |^[0-9]" | head -20
echo

hr; echo "## 10. 已有的工作空间"
ls -d ~/catkin_ws ~/ros2_ws ~/dzpx4 ~/ws 2>/dev/null || echo "  未发现常见工作空间"
for d in ~/catkin_ws/src ~/ros2_ws/src ~/dzpx4/src; do
  [ -d "$d" ] && echo "-- $d --" && ls "$d" | head -20
done
echo

echo "==================== 探测结束 ===================="
echo "把本文件完整贴回给我即可。"
