#!/usr/bin/env bash
# 车端深度探测（第二轮）：第一轮发现车端「ROS_DISTRO 未设置 / 未找到 ros2」，
# 需要区分「真的没装 ROS」还是「非交互 shell 没加载 /opt/ros/*/setup.bash」。
# 顺便把用户名下的既有代码、串口归属、NPU 能力摸清。
#
# 全部只读。用法见 tools/remote_run.py。

hr() { echo "----------------------------------------------------"; }

echo "==================== 车端深度探测 ===================="
date
echo

hr; echo "## 1. ROS 到底装没装（绕开 PATH）"
ls -d /opt/ros/* 2>/dev/null || echo "  /opt/ros 不存在 => 车端确实没装 ROS"
for d in /opt/ros/*; do
  [ -d "$d" ] || continue
  echo "  -- $d --"
  ls "$d" 2>/dev/null | head -12
  [ -f "$d/setup.bash" ] && echo "     setup.bash: 有" || echo "     setup.bash: 无"
done
echo "-- 直接找可执行文件 --"
find /opt/ros /usr/bin /usr/local/bin -maxdepth 3 -name "ros2" -o -maxdepth 3 -name "roslaunch" 2>/dev/null | head
echo "-- 包管理器痕迹 --"
dpkg -l 2>/dev/null | grep -icE "ros-|ros2" | sed 's/^/  dpkg 里 ros 相关包数: /'
ls /etc/apt/sources.list.d/ 2>/dev/null | grep -i ros || echo "  无 ROS 源"
echo

hr; echo "## 2. 登录 shell 里的环境（对比上面，判断是不是 PATH 问题）"
bash -lc 'echo "ROS_DISTRO=${ROS_DISTRO:-<空>}"; echo "ros2=$(command -v ros2 || echo 无)"; echo "PYTHONPATH=$PYTHONPATH"' 2>&1 | head -5
grep -nE "ros|setup.bash" ~/.bashrc 2>/dev/null | head -10 || echo "  ~/.bashrc 里没有 ros 相关"
echo

hr; echo "## 3. 家目录里已有的东西（车端既有代码/脚本）"
ls -la ~ | head -40
echo "-- 递归找 launch / setup / 协议脚本 --"
find ~ -maxdepth 3 \( -name "*.launch" -o -name "*.launch.py" -o -name "*.py" -o -name "*.sh" -o -name "package.xml" \) \
  -not -path "*/.cache/*" -not -path "*/.local/lib/*" 2>/dev/null | head -40
echo

hr; echo "## 4. 串口归属（两个 ACM 口分别是谁）"
for d in /dev/ttyACM* /dev/ttyUSB*; do
  [ -e "$d" ] || continue
  echo "-- $d --"
  udevadm info -q property -n "$d" 2>/dev/null | grep -E "ID_VENDOR|ID_MODEL|ID_SERIAL|ID_PATH=" | head -6
done
echo "-- by-id 软链 --"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "  无 by-id"
echo "-- 谁在占用 --"
command -v lsof >/dev/null && lsof /dev/ttyACM* 2>/dev/null | head -10 || echo "  lsof 不可用"
command -v fuser >/dev/null && fuser -v /dev/ttyACM0 /dev/ttyACM1 2>&1 | head -10 || true
echo

hr; echo "## 5. NPU / RKNN 能力（Orange Pi 5 应为 RK3588S）"
grep -iE "model|hardware" /proc/cpuinfo | head -3
ls -l /dev/rknpu* /dev/mpp_service /dev/dri 2>/dev/null | head -10 || echo "  无 rknpu 设备节点"
ls -l /usr/lib/librknnrt.so /usr/lib/aarch64-linux-gnu/librknnrt.so 2>/dev/null
python3 -c "from rknnlite.api import RKNNLite; print('rknnlite: 可用')" 2>&1 | tail -1
pip3 list 2>/dev/null | grep -iE "rknn|numpy|opencv|torch|onnx|ultralytics" || echo "  pip3 无相关包"
command -v ffmpeg && ffmpeg -hide_banner -version 2>/dev/null | head -1
echo

hr; echo "## 6. 相机能力（AR0234）"
command -v v4l2-ctl >/dev/null && { echo "-- video0 支持格式 --"; v4l2-ctl -d /dev/video0 --list-formats-ext 2>/dev/null | head -30; }
echo "-- 尝试抓一帧（5 秒超时，只读不写盘） --"
timeout 12 ffmpeg -hide_banner -loglevel error -f v4l2 -i /dev/video0 -frames:v 1 -f null - 2>&1 | head -5 && echo "  抓帧结束（无输出=成功）"
echo

hr; echo "## 7. 正在运行的服务与进程（有没有官方程序在跑）"
systemctl list-units --type=service --state=running 2>/dev/null | grep -iE "ros|dz|judge|chassis|vnc|robot" || echo "  无相关服务"
ps -eo pid,comm,args --sort=-%cpu 2>/dev/null | head -15
echo

hr; echo "## 8. Python 与磁盘"
python3 --version; pip3 --version 2>/dev/null | head -1
df -h / /home 2>/dev/null | head -5
echo

echo "==================== 探测结束 ===================="
