#!/usr/bin/env bash
# 飞控相关代码/环境侦察（严格只读）
export LC_ALL=C
echo "########## [0] 主机 ##########"
echo "host: $(hostname)  date: $(date '+%F %T %Z')  uptime: $(uptime -p)"

echo
echo "########## [1] 飞控硬件是否在线 ##########"
echo "--- /dev/ttyACM* /dev/ttyUSB* ---"
ls -l /dev/ttyACM* /dev/ttyUSB* 2>&1 | sed 's/^/   /'
echo "--- lsusb ---"
lsusb | sed 's/^/   /'
echo "--- dmesg 中的 ttyACM 记录（最近 8 条） ---"
dmesg 2>/dev/null | grep -i ttyacm | tail -8 | sed 's/^/   /' || echo "   (无)"
echo "--- PX4/uXRCE/mavros 进程 ---"
ps -ef | grep -iE 'xrce|mavros|px4|MicroXRCE' | grep -v grep | sed 's/^/   /' || echo "   (无)"

echo
echo "########## [2] shell 历史中的运行方式 ##########"
echo "--- 历史里出现过的 ros2 / PX4 / 串口 命令（去重，最近 80 条） ---"
grep -aE 'ros2|MicroXRCE|px4|PX4|ttyACM|mavlink|param' "$HOME/.bash_history" 2>/dev/null | tail -80 | sed 's/^/   /'

echo
echo "########## [3] 工作空间包清单（源码树体积/文件数/最近改动） ##########"
for p in "$HOME"/dzpx4/src/*/; do
  n=$(basename "$p")
  case "$n" in log|maps) continue;; esac
  sz=$(du -sh "$p" 2>/dev/null | cut -f1)
  nf=$(find "$p" -type f 2>/dev/null | grep -vcE '/build/|/\.git/')
  mt=$(find "$p" -type f -printf '%TY-%Tm-%Td %TH:%TM\n' 2>/dev/null | sort | tail -1)
  printf "   %-32s %6s  files=%-5s last=%s\n" "$n" "$sz" "$nf" "$mt"
done

echo
echo "########## [4] build/install 状态（是否编译过、何时） ##########"
echo "--- install 下已安装的包 ---"
ls -1 "$HOME/dzpx4/install" 2>/dev/null | sed 's/^/   /'
echo "--- 各包二进制/可执行 ---"
for d in "$HOME"/dzpx4/install/*/lib/*/; do
  [ -d "$d" ] || continue
  ex=$(find "$d" -maxdepth 1 -type f -executable 2>/dev/null | head -5 | xargs -r -n1 basename | tr '\n' ' ')
  [ -n "$ex" ] && echo "   $(basename "$(dirname "$d")")/$(basename "$d"): $ex"
done
echo "--- 关键 install 目录时间戳 ---"
for d in px4_control px4_path px4_odom_convert px4_imu_bridge px4_msgs fast_lio_localization livox_ros_driver2; do
  [ -d "$HOME/dzpx4/install/$d" ] && echo "   $d -> $(stat -c %y "$HOME/dzpx4/install/$d" | cut -d. -f1)"
done

echo
echo "########## [5] 最近一次编译日志里的错误/警告 ##########"
L=$(ls -dt "$HOME"/dzpx4/log/build_* 2>/dev/null | head -1)
echo "   最新构建日志目录: $L"
if [ -n "$L" ] && [ -f "$L/logger_all.log" ]; then
  echo "   --- 失败摘要 ---"
  grep -aE 'Failed|failed|error:|Error' "$L/logger_all.log" | tail -15 | sed 's/^/     /' || echo "     (无)"
  echo "   --- 摘要尾部 ---"
  tail -12 "$L/logger_all.log" | sed 's/^/     /'
fi

echo
echo "########## [6] 雷达/定位/地图数据 ##########"
echo "--- FAST_LIO_LOCALIZATION_ROS2 体积构成 ---"
du -sh "$HOME"/dzpx4/src/FAST_LIO_LOCALIZATION_ROS2/* 2>/dev/null | sed 's/^/   /'
echo "--- maps 目录 ---"
ls -la "$HOME/dzpx4/src/maps" 2>/dev/null | sed 's/^/   /'
echo "--- 是否有 PX4 参数文件 / 标定文件 ---"
find "$HOME" -maxdepth 3 -type f \( -name '*.params' -o -name '*param*' -o -name '*.calib' \) 2>/dev/null | grep -v dzpx4/build | head -20 | sed 's/^/   /'

echo
echo "########## [7] 飞控代码涉及的话题名（源码里的字符串常量） ##########"
grep -rhoE '"/(fmu|px4|livox|Odometry|cloud|path|cmd|control)[A-Za-z0-9_/]*"' \
  "$HOME/dzpx4/src/px4_control" "$HOME/dzpx4/src/px4_path" \
  "$HOME/dzpx4/src/px4_odom_convert" "$HOME/dzpx4/src/px4_imu_bridge" 2>/dev/null \
  | sort | uniq -c | sort -rn | sed 's/^/   /'

echo
echo "########## 侦察结束 ##########"
