#!/usr/bin/env bash
# 补齐官方栈缺失的依赖包（**只编缺的那几个**，不动整仓、不改源码）
#
# 为什么需要（2026-09-28 实地诊断）：
#   `dzactuator bringup.launch.py` 起不来：PackageNotFoundError: package 'robot_localization' not found
#   → 底盘节点全没起来 → 没有 /imu_data → 定位节点收不到 IMU → /odom/localization 永远无数据
#   → 脚本每轮都报"读不到定位"，真发车因此被硬门禁拦住。
#   根因还是官方 `install/` 残缺（9-28 白天修的是 det_node/dzactuator，这次轮到 robot_localization）。
#
# 做法：从 `~/dzrobot/src` 里找出 bringup 引用但**没装**的包，逐个 `--packages-select` 编。
# 环境变量：AG_DRY_RUN=1  只诊断不编译
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
DRY="${AG_DRY_RUN:-0}"
WS="$HOME/dzrobot"
hr() { echo "----------------------------------------------------"; }

echo "==================== 补齐官方栈依赖 ===================="
date
echo

hr; echo "## 1. bringup 需要哪些包、哪些没装"
NEED="dzactuator robot_localization rplidar_ros tf2_ros"
MISSING=""
for p in $NEED; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then
    printf "  [OK]   %-24s %s\n" "$p" "$(ros2 pkg prefix "$p" 2>/dev/null)"
  else
    printf "  [缺]   %-24s\n" "$p"
    case "$p" in
      dzactuator|robot_localization|rplidar_ros) MISSING="$MISSING $p" ;;
      *) echo "         （$p 属于 ROS 基础，缺了要装 ros-humble 依赖，不是本脚本能修的）" ;;
    esac
  fi
done
echo

hr; echo "## 2. 底盘串口设备（launch 默认 /dev/dzactuator）"
for d in /dev/dzactuator /dev/ttyACM0 /dev/ttyACM1; do
  if [ -e "$d" ]; then
    printf "  [OK]   %-20s -> %s\n" "$d" "$(readlink -f "$d" 2>/dev/null)"
  else
    echo "  [缺]   $d"
  fi
done
echo

hr; echo "## 3. 源码里有没有这些包"
for p in $MISSING; do
  found="$(find "$WS/src" -maxdepth 3 -name package.xml -exec grep -l "<name>$p</name>" {} + 2>/dev/null | head -1)"
  if [ -n "$found" ]; then
    echo "  [有] $p  <- ${found%/package.xml}"
  else
    echo "  [无] $p  在 $WS/src 下没找到源码（可能需要从厂家拿）"
  fi
done
echo

if [ -z "${MISSING// /}" ]; then
  hr; echo "## 结论：什么都不缺，不用编"
  exit 0
fi

hr; echo "## 4. 编译缺失的包：$MISSING"
if [ "$DRY" = "1" ]; then
  echo "  （AG_DRY_RUN=1，只诊断不编译）"
  exit 0
fi
# 只在正主那份源码里编（--base-paths src），避免重复包名把 colcon 顶掉
LOG=/tmp/ag_fixdeps.log
cd "$WS" || exit 1
# shellcheck disable=SC2086
if [ "${AG_BACKGROUND:-0}" = "1" ]; then
  # 后台编译：这条链路（车端 Wi-Fi 只有 1~5 Mbps）下前台等编译太磨人，
  # 而且连接一抖就被打断、编译也跟着断。放后台后命令**立刻返回**，
  # 之后用 AG_STATUS=1 去查日志就行。
  setsid nohup bash -c "colcon build --base-paths src --packages-select $MISSING \
    --cmake-args -DCMAKE_BUILD_TYPE=Release > $LOG 2>&1; echo \$? > ${LOG}.rc" \
    > /dev/null 2>&1 &
  sleep 2
  echo "  已在车端后台开始编译，日志 $LOG（退出码写到 ${LOG}.rc）"
  echo "  查进度：python tools/remote_run.py --host <IP> --user duzhong --script tools/car_fix_official_deps.sh --env AG_STATUS=1"
  exit 0
fi
if [ "${AG_STATUS:-0}" = "1" ]; then
  echo "  -- 编译状态 --"
  if [ -f "${LOG}.rc" ]; then
    echo "  已结束，退出码 = $(cat "${LOG}.rc")"
  else
    echo "  仍在编译（还没写退出码）"
  fi
  tail -8 "$LOG" 2>/dev/null | sed 's/^/    /'
  exit 0
fi
colcon build --base-paths src --packages-select $MISSING \
  --cmake-args -DCMAKE_BUILD_TYPE=Release > "$LOG" 2>&1
RC=$?
echo "  colcon 退出码 = $RC   （完整日志 $LOG）"
tail -12 "$LOG" | sed 's/^/    /'
echo

if [ "$RC" != "0" ]; then
  hr; echo "## 编译失败 —— 常见原因与下一步"
  echo "  * 缺系统依赖（最常见）：看日志里的 'Could not find a package configuration file'，"
  echo "    然后用 rosdep 装：sudo rosdep install --from-paths src --ignore-src -y -r"
  echo "  * 重复包名：确认 ~/dzrobot 下只有 src 一份源码（src_1/src_4 已进回收站就没事）"
  exit 1
fi

hr; echo "## 5. 复验：这几个包现在能不能找到"
for p in $MISSING; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then
    printf "  [OK]   %-24s %s\n" "$p" "$(ros2 pkg prefix "$p" 2>/dev/null)"
  else
    printf "  [仍缺] %-24s\n" "$p"
  fi
done
echo
echo "  下一步：python car.py preflight <车IP>   # 复验定位到底有没有数据"
echo "==================== 结束 ===================="
