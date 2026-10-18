#!/usr/bin/env bash
# =============================================================================
# 解决 serial 依赖：确定正确的 apt 包名并安装
# =============================================================================
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/fix_serial.sh"
#
# 背景：
#   dzmission / raw_transport / referee_board / cmd_link 都用
#       #include "serial/serial.h"  +  serial::Serial
#   而 `serial` 是 ROS 1 时代的库（catkin 构建），**没有单独的 ros-humble-serial**。
#   ROS 2 里它由 **ros-humble-serial-driver** 提供（名字不同，容易找错）。
#
# 本脚本会：
#   1. 在 apt 源里搜索所有候选包名
#   2. 安装正确的那个
#   3. 用 cmake 真正验证 find_package(serial) 能否成功
# =============================================================================

set +e
BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %s\n" "$1"; }
bad()  { printf "  ${RED}✗${RST} %s\n" "$1"; }
warn() { printf "  ${YEL}!${RST} %s\n" "$1"; }
info() { printf "  ${CYN}·${RST} %s\n" "$1"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

source /opt/ros/humble/setup.bash 2>/dev/null

printf "${BOLD}解决 serial 依赖${RST}\n"

# ---------------------------------------------------------------- 1. 判断是否已可用
sec "1. 当前状态"

if ros2 pkg prefix serial >/dev/null 2>&1; then
  ok "serial 已经可用: $(ros2 pkg prefix serial)"
  DONE=1
elif [ -f /opt/ros/humble/include/serial/serial.h ]; then
  ok "找到 /opt/ros/humble/include/serial/serial.h（但 ros2 pkg 查不到）"
  DONE=1
else
  warn "serial 尚未安装"
  DONE=0
fi

if [ "$DONE" = "1" ]; then
  sec "2. 用 cmake 验证 find_package(serial) 能否成功"
  T=$(mktemp -d)
  mkdir -p "$T/proj"
  cat > "$T/proj/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.8)
project(ser_test)
find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
message(STATUS "SERIAL_FOUND_DIR = ${serial_DIR}")
EOF
  if cmake -S "$T/proj" -B "$T/build" >"$T/log" 2>&1; then
    ok "find_package(serial) 成功"
    grep -i 'SERIAL_FOUND_DIR' "$T/log" | sed 's/^/      /'
  else
    bad "find_package(serial) 失败 —— 见下"
    tail -15 "$T/log" | sed 's/^/      /'
  fi
  rm -rf "$T"
  printf "\n  ${GRN}${BOLD}可以继续编译了：bash build_ws.sh${RST}\n\n"
  exit 0
fi

# ---------------------------------------------------------------- 2. 搜索候选包名
sec "2. 在 apt 源里搜索候选包名"

printf "  apt-cache search 结果：\n"
apt-cache search '^ros-humble-serial' 2>/dev/null | sed 's/^/      /'
printf "\n  逐个检查候选：\n"

CANDS=(ros-humble-serial-driver ros-humble-serial ros-humble-serial-msgs)
FOUND=""
for c in "${CANDS[@]}"; do
  POL=$(apt-cache policy "$c" 2>/dev/null | awk '/Candidate:/{print $2}')
  if [ -n "$POL" ] && [ "$POL" != "(none)" ]; then
    ok "$c  候选版本 $POL"
    [ -z "$FOUND" ] && FOUND="$c"
  else
    info "$c  不可用"
  fi
done

# 兜底：模糊搜索任何含 serial 的 ROS 包
if [ -z "$FOUND" ]; then
  printf "\n  模糊搜索 'ros-humble-*serial*'：\n"
  apt-cache search 'ros-humble.*serial' 2>/dev/null | sed 's/^/      /'
fi

# ---------------------------------------------------------------- 3. 安装
sec "3. 安装"

if [ -n "$FOUND" ]; then
  printf "  将安装: %s\n" "$FOUND"
  sudo apt install -y "$FOUND"
  RC=$?
else
  bad "在 apt 源里找不到任何可提供 serial 的 ROS 2 包"
  printf "\n  ${BOLD}需要改用其它方案（见脚本末尾说明）${RST}\n"
  RC=1
fi

# ---------------------------------------------------------------- 4. 验证
sec "4. 验证"

if ros2 pkg prefix serial >/dev/null 2>&1 || [ -f /opt/ros/humble/include/serial/serial.h ]; then
  ok "serial 头文件已就位"
  ls -la /opt/ros/humble/include/serial/ 2>/dev/null | head -5 | sed 's/^/      /'
  printf "\n  ${GRN}${BOLD}成功 —— 可以继续：bash build_ws.sh${RST}\n"
else
  bad "仍未解决"
  printf "\n  ${BOLD}备选方案（按推荐顺序）${RST}\n"
  printf "  ${CYN}A) 从源码编译并 install 到系统（一次性，最贴近原设计）${RST}\n"
  printf "     git clone https://github.com/wjwwood/serial.git\n"
  printf "     cd serial && mkdir build && cd build\n"
  printf "     cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local\n"
  printf "     make -j2 && sudo make install && sudo ldconfig\n"
  printf "     # 然后让 CMake 找到它：export CMAKE_PREFIX_PATH=/usr/local:\$CMAKE_PREFIX_PATH\n"
  printf "     # 注意：该库原本是 catkin 包，需确认其 CMakeLists 支持独立构建\n"
  printf "\n  ${CYN}B) 自己写一个极简串口封装（最可控，约 200 行）${RST}\n"
  printf "     本代码只用到 9 个 API：\n"
  printf "       Serial / Timeout::simpleTimeout / setPort / setBaudrate / setTimeout\n"
  printf "       open / close / isOpen / read / write / available\n"
  printf "     用 POSIX termios 实现这些即可，然后改 4 个文件的 #include。\n"
  printf "     优点：无外部依赖、完全可控、可加错误处理\n"
fi

printf "\n"
exit $RC
