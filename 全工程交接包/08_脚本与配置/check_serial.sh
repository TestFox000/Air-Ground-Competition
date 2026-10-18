#!/usr/bin/env bash
# =============================================================================
# serial 包单独编译验证（不需要 ROS，不需要硬件）
# =============================================================================
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/check_serial.sh"
#
# ★ 为什么先单独验证 serial：
#   dzmission 的 4 个文件依赖 serial::Serial，而这是我**新写的**代码，
#   从未编译过。先把它单独编过 + 跑自测，再编 dzmission，
#   能把"新代码的问题"和"dzmission 既有代码的问题"**分开定位**。
#
# 这个脚本**不需要 ROS**（serial 包只依赖 POSIX + CMake）。
# =============================================================================

set +e
BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %s\n" "$1"; }
bad()  { printf "  ${RED}✗${RST} %s\n" "$1"; }
warn() { printf "  ${YEL}!${RST} %s\n" "$1"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

WINPROJ="/mnt/c/Users/Ran/Desktop/无人车/src-car"
WORK="$HOME/serial_check"

printf "${BOLD}serial 包单独编译验证${RST}\n"

# ---------------------------------------------------------------- 0. 源文件齐否
sec "0. 源文件完整性"

declare -A FILES=(
  ["CMakeLists.txt"]="包定义"
  ["package.xml"]="包元数据"
  ["cmake/serialConfig.cmake.in"]="CMake 导出模板"
  ["include/serial/serial.h"]="头文件（接口）"
  ["src/serial.cpp"]="实现（termios）"
  ["test/serial_selftest.cpp"]="自测程序"
)
MISS=0
for f in "${!FILES[@]}"; do
  if [ -f "$WINPROJ/serial/$f" ]; then
    ok "$f  (${FILES[$f]})"
  else
    bad "$f  缺失  (${FILES[$f]})"
    MISS=$((MISS+1))
  fi
done

if [ "$MISS" -gt 0 ]; then
  bad "有 $MISS 个文件缺失，停止"
  exit 1
fi

# ---------------------------------------------------------------- 1. 拷到 Linux 侧
sec "1. 拷到 Linux 侧（避免 /mnt/c 的 IO 开销，且避免中文路径干扰 CMake）"

rm -rf "$WORK"
mkdir -p "$WORK"
cp -r "$WINPROJ/serial" "$WORK/serial"
ok "已拷到 $WORK/serial"

# ---------------------------------------------------------------- 2. 直接 g++ 编译
sec "2. 直接用 g++ 编译（先排除 CMake 变量）"

cd "$WORK/serial" || exit 1
printf "  g++ -std=c++17 -O1 -Wall -Wextra -I include \\\\\n"
printf "      src/serial.cpp test/serial_selftest.cpp -o serial_selftest -lpthread\n\n"

if g++ -std=c++17 -O1 -Wall -Wextra \
      -I include \
      src/serial.cpp test/serial_selftest.cpp \
      -o serial_selftest -lpthread 2> gpp.log; then
  ok "★ g++ 编译成功（含 -Wall -Wextra）"
  if [ -s gpp.log ]; then
    printf "  ${YEL}警告：${RST}\n"
    head -20 gpp.log | sed 's/^/      /'
  else
    ok "无编译警告"
  fi
else
  bad "★ g++ 编译失败 —— 完整错误："
  printf "\n"
  cat gpp.log
  printf "\n  ${BOLD}请把上面错误贴回给助手。${RST}\n\n"
  exit 1
fi

# ---------------------------------------------------------------- 3. 运行自测
sec "3. 运行自测（不需要硬件）"

# ★ 关键：绝不能用 `./x | tee f`，管道会吞掉退出码
./serial_selftest > selftest.log 2>&1
RC=$?
cat selftest.log
printf "\n"
if [ "$RC" = "0" ]; then
  ok "★ 自测全部通过（退出码 0）"
else
  bad "★ 自测有失败项（退出码 $RC）"
fi

# ---------------------------------------------------------------- 4. CMake 构建验证
sec "4. CMake 构建 + install 验证（验证 find_package 能否成功）"

mkdir -p build && cd build || exit 1
PREFIX="$WORK/install"

if cmake .. -DCMAKE_INSTALL_PREFIX="$PREFIX" -DCMAKE_BUILD_TYPE=Release > cmake.log 2>&1; then
  ok "cmake configure 成功"
  if cmake --build . -j1 > build.log 2>&1; then
    ok "cmake build 成功"
    if cmake --install . > install.log 2>&1; then
      ok "cmake install 成功"
    else
      bad "cmake install 失败"; tail -15 install.log | sed 's/^/      /'
    fi
  else
    bad "cmake build 失败"; tail -20 build.log | sed 's/^/      /'
  fi
else
  bad "cmake configure 失败 —— 见下"
  tail -25 cmake.log | sed 's/^/      /'
fi

printf "\n  install 产物：\n"
find "$PREFIX" -type f 2>/dev/null | sed "s|$PREFIX|  <prefix>|" | head -20 | sed 's/^/    /'

# ---------------------------------------------------------------- 5. find_package 真实测试
sec "5. ★ find_package(serial) 真实测试（这是 dzmission 依赖的关键）"

if [ ! -d "$PREFIX" ]; then
  warn "install 目录不存在，跳过"
else
  T="$WORK/consumer"
  mkdir -p "$T"
  cat > "$T/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.8)
project(consumer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

# ★ 这就是 dzmission 里的写法
find_package(serial REQUIRED)
message(STATUS ">>> serial_DIR = ${serial_DIR}")

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE serial::serial)
EOF
  cat > "$T/main.cpp" <<'EOF'
#include "serial/serial.h"
#include <cstdio>
int main() {
  serial::Serial s;
  s.setPort("/dev/null");
  s.setBaudrate(115200);
  s.setTimeout(serial::Timeout::simpleTimeout(10));
  std::printf("consumer ok, isOpen=%d, baud=%u\n",
              (int)s.isOpen(), s.getBaudrate());
  return 0;
}
EOF
  mkdir -p "$T/build"
  if cmake -S "$T" -B "$T/build" -DCMAKE_PREFIX_PATH="$PREFIX" > "$T/cfg.log" 2>&1; then
    ok "★ find_package(serial REQUIRED) 成功"
    grep '>>> serial_DIR' "$T/cfg.log" | sed 's/^/      /'
    if cmake --build "$T/build" -j1 > "$T/b.log" 2>&1; then
      ok "下游链接 serial::serial 成功"
      if "$T/build/consumer" > "$T/run.log" 2>&1; then
        ok "下游程序运行成功: $(cat "$T/run.log")"
      else
        bad "下游程序运行失败"; cat "$T/run.log" | sed 's/^/      /'
      fi
    else
      bad "下游编译/链接失败"; tail -20 "$T/b.log" | sed 's/^/      /'
    fi
  else
    bad "★ find_package(serial) 失败 —— 见下"
    tail -25 "$T/cfg.log" | sed 's/^/      /'
    printf "\n  ${YEL}若失败，最可能是 Config 文件安装位置不对。${RST}\n"
    printf "  已安装到：\n"
    find "$PREFIX" -name 'serialConfig.cmake' -o -name 'serialTargets.cmake' 2>/dev/null | sed 's/^/      /'
  fi
fi

# ---------------------------------------------------------------- 汇总
sec "汇总"
printf "  工作目录 : %s\n" "$WORK"
printf "  自测退出码 : %s\n" "$RC"
printf "\n"
if [ "$RC" = "0" ]; then
  printf "  ${GRN}${BOLD}serial 包 ✅ 可用${RST}\n"
  printf "\n  ${BOLD}下一步：把 serial 纳入工作空间一起编译${RST}\n"
  printf "    bash \"/mnt/c/Users/Ran/Desktop/无人车/src-car/build_ws.sh\"\n"
  printf "    （build_ws.sh 已改成先编 serial、再编 dzmission）\n"
else
  printf "  ${RED}${BOLD}serial 包有问题，先解决再编 dzmission${RST}\n"
fi
printf "\n"

exit $RC
