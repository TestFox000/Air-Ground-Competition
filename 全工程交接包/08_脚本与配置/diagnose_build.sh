#!/usr/bin/env bash
# =============================================================================
# 诊断：为什么 serial 没被 colcon 选上
# =============================================================================
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/diagnose_build.sh"
#
# 直接回答这几个问题：
#   1. ~/ws/src 里到底有哪些包目录？
#   2. colcon 能否发现这些包？（colcon list）
#   3. 哪些包之间有依赖关系？（colcon list --topological-order）
#   4. 单独 build serial 能不能成功？
# =============================================================================

set +e
BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %s\n" "$1"; }
bad()  { printf "  ${RED}✗${RST} %s\n" "$1"; }
warn() { printf "  ${YEL}!${RST} %s\n" "$1"; }
info() { printf "  ${CYN}·${RST} %s\n" "$1"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

WS="$HOME/ws"
WINPROJ="/mnt/c/Users/Ran/Desktop/无人车/src-car"

printf "${BOLD}诊断：serial 为什么没被选上${RST}\n"

# ---------------------------------------------------------------- 1
sec "1. $WS/src 下的内容"

if [ ! -d "$WS/src" ]; then
  bad "$WS/src 不存在 —— 先跑 build_ws.sh 拷贝源码"
  exit 1
fi
ls -la "$WS/src" | sed 's/^/  /'

printf "\n  逐个看是否是包（有无 package.xml / CMakeLists.txt）：\n"
for d in "$WS/src"/*/; do
  [ -d "$d" ] || continue
  n=$(basename "$d")
  px="no"; cm="no"
  [ -f "$d/package.xml" ] && px="yes"
  [ -f "$d/CMakeLists.txt" ] && cm="yes"
  printf "    %-24s package.xml=%-4s CMakeLists=%s\n" "$n" "$px" "$cm"
done

# ---------------------------------------------------------------- 2
sec "2. colcon 能否发现这些包"

source /opt/ros/humble/setup.bash 2>/dev/null
cd "$WS" || exit 1

printf "  colcon list：\n"
colcon list 2>&1 | sed 's/^/    /'

printf "\n  ★ 若上面没有出现 serial，说明 colcon 根本没发现它\n"

# ---------------------------------------------------------------- 3
sec "3. 依赖关系（拓扑顺序）"

printf "  colcon list --topological-order：\n"
colcon list --topological-order 2>&1 | sed 's/^/    /'

# ---------------------------------------------------------------- 4
sec "4. serial 是否被识别为依赖"

printf "  dzmission 依赖 serial —— colcon 的识别结果：\n"
colcon graph --packages-select dzmission 2>&1 | head -20 | sed 's/^/    /'

# ---------------------------------------------------------------- 5
sec "5. 单独 build serial"

printf "  colcon build --packages-select serial：\n"
colcon build --packages-select serial \
  --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  2>&1 | tail -25 | sed 's/^/    /'
RC=$?

printf "\n"
if [ -d "$WS/install/serial" ]; then
  ok "install/serial 存在"
  printf "    share/serial/ 内容：\n"
  ls "$WS/install/serial/share/serial/" 2>/dev/null | sed 's/^/      /'
  if [ -f "$WS/install/serial/share/serial/package.sh" ]; then
    ok "★ package.sh **存在** —— serial 装对了"
  else
    bad "★ package.sh **仍不存在** —— serial 的 CMakeLists 没生成它"
    printf "      这说明 ament_package() 仍未被正确执行。\n"
    printf "      查 CMake 配置日志：\n"
    find "$WS/log/latest_build/serial" -name '*.log' 2>/dev/null | head -5 | while read -r f; do
      printf "        %s\n" "$f"
    done
    printf "      看 command.log 里的 configure 输出：\n"
    cat "$WS/log/latest_build/serial/command.log" 2>/dev/null | tail -25 | sed 's/^/        /'
  fi
else
  bad "install/serial 不存在 —— serial 编译失败"
fi

# ---------------------------------------------------------------- 6
sec "6. 核对源码里的 CMakeLists 是否真的含 ament_package()"

CM="$WS/src/serial/CMakeLists.txt"
if [ -f "$CM" ]; then
  printf "  $CM 里的关键行：\n"
  grep -nE 'ament_package|find_package\(ament_cmake|ament_export_targets|if\(ament' "$CM" | sed 's/^/    /'
  printf "\n  行数: %s\n" "$(wc -l < "$CM")"
else
  bad "$CM 不存在"
fi

sec "汇总"
printf "  请把本脚本**完整输出**贴回给助手。\n\n"
exit 0
