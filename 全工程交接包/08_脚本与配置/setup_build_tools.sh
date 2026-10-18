#!/usr/bin/env bash
# =============================================================================
# 阶段 1：编译工具链 + 最小编译验证（不需要 ROS）
# =============================================================================
# 在 Ubuntu 22.04 (jammy) 里运行：
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/setup_build_tools.sh"
#
# 作用：装 g++/cmake/ninja，然后**当场编译并运行 bridge_selftest**，
#       验证 dzmission 里那部分纯标准库代码能否编译、过门判定是否正确。
# 这一步不依赖 ROS，5 分钟能出结果。
# =============================================================================

set +e

BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %s\n" "$1"; }
bad()  { printf "  ${RED}✗${RST} %s\n" "$1"; }
warn() { printf "  ${YEL}!${RST} %s\n" "$1"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

WINPROJ="/mnt/c/Users/Ran/Desktop/无人车/src-car"

printf "${BOLD}阶段 1：编译工具链 + 最小编译验证${RST}\n"

# ---------------------------------------------------------------- 0. 环境确认
sec "0. 环境确认"

if [ -r /etc/os-release ]; then . /etc/os-release; fi
printf "  发行版        : %s\n" "${PRETTY_NAME:-unknown}"
printf "  代号          : %s\n" "${VERSION_CODENAME:-unknown}"
printf "  内存          : %s\n" "$(free -h | awk '/^Mem:/{print $2" (可用 "$7")"}')"
printf "  swap          : %s\n" "$(free -h | awk '/^Swap:/{print $2}')"
printf "  CPU           : %s 核\n" "$(nproc)"
printf "  磁盘(/)可用   : %s\n" "$(df -h / | awk 'NR==2{print $4}')"

if [ "${VERSION_CODENAME:-}" = "jammy" ]; then
  ok "Ubuntu 22.04 (jammy) —— 与 ROS 2 Humble 匹配"
else
  warn "代号不是 jammy（是 ${VERSION_CODENAME:-?}）—— ROS 2 Humble 需要 22.04"
fi

# 内存提示
MEM_MB=$(awk '/^MemTotal:/{printf "%d", $2/1024}' /proc/meminfo)
if [ "$MEM_MB" -lt 5000 ]; then
  warn "内存 ${MEM_MB}MB 偏小 —— 编译 nav2 等大包时务必限制并行度（见阶段 3）"
fi

# ---------------------------------------------------------------- 1. 装工具链
sec "1. 安装编译工具链"

NEED=()
command -v gcc    >/dev/null 2>&1 || NEED+=(gcc)
command -v g++    >/dev/null 2>&1 || NEED+=(g++)
command -v make   >/dev/null 2>&1 || NEED+=(make)
command -v cmake  >/dev/null 2>&1 || NEED+=(cmake)
command -v ninja  >/dev/null 2>&1 || NEED+=(ninja)
command -v pkg-config >/dev/null 2>&1 || NEED+=(pkg-config)

if [ ${#NEED[@]} -eq 0 ]; then
  ok "工具链已齐（跳过 apt）"
else
  warn "缺少: ${NEED[*]}  —— 开始 apt 安装"
  sudo apt update || { bad "apt update 失败"; exit 1; }
  sudo apt install -y build-essential cmake ninja-build pkg-config git curl || {
    bad "apt install 失败"; exit 1; }
  ok "工具链安装完成"
fi

# 版本确认
sec "2. 工具链版本"
for t in gcc g++ make cmake ninja pkg-config git python3; do
  if command -v "$t" >/dev/null 2>&1; then
    printf "  %-12s %s\n" "$t" "$("$t" --version 2>&1 | head -1 | cut -c1-70)"
  else
    printf "  %-12s ${RED}未安装${RST}\n" "$t"
  fi
done

# C++17 实测
if command -v g++ >/dev/null 2>&1; then
  T=$(mktemp -d)
  echo 'int main(){ return 0; }' > "$T/t.cpp"
  if g++ -std=c++17 "$T/t.cpp" -o "$T/t" 2>/dev/null; then
    ok "g++ 支持 -std=c++17"
  else
    bad "g++ 不支持 -std=c++17（版本过旧）"
  fi
  rm -rf "$T"
fi

# ---------------------------------------------------------------- 3. 最小编译验证
sec "3. ★ 最小编译验证（bridge_selftest，不需要 ROS）"

if [ ! -d "$WINPROJ" ]; then
  bad "项目目录不可见: $WINPROJ"
  exit 1
fi
ok "项目目录可见"

# ★ 不在 /mnt/c 下编译：拷到 Linux 侧
BUILDROOT="$HOME/dz_build"
mkdir -p "$BUILDROOT"
cp "$WINPROJ/dzmission/src/bridge_tracker.cpp" "$BUILDROOT/" 2>/dev/null
cp "$WINPROJ/dzmission/test/bridge_selftest.cpp" "$BUILDROOT/" 2>/dev/null
mkdir -p "$BUILDROOT/dzmission"
cp -r "$WINPROJ/dzmission/include" "$BUILDROOT/dzmission/" 2>/dev/null

if [ ! -f "$BUILDROOT/bridge_tracker.cpp" ]; then
  bad "无法拷贝源文件（检查路径中的中文是否被正确转义）"
  ls -la "$WINPROJ/dzmission/src/" 2>&1 | head -5
  exit 1
fi
ok "源文件已拷到 $BUILDROOT（避免 /mnt/c 的 IO 开销）"

cd "$BUILDROOT" || exit 1
printf "\n  ${BOLD}编译命令：${RST}\n"
printf "    g++ -std=c++17 -O1 -Wall -I dzmission/include \\\\\n"
printf "        bridge_tracker.cpp bridge_selftest.cpp -o bridge_selftest\n\n"

if g++ -std=c++17 -O1 -Wall -Wextra \
      -I dzmission/include \
      bridge_tracker.cpp bridge_selftest.cpp \
      -o bridge_selftest 2>compile.log; then
  ok "★ 编译成功"
  printf "\n  ${BOLD}运行结果：${RST}\n"
  # ★ 关键：绝不能写 `./x | tee f`，那样管道的退出码来自 tee，
  #   会把自测的失败码**吞掉**，导致"有失败项却报成功"。
  #   （第一次运行时就是这样误报了，必须显式取退出码）
  ./bridge_selftest > run.log 2>&1
  RUN_RC=$?
  cat run.log
  if [ "$RUN_RC" = "0" ]; then
    ok "★ 自测全部通过（退出码 0）"
  else
    bad "★ 自测有失败项（退出码 $RUN_RC）—— 判定逻辑有问题，见上"
    printf "  ${BOLD}不要上线${RST}，请把上面输出贴回给助手。\n"
  fi
else
  bad "★ 编译失败 —— 完整错误如下："
  printf "\n"
  cat compile.log
  printf "\n"
  printf "  ${BOLD}请把上面这段错误完整贴回给助手。${RST}\n"
fi

# ---------------------------------------------------------------- 4. 汇总
sec "4. 汇总"
printf "  编译目录 : %s\n" "$BUILDROOT"
printf "  编译日志 : %s\n" "$BUILDROOT/compile.log"
printf "\n  ${BOLD}下一步${RST}：装 ROS 2 Humble（见 setup_ros2_humble.sh）\n\n"

exit 0
