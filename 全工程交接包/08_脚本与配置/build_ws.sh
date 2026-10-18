#!/usr/bin/env bash
# =============================================================================
# 阶段 3：搭建工作空间并编译（★ 针对 8GB 内存优化）
# =============================================================================
# 用法：
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/build_ws.sh"           # 只编译 dzmission
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/build_ws.sh" rknn_pt   # 指定包
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/build_ws.sh" all       # 全部（★ 8GB 风险高）
#
# ★★ 8GB 内存的核心策略 ★★
#   1. 源码拷到 Linux 侧（~/ws），**不在 /mnt/c 下编译**（IO 慢 5~10 倍）
#   2. 用 -j2 / -l2 严格限制并行度 —— 并行是 OOM 的头号原因
#   3. 用 --symlink-install 省空间（install 目录做软链）
#   4. 单包逐个编译，不要一次全量（失败面小、定位快）
#   5. 编译前先按包依赖顺序处理
# =============================================================================

set +e

BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'
# ★ 这几个函数必须支持**格式化参数**：
#   写成 `printf "  ... %s\n" "$1"` 只取第一个参数，多余的会被静默丢弃，
#   于是 `ok "包 %s 完成" "$p"` 会原样打印出 `包 %s 完成`（很误导）。
#   用 "$@" 把全部参数交给 printf 才能正确格式化。
#   本脚本在这一点上错过两次（bad 一次、ok 一次），特此说明。
ok()   { printf "  ${GRN}✓${RST} $*\n"; }
bad()  { printf "  ${RED}✗${RST} $*\n"; }
warn() { printf "  ${YEL}!${RST} $*\n"; }
info() { printf "  ${CYN}·${RST} $*\n"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

TARGET="${1:-dzmission}"
WINPROJ="/mnt/c/Users/Ran/Desktop/无人车/src-car"
WS="$HOME/ws"

# ★★ 并行度上限（关键）★★
#   本机 WSL 只看到约 3.7GB 内存、1GB swap。
#   实测经验：编译 ROS 2 包时，每个 g++/cc1plus 进程峰值约 0.5~1.5GB。
#   并行 2 个包时，若每个包内部再并行，很容易突破 3.7GB -> OOM-kill。
#   => 默认 JOBS=1（串行编译），最稳。若加了 swap 且编译顺利，可手动改回 2。
JOBS=1
LOAD=1

printf "${BOLD}阶段 3：工作空间编译（目标: %s，并行度: -j%d）${RST}\n" "$TARGET" "$JOBS"

# ---------------------------------------------------------------- 0. 内存检查
sec "0. 内存与 swap 检查"

MEM_MB=$(awk '/^MemTotal:/{printf "%d", $2/1024}' /proc/meminfo)
SWAP_MB=$(awk '/^SwapTotal:/{printf "%d", $2/1024}' /proc/meminfo)
printf "  内存 : %s MB\n" "$MEM_MB"
printf "  swap : %s MB\n" "$SWAP_MB"

if [ "$SWAP_MB" -lt 1024 ]; then
  warn "swap 小于 1GB —— 编译大包时容易 OOM 被杀"
  printf "     建议在 Windows 侧创建 %%USERPROFILE%%\\.wslconfig 并重启 WSL：\n"
  printf "       [wsl2]\n       swap=8GB\n       memory=5GB\n"
  printf "     （8GB 物理内存下 memory 建议 4~5GB，给 Windows 留够）\n"
fi

# 编译前释放页缓存（可选，帮助不大但无害）
sync 2>/dev/null

# ---------------------------------------------------------------- 1. ROS 环境
sec "1. ROS 2 环境"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  bad "/opt/ros/humble/setup.bash 不存在 —— 请先运行 setup_ros2_humble.sh"
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
ok "已 source /opt/ros/humble/setup.bash (ROS_DISTRO=$ROS_DISTRO)"

if ! command -v colcon >/dev/null 2>&1; then
  bad "colcon 未安装 —— sudo apt install -y python3-colcon-common-extensions"
  exit 1
fi
ok "colcon 可用"

# ---------------------------------------------------------------- 2. 拷贝源码
sec "2. 拷贝源码到 Linux 侧（避免 /mnt/c 的 IO 开销）"

if [ ! -d "$WINPROJ" ]; then
  bad "项目目录不可见: $WINPROJ"
  exit 1
fi

mkdir -p "$WS/src"

# ★ 只拷需要的包，避免把第三方大包（nav2/slam）拖进来
WANT=()
case "$TARGET" in
  all)
    WANT=(serial dzmission rknn_pt dzactuator dzjudgment dzrknn_dataprocess dztrack)
    warn "全部编译包含 rknn_pt（需要 RKNN SDK/rknnrt），在 PC 上**很可能失败**"
    warn "  → 若只想验证编排层，建议用默认目标 dzmission"
    ;;
  dzmission)
    # ★ dzmission 依赖 serial 库（find_package(serial REQUIRED)），
    #   而 ROS 2 Humble 没有对应的 apt 包，所以 serial 是本工作空间内的
    #   自建包（serial/ 目录）。**必须先编译它**，否则 dzmission 找不到。
    WANT=(serial dzmission)
    ;;
  *)
    WANT=($TARGET)
    ;;
esac

printf "  要拷贝的包: %s\n" "${WANT[*]}"

for p in "${WANT[@]}"; do
  if [ -d "$WINPROJ/$p" ]; then
    rm -rf "$WS/src/$p"
    cp -r "$WINPROJ/$p" "$WS/src/$p" && ok "拷贝 $p"
  else
    bad "源包里找不到 $p"
  fi
done

# fake_amcl_node 没有 package.xml，colcon 会跳过它 —— 顺带提示
if [ -d "$WINPROJ/fake_amcl_node" ] && [ ! -f "$WINPROJ/fake_amcl_node/package.xml" ]; then
  warn "fake_amcl_node 缺 package.xml（colcon 会跳过）—— 若 launch 引用它需补上"
fi

# ---------------------------------------------------------------- 3. 依赖检查
sec "3. 包依赖检查（rosdep）"

DEP_FAIL=0
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  printf "  初始化 rosdep…\n"
  sudo rosdep init 2>/dev/null
fi
rosdep update >/dev/null 2>&1

printf "  检查依赖（只报告，不自动装，避免误装大包）…\n"
rosdep check --from-paths "$WS/src" --ignore-src 2>&1 | head -30
DEP_FAIL=${PIPESTATUS[0]}
if [ "$DEP_FAIL" != "0" ]; then
  warn "有缺失依赖。可执行（谨慎，会装不少东西）："
  printf "       sudo rosdep install --from-paths ~/ws/src --ignore-src -r -y\n"
else
  ok "依赖检查通过"
fi

# ---------------------------------------------------------------- 4. 编译
sec "4. 编译"

cd "$WS" || exit 1

# =============================================================================
# ★★ 先清理陈旧的 build/install（这是本脚本的一个真实坑）★★
# -----------------------------------------------------------------------------
# 症状：改了包的 CMakeLists 后重跑，colcon 仍用旧产物，报
#         Failed to find .../install/<pkg>/share/<pkg>/package.sh
#       且输出里只 "Starting >>> <依赖它的包>"，**被改的包根本没被编**。
#
# 原因：colcon 按 install/<pkg>/ 下的标记判断"是否已构建"。
#       若上一次是用**旧的、有缺陷的** CMakeLists 装的，
#       标记存在但 package.sh 缺失 -> colcon 认为无需重编 ->
#       依赖它的包读不到 package.sh -> 报错。
#
# 所以：**每次构建前清理 build/ 与 install/**，除非显式要求增量。
#   KEEP_BUILD=1 bash build_ws.sh   # 需要增量编译时这样用
# =============================================================================
if [ "${KEEP_BUILD:-0}" != "1" ]; then
  printf "  清理陈旧的 build/ install/（避免 colcon 复用有缺陷的旧产物）\n"
  rm -rf "$WS/build" "$WS/install"
  ok "已清理（如需增量编译：KEEP_BUILD=1 bash build_ws.sh）"
else
  warn "KEEP_BUILD=1 —— 保留 build/install 做增量编译"
  warn "  若改了 CMakeLists 后出现 package.sh 之类找不到的错误，请去掉该变量重跑"
fi

# ★★ 8GB 内存的关键参数 ★★
COLCON_ARGS=(
  --symlink-install                      # 省磁盘
  --parallel-workers "$JOBS"             # 限制并行包数
  --cmake-args -DCMAKE_BUILD_TYPE=Release
              -DCMAKE_CXX_FLAGS_RELEASE="-O2 -g0"
  --event-handlers console_direct+       # 实时输出，便于看 OOM
)
MAKE_ARGS=(--executor sequential)
# 让 make 也限制并行与负载（双重保险）
export MAKEFLAGS="-j$JOBS -l$LOAD"
export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"

printf "  编译顺序: %s\n" "${WANT[*]}"
printf "  ★ serial 先于 dzmission：后者 find_package(serial REQUIRED)\n"
printf "  colcon build %s\n" "${COLCON_ARGS[*]}"
printf "  MAKEFLAGS=%s\n" "$MAKEFLAGS"
printf "\n  ${YEL}提示${RST}：若看到 'Killed' 或 'signal 9'，就是 OOM，\n"
printf "        把 JOBS 改成 1 重试（本脚本顶部 JOBS 变量）\n\n"

# =============================================================================
# ★★ 逐个包单独编译（不再把多个 --packages-select 拼在一起）★★
# -----------------------------------------------------------------------------
# 踩过的坑（有 set -x 实证）：
#   写 `colcon build --packages-select serial --packages-select dzmission`，
#   参数确实原样传进去了（set -x 可见），但 colcon **只启动 dzmission**，
#   且在构建前的检查阶段就去找 install/serial/share/serial/package.sh，
#   找不到就整体失败：
#       Starting >>> dzmission
#       ERROR: Failed to find .../install/serial/share/serial/package.sh
#
#   即：colcon 把 serial 当成了"必须已构建好的环境依赖"，
#       而不是"本次要一起构建的包"。
#
#   ⇒ 改为**对每个包单独调用一次 colcon build**，顺序由我们保证：
#       colcon build --packages-select serial      # 先把它装好
#       colcon build --packages-select dzmission   # 再编依赖它的
#     这样 dzmission 那一步开始时，install/serial 已完整存在，不会再报错。
#
#   ★ 注意 `--packages-select` 会**跳过依赖解析**（不自动编依赖），
#     这正是我们需要的：依赖由 WANT 的顺序显式管理。
# =============================================================================
printf "  ${CYN}逐个编译（顺序即依赖顺序）：${RST}\n"
for p in "${WANT[@]}"; do
  printf "    -> %s\n" "$p"
done
printf "\n"

START=$(date +%s)
RC=0
FAILED_PKG=""
for p in "${WANT[@]}"; do
  printf "\n  ${BOLD}===== 编译 %s =====${RST}\n" "$p"
  colcon build --packages-select "$p" "${COLCON_ARGS[@]}"
  rc=$?
  if [ "$rc" != "0" ]; then
    RC=$rc
    FAILED_PKG="$p"
    bad "包 %s 编译失败（退出码 %s）—— 停止后续编译" "$p" "$rc"
    break
  fi
  ok "包 %s 完成" "$p"
done
ELAPSED=$(( $(date +%s) - START ))

printf "\n"
if [ "$RC" = "0" ]; then
  ok "★ 编译成功（耗时 ${ELAPSED}s）"
else
  bad "★ 编译失败：包 %s（退出码 $RC，耗时 ${ELAPSED}s）" "$FAILED_PKG"
  printf "\n  ${BOLD}收集错误摘要：${RST}\n"
  find "$WS/log/latest_build" -name 'stderr.log' 2>/dev/null | while read -r f; do
    if [ -s "$f" ]; then
      printf "\n  --- %s ---\n" "$f"
      grep -iE 'error|错误|undefined reference|No such file' "$f" | head -20 | sed 's/^/    /'
    fi
  done
  printf "\n  ${BOLD}把上面的错误贴回给助手。${RST}\n"
fi

# ---------------------------------------------------------------- 5. 验证产物
sec "5. 验证产物"

# =============================================================================
# ★★ 关键检查：确认每个**目标包都真的被构建了** ★★
# -----------------------------------------------------------------------------
# 为什么需要这个：
#   上一轮出现过"colcon 报 0 packages finished 却没编任何目标包"的情况 ——
#   colcon 认为包已构建（install/ 里有陈旧标记），于是跳过；
#   而实际产物缺失，结果依赖它的包报：
#       Failed to find .../install/<pkg>/share/<pkg>/package.sh
#   只报"编译成功"不够，必须**逐个确认产物存在**。
# =============================================================================
MISSING_INSTALL=()
for p in "${WANT[@]}"; do
  if [ -d "$WS/install/$p" ]; then
    ok "已安装: $p"
  else
    bad "**未安装**: $p  <- 该包没有被构建（或构建失败）"
    MISSING_INSTALL+=("$p")
  fi
done

if [ ${#MISSING_INSTALL[@]} -gt 0 ]; then
  printf "\n  ${RED}${BOLD}★ 有 %d 个目标包没有产物${RST}\n" "${#MISSING_INSTALL[@]}"
  printf "     ${BOLD}最可能的原因${RST}：colcon 复用了陈旧的 build/install。\n"
  printf "     本脚本默认会先清理；若你用了 KEEP_BUILD=1，请去掉它重跑。\n"
  printf "     也可手动清理：rm -rf ~/ws/build ~/ws/install ~/ws/log\n\n"
fi

if [ -d "$WS/install" ]; then
  printf "  已安装的包：\n"
  ls "$WS/install" 2>/dev/null | sed 's/^/    /'
  printf "\n  可执行文件：\n"
  find "$WS/install" -type f -perm -u+x -path '*/lib/*' 2>/dev/null | head -20 | sed 's/^/    /'
fi

# 若有 bridge_selftest 就顺手跑
SELFTEST=$(find "$WS/install" -name 'bridge_selftest' -type f 2>/dev/null | head -1)
if [ -n "$SELFTEST" ]; then
  sec "6. ★ 运行 bridge_selftest"
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash" 2>/dev/null
  if "$SELFTEST"; then ok "bridge_selftest 通过"; else bad "bridge_selftest 失败"; fi
else
  printf "\n  ${YEL}!${RST} 未找到 bridge_selftest（可能未编译进 install）\n"
fi

sec "汇总"
printf "  工作空间 : %s\n" "$WS"
printf "  编译耗时 : %ss\n" "$ELAPSED"
printf "  编译日志 : %s\n" "$WS/log/latest_build"
printf "\n  ${BOLD}每次开新终端：${RST}\n"
printf "    source /opt/ros/humble/setup.bash\n"
printf "    source %s/install/setup.bash\n\n" "$WS"

exit $RC
