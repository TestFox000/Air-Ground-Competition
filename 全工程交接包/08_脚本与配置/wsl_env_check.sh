#!/usr/bin/env bash
# =============================================================================
# WSL Ubuntu 环境自检（为编译 dzmission / ROS 2 Humble 做准备）
# =============================================================================
# 用法：  bash wsl_env_check.sh
#         （或先 chmod +x wsl_env_check.sh 再 ./wsl_env_check.sh）
#
# 输出会告诉你：哪些已就绪、哪些还缺、缺的怎么装。
# =============================================================================

set +e

BOLD=$'\033[1m'; DIM=$'\033[2m'; RED=$'\033[31m'; GRN=$'\033[32m'
YEL=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'

ok()   { printf "  ${GRN}✓${RST} %-34s %s\n" "$1" "$2"; }
bad()  { printf "  ${RED}✗${RST} %-34s %s\n" "$1" "$2"; }
warn() { printf "  ${YEL}!${RST} %-34s %s\n" "$1" "$2"; }
info() { printf "  ${DIM}·${RST} %-34s %s\n" "$1" "$2"; }
sec()  { printf "\n${BOLD}${CYN}=== %s ===${RST}\n" "$1"; }

MISSING=()
SUGGEST=()

need() {  # need <检查名> <是否满足0/1> <实际值> <缺失时的安装建议>
  if [ "$2" = "0" ]; then ok "$1" "$3"; else bad "$1" "$3"; MISSING+=("$1"); [ -n "$4" ] && SUGGEST+=("$4"); fi
}

printf "${BOLD}WSL Ubuntu 环境自检${RST}\n"
printf "${DIM}时间: %s${RST}\n" "$(date '+%F %T')"

# ---------------------------------------------------------------- 1. 系统
sec "1. 系统信息"

if [ -r /etc/os-release ]; then
  . /etc/os-release
  DISTRO="$PRETTY_NAME"
  CODENAME="${VERSION_CODENAME:-unknown}"
else
  DISTRO="unknown"; CODENAME="unknown"
fi
info "发行版" "$DISTRO"
info "代号"   "$CODENAME"
info "内核"   "$(uname -r)"
info "架构"   "$(uname -m)"
info "CPU 核数" "$(nproc)"
info "内存"   "$(free -h 2>/dev/null | awk '/^Mem:/{print $2}')"
info "磁盘可用(/) " "$(df -h / 2>/dev/null | awk 'NR==2{print $4}')"

case "$CODENAME" in
  jammy) ok "Ubuntu 版本适配 ROS 2 Humble" "22.04 (jammy) — 官方完整支持" ;;
  noble) warn "Ubuntu 版本" "24.04 (noble) — Humble 官方目标为 22.04，用 Jazzy 更顺；仍可装 Humble 但麻烦" ;;
  focal) warn "Ubuntu 版本" "20.04 — 对应 ROS 2 Foxy，不是 Humble" ;;
  *)     warn "Ubuntu 版本" "$CODENAME — 请确认与 ROS 2 版本匹配" ;;
esac

# ---------------------------------------------------------------- 2. WSL
sec "2. WSL 环境"

if grep -qi microsoft /proc/version 2>/dev/null; then
  ok "运行在 WSL 内" "$(grep -o 'WSL[0-9]*' /proc/version | head -1)"
else
  warn "运行在 WSL 内" "未检出（可能是原生 Ubuntu，也可以）"
fi

if grep -qi systemd /proc/1/comm 2>/dev/null || [ -d /run/systemd/system ]; then
  ok "systemd" "已启用（ROS 2 需要）"
else
  bad "systemd" "未启用 —— ROS 2 依赖 systemd 服务"
  SUGGEST+=("启用 systemd：编辑 /etc/wsl.conf 加 [boot] systemd=true，然后在 Windows 执行 wsl --shutdown 再进入")
fi

if command -v systemctl >/dev/null 2>&1; then
  info "systemctl" "$(systemctl --version 2>/dev/null | head -1)"
fi

# ---------------------------------------------------------------- 3. 编译工具链
sec "3. 编译工具链（当前最可能缺的）"

check_tool() {  # check_tool <命令> <显示名> <版本参数>
  if command -v "$1" >/dev/null 2>&1; then
    V=$("$1" $3 2>&1 | head -1 | cut -c1-60)
    ok "$2" "$V"
    return 0
  else
    bad "$2" "未安装"
    MISSING+=("$2")
    return 1
  fi
}

HAVE_GXX=1
check_tool gcc    "gcc"        "--version" || HAVE_GXX=0
check_tool g++    "g++"        "--version" || HAVE_GXX=0
check_tool make   "make"       "--version"
check_tool cmake  "cmake"      "--version"
check_tool ninja  "ninja"      "--version"
check_tool git    "git"        "--version"
check_tool python3 "python3"   "--version"
check_tool pkg-config "pkg-config" "--version"

if [ "$HAVE_GXX" = "1" ]; then
  info "C++ 标准支持" "$(echo 'int main(){return 0;}' | g++ -std=c++17 -x c++ - -o /tmp/_t 2>&1 && echo 'c++17 可用' || echo 'c++17 不可用')"
  rm -f /tmp/_t
fi

# ---------------------------------------------------------------- 4. 依赖库
sec "4. 编译 dzmission 需要的库"

check_pkg() {  # check_pkg <pkg-config名> <显示名> <apt包名>
  if pkg-config --exists "$1" 2>/dev/null; then
    ok "$2" "$(pkg-config --modversion "$1" 2>/dev/null)"
  else
    # 退一步：看头文件是否存在
    if [ -n "$4" ] && [ -e "$4" ]; then
      ok "$2" "头文件存在（无 .pc）"
    else
      bad "$2" "未安装"
      MISSING+=("$2")
      [ -n "$3" ] && SUGGEST+=("sudo apt install -y $3")
    fi
  fi
}

check_pkg eigen3        "Eigen3"        "libeigen3-dev"      "/usr/include/eigen3"
check_pkg yaml-cpp      "yaml-cpp"      "libyaml-cpp-dev"    "/usr/include/yaml-cpp"
check_pkg libpcre2-8    "PCRE2"         "libpcre2-dev"
check_pkg zlib          "zlib"          "zlib1g-dev"

# OpenCV（rknn_pt / 视觉链路用）
if pkg-config --exists opencv4 2>/dev/null; then
  ok "OpenCV4" "$(pkg-config --modversion opencv4)"
elif [ -e /usr/include/opencv4 ]; then
  ok "OpenCV4" "头文件存在（无 .pc）"
else
  bad "OpenCV4" "未安装"
  SUGGEST+=("sudo apt install -y libopencv-dev")
fi

# tf2 / geometry（ROS 自带，这里只做提示）
info "tf2 / rclcpp 等" "由 ROS 2 提供，见第 5 节"

# ---------------------------------------------------------------- 5. ROS 2
sec "5. ROS 2 Humble"

if [ -f /opt/ros/humble/setup.bash ]; then
  ok "ROS 2 Humble" "/opt/ros/humble 存在"
  # 在干净子 shell 里 source，避免污染当前会话
  ROSDISTRO=$(bash -c 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; echo $ROS_DISTRO')
  info "ROS_DISTRO" "${ROSDISTRO:-?}"
  if bash -c 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; command -v ros2 >/dev/null'; then
    ok "ros2 命令" "$(bash -c 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; ros2 --version' 2>&1 | head -1)"
  else
    bad "ros2 命令" "找不到（ROS 安装不完整）"
  fi
  for P in rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs nav2_msgs; do
    if [ -d "/opt/ros/humble/share/$P" ] || [ -d "/opt/ros/humble/include/$P" ]; then
      ok "  pkg: $P" "存在"
    else
      bad "  pkg: $P" "缺失"
      MISSING+=("ros-$P")
    fi
  done
  if command -v colcon >/dev/null 2>&1; then
    ok "colcon" "$(colcon version-check 2>/dev/null | head -1 || echo '已安装')"
  else
    bad "colcon" "未安装"
    SUGGEST+=("sudo apt install -y python3-colcon-common-extensions")
  fi
else
  bad "ROS 2 Humble" "/opt/ros/humble 不存在"
  SUGGEST+=("按官方步骤安装 ROS 2 Humble：见 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html")
fi

# ---------------------------------------------------------------- 6. 访问 Windows 项目
sec "6. 访问 Windows 上的项目"

WINPROJ="/mnt/c/Users/Ran/Desktop/无人车/src-car"
ALTROOT="/mnt/c/Users/Ran/Desktop"
if [ -d "$WINPROJ" ]; then
  ok "项目目录可见" "$WINPROJ"
  info "文件数" "$(find "$WINPROJ" -type f 2>/dev/null | wc -l)"
  info "软件包" "$(ls -d "$WINPROJ"/*/ 2>/dev/null | xargs -n1 basename 2>/dev/null | tr '\n' ' ')"
else
  bad "项目目录" "$WINPROJ 不存在或不可访问"
  if [ -d "$ALTROOT" ]; then
    info "上级目录内容" "$(ls "$ALTROOT" 2>/dev/null | head -20 | tr '\n' ' ')"
    SUGGEST+=("确认 Windows 路径：/mnt/c/Users/Ran/Desktop/无人车/src-car")
  else
    warn "挂载" "/mnt/c 不可见 —— 检查 /etc/wsl.conf 的 automount 设置"
  fi
fi

info "跨文件系统性能提示" "在 /mnt/c 下编译很慢；建议把源码拷到 ~/ws 里再 build"

# ---------------------------------------------------------------- 7. 单项编译测试
sec "7. ★ 最小编译测试（关键一步）"

if command -v g++ >/dev/null 2>&1 && [ -d "$WINPROJ" ]; then
  TMP=$(mktemp -d)
  if g++ -std=c++17 -O1 -Wall -I "$WINPROJ/dzmission/include" \
        "$WINPROJ/dzmission/src/bridge_tracker.cpp" \
        "$WINPROJ/dzmission/test/bridge_selftest.cpp" \
        -o "$TMP/bridge_selftest" 2>"$TMP/err.txt"; then
    ok "编译 bridge_selftest" "成功"
    if "$TMP/bridge_selftest" >"$TMP/run.txt" 2>&1; then
      ok "运行 bridge_selftest" "$(tail -3 "$TMP/run.txt" | tr '\n' ' ')"
    else
      bad "运行 bridge_selftest" "退出码非 0，见下"
      tail -15 "$TMP/run.txt" | sed 's/^/      /'
    fi
  else
    bad "编译 bridge_selftest" "失败 —— 前 20 行错误："
    head -20 "$TMP/err.txt" | sed 's/^/      /'
    SUGGEST+=("若报 'command not found' 之类 -> 先装 g++：sudo apt install -y build-essential")
  fi
  rm -rf "$TMP"
else
  warn "编译 bridge_selftest" "跳过（缺 g++ 或项目目录不可见）"
fi

# ---------------------------------------------------------------- 汇总
sec "汇总"

if [ ${#MISSING[@]} -eq 0 ]; then
  printf "  ${GRN}${BOLD}环境已就绪 —— 没有缺失项${RST}\n"
else
  printf "  ${YEL}${BOLD}缺失 %d 项：${RST}\n" "${#MISSING[@]}"
  for m in "${MISSING[@]}"; do printf "    - %s\n" "$m"; done
fi

if [ ${#SUGGEST[@]} -gt 0 ]; then
  printf "\n  ${BOLD}建议执行的命令：${RST}\n"
  # 去重
  printf '%s\n' "${SUGGEST[@]}" | awk '!seen[$0]++' | sed 's/^/    /'
fi

printf "\n  ${DIM}把本脚本的完整输出贴回给助手，即可得到针对性的下一步。${RST}\n\n"

exit 0
