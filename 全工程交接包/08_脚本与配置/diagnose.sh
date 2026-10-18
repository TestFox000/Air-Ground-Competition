#!/usr/bin/env bash
# =============================================================================
# 诊断：ROS 2 安装后的遗留问题
# =============================================================================
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/diagnose.sh"
#
# 检查三件事：
#   1. 哪些"额外依赖"没装上（上次脚本报了"部分失败"）
#   2. rosdep update 为什么失败
#   3. nav2_msgs 该怎么提供（apt 二进制 vs 工作空间源码）
# =============================================================================

set +e
BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %s\n" "$1"; }
bad()  { printf "  ${RED}✗${RST} %s\n" "$1"; }
warn() { printf "  ${YEL}!${RST} %s\n" "$1"; }
info() { printf "  ${CYN}·${RST} %s\n" "$1"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

WINPROJ="/mnt/c/Users/Ran/Desktop/无人车/src-car"

printf "${BOLD}诊断：ROS 2 安装后的遗留问题${RST}\n"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null
printf "  ROS_DISTRO = %s\n" "${ROS_DISTRO:-未设置}"

# ---------------------------------------------------------------- 1. 额外依赖
sec "1. 额外依赖逐项检查"

check_apt() {  # <apt包名> <说明>
  if dpkg -s "$1" >/dev/null 2>&1; then
    V=$(dpkg-query -W -f='${Version}' "$1" 2>/dev/null)
    ok "$1  ($V)"
  else
    bad "$1  未安装  <- $2"
  fi
}

check_apt libeigen3-dev      "Eigen3 线性代数"
check_apt libyaml-cpp-dev    "YAML 解析"
check_apt libpcre2-dev       "正则（Console Bridge 依赖）"
check_apt zlib1g-dev         "压缩"
check_apt libopencv-dev      "OpenCV4（视觉链路）"
check_apt python3-colcon-common-extensions "colcon 构建工具"
check_apt python3-rosdep     "rosdep"
check_apt python3-vcstool    "vcstool"
check_apt ros-humble-tf2-tools "tf2 工具"
check_apt ros-humble-tf-transformations "tf 转换"

printf "\n  头文件存在性（比 dpkg 更直接）：\n"
for h in /usr/include/eigen3 /usr/include/yaml-cpp /usr/include/pcre2.h \
         /usr/include/zlib.h /usr/include/opencv4; do
  if [ -e "$h" ]; then ok "$h"; else bad "$h 不存在"; fi
done

# ---------------------------------------------------------------- 2. nav2_msgs
sec "2. nav2_msgs 的两种来源"

APT_AVAIL=$(apt-cache policy ros-humble-nav2-msgs 2>/dev/null | awk '/Candidate:/{print $2}')
printf "  (a) apt 二进制包 ros-humble-nav2-msgs\n"
if [ -n "$APT_AVAIL" ] && [ "$APT_AVAIL" != "(none)" ]; then
  ok "    可安装，候选版本: $APT_AVAIL"
  printf "        安装命令: sudo apt install -y ros-humble-nav2-msgs\n"
else
  warn "    apt 源里查不到（可能 apt update 没跑或源未配好）"
fi

printf "  (b) 工作空间源码\n"
NMSRC="$WINPROJ/navigation2/nav2_msgs"
if [ -f "$NMSRC/package.xml" ] && [ -f "$NMSRC/action/NavigateToPose.action" ]; then
  ok "    源码完整: $NMSRC"
  if [ -f "$NMSRC/package.xml" ]; then
    VER=$(grep -oP '(?<=<version>)[^<]+' "$NMSRC/package.xml" 2>/dev/null | head -1)
    DEP=$(grep -oP '(?<=<depend>)[^<]+' "$NMSRC/package.xml" 2>/dev/null | tr '\n' ' ')
    printf "        版本: %s\n" "$VER"
    printf "        依赖: %s\n" "$DEP"
  fi
else
  bad "    源码不完整"
fi

# 是否已装到 /opt/ros
if [ -d /opt/ros/humble/share/nav2_msgs ]; then
  ok "    /opt/ros/humble 里已有 nav2_msgs"
else
  info "    /opt/ros/humble 里没有 nav2_msgs（所以上一步报了缺失）"
fi

# ---------------------------------------------------------------- 3. rosdep
sec "3. rosdep 诊断"

if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  ok "rosdep 源列表已初始化"
  head -5 /etc/ros/rosdep/sources.list.d/20-default.list | sed 's/^/      /'
else
  bad "rosdep 未初始化（没有 20-default.list）"
  printf "        修复: sudo rosdep init\n"
fi

printf "  测试 rosdep update（限时 40s）：\n"
timeout 40 rosdep update 2>&1 | tail -12 | sed 's/^/      /'
RC=${PIPESTATUS[0]}
if [ "$RC" = "0" ]; then
  ok "rosdep update 成功"
elif [ "$RC" = "124" ]; then
  bad "rosdep update 超时 —— 网络问题（raw.githubusercontent.com 被墙？）"
  printf "        ${BOLD}绕过办法${RST}：本项目的依赖基本都能直接 apt 装，\n"
  printf "                   可以不依赖 rosdep（见下节）\n"
else
  bad "rosdep update 失败（退出码 $RC）"
fi

# 常见根因探测
printf "\n  根因探测：\n"
if curl -sS -m 8 -o /dev/null -w '%{http_code}' https://raw.githubusercontent.com >/dev/null 2>&1; then
  ok "    能访问 raw.githubusercontent.com"
else
  bad "    访问 raw.githubusercontent.com 失败 —— 这是 rosdep update 失败的最常见原因"
  printf "        （rosdep 的源列表托管在 GitHub raw，国内常被墙）\n"
fi

# ---------------------------------------------------------------- 4. 汇总建议
sec "4. 建议的修复命令"

printf "  ${BOLD}① 装 nav2_msgs（选一个）${RST}\n"
printf "     A) 二进制（快，推荐）:\n"
printf "        sudo apt install -y ros-humble-nav2-msgs\n"
printf "     B) 源码（apt 装不上时）: 把 nav2_msgs 拷进工作空间后 colcon build\n"

printf "\n  ${BOLD}② 补装失败的额外依赖${RST}\n"
printf "     sudo apt install -y libeigen3-dev libyaml-cpp-dev libpcre2-dev \\\\\n"
printf "         zlib1g-dev libopencv-dev python3-colcon-common-extensions\n"

printf "\n  ${BOLD}③ rosdep（可跳过）${RST}\n"
printf "     本项目依赖简单，若 rosdep update 一直失败，可以：\n"
printf "       · 不跑 rosdep，直接手动 apt 装缺的包\n"
printf "       · 或用镜像源（把 20-default.list 里的 raw.githubusercontent.com\n"
printf "         换成国内镜像），但需自行确认镜像可靠性\n"

printf "\n  ${BOLD}把本节以上的完整输出贴回给助手。${RST}\n\n"

exit 0
