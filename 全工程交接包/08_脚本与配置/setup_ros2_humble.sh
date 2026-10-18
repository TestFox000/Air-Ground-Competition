#!/usr/bin/env bash
# =============================================================================
# 阶段 2：安装 ROS 2 Humble + 关键依赖（Ubuntu 22.04 jammy）
# =============================================================================
# 用法：
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/setup_ros2_humble.sh"
#
# ★ 依据官方文档：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
#   步骤：locale -> sources -> apt install ros-humble-desktop -> ros-dev-tools
#
# ★ 8GB 内存注意事项（重要）：
#   · ros-humble-desktop 比 ros-base 大很多（含 RViz/Gazebo）
#     若磁盘/内存紧张，可只装 ros-base（本脚本支持 --base 参数）
#   · 安装过程 apt 本身不需要太多内存；**真正吃内存的是 colcon build**
#   · 编译阶段必须限制并行度（见阶段 3）
# =============================================================================

set +e

BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %s\n" "$1"; }
bad()  { printf "  ${RED}✗${RST} %s\n" "$1"; }
warn() { printf "  ${YEL}!${RST} %s\n" "$1"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

MODE="${1:-desktop}"     # desktop | base
ROS_DISTRO_NAME="humble"
ROS_DIR="/opt/ros/${ROS_DISTRO_NAME}"

printf "${BOLD}阶段 2：安装 ROS 2 Humble（模式: %s）${RST}\n" "$MODE"
printf "  官方文档: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html\n"

# ---------------------------------------------------------------- 0. 前置检查
sec "0. 前置检查"

if [ -r /etc/os-release ]; then . /etc/os-release; fi
printf "  发行版 : %s (%s)\n" "${PRETTY_NAME:-?}" "${VERSION_CODENAME:-?}"

if [ "${VERSION_CODENAME:-}" != "jammy" ]; then
  bad "ROS 2 Humble 的 deb 包**只对 Ubuntu 22.04 (jammy) 提供**"
  printf "     当前是 %s。请改用 22.04，或改选与当前版本匹配的 ROS 2。\n" "${VERSION_CODENAME:-?}"
  exit 1
fi
ok "Ubuntu 22.04 (jammy) —— 可以安装 Humble"

if [ -d "$ROS_DIR" ]; then
  warn "$ROS_DIR 已存在 —— 可能已装过。跳过安装，只做校验。"
  ALREADY=1
else
  ALREADY=0
fi

# ---------------------------------------------------------------- 1. locale
sec "1. 设置 UTF-8 locale（官方要求）"

CUR_LOCALE=$(locale 2>/dev/null | awk -F= '/^LANG=/{print $2}')
printf "  当前 LANG = %s\n" "${CUR_LOCALE:-未设置}"

sudo apt update -qq
sudo apt install -y locales curl gnupg lsb-release software-properties-common

if ! locale -a 2>/dev/null | grep -qi 'en_US.utf8'; then
  sudo locale-gen en_US en_US.UTF-8
  ok "已生成 en_US.UTF-8"
else
  ok "en_US.UTF-8 已存在"
fi
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 2>/dev/null
export LANG=en_US.UTF-8
ok "locale 设置完成（export LANG=en_US.UTF-8）"

# ---------------------------------------------------------------- 2. ROS 源
sec "2. 添加 ROS 2 apt 源"

sudo add-apt-repository -y universe >/dev/null 2>&1

# ★ 官方现在推荐用 ros2-apt-source 包（自动配好源与密钥）
#   旧写法（手动加 key 与 sources.list）已不推荐。
if [ "$ALREADY" = "0" ]; then
  printf "  获取最新 ros2-apt-source 版本号…\n"
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F "tag_name" | awk -F'"' '{print $4}')
  if [ -z "$ROS_APT_SOURCE_VERSION" ]; then
    bad "无法获取 ros2-apt-source 版本（网络问题？）"
    printf "     可手工执行：\n"
    printf "       export ROS_APT_SOURCE_VERSION=\$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F \"tag_name\" | awk -F'\"' '{print \$4}')\n"
    printf "       curl -L -o /tmp/ros2-apt-source.deb \"https://github.com/ros-infrastructure/ros-apt-source/releases/download/\${ROS_APT_SOURCE_VERSION}/ros2-apt-source_\${ROS_APT_SOURCE_VERSION}.\$(. /etc/os-release && echo \${UBUNTU_CODENAME:-\${VERSION_CODENAME}})_all.deb\"\n"
    printf "       sudo dpkg -i /tmp/ros2-apt-source.deb\n"
    exit 1
  fi
  ok "ros2-apt-source 版本: $ROS_APT_SOURCE_VERSION"

  CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
  DEB_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
  printf "  下载 %s\n" "$DEB_URL"
  if curl -L -o /tmp/ros2-apt-source.deb "$DEB_URL"; then
    sudo dpkg -i /tmp/ros2-apt-source.deb && ok "ROS 2 源已配置"
  else
    bad "下载 ros2-apt-source.deb 失败"
    exit 1
  fi
else
  ok "跳过（ROS 目录已存在）"
fi

# ---------------------------------------------------------------- 3. 安装 ROS 2
sec "3. 安装 ROS 2 Humble（模式: $MODE）"
printf "  ${YEL}注意${RST}：官方警告 —— 全新系统上应先 upgrade 再装，\n"
printf "        否则可能触发**关键系统包被移除**。\n\n"

sudo apt update

# ★ 官方警告：新系统必须先 upgrade systemd/udev 相关
if [ "$ALREADY" = "0" ]; then
  printf "  执行 apt upgrade（官方要求，避免移除 systemd/udev）…\n"
  sudo apt upgrade -y || warn "apt upgrade 有警告，继续"
fi

if [ "$ALREADY" = "0" ]; then
  if [ "$MODE" = "base" ]; then
    PKG="ros-humble-ros-base"
  else
    PKG="ros-humble-desktop"
  fi
  printf "  安装 %s …（这一步较大，请耐心）\n" "$PKG"
  if sudo apt install -y "$PKG"; then
    ok "$PKG 安装完成"
  else
    bad "$PKG 安装失败"
    exit 1
  fi

  printf "  安装 ros-dev-tools（编译器与构建工具）…\n"
  sudo apt install -y ros-dev-tools && ok "ros-dev-tools 安装完成"
else
  ok "跳过（已安装）"
fi

# ---------------------------------------------------------------- 4. 关键依赖
sec "4. 安装项目需要的额外依赖"

EXTRA=(
  libeigen3-dev            # Eigen3
  libyaml-cpp-dev          # yaml-cpp
  libpcre2-dev             # PCRE2
  zlib1g-dev               # zlib
  libopencv-dev            # OpenCV4（rknn_pt / 视觉链路）
  python3-colcon-common-extensions
  python3-rosdep
  python3-vcstool
  ros-humble-tf2-tools
  # ★ ros-humble-serial：dzmission 的 CMakeLists 有 find_package(serial REQUIRED)
  #   用于与 STM32 底盘串口通信。**漏了它 dzmission 编不过**。
  ros-humble-serial
  # ★ rknn_pt 的 det_node.cc 真的在用 cv_bridge（不是死依赖）
  ros-humble-cv-bridge
  ros-humble-image-transport
)
printf "  安装: %s\n" "${EXTRA[*]}"
if sudo apt install -y "${EXTRA[@]}"; then
  ok "额外依赖安装完成"
else
  warn "部分依赖安装失败（可能个别包名在该版本不同）—— 逐个看提示"
fi

# rosdep 初始化（只做一次）
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  printf "  初始化 rosdep…\n"
  sudo rosdep init 2>/dev/null
fi
rosdep update 2>/dev/null && ok "rosdep 已更新" || warn "rosdep update 失败（可稍后重试）"

# ---------------------------------------------------------------- 5. 校验
sec "5. 安装校验"

if [ -f "$ROS_DIR/setup.bash" ]; then
  ok "$ROS_DIR/setup.bash 存在"
  # 在干净子 shell 里验证，避免污染
  if bash -lc "source $ROS_DIR/setup.bash >/dev/null 2>&1 && command -v ros2 >/dev/null"; then
    ok "ros2 命令可用"
    bash -lc "source $ROS_DIR/setup.bash >/dev/null 2>&1; ros2 --version" 2>&1 | head -1 | sed 's/^/      /'
  else
    bad "ros2 命令不可用"
  fi

  printf "\n  关键包检查：\n"
  for P in rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros; do
    if [ -d "$ROS_DIR/share/$P" ] || [ -d "$ROS_DIR/include/$P" ]; then
      printf "    ${GRN}✓${RST} %s\n" "$P"
    else
      printf "    ${RED}✗${RST} %s\n" "$P"
    fi
  done

  # nav2_msgs 属于 navigation2，desktop 里通常有
  if [ -d "$ROS_DIR/share/nav2_msgs" ]; then
    printf "    ${GRN}✓${RST} nav2_msgs\n"
  else
    printf "    ${YEL}!${RST} nav2_msgs 缺失（本工作空间内有 navigation2 源码，可由源码提供）\n"
  fi
else
  bad "$ROS_DIR/setup.bash 不存在 —— 安装未完成"
fi

sec "6. 汇总与下一步"
printf "  ROS 2 Humble : %s\n" "$([ -f "$ROS_DIR/setup.bash" ] && echo 已安装 || echo 未安装)"
printf "\n  ${BOLD}每次开新终端都要：${RST}\n"
printf "    source /opt/ros/humble/setup.bash\n"
printf "\n  ${BOLD}下一步${RST}：搭建工作空间并编译（见 build_ws.sh）\n"
printf "    ★ 8GB 内存务必限制并行度，否则 colcon build 会 OOM\n\n"

exit 0
