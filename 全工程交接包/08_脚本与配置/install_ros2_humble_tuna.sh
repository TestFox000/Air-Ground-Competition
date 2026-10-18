#!/usr/bin/env bash
# =============================================================================
# install_ros2_humble_tuna.sh
#   —— 用**清华 TUNA 镜像**安装 ROS 2 Humble（不依赖 GitHub）
# =============================================================================
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/install_ros2_humble_tuna.sh"
#
# ★ 为什么要用镜像：
#   官方安装步骤要从 **github.com/ros-infrastructure/ros-apt-source** 下载
#   源配置包，而本机网络**访问不了 GitHub**（rosdep update 也是因
#   raw.githubusercontent.com 不可达才失败）。清华镜像把 ROS 2 的
#   apt 仓库完整同步了，且在国内可达。
#
# ★ 前提：必须是 Ubuntu 22.04 (jammy)
#   Humble 只对 22.04 发布 deb 包；24.04 要用 Jazzy（那是另一套 API）。
#   本脚本会先检查版本，不符就停下并说明。
#
# 装完后：source /opt/ros/humble/setup.bash
# =============================================================================

set +e
BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %s\n" "$1"; }
bad()  { printf "  ${RED}✗${RST} %s\n" "$1"; }
warn() { printf "  ${YEL}!${RST} %s\n" "$1"; }
info() { printf "  ${CYN}·${RST} %s\n" "$1"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

printf "${BOLD}ROS 2 Humble 安装（清华 TUNA 镜像）${RST}\n"

# ---------------------------------------------------------------- 0. 版本检查
sec "0. 系统版本检查"

CODENAME=""
if [ -f /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"
fi
printf "  发行版 : %s\n" "${PRETTY_NAME:-未知}"
printf "  代号   : %s\n" "${CODENAME:-未知}"

if [ "$CODENAME" = "jammy" ]; then
  ok "Ubuntu 22.04 (jammy) —— 可以安装 Humble"
elif [ "$CODENAME" = "noble" ]; then
  bad "这是 Ubuntu 24.04 (noble)"
  printf "\n  ${BOLD}ROS 2 Humble 的 deb 包只对 22.04 发布。${RST}\n"
  printf "  24.04 对应的发行版是 ${BOLD}Jazzy${RST}，但本工程是按 Humble 写的，\n"
  printf "  换 Jazzy 需要改不少 API，${BOLD}不推荐${RST}。\n\n"
  printf "  建议：重装/切到 Ubuntu 22.04。WSL 下可以：\n"
  printf "    wsl --install -d Ubuntu-22.04\n"
  printf "  然后在这个 22.04 里跑本脚本。\n\n"
  exit 1
else
  bad "未识别的发行版代号：${CODENAME:-空}"
  printf "  本脚本只支持 Ubuntu 22.04 (jammy)。\n"
  exit 1
fi

# ---------------------------------------------------------------- 1. 基础工具
sec "1. 基础工具"

printf "  安装 locales curl gnupg lsb-release software-properties-common …\n"
if sudo apt update -qq && sudo apt install -y -qq \
      locales curl gnupg lsb-release software-properties-common ca-certificates; then
  ok "基础工具就绪"
else
  bad "基础工具安装失败（先解决 apt 源问题）"
  exit 1
fi

# locale 必须是 UTF-8，否则 ros2 CLI 会有编码问题
sudo locale-gen en_US en_US.UTF-8 >/dev/null 2>&1
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 >/dev/null 2>&1
ok "locale 已设为 en_US.UTF-8"

# ---------------------------------------------------------------- 2. 加 ROS 2 源
sec "2. 添加 ROS 2 apt 源（清华镜像）"

MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"
KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
SOURCES="/etc/apt/sources.list.d/ros2.list"

# ---- 2.1 GPG 公钥 ----
# ★ 官方步骤从 github.com/ros/rosdistro 取 key，本机不可达。
#   改为从 Ubuntu keyserver 取（同一把密钥，指纹 0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654）。
KEY_OK=0
if [ -f "$KEYRING" ] && gpg --list-keys --with-colons 2>/dev/null | grep -q "ros"; then
  ok "GPG 公钥已存在"
  KEY_OK=1
else
  printf "  取 ROS GPG 公钥（keyserver.ubuntu.com）…\n"
  if sudo gpg --no-default-keyring --keyring "$KEYRING" \
        --keyserver keyserver.ubuntu.com \
        --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 >/dev/null 2>&1; then
    ok "公钥已导入（keyserver.ubuntu.com）"
    KEY_OK=1
  else
    warn "keyserver.ubuntu.com 失败，改用清华的 keyserver 镜像"
    if sudo gpg --no-default-keyring --keyring "$KEYRING" \
          --keyserver hkps://keyserver.ubuntu.com \
          --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 >/dev/null 2>&1; then
      ok "公钥已导入（hkps）"
      KEY_OK=1
    fi
  fi
fi

if [ "$KEY_OK" != "1" ]; then
  bad "无法获取 ROS GPG 公钥"
  printf "\n  ${BOLD}手动方案${RST}：把公钥文件拷进 WSL 再导入。\n"
  printf "  在**能上网的机器**上下载：\n"
  printf "    https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc\n"
  printf "  存到 C:\\\\Users\\\\Ran\\\\ros.asc，然后在 Ubuntu 里：\n"
  printf "    sudo gpg --no-default-keyring --keyring %s --import /mnt/c/Users/Ran/ros.asc\n" "$KEYRING"
  printf "  再重跑本脚本。\n\n"
fi

# ---- 2.2 写 sources.list ----
if [ "$KEY_OK" = "1" ]; then
  printf "  写入 %s\n" "$SOURCES"
  echo "deb [arch=$(dpkg --print-architecture) signed-by=$KEYRING] $MIRROR $CODENAME main" \
    | sudo tee "$SOURCES" >/dev/null && ok "apt 源已配置（$MIRROR）"

  printf "  更新 apt 索引…\n"
  if sudo apt update -qq 2>&1 | tail -3; then
    ok "apt 索引已更新"
  else
    warn "apt update 有警告，继续尝试"
  fi
fi

# ---------------------------------------------------------------- 3. 装 ROS 2
sec "3. 安装 ROS 2 Humble"

if [ -d /opt/ros/humble ]; then
  ok "/opt/ros/humble 已存在 —— 跳过重复安装"
else
  printf "  尝试 ros-humble-desktop（含 rviz/gazebo，约 1.5GB）…\n"
  if sudo apt install -y ros-humble-desktop; then
    ok "ros-humble-desktop 安装完成"
  else
    warn "desktop 失败，改用 ros-humble-ros-base（更小，够本工程用）"
    if sudo apt install -y ros-humble-ros-base; then
      ok "ros-humble-ros-base 安装完成"
    else
      bad "ROS 2 安装失败"
      printf "\n  请把上面的 apt 报错贴回给助手。\n\n"
      exit 1
    fi
  fi
fi

# ---------------------------------------------------------------- 4. 本工程需要的额外依赖
sec "4. 本工程额外依赖"

# ★ 说明：
#   · serial  —— ROS 2 没有 apt 包（wjwwood/serial 是 ROS 1 的）。
#                本工程已在 src-car/serial/ 下**自建**了一个接口兼容的实现，
#                由 build_ws.sh 一起编译，**不需要 apt 装**。
#   · nav2_msgs —— dzmission 的 CMakeLists 有 find_package(nav2_msgs REQUIRED)
EXTRA=(
  python3-colcon-common-extensions
  python3-rosdep
  ros-humble-nav2-msgs
  ros-humble-tf2
  ros-humble-tf2-ros
  ros-humble-tf2-geometry-msgs
  ros-humble-visualization-msgs
  ros-humble-nav-msgs
  ros-humble-sensor-msgs
  ros-humble-geometry-msgs
  ros-humble-std-msgs
  ros-humble-rclcpp-action
  build-essential
  cmake
  git
)

printf "  逐个安装（缺哪个装哪个）…\n"
FAILED=()
for p in "${EXTRA[@]}"; do
  if dpkg -s "$p" >/dev/null 2>&1; then
    info "$p 已装"
  elif sudo apt install -y "$p" >/dev/null 2>&1; then
    ok "$p"
  else
    bad "$p 装不上"
    FAILED+=("$p")
  fi
done

if [ ${#FAILED[@]} -gt 0 ]; then
  printf "\n  ${YEL}以下没装上（可能不影响编译，后面再看）：${RST}\n"
  for f in "${FAILED[@]}"; do printf "    - %s\n" "$f"; done
fi

# ---------------------------------------------------------------- 5. rosdep
sec "5. rosdep（★ 本机可跳过）"

printf "  ${CYN}说明${RST}：官方要 'sudo rosdep init && rosdep update'，\n"
printf "  但它们会去 raw.githubusercontent.com 取索引 —— **本机不可达**。\n"
printf "  本工程的依赖已用上面的 apt 列表覆盖，**不需要 rosdep**。\n\n"

if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  info "rosdep 已初始化过"
else
  warn "跳过 rosdep 初始化（不影响 build_ws.sh）"
fi

# ---------------------------------------------------------------- 6. 写 .bashrc
sec "6. 配置 ~/.bashrc"

MARK="# ---- ROS 2 Humble (added by install_ros2_humble_tuna.sh) ----"
if grep -qF "$MARK" "$HOME/.bashrc" 2>/dev/null; then
  ok ".bashrc 已配置过"
else
  {
    echo ""
    echo "$MARK"
    echo 'source /opt/ros/humble/setup.bash'
    echo 'if [ -f "$HOME/ws/install/setup.bash" ]; then'
    echo '  source "$HOME/ws/install/setup.bash"'
    echo 'fi'
    echo "# ---- end ROS 2 ----"
  } >> "$HOME/.bashrc"
  ok "已写入 ~/.bashrc（新终端自动 source）"
fi

# ---------------------------------------------------------------- 7. 验证
sec "7. 验证"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  printf "  ROS_DISTRO      : %s\n" "${ROS_DISTRO:-未设置}"
  if command -v ros2 >/dev/null 2>&1; then
    ok "ros2 命令可用: $(command -v ros2)"
  else
    bad "ros2 命令仍不可用"
  fi
  if command -v colcon >/dev/null 2>&1; then
    ok "colcon 可用: $(command -v colcon)"
  else
    bad "colcon 不可用"
  fi
  if dpkg -s ros-humble-nav2-msgs >/dev/null 2>&1; then
    ok "nav2_msgs 已装（dzmission 编译需要）"
  else
    warn "nav2_msgs 未装 —— dzmission 的 find_package(nav2_msgs) 会失败"
  fi
else
  bad "/opt/ros/humble 不存在 —— 安装未成功"
fi

sec "汇总"
printf "  ${BOLD}下一步：${RST}\n"
printf "    1) 开新终端（或 source ~/.bashrc）\n"
printf "    2) 编译本工程：\n"
printf "       bash \"/mnt/c/Users/Ran/Desktop/无人车/src-car/build_ws.sh\"\n"
printf "    3) 启动：\n"
printf "       ros2 launch dzmission bringup_mission.launch.py\n\n"
printf "  ${CYN}提示${RST}：serial 库由本工程自带（src-car/serial/），无需 apt 安装。\n\n"
exit 0
