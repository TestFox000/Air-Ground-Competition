#!/usr/bin/env bash
# =============================================================================
# fix_ros_key.sh —— 解决"取不到 ROS GPG 公钥"
# =============================================================================
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/fix_ros_key.sh"
#
# ★ 背景：
#   装 ROS 2 需要先给 apt 配好 ROS 仓库，而仓库要求 GPG 签名验证。
#   本机网络：
#     · github.com / raw.githubusercontent.com  不可达
#     · keyserver.ubuntu.com                    不可达
#   所以官方那套取公钥的办法全部失效。
#
# ★ 本脚本按顺序尝试 4 条路，哪条通用哪条：
#     路 1：从**清华镜像**直接下载公钥文件（镜像站常有）
#     路 2：从**中科大/阿里云**镜像下载
#     路 3：用 `trusted=yes` 跳过签名验证（最省事，仅适合信任清华源）
#     路 4：完全离线 —— 你手动准备 ros.asc，脚本负责导入
#
#   路 3 说明：`[trusted=yes]` 会让 apt 不再校验该仓库的签名。
#   因为仓库地址是清华镜像（我们主动信任它），且下载仍走 HTTPS，
#   对本用途是可接受的折中。若你更在意安全，请走路 1/2/4。
# =============================================================================

set +e
BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %s\n" "$1"; }
bad()  { printf "  ${RED}✗${RST} %s\n" "$1"; }
warn() { printf "  ${YEL}!${RST} %s\n" "$1"; }
info() { printf "  ${CYN}·${RST} %s\n" "$1"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
SOURCES="/etc/apt/sources.list.d/ros2.list"
TUNA="https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"

printf "${BOLD}修复 ROS GPG 公钥 / apt 源${RST}\n"

# ---------------------------------------------------------------- 版本
sec "0. 环境"
CODENAME=""
if [ -f /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"
fi
printf "  发行版代号 : %s\n" "${CODENAME:-未知}"
ARCH="$(dpkg --print-architecture)"
printf "  架构       : %s\n" "$ARCH"
if [ "$CODENAME" != "jammy" ]; then
  warn "本脚本预期 jammy(22.04)，当前是 ${CODENAME:-未知} —— 继续但请留意"
fi

# ---------------------------------------------------------------- 网络可达性诊断
sec "0.5 网络可达性诊断（关键：决定用哪条路）"

probe() {
  # $1 = 名称, $2 = URL
  local name="$1" url="$2" code
  code=$(curl -fsS -o /dev/null -w '%{http_code}' \
              --connect-timeout 6 --max-time 15 "$url" 2>/dev/null)
  if [ -n "$code" ] && [ "$code" != "000" ]; then
    printf "  ${GRN}✓${RST} %-34s HTTP %s\n" "$name" "$code"
    return 0
  else
    printf "  ${RED}✗${RST} %-34s 不可达\n" "$name"
    return 1
  fi
}

printf "  逐个探测（每个最多 15 秒）：\n"
TUNA_OK=0
probe "清华 TUNA (ros2 仓库)" "https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/dists/jammy/Release" && TUNA_OK=1
probe "清华 TUNA (主页)"       "https://mirrors.tuna.tsinghua.edu.cn/" || true
probe "中科大 USTC"            "https://mirrors.ustc.edu.cn/ros2/ubuntu/dists/jammy/Release" || true
probe "阿里云"                 "https://mirrors.aliyun.com/ros2/ubuntu/dists/jammy/Release" || true
probe "GitHub raw"             "https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc" || true
probe "keyserver.ubuntu.com"   "https://keyserver.ubuntu.com/" || true

printf "\n"
if [ "$TUNA_OK" = "1" ]; then
  ok "清华 TUNA 可达 —— 后续用镜像源，不需要公钥也能装（trusted=yes 兜底）"
else
  warn "清华 TUNA **不可达** —— 需要换镜像站或走离线方案"
  printf "    请把上面这份探测结果贴回给助手，我据此换方案。\n"
fi

# ---------------------------------------------------------------- 路 1/2：下载公钥
sec "1-2. 尝试从镜像下载公钥"

# 候选 URL（清华 / 中科大 / 阿里云 / Ubuntu keyserver 的 https 形式）
CANDIDATES=(
  "https://mirrors.tuna.tsinghua.edu.cn/ros/ros.asc"
  "https://mirrors.tuna.tsinghua.edu.cn/ros2/ros.asc"
  "https://mirrors.ustc.edu.cn/ros/ros.asc"
  "https://mirrors.aliyun.com/ros/ros.asc"
  "https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc"
)
KEY_GOT=0
TMPKEY="$(mktemp /tmp/roskey.XXXXXX.asc)"

for u in "${CANDIDATES[@]}"; do
  printf "  尝试 %s\n" "$u"
  if curl -fsSL --connect-timeout 8 --max-time 25 -o "$TMPKEY" "$u" 2>/dev/null; then
    if [ -s "$TMPKEY" ] && head -c 40 "$TMPKEY" | grep -q 'BEGIN PGP'; then
      ok "下载成功（$(wc -c < "$TMPKEY") 字节）"
      KEY_GOT=1
      break
    else
      warn "内容不像 PGP 公钥，跳过"
    fi
  else
    info "不可达"
  fi
done

if [ "$KEY_GOT" = "1" ]; then
  sudo gpg --dearmor --yes -o "$KEYRING" < "$TMPKEY" 2>/dev/null
  sudo chmod 644 "$KEYRING"
  if [ -s "$KEYRING" ]; then
    ok "公钥已导入 $KEYRING"
  else
    bad "导入失败"
    KEY_GOT=0
  fi
fi

# ---------------------------------------------------------------- 手动兜底检查
if [ "$KEY_GOT" != "1" ]; then
  printf "\n"
  warn "镜像都没取到公钥。检查你是否已手动放了文件："
  for cand in /mnt/c/Users/Ran/ros.asc "$HOME/ros.asc" /tmp/ros.asc; do
    if [ -f "$cand" ]; then
      ok "找到 $cand"
      sudo gpg --dearmor --yes -o "$KEYRING" < "$cand"
      KEY_GOT=1
      break
    else
      info "没有 $cand"
    fi
  done
fi

# ---------------------------------------------------------------- 路 3：trusted=yes
sec "3. 配置 apt 源"

if [ "$KEY_GOT" = "1" ]; then
  echo "deb [arch=$ARCH signed-by=$KEYRING] $TUNA $CODENAME main" \
    | sudo tee "$SOURCES" >/dev/null
  ok "已写入（signed-by 校验）: $SOURCES"
  printf "     %s\n" "$(cat "$SOURCES")"
else
  warn "无公钥 —— 改用 trusted=yes（跳过签名校验）"
  printf "  ${CYN}说明${RST}：仓库地址是清华镜像、走 HTTPS，我们主动信任它。\n"
  printf "        这不如签名校验严格，但能让安装继续。\n\n"
  echo "deb [arch=$ARCH trusted=yes] $TUNA $CODENAME main" \
    | sudo tee "$SOURCES" >/dev/null
  ok "已写入（trusted=yes）: $SOURCES"
  printf "     %s\n" "$(cat "$SOURCES")"
fi

# ---------------------------------------------------------------- 4. apt update
sec "4. apt update"

if sudo apt update 2>&1 | tail -8; then
  ok "apt update 完成"
else
  bad "apt update 失败 —— 见上面报错"
fi

printf "\n  能否看到 ROS 2 包：\n"
if apt-cache policy ros-humble-ros-base 2>/dev/null | grep -q Candidate; then
  CAND=$(apt-cache policy ros-humble-ros-base 2>/dev/null | awk '/Candidate:/{print $2}')
  ok "ros-humble-ros-base 候选版本: $CAND"
  printf "\n  ${GRN}${BOLD}源已可用 —— 继续装 ROS 2：${RST}\n"
  printf "    bash \"/mnt/c/Users/Ran/Desktop/无人车/src-car/install_ros2_humble_tuna.sh\"\n\n"
else
  bad "apt 里仍看不到 ros-humble-*"
  printf "\n  ${BOLD}手动方案（离线导入公钥）${RST}\n"
  printf "  1) 在任何能上网的机器上打开并保存为 ros.asc：\n"
  printf "       https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc\n"
  printf "     或从已装 ROS 2 的机器上导出：\n"
  printf "       gpg --export --armor C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 > ros.asc\n"
  printf "  2) 放到 Windows 的 C:\\\\Users\\\\Ran\\\\ros.asc\n"
  printf "  3) 重跑本脚本（它会自动找到并导入）\n\n"
  printf "  也可以直接告诉我：你的网络能不能访问 **mirrors.tuna.tsinghua.edu.cn**？\n"
  printf "  若连清华镜像也不通，我们得换别的路子（例如离线 deb 或换镜像站）。\n\n"
fi

rm -f "$TMPKEY"
exit 0
