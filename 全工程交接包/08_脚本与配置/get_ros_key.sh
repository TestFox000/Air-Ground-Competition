#!/usr/bin/env bash
# =============================================================================
# get_ros_key.sh —— 取 ROS GPG 公钥（keyserver.ubuntu.com 可达时最省事）
# =============================================================================
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/get_ros_key.sh"
#
# ★ 依据上一次诊断：
#     ✓ keyserver.ubuntu.com   HTTP 200  <- 可达！
#     ✗ GitHub raw             不可达
#     ✗ 各镜像的 ros.asc       不可达（镜像站不提供该文件）
#   所以走 **keyserver**，但要用对协议：
#     · `hkps://` 是 443/TLS，可能被拦；
#     · `hkp://`  是 80/明文，更容易通 —— 本脚本优先用它。
#
# ROS 2 仓库签名密钥指纹：
#     C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#   （报错里的 NO_PUBKEY F42ED6FBAB17C654 是它的**后 16 位**）
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
FPR="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
SHORT="F42ED6FBAB17C654"

printf "${BOLD}获取 ROS GPG 公钥${RST}\n"

CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
ARCH="$(dpkg --print-architecture)"

# ---------------------------------------------------------------- 已有？
sec "0. 现状"
if [ -s "$KEYRING" ] && sudo gpg --no-default-keyring --keyring "$KEYRING" \
      --list-keys 2>/dev/null | grep -q "$SHORT"; then
  ok "公钥已存在且包含 $SHORT"
  HAVE=1
else
  info "尚无 $SHORT 公钥"
  HAVE=0
fi

# ---------------------------------------------------------------- 多路取 key
if [ "$HAVE" != "1" ]; then
  sec "1. 从 keyserver 取公钥（按成功率排序）"

  # 顺序：明文 HKP(80) 最易通 -> HKP over https -> 其他 keyserver -> HTTP API
  TRIES=(
    "hkp://keyserver.ubuntu.com:80"
    "hkp://keyserver.ubuntu.com"
    "hkps://keyserver.ubuntu.com"
    "hkp://pgp.mit.edu:80"
    "hkps://keys.openpgp.org"
  )

  GOT=0
  for ks in "${TRIES[@]}"; do
    printf "  尝试 %s …\n" "$ks"
    # --recv-keys 用**完整指纹**更可靠
    if sudo gpg --no-default-keyring --keyring "$KEYRING" \
          --keyserver "$ks" --recv-keys "$FPR" >/dev/null 2>&1; then
      if sudo gpg --no-default-keyring --keyring "$KEYRING" \
            --list-keys 2>/dev/null | grep -q "$SHORT"; then
        ok "成功（$ks）"
        GOT=1
        break
      fi
    fi
    # 有些 keyserver 需要先 --keyserver-options
    if sudo gpg --no-default-keyring --keyring "$KEYRING" \
          --keyserver "$ks" --keyserver-options timeout=15 \
          --recv-keys "$FPR" >/dev/null 2>&1; then
      if sudo gpg --no-default-keyring --keyring "$KEYRING" \
            --list-keys 2>/dev/null | grep -q "$SHORT"; then
        ok "成功（$ks，带 timeout 选项）"
        GOT=1
        break
      fi
    fi
    info "失败"
  done

  # ---- 兜底：keyserver.ubuntu.com 的 HTTP API 直接返回公钥 ----
  if [ "$GOT" != "1" ]; then
    sec "2. 兜底：用 keyserver.ubuntu.com 的 HTTP API（纯 curl）"
    TMP="$(mktemp /tmp/roskey.XXXXXX.asc)"
    for url in \
      "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x${FPR}&options=mr" \
      "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x${SHORT}&options=mr" \
      "http://keyserver.ubuntu.com:11371/pks/lookup?op=get&search=0x${FPR}"
    do
      printf "  尝试 %s\n" "${url:0:78}…"
      if curl -fsSL --connect-timeout 8 --max-time 30 -o "$TMP" "$url" 2>/dev/null \
         && [ -s "$TMP" ] && grep -q 'BEGIN PGP' "$TMP"; then
        ok "下载到公钥（$(wc -c < "$TMP") 字节）"
        sudo gpg --dearmor --yes -o "$KEYRING" < "$TMP"
        GOT=1
        break
      fi
      info "失败"
    done
    rm -f "$TMP"
  fi

  if [ "$GOT" = "1" ]; then
    sudo chmod 644 "$KEYRING"
    HAVE=1
  fi
fi

# ---------------------------------------------------------------- 写源
sec "3. 配置 apt 源（有公钥就严格校验，没有才降级）"

if [ "$HAVE" = "1" ]; then
  echo "deb [arch=$ARCH signed-by=$KEYRING] $TUNA $CODENAME main" \
    | sudo tee "$SOURCES" >/dev/null
  ok "已用 signed-by 严格校验"
else
  warn "仍无公钥 —— 继续用 trusted=yes（不影响安装，只是少了签名校验）"
  echo "deb [arch=$ARCH trusted=yes] $TUNA $CODENAME main" \
    | sudo tee "$SOURCES" >/dev/null
fi
printf "     %s\n" "$(cat "$SOURCES")"

# ---------------------------------------------------------------- update
sec "4. apt update 验证"
UPD=$(sudo apt update 2>&1)
printf "%s\n" "$UPD" | tail -6
if printf "%s" "$UPD" | grep -q "NO_PUBKEY"; then
  warn "仍有 NO_PUBKEY 警告 —— 但源可用，安装不受影响"
elif [ "$HAVE" = "1" ]; then
  ok "无签名警告 —— 公钥已生效"
fi

printf "\n  可用性检查：\n"
CAND=$(apt-cache policy ros-humble-ros-base 2>/dev/null | awk '/Candidate:/{print $2}')
if [ -n "$CAND" ] && [ "$CAND" != "(none)" ]; then
  ok "ros-humble-ros-base 候选版本: $CAND"
else
  bad "apt 里看不到 ros-humble-* —— 源没配好"
fi

sec "汇总"
if [ "$HAVE" = "1" ]; then
  printf "  ${GRN}${BOLD}公钥已就绪${RST} —— 可以正式安装 ROS 2：\n"
else
  printf "  ${YEL}${BOLD}没有公钥，但源可用${RST} —— 也能装（trusted=yes），继续即可：\n"
fi
printf "    bash \"/mnt/c/Users/Ran/Desktop/无人车/src-car/install_ros2_humble_tuna.sh\"\n\n"
exit 0
