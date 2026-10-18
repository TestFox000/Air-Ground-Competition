#!/usr/bin/env bash
# =============================================================================
# diag_odom.sh —— 诊断"为什么车端收不到 odom"
# =============================================================================
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/diag_odom.sh"
#
# 需要在**离线测试正在跑**的时候执行（车端 + fake_sensors 都在跑）。
# =============================================================================

set +e
BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %s\n" "$1"; }
bad()  { printf "  ${RED}✗${RST} %s\n" "$1"; }
warn() { printf "  ${YEL}!${RST} %s\n" "$1"; }
info() { printf "  ${CYN}·${RST} %s\n" "$1"; }
sec()  { printf "\n${BOLD}=== %s ===\n${RST}" "$1"; }

source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/ws/install/setup.bash" 2>/dev/null

printf "${BOLD}诊断：车端为什么收不到 odom${RST}\n"

# ---------------------------------------------------------------- 1
sec "1. 话题列表（找 odom 相关）"
ros2 topic list 2>/dev/null | grep -iE 'odom|imu|offset' | sed 's/^/  /' || bad "没找到 odom/imu 话题"

# ---------------------------------------------------------------- 2
sec "2. 各话题的发布者/订阅者"
for t in /odom /imu_data /offset_center; do
  printf "  ${BOLD}%s${RST}\n" "$t"
  ros2 topic info "$t" -v 2>/dev/null | grep -E 'Type|Publisher count|Subscription count|node namespace|node name|Reliability|Durability' \
    | sed 's/^/      /' || info "查不到"
done

# ---------------------------------------------------------------- 3
sec "3. 话题频率（关键：odom 有没有在发）"
for t in /odom /imu_data /offset_center; do
  printf "  %-16s " "$t"
  timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' || printf "${RED}无数据${RST}\n"
done

# ---------------------------------------------------------------- 4
sec "4. /odom 的实际内容（看字段名对不对）"
timeout 8 ros2 topic echo /odom --once 2>/dev/null | head -30 | sed 's/^/  /' || bad "收不到 /odom"

# ---------------------------------------------------------------- 5
sec "5. 车端定位节点订阅的话题名"
ros2 param get /dzmission loc.odom_topic 2>/dev/null | sed 's/^/  /' || info "读不到参数"
ros2 param get /dzmission loc.imu_topic 2>/dev/null | sed 's/^/  /' || true
ros2 param get /dzmission loc.amcl_topic 2>/dev/null | sed 's/^/  /' || true

# ---------------------------------------------------------------- 6
sec "6. 定位健康标志（0=不健康 1=健康）"
timeout 6 ros2 topic echo /localization/healthy --once 2>/dev/null | sed 's/^/  /' || bad "收不到 /localization/healthy"

# ---------------------------------------------------------------- 7
sec "7. 过滤后里程（车端自己发的）"
timeout 6 ros2 topic hz /odometry/filtered 2>/dev/null | grep -m1 'average rate' \
  | sed 's/^/  /' || warn "/odometry/filtered 没有数据"

# ---------------------------------------------------------------- 8
sec "8. 节点列表"
ros2 node list 2>/dev/null | sed 's/^/  /'

# ---------------------------------------------------------------- 9
sec "9. ★ 关键：检查 /odom 的 QoS 是否匹配"
cat <<'EOF'
  车端订阅 /odom 用的是 **BEST_EFFORT**（Reliability: BEST_EFFORT）。
  fake_sensors.py 也用 BEST_EFFORT —— 理论上匹配。

  若第 2 节显示 Publisher 的 Reliability 是 RELIABLE，
  而 Subscription 是 BEST_EFFORT，则 **不匹配 → 静默丢弃**。

  兼容规则（DDS）：
    Publisher RELIABLE   + Sub BEST_EFFORT  -> 不兼容（丢弃）
    Publisher BEST_EFFORT + Sub BEST_EFFORT -> 兼容
    Publisher RELIABLE   + Sub RELIABLE     -> 兼容
EOF

sec "汇总"
printf "  把 ${BOLD}第 1~5 节${RST} 的输出贴回给助手。\n"
printf "  重点：\n"
printf "    · 第 3 节的 /odom 有没有频率（无 = 没在发）\n"
printf "    · 第 2 节的 Reliability 是否匹配\n"
printf "    · 第 5 节车端订阅的话题名是不是 odom\n\n"
exit 0
