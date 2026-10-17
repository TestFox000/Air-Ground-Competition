#!/usr/bin/env bash
# =============================================================================
#  语音试音工具   test_voice.sh
# -----------------------------------------------------------------------------
#  用途：把底盘语音模块的 ID 逐个试一遍，挑一句合适的当"任务完成提示音"，
#        挑好后填到  config/robot.yaml  的  finish_voice_id。
#
#  为什么需要它：底盘代码里只出现过 2 个 ID（1 = 切换自主模式，15 = cmd_vel
#  相关的档位播报），其余 ID 对应哪句话没有任何实机依据。与其瞎猜一个填进去，
#  不如花 1 分钟在车上试出来。
#
#  用法:
#      ./test_voice.sh 1 2 3 15 20      # 依次试这几个 ID
#      ./test_voice.sh sweep            # 从 1 试到 30（默认）
#      ./test_voice.sh sweep 1 20       # 从 1 试到 20
#
#  ★ 底盘有闸门：同一个 ID 在 2 秒内不会重播，所以本脚本每次间隔 3 秒。
#  ★ 播报需要底盘(dzactuator)在跑 —— 先  ./start_all.sh start  或单独起底盘。
# =============================================================================
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOPIC="/voice_switch"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

if ! timeout 8 ros2 node list 2>/dev/null | grep -q dzactuator; then
  echo "⚠️  底盘节点 dzactuator 不在线 —— 语音播不出来。"
  echo "    先执行:  ~/dzrobot/start_all.sh start"
  exit 1
fi

# 解析要试的 ID 列表
IDS=()
if [ $# -eq 0 ] || [ "${1:-}" = "sweep" ]; then
  FROM="${2:-1}"; TO="${3:-30}"
  for i in $(seq "$FROM" "$TO"); do IDS+=("$i"); done
else
  IDS=("$@")
fi

echo ""
echo "=============================================================="
echo "  语音试音  ——  共 ${#IDS[@]} 个 ID，每个间隔 3 秒"
echo "  话题: $TOPIC    (Ctrl+C 可随时中断)"
echo "=============================================================="
echo ""
echo "  提示：听哪一句适合当"跑完全程"的提示音，记下它的 ID。"
echo ""

i=0
for id in "${IDS[@]}"; do
  i=$((i + 1))
  printf "  [%2d/%2d]  播放 ID = %-3s ...\n" "$i" "${#IDS[@]}" "$id"
  ros2 topic pub --once "$TOPIC" std_msgs/msg/UInt8 "{data: $id}" >/dev/null 2>&1
  sleep 3
done

echo ""
echo "=============================================================="
echo "  试音结束。满意的 ID 填到:"
echo "      $WS/config/robot.yaml   ->   path_executor -> finish_voice_id"
echo ""
echo "  改完不用重启整套，只重启循迹即可:"
echo "      $WS/tools/restart_node.sh path"
echo "=============================================================="
echo ""
