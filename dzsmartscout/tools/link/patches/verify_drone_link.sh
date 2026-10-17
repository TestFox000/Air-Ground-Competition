#!/bin/bash
# ==============================================================================
#  只读验证：无人机侧 dzlink.env + dz_send_waypoint.py 补丁
#  用法：bash verify_drone_link.sh
#  安全性：dry-run 不发包；第 4 步会真发 1 条 String 到 /nav/waypoint，
#          车侧栈没跑时无任何订阅者 → 不产生任何影响（这一步就是要看诊断行）。
# ==============================================================================
set -u
cd "$HOME/dzlink_tools" || { echo "❌ 进不去 ~/dzlink_tools"; exit 1; }

echo "=== 0) 链路环境文件 ==="
ls -l dzlink.env
grep -n 'export' dzlink.env
md5sum dzlink.env

echo
echo "=== 1) 补丁落点 ==="
grep -n 'domain_report\|DZLINK_ENV' dz_send_waypoint.py

echo
echo "=== 2) contract（第一行应该是域号）==="
python3 dz_send_waypoint.py contract | head -6

echo
echo "=== 3) route --dry-run（不发包）==="
python3 dz_send_waypoint.py route "N10 M10 L10 L9 K9" --dry-run

echo
echo "=== 4) 真实 rclpy 发送（车侧栈未跑 → 预期 订阅者匹配数=0 + 打印域号 + exit 3）==="
# ★ source ROS 环境必须在 set +u 下进行：/opt/ros/humble/setup.bash 第 8 行会展开
#   未定义变量 AMENT_TRACE_SETUP_FILES，与本脚本的 set -u 冲突（车侧 start_all.sh
#   的 135-137 行注释就是这个坑，本轮实测复现了）。source 完立刻恢复。
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
python3 dz_send_waypoint.py route "N10 M10" --wait-match 2 --repeat 1
echo "SEND_EXIT=$?"

echo
echo "=== 5) 反证：显式把域号压成 0，应该是【报警】而不是静默走错域 ==="
ROS_DOMAIN_ID=0 python3 dz_send_waypoint.py contract | head -2
