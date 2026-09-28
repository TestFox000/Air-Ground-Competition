#!/usr/bin/env bash
# 官方栈依赖体检（只读）：查 `dzactuator bringup.launch.py` 到底需要哪些包、哪些没装。
#
# 背景（2026-09-28）：真发车前体检发现 `bringup.launch.py` 起不来：
#     PackageNotFoundError: package 'robot_localization' not found
#   → 底盘节点全没起来 → 没有 /imu_data → 定位节点收不到 IMU → /odom/localization 永远无数据
# 这就是每一轮"读不到定位"的真因：官方 install/ 是**残缺**的，缺的包不止一个。
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
hr() { echo "----------------------------------------------------"; }

echo "==================== 官方栈依赖体检（只读） ===================="
date
echo

hr; echo "## 1. bringup.launch.py 里引用到的包"
LP="$HOME/dzrobot/src/dzactuator/launch/bringup.launch.py"
if [ -f "$LP" ]; then
  # 三种常见写法都抓：package='x' / get_package_share_directory('x') / 'x', 'launch/..'
  grep -oE "package=['\"][a-zA-Z0-9_]+['\"]|get_package_share_directory\(['\"][a-zA-Z0-9_]+['\"]\)" "$LP" \
    | grep -oE "[a-zA-Z0-9_]+['\"]\)?$" | tr -d "'\")" | sort -u | sed 's/^/    /'
else
  echo "    [FAIL] 找不到 $LP"
fi
echo "  -- launch 文件开头 20 行（肉眼看依赖）--"
head -20 "$LP" 2>/dev/null | sed 's/^/    /'
echo

hr; echo "## 2. 官方工作区现状"
echo "  -- ~/dzrobot/src 顶层 --"
ls "$HOME/dzrobot/src" 2>/dev/null | sed 's/^/    /'
echo "  -- ~/dzrobot/install 顶层（已装好的）--"
ls "$HOME/dzrobot/install" 2>/dev/null | sed 's/^/    /'
echo "  -- ~/install 顶层（另一套工作区）--"
ls "$HOME/install" 2>/dev/null | sed 's/^/    /'
echo

hr; echo "## 3. 有没有别的 launch 也引用同一个包（判断影响面）"
grep -rl "robot_localization" "$HOME/dzrobot/src"/*/launch/*.py 2>/dev/null | sed 's/^/    /' \
  || echo "    （没找到其它引用）"
echo

hr; echo "## 4. 结论"
echo "  上面第 1 节列出的包，凡是不在「install 顶层」里的，就是**缺的**。"
echo "  缺什么就在官方工作区里单独编什么（不要整仓 colcon build，太慢也容易踩重复包名）："
echo "    colcon build --base-paths src --packages-select <包名> [<包名>...]"
echo "  编完再跑 tools/car_preflight.sh 复验（重点看 /odom/localization 有没有数据）。"
echo "==================== 结束 ===================="
