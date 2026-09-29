#!/usr/bin/env bash
# 查 `path_executor.py` 该怎么起（实车不动的真因排查，只读）
#
# 现象：实跑时没人执行 /nav/waypoint，车不动。
# 真因：脚本里 `python3 .../path_executor.py` 直接跑源码，启动时就崩了 ——
#       ModuleNotFoundError: No module named 'car_localization.map_utils'
# 本脚本把"模块在哪 / 官方 launch 怎么起 / 有没有现成可执行入口"一次查清。
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
hr() { echo "----------------------------------------------------"; }

echo "==================== path_executor 启动方式排查 ===================="
date
echo

hr; echo "## 1. 源码目录里有什么"
ls -l "$HOME/dzrobot/src/car_localization" 2>/dev/null | sed 's/^/    /'
echo "  -- car_localization/ 包内 --"
ls -l "$HOME/dzrobot/src/car_localization/car_localization" 2>/dev/null | sed 's/^/    /'
echo

hr; echo "## 2. map_utils 到底在不在"
find "$HOME/dzrobot/src" "$HOME/install" "$HOME/dzrobot/install" \
  -name "map_utils*" 2>/dev/null | sed 's/^/    /' || true
echo "  （上面没有就说明源码里有但没被安装，或压根不存在）"
echo

hr; echo "## 3. 已安装的 car_localization 包里有什么"
for d in "$HOME/install/car_localization" "$HOME/dzrobot/install/car_localization"; do
  [ -d "$d" ] || continue
  echo "  -- $d --"
  find "$d" -name "*.py" 2>/dev/null | head -12 | sed 's/^/    /'
done
echo

hr; echo "## 4. 官方 launch 文件列表 + 谁引用了 path_executor"
ls "$HOME/dzrobot/src/car_localization/launch" 2>/dev/null | sed 's/^/    /'
echo "  -- 引用 path_executor 的 launch --"
grep -rl "path_executor" "$HOME/dzrobot/src/car_localization/launch" 2>/dev/null | sed 's/^/    /' || echo "    （launch 里没引用）"
echo "  -- 那些 launch 里的节点定义 --"
grep -rn -A3 "path_executor" "$HOME/dzrobot/src/car_localization/launch" 2>/dev/null | head -20 | sed 's/^/    /'
echo

hr; echo "## 5. 有没有现成的可执行入口"
ros2 pkg executables car_localization 2>/dev/null | sed 's/^/    /' || echo "    （查不到）"
echo

hr; echo "## 6. 直接试两种起法，看哪种能活 5 秒"
echo "  A) PYTHONPATH 带上源码目录再跑源码："
PYTHONPATH="$HOME/dzrobot/src/car_localization:${PYTHONPATH}" \
  timeout 6 python3 "$HOME/dzrobot/src/car_localization/car_localization/path_executor.py" 2>&1 \
  | tail -6 | sed 's/^/      /'
echo
echo "  B) 不设 PYTHONPATH 直接跑："
timeout 6 python3 "$HOME/dzrobot/src/car_localization/car_localization/path_executor.py" 2>&1 \
  | tail -4 | sed 's/^/      /'
echo
echo "==================== 结束 ===================="
