#!/usr/bin/env bash
# 查清「可执行入口到底装到哪、ros2 run 能不能发现」——只列目录与索引，不启动节点。
# 背景：car_verify.sh 里 `ros2 pkg executables` 返回空，但如果 ros2 run 也发现不了，
#       launch 会在启动瞬间失败（这是只有真机才会暴露的问题）。

WS="${AG_WS:-/tmp/ag_ws}"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null

echo "==================== 入口安装位置排查 ===================="
echo

echo "## 1. install/ag_car 目录结构"
find "$WS/install/ag_car" -maxdepth 3 -type d | sed 's/^/  /'
echo "-- 所有名为 car_node 的文件 --"
find "$WS/install/ag_car" -name "car_node*" | sed 's/^/  /'
echo

echo "## 2. 家目录里可执行文件在哪一层（对比：官方包 dzjudgment）"
find "$HOME/dzrobot/install" -maxdepth 3 -type d -name "dzjudgment" 2>/dev/null | head -3 | sed 's/^/  /'
ls "$HOME/dzrobot/install/dzjudgment/lib/dzjudgment" 2>/dev/null | sed 's/^/    lib: /'
ls "$HOME/dzrobot/install/dzjudgment/bin" 2>/dev/null | sed 's/^/    bin: /'
echo

echo "## 3. ament 索引能查到的可执行文件（ros2 run 用的就是这个）"
python3 - <<'PY'
from ament_index_python.packages import get_package_prefix
import os
for pkg in ("ag_car", "ag_mission", "dzjudgment"):
    try:
        prefix = get_package_prefix(pkg)
    except Exception as e:
        print(f"  {pkg}: 未注册 ({e})")
        continue
    print(f"  {pkg} prefix = {prefix}")
    for sub in ("lib", "bin"):
        d = os.path.join(prefix, "lib", pkg if sub == "lib" else "")
        d2 = os.path.join(prefix, "bin") if sub == "bin" else d
        for cand in {d, d2}:
            if os.path.isdir(cand):
                files = sorted(f for f in os.listdir(cand) if os.access(os.path.join(cand, f), os.X_OK))
                if files:
                    print(f"      {cand}: {files}")
PY
echo

echo "## 4. ros2 run 的实际行为（--help，不启动节点）"
for pair in "ag_car car_node" "ag_mission mission_node" "ag_bridge fake_intel_node"; do
  set -- $pair
  out=$(timeout 20 ros2 pkg executables "$1" 2>&1 | head -3)
  echo "  ros2 pkg executables $1 -> '${out:-<空>}'"
done
echo

echo "## 5. setuptools 的 entry_points 是否真的生成了脚本"
grep -n "console_scripts" -A4 "$WS/src/ag_car/setup.py" | sed 's/^/  /'
find "$WS/build/ag_car" -name "car_node*" 2>/dev/null | head -3 | sed 's/^/  built: /'
echo

echo "==================== 结束 ===================="
