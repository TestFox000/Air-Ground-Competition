#!/usr/bin/env bash
# 精确定位 FAST-LIO 定位链路 python 依赖问题（只做 import 测试与日志取证，不启动任何节点）
export LC_ALL=C
SRC="$HOME/dzpx4/src/FAST_LIO_LOCALIZATION_ROS2/scripts"

echo "########## [1] source ROS 之后的 import 测试 ##########"
source /opt/ros/humble/setup.bash >/dev/null 2>&1
echo "   ROS_DISTRO=$ROS_DISTRO  python3=$(command -v python3)"
for m in rclpy numpy open3d tf_transformations; do
  out=$(python3 -c "import $m;print(m if False else getattr($m,'__version__','?'))" 2>&1 | tail -1)
  printf "   %-20s -> %s\n" "$m" "$out"
done
echo "   --- transform_fusion.py 的兜底 import 路径 ---"
python3 -c "from tf2_ros import transformations; print('tf2_ros.transformations OK')" 2>&1 | tail -1

echo
echo "########## [2] 不 source ROS 的对照 ##########"
for m in open3d tf_transformations; do
  out=$(/usr/bin/python3 -c "import $m" 2>&1 | tail -1)
  printf "   %-20s -> %s\n" "$m" "$out"
done

echo
echo "########## [3] 三个 python 脚本的 import 期依赖（静态判定谁会立刻崩） ##########"
for s in global_map_publisher global_localization transform_fusion publish_initial_pose; do
  echo "   --- $s.py ---"
  grep -nE "^(import|from) " "$SRC/$s.py" | sed 's/^/      /'
done

echo
echo "########## [4] 日志取证：这几个节点历史上有没有真的起来过 ##########"
echo "   --- 顶层节点日志文件（按节点名聚合） ---"
ls "$HOME"/.ros/log/*.log 2>/dev/null | sed -e 's#.*/##' -e 's/_[0-9]*_[0-9]*\.log$//' | sort | uniq -c | sort -rn | sed 's/^/     /'
echo "   --- 有没有任何日志提到 open3d / ModuleNotFoundError ---"
grep -rl "ModuleNotFoundError" "$HOME/.ros/log" 2>/dev/null | head -5 | sed 's/^/     /'
grep -rl "open3d" "$HOME/.ros/log" 2>/dev/null | head -5 | sed 's/^/     /'
echo "   --- 2026-09-26 那次启动 fast_lio_localization 的 launch.log 内容 ---"
f=$(grep -rl "global_localization" "$HOME/.ros/log" 2>/dev/null | head -1)
echo "     文件: $f"
[ -n "$f" ] && sed -n '1,40p' "$f" | sed 's/^/       /'

echo
echo "########## [5] 那条 launch.log 所在 run 目录的全部内容 ##########"
d=$(dirname "$f" 2>/dev/null)
[ -n "$d" ] && ls -la "$d" | sed 's/^/     /'

echo
echo "取证结束"
