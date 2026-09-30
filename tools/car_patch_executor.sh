#!/usr/bin/env bash
# 生成 path_executor 的**补丁副本**，用于调速度与前视距离 —— 绝不改官方源码
#
# 为什么必须绕开官方源码：
#   1) 车是公用的，`~/dzrobot` 全程只读（看板 §0.1），改了别人就跟着遭殃
#   2) 这两个量是**硬编码**，不是 ROS 参数，没法用 `-p` 传：
#        self.lookahead_dist = 0.15   # 前视距离，只有 15 cm
#        self.linear_speed   = 0.3    # m/s → 底盘字节 21/255
#   前视距离太短是"车原地转圈"的直接原因（pure pursuit 增益过大、持续打方向）；
#   速度太低则让车推不动（180 s 跑不完 13 段）。
#
# 做法：把官方 path_executor.py + map_utils.py 复制到 /tmp/ag_ws/patched，
#       只改那两行，然后让启动脚本优先用这份。**官方文件一个字节都不动。**
#
# 用法：
#   python tools/remote_run.py --host <IP> --user duzhong \
#       --script tools/car_patch_executor.sh --env AG_LINEAR_SPEED=0.6 --env AG_LOOKAHEAD=0.40
set -o pipefail

# 默认对齐官方 dztrack.launch.py 的 "pursuitSpeed": 0.8
# （官方发布包 dzrobot-bisai-20260617/src/dztrack/launch/dztrack.launch.py）
SPEED="${AG_LINEAR_SPEED:-0.8}"
LOOK="${AG_LOOKAHEAD:-0.40}"
SRC="$HOME/dzrobot/src/car_localization/car_localization"
DST="${AG_PATCH_DIR:-/tmp/ag_ws/patched}/car_localization"

echo "==================== 生成执行器补丁副本（不动官方源码） ===================="
echo "  linear_speed   : ${SPEED} m/s  （底盘字节约 $(awk -v s=$SPEED 'BEGIN{printf "%.0f", s*23/0.33}')/255）"
echo "  lookahead_dist : ${LOOK} m"
echo

if [ ! -f "$SRC/path_executor.py" ]; then
  echo "  [FAIL] 找不到官方 $SRC/path_executor.py"; exit 1
fi

mkdir -p "$DST" || { echo "  [FAIL] 建不了 $DST"; exit 1; }
cp -f "$SRC/path_executor.py" "$DST/path_executor.py" || exit 1
cp -f "$SRC/map_utils.py"     "$DST/map_utils.py"     2>/dev/null || echo "  [!] 没有 map_utils.py（后面会 import 失败）"
if [ -f "$SRC/__init__.py" ]; then cp -f "$SRC/__init__.py" "$DST/__init__.py"; else : > "$DST/__init__.py"; fi

# 只替换那两行（行首 8 个空格，类属性赋值）
sed -i "s|^        self\.lookahead_dist = .*|        self.lookahead_dist = ${LOOK}   # patched: 官方 0.15 太短，会导致转圈|" "$DST/path_executor.py"
sed -i "s|^        self\.linear_speed = .*|        self.linear_speed = ${SPEED}   # patched: 官方 0.3 太慢，跑不完路线|" "$DST/path_executor.py"

echo "  -- 替换结果（应看到 patched 注释）--"
grep -n "self.lookahead_dist = \|self.linear_speed = " "$DST/path_executor.py" | head -4 | sed 's/^/    /'
echo
echo "  -- 校验：补丁副本必须能导入 map_utils（否则一启动就崩）--"
PYTHONPATH="$(dirname "$DST"):${PYTHONPATH}" python3 -c "import car_localization.map_utils; print('    [OK] map_utils 可导入')" 2>&1 | sed 's/^/  /'
echo
echo "  [OK] 补丁在 $DST"
echo "  启动脚本会优先用它；想回到官方版本，删掉这个目录即可。"
echo "==================== 结束 ===================="
