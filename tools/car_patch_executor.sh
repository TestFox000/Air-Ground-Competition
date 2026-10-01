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
# 必须 source ROS：否则末尾的导入校验会因为找不到 rclpy 而**误报失败**
# （serial_reader 依赖 rclpy，而 rclpy 只在 ROS 环境里）
source /opt/ros/humble/setup.bash 2>/dev/null

# 默认对齐官方 dztrack.launch.py 的 "pursuitSpeed": 0.8
# （官方发布包 dzrobot-bisai-20260617/src/dztrack/launch/dztrack.launch.py）
SPEED="${AG_LINEAR_SPEED:-0.8}"
LOOK="${AG_LOOKAHEAD:-0.40}"
# ⚠️ 到达阈值必须 **大于车的最小转弯半径**，否则车永远"到不了"：
#    官方阈值 0.10 + π*0.073/4 ≈ 0.157 m，而最小转弯半径 ≈ 0.157/tan(30°) ≈ 0.27 m
#    （实测绕圈半径 0.30 m）—— 纯追踪无法螺旋进一个自己转不进去的点，
#    于是绕着目标稳定画圆，永远不会触发 arrived（2026-10-01 实测确认）。
ARRIVE="${AG_ARRIVE_M:-0.40}"
SRC="$HOME/dzrobot/src/car_localization/car_localization"
DST="${AG_PATCH_DIR:-/tmp/ag_ws/patched}/car_localization"

echo "==================== 生成执行器补丁副本（不动官方源码） ===================="
echo "  linear_speed   : ${SPEED} m/s  （底盘字节约 $(awk -v s=$SPEED 'BEGIN{printf "%.0f", s*23/0.33}')/255）"
echo "  lookahead_dist : ${LOOK} m"
echo "  arrival_thresh : ${ARRIVE} m  （必须 > 最小转弯半径 ≈0.27 m）"
echo

if [ ! -f "$SRC/path_executor.py" ]; then
  echo "  [FAIL] 找不到官方 $SRC/path_executor.py"; exit 1
fi

mkdir -p "$DST" || { echo "  [FAIL] 建不了 $DST"; exit 1; }
# ⚠️ 必须复制**整个包**，不能只挑 path_executor.py + map_utils.py：
#    启动脚本会把这个目录放进 PYTHONPATH 最前面，于是 `car_localization` 整个包名
#    都解析到这里。少了 serial_reader.py，定位节点一启动就
#    ModuleNotFoundError → 直接死 → 读不到定位 → 真发车门禁拒绝发车。
#    （2026-09-30 踩过：补丁把定位"吃"掉了，查了半天才发现是自己造成的）
cp -f "$SRC"/*.py "$DST/" || exit 1
[ -f "$SRC/__init__.py" ] || : > "$DST/__init__.py"
echo "  -- 复制的模块（必须含 serial_reader，否则定位会崩）--"
ls -1 "$DST" | sed 's/^/      /'

# 只替换那两行（行首 8 个空格，类属性赋值）
sed -i "s|^        self\.lookahead_dist = .*|        self.lookahead_dist = ${LOOK}   # patched: 官方 0.15 太短，会导致转圈|" "$DST/path_executor.py"
sed -i "s|^        self\.linear_speed = .*|        self.linear_speed = ${SPEED}   # patched: 官方 0.3 太慢，跑不完路线|" "$DST/path_executor.py"
# 到达阈值：官方算出来 ≈0.157 m，比最小转弯半径（≈0.27 m）还小 → 永远到不了
sed -i "s|^        self\.arrival_threshold = .*|        self.arrival_threshold = ${ARRIVE}   # patched: 官方 ~0.157 m < 最小转弯半径，车绕圈到不了|" "$DST/path_executor.py"

echo "  -- 替换结果（应看到 patched 注释）--"
grep -n "self.lookahead_dist = \|self.linear_speed = \|self.arrival_threshold = " "$DST/path_executor.py" | head -5 | sed 's/^/    /'
echo
echo "  -- 校验：补丁包必须**两个**模块都能导入 --"
echo "     （map_utils 给执行器、serial_reader 给定位节点，少一个就会静默崩掉一个节点）"
PYTHONPATH="$(dirname "$DST"):${PYTHONPATH}" python3 -c "
import importlib
for m in ('car_localization.map_utils', 'car_localization.serial_reader'):
    try:
        importlib.import_module(m); print('    [OK]', m)
    except Exception as e:
        print('    [FAIL]', m, '->', e)
" 2>&1 | sed 's/^/  /'
echo
echo "  [OK] 补丁在 $DST"
echo "  启动脚本会优先用它；想回到官方版本，删掉这个目录即可。"
echo "==================== 结束 ===================="
