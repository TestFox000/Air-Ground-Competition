#!/usr/bin/env bash
# 让执行器**自报内部状态**：瞄准点 / 转向量 / 状态机 / 距目标距离
#
# 为什么需要：2026-10-01 实测把"单位、符号、yaw 映射、轴距模型"逐条排除后，
# 车仍然绕圈（前视 0.15 与 0.50 都一样）。剩下的可能都在执行器**内部状态**里，
# 而它平时不打任何周期性日志 —— 只能猜。这个脚本用 monkey-patch 包一层，
# 不动车上任何文件，让它每 0.5 s 报一次"它以为的情况"。
#
# 用法：
#   python tools/remote_run.py --host <IP> --user duzhong --script tools/car_debug_executor.sh
#   （先跑过 tools/car_patch_executor.sh 更好：会用补丁副本的参数）
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null

TARGET="${AG_TARGET:-N9}"
RUN_S="${AG_RUN_S:-18}"
PKG_DIR="${AG_PATCH_DIR:-/tmp/ag_ws/patched}"      # 补丁副本所在目录（其下是 car_localization/）
PE_DIR="$HOME/dzrobot/src/car_localization"

OUT=/tmp/ag_dbg
rm -rf "$OUT"; mkdir -p "$OUT"
PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_dbg_${n}.log" 2>&1 & PGRPS="$PGRPS $!"; echo "    $n 进程组=$!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 3
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  pkill -f "ag_dbg.py" 2>/dev/null
  echo "  已停止本脚本启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

# 用哪个包：补丁副本优先（要把它所在目录放进 PYTHONPATH）
if [ -f "$PKG_DIR/car_localization/path_executor.py" ]; then
  PKG_ROOT="$PKG_DIR"; SRC="$PKG_DIR/car_localization/path_executor.py"
else
  PKG_ROOT="$PE_DIR"; SRC="$PE_DIR/car_localization/path_executor.py"
fi
export PYTHONPATH="$PKG_ROOT:${PYTHONPATH}"

echo "==================== 执行器内部状态自报 ===================="
date
echo "  包根: $PKG_ROOT"
grep -n "self.lookahead_dist = \|self.linear_speed = " "$SRC" 2>/dev/null | sed 's/^/    /'
echo

cat > /tmp/ag_dbg.py <<'DBGPY'
import math, sys, time
sys.path.insert(0, __import__("os").environ.get("AG_PKG_ROOT", "/tmp/ag_ws/patched"))
import car_localization.path_executor as pe

Node = pe.PathExecutorNode
_t = [0.0]


def _rate(period=0.5):
    now = time.time()
    if now - _t[0] < period:
        return None
    _t[0] = now
    return now


def _state(self):
    fin = self.path_points[-1] if self.path_points else None
    d = math.hypot(self.current_x - fin['x'], self.current_y - fin['y']) if fin else -1.0
    return ("pos=(%.2f,%.2f)m yaw=%.0f° corner=%s/%d arrived=%s idx=%d/%d dist2end=%.2fm"
            % (self.current_x, self.current_y, math.degrees(self.current_yaw),
               getattr(self, "corner_mode", "?"), getattr(self, "corner_phase", -1),
               self.arrived, self.current_target_idx, len(self.path_points), d))


_pp = Node.pure_pursuit_control


def pp(self, target_point):
    lin, steer = _pp(self, target_point)
    if _rate() is not None:
        aim = "(%.2f,%.2f)m" % (target_point['x'], target_point['y']) if target_point else "None"
        alpha = ""
        if target_point:
            dx = target_point['x'] - self.current_x
            dy = target_point['y'] - self.current_y
            alpha = " bearing=%.0f°" % math.degrees(
                math.atan2(dy, dx) - self.current_yaw)
        self.get_logger().info("[PP] aim=%s%s steer=%+.1f° lin=%.3f | %s"
                               % (aim, alpha, steer, lin, _state(self)))
    return lin, steer


_cl = Node.control_loop


def cl(self):
    _cl(self)
    if _rate() is not None:
        self.get_logger().info("[LOOP] %s" % _state(self))


Node.pure_pursuit_control = pp
Node.control_loop = cl
pe.main()
DBGPY

export AG_PKG_ROOT="$PKG_ROOT"

echo "$PKG_ROOT" >/dev/null

hr; echo "## 1. 起底盘 + 定位"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 12
grep -E "serial port opened|can not open" /tmp/ag_dbg_actuator.log 2>/dev/null | tail -1 | sed 's/^/    /'
start localization ros2 launch car_localization localization.launch.py
sleep 8
echo

hr; echo "## 2. 起（带日志包装的）执行器并下令 $TARGET"
start executor bash -c 'AG_PKG_ROOT='"$PKG_ROOT"' python3 -u /tmp/ag_dbg.py'
sleep 8
timeout 5 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:N10,y'}" >/dev/null 2>&1
echo "    已发 reset:N10,y"
sleep 3
timeout 5 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: '$TARGET,y'}" >/dev/null 2>&1
echo "    已发 $TARGET,y，观测 ${RUN_S}s"
sleep "$RUN_S"
echo

hr; echo "## 3. 执行器自报（节选）"
grep -E "\[PP\]|\[LOOP\]|Navigating|ARRIVED|PIVOT|sharp|corner" /tmp/ag_dbg_executor.log \
  | sed 's/.*\[INFO\] //; s/\x1b\[[0-9;]*m//g' | tail -30 | sed 's/^/    /'
echo
echo "  完整日志：/tmp/ag_dbg_executor.log（取回：--get /tmp/ag_dbg --to logs/car）"
echo "==================== 结束（节点将自动停止） ===================="
