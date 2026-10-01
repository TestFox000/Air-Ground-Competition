#!/usr/bin/env bash
# 抓执行器真实输出的 linear.x / angular.z，并判断"让它往哪转、车是不是往那转"
#
# 底盘约定（dzactuator，读自源码）：
#     TargetAngle = angular.z + 60，限幅 [25,95] → angular.z 是**舵机角度（度）**
#     linear.x 是 m/s：TargetSpeed = |v*23/0.33|
#
# 本脚本用三个量判读：
#   B1 单位：|angular.z| 峰值 >8 → 是度；<8 → 弧度（与底盘约定不符，会把舵机顶死）
#   B2 符号：angular.z 的符号 vs 车实际 yaw 的变化方向
#           反号 = 正反馈 = 车必然画圆（这一项命中，改符号立刻见效）
#   B3 速度：linear.x 实值 vs 命令速度（速度折算有没有差 2.86 倍的格/米泄漏）
#
# ⚠️ 会让车动（发一格命令）。需人在场、车周围留空间。
#
# ⚠️ 采样必须**先于执行器启动**（2026-10-01 实测教训）：
#    车端 Wi-Fi 上节点一多，DDS 发现就会丢，后启动的 `ros2 topic echo`
#    常常一条都收不到（话题明明 50 Hz，采样文件却是空的）。
#    所以这里一上来就把三个 echo 挂在后台，贯穿全程。
#
# 用法：
#   python tools/remote_run.py --host <IP> --user duzhong --script tools/car_probe_turn.sh
#   python tools/remote_run.py --host <IP> --user duzhong --script tools/car_probe_turn.sh \
#       --env AG_TARGET=M10 --env AG_RUN_S=15
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
[ -f "${AG_WS:-/tmp/ag_ws}/install/setup.bash" ] && source "${AG_WS:-/tmp/ag_ws}/install/setup.bash" 2>/dev/null

TARGET="${AG_TARGET:-N9}"
# AG_ROUTE：直接给整条路线（如 "M8 L7,y"），用来验证多段路点能不能走完
ROUTE_CMD="${AG_ROUTE:-$TARGET,y}"
RUN_S="${AG_RUN_S:-15}"
SETUP_S="${AG_SETUP_S:-10}"

# 执行器：优先用补丁副本（与 car_move_dryrun.sh 同一套规则）
PE_OFFICIAL="$HOME/dzrobot/src/car_localization/car_localization/path_executor.py"
PE_PATCH="${AG_PATCH_DIR:-/tmp/ag_ws/patched}/car_localization/path_executor.py"
if [ -f "$PE_PATCH" ]; then
  export PYTHONPATH="$(dirname "$(dirname "$PE_PATCH")"):${PYTHONPATH}"
  PE="$PE_PATCH"
else
  export PYTHONPATH="$HOME/dzrobot/src/car_localization:${PYTHONPATH}"
  PE="$PE_OFFICIAL"
fi

OUT=/tmp/ag_turn
rm -rf "$OUT"; mkdir -p "$OUT"
PGRPS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_turn_${n}.log" 2>&1 & PGRPS="$PGRPS $!"; echo "    $n 进程组=$!"; }
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 3
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  pkill -f "path_executor.py" 2>/dev/null
  pkill -f "ros2 topic echo" 2>/dev/null
  echo "  已停止本脚本启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 转向指令实测（车会动一小段） ===================="
date
echo "  目标 ${TARGET}（一格），发命令后观测 ${RUN_S}s"
echo "  执行器: $PE"
echo

hr; echo "## 1. 起底盘 + 定位"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 12
grep -E "serial port opened|can not open" /tmp/ag_turn_actuator.log 2>/dev/null | tail -2 | sed 's/^/    /'
start localization ros2 launch car_localization localization.launch.py
sleep 8
echo "  -- 定位验活（必须 50 Hz，否则后面全是空数据）--"
timeout 8 ros2 topic hz /odom/localization 2>/dev/null | grep -m1 "average rate" | sed 's/^/    /'
echo

hr; echo "## 2. 先挂采样（关键：必须在起执行器之前）"
ECHO_T=$((SETUP_S + RUN_S + 20))
# ⚠️ /pursuitAngle 是 **BEST_EFFORT**（dzactuator 用 KeepLast(1).best_effort() 订阅）。
#    `ros2 topic echo` 默认 RELIABLE，QoS 不匹配时**一条都收不到** ——
#    表现为"执行器明明在发，采样文件却是空的"（2026-10-01 就是栽在这里：
#    连跑三轮全是 0 样本）。必须显式 --qos-reliability best_effort。
# ⚠️ 车端这两个话题都是 **BEST_EFFORT**（/pursuitAngle 是 KeepLast(1).best_effort()；
#    /odom/localization 同理，car_move_dryrun.sh 的 read_pos 早就留了 best_effort 兜底）。
#    `ros2 topic echo` 默认 RELIABLE，QoS 不匹配时**一条都收不到** ——
#    表现为"节点明明 50 Hz 在发，采样文件却是空的"（2026-10-01 连跑三轮全是 0 样本）。
#    订阅端用 best_effort 对 RELIABLE 发布者也兼容，所以三路统一用 best_effort。
# ⚠️ 不要再用 `ros2 topic echo` 采样，两个坑都踩过：
#   ① 这两个话题都是 BEST_EFFORT，CLI 默认 RELIABLE 订阅 → QoS 不匹配，一条都收不到
#      （"节点 50 Hz 在发、采样文件却是空的"，2026-10-01 连跑三轮全是 0 样本）；
#   ② `ros2` CLI 依赖 daemon，我们反复 kill 进程组会把 daemon 搞坏，之后 CLI
#      直接抛 `!rclpy.ok()` 崩掉。所以这里内嵌 rclpy 订阅器，一个节点订两路。
cat > /tmp/ag_turn_sub.py <<'SUBPY'
import sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

secs = float(sys.argv[1]); out = sys.argv[2]
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
rclpy.init()
n = Node("ag_turn_probe")
f = open(out, "w", buffering=1)
t0 = time.time()


def yaw_of(q):
    return math.degrees(2.0 * math.atan2(q.z, q.w))


def cb_twist(m):
    f.write("twist %.3f %.6f %.6f\n" % (time.time() - t0, m.linear.x, m.angular.z))


def cb_odom(m):
    p = m.pose.pose.position
    f.write("odom %.3f %.6f %.6f %.3f\n"
            % (time.time() - t0, p.x, p.y, yaw_of(m.pose.pose.orientation)))


n.create_subscription(Twist, "/pursuitAngle", cb_twist, qos)
n.create_subscription(Odometry, "/odom/localization", cb_odom, qos)
while time.time() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.1)
f.close()
n.destroy_node()
rclpy.shutdown()
SUBPY
( timeout $((ECHO_T + 10)) python3 -u /tmp/ag_turn_sub.py "$ECHO_T" "$OUT/dump.txt" \
    > "$OUT/sub.log" 2>&1 ) &
E1=$!
sleep 3
echo "    已挂上订阅器（/pursuitAngle + /odom/localization，best_effort，${ECHO_T}s）"
echo

hr; echo "## 3. 起执行器并下令"
start executor python3 -u "$PE"
sleep "$SETUP_S"
timeout 5 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:N10,y'}" 2>&1 | sed 's/^/    /'
sleep 3
timeout 5 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: '$ROUTE_CMD'}" 2>&1 | sed 's/^/    /'
echo "    已发：$ROUTE_CMD"
sleep "$RUN_S"
kill $E1 2>/dev/null; wait $E1 2>/dev/null
cp -f /tmp/ag_turn_executor.log "$OUT/" 2>/dev/null
echo

hr; echo "## 4. 判读"
python3 - "$OUT" <<'PY'
import math, os, re, sys

out = sys.argv[1]


def load_dump(path):
    """读订阅器落盘：twist(t, lin, ang) 与 odom(t, x, y, yaw_deg)。"""
    tw, od = [], []
    try:
        txt = open(path, encoding="utf-8", errors="replace").read()
    except OSError:
        return tw, od
    for line in txt.splitlines():
        p = line.split()
        if len(p) == 4 and p[0] == "twist":
            tw.append((float(p[1]), float(p[2]), float(p[3])))
        elif len(p) == 5 and p[0] == "odom":
            od.append((float(p[1]), float(p[2]), float(p[3]), float(p[4])))
    return tw, od


def unwrap_delta(yaws):
    """展开后的首尾差：抗 ±180 跳变。"""
    if len(yaws) < 2:
        return 0.0
    prev = yaws[0]
    tot = 0.0
    for y in yaws[1:]:
        d = y - prev
        while d > 180.0:
            d -= 360.0
        while d < -180.0:
            d += 360.0
        tot += d
        prev = y
    return tot


tw, od = load_dump(os.path.join(out, "dump.txt"))
lin = [t[1] for t in tw]
ang = [t[2] for t in tw]

if not ang:
    print("    [FAIL] 没抓到 /pursuitAngle —— 订阅器没接上。")
    print("           先看 $OUT/sub.log（订阅器异常）与 /tmp/ag_turn_localization.log，")
    print("           确认定位是 50 Hz 且执行器活着，然后重跑一次。")
    if od:
        print(f"           （定位倒是有 {len(od)} 个样本，说明订阅器在工作、只是没收到 Twist）")
    sys.exit(0)

peak = max(abs(a) for a in ang)
mean = sum(ang) / len(ang)
print(f"    /pursuitAngle: {len(ang)} 帧（定位 {len(od)} 帧）")
print(f"      linear.x  范围 [{min(lin):.3f}, {max(lin):.3f}] m/s" if lin else "      linear.x 无样本")
print(f"      angular.z 范围 [{min(ang):.3f}, {max(ang):.3f}]，均值 {mean:+.3f}")
print(f"      前 8 个 angular.z: {[round(a, 2) for a in ang[:8]]}")
print()
print("  [B1 单位] angular.z 是度还是弧度")
if peak > 8:
    print(f"    峰值 {peak:.1f} → **度**，与底盘约定（z+60）一致 ✓")
else:
    print(f"    峰值 {peak:.3f} → 弧度量级。底盘会当度用（几乎不转向），需 ×180/π")
print()
print("  [B2 符号] 指令方向 vs 车实际转向")
ys = [o[3] for o in od]          # 订阅器直接给的 yaw（度）
if len(ys) > 5:
    # 取后半段（发命令之后）看转向
    tail = ys[len(ys) // 2:]
    d = unwrap_delta(tail)
    print(f"    angular.z 均值 {mean:+.3f}（>0 = 让车左转）")
    print(f"    后半程 yaw 变化 {d:+.1f}°（>0 = 车确实在左转）")
    if abs(d) < 3:
        print("      → 车几乎没转向：可能没收到命令，或速度太小")
    elif mean * d > 0:
        print("      → 方向一致，符号没接反 ✓")
    else:
        print("      → ★符号反了★：让往一边转、车往另一边 = 正反馈 = 画圆根因")
else:
    print("    （yaw 样本不足，跳过）")
print()
p = [(o[1], o[2]) for o in od]
if len(p) > 5:
    walked = sum(math.dist(p[i], p[i + 1]) for i in range(len(p) - 1))
    netd = math.dist(p[0], p[-1])
    print(f"  [B3 位移] 走了 {walked:.2f} block（{walked*0.35:.2f} m），净移动 {netd:.2f} block（{netd*0.35:.2f} m）")
    print(f"    行进效率 {netd / walked if walked else 0:.2f}（1.0 = 直着过去，<0.5 = 在绕）")
PY
echo
echo "  证据目录: $OUT"
ls -1 "$OUT" 2>/dev/null | sed 's/^/      /'
echo "  取回：python tools/remote_run.py --host <IP> --user duzhong --get $OUT --to logs/car"
echo "==================== 结束（节点将自动停止） ===================="
