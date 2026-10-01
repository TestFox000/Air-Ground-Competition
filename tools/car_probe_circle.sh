#!/usr/bin/env bash
# 查「车为什么画圆」：把三个候选根因一次分清
#
# 背景（2026-09-30 真发车复盘）：
#   60 s 真发车报「位移 1.741 m」，看着像"车动了"。但 tools/traj_report.py 复盘发现：
#     路径总长 12.684 m、净位移 1.741 m、**净转角 +936°、转向方向零次反转**
#   —— 车根本没跑路线，是在半径约 0.78 m 的圈里单向画了 2.6 圈。
#   "位移"这个数**发现不了这件事**（本项目的路线终点 N11 就在起点 N10 旁边，
#   跑完全程的净位移本来也就 0.35 m）。
#
# 画圆只有三种可能，本脚本逐个给判据：
#   A) 定位 yaw 在漂   —— 车其实走直线，是**上报的朝向**一直在转
#                         判据：车**完全不动**时，yaw 自己也在单调变化
#   B) 前视距离不对   —— 0.35 m 的格距配 0.40 m 前视，目标点一直"够不到"
#                         判据：/pursuitAngle 的 angular 恒定同号、车走得动但就是到不了
#   C) 单位不匹配      —— 目标点在格坐标、位置是米（或反之），永远差一个 0.35 倍
#                         判据：执行器日志里 dist-to-goal 不下降，或量级差 3 倍
#
# ★ 安全设计：默认**一步都不发**（AG_SEND=0）。
#   阶段 1 只起底盘+定位、车静止不动，光看 yaw 漂不漂 —— 这已经能判掉 A。
#   要发命令才加 AG_SEND=1（那时人在车边、遥控器在手）。
#
# 用法：
#   python tools/remote_run.py --host <IP> --user duzhong --script tools/car_probe_circle.sh
#   # 要发一条单格命令看转向（人必须在场）
#   python tools/remote_run.py --host <IP> --user duzhong --script tools/car_probe_circle.sh \
#       --env AG_SEND=1
#
# 环境变量：
#   AG_SEND=1     发一条 2 格的命令（默认不发，车不动）
#   AG_SEND_WP=   自定义命令，默认 "N9,y"（从 N10 往前 1 格）
#   AG_IDLE_S=10  静息观测时长
#   AG_RUN_S=20   发命令后的观测时长
#   AG_NO_EKF=1   不起官方 EKF（/odometry/filtered 就没有，判据 A 少一路对照）

# ⚠️ 不要用 set -u（ROS setup.bash 会引用未定义变量，非交互 shell 直接退出）
set -o pipefail

SEND="${AG_SEND:-0}"
WP="${AG_SEND_WP:-N9,y}"
IDLE_S="${AG_IDLE_S:-10}"
RUN_S="${AG_RUN_S:-20}"
NO_EKF="${AG_NO_EKF:-0}"

source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
[ -f "${AG_WS:-/tmp/ag_ws}/install/setup.bash" ] && source "${AG_WS:-/tmp/ag_ws}/install/setup.bash" 2>/dev/null

# 执行器：优先用补丁副本（与 car_move_dryrun.sh 保持同一套规则）
PE_OFFICIAL="$HOME/dzrobot/src/car_localization/car_localization/path_executor.py"
PE_PATCH="${AG_PATCH_DIR:-/tmp/ag_ws/patched}/car_localization/path_executor.py"
if [ -f "$PE_PATCH" ]; then
  export PYTHONPATH="$(dirname "$(dirname "$PE_PATCH")"):${PYTHONPATH}"
  PE="$PE_PATCH"
else
  export PYTHONPATH="$HOME/dzrobot/src/car_localization:${PYTHONPATH}"
  PE="$PE_OFFICIAL"
fi

OUT=/tmp/ag_circle
rm -rf "$OUT"; mkdir -p "$OUT"
PIDS=""
start() { local n="$1"; shift; setsid "$@" > "/tmp/ag_circle_${n}.log" 2>&1 & local pg=$!; PIDS="$PIDS $pg"; echo "    $n 进程组=$pg"; }
cleanup() {
  for pg in $PIDS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 3
  for pg in $PIDS; do kill -9 -"$pg" 2>/dev/null; done
  pkill -f "path_executor.py" 2>/dev/null
  echo "  已停止本次启动的节点"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 画圆排查（默认不动车） ===================="
date
echo "  模式: $([ "$SEND" = "1" ] && echo "★会发命令 $WP（必须人在场）★" || echo '只观测，车不动')"
echo "  执行器: $PE"
echo

hr; echo "## 1. 起底盘 + 定位（没人发速度 → 车不动）"
start actuator ros2 launch dzactuator bringup.launch.py
sleep 10
grep -E "serial port opened|can not open" /tmp/ag_circle_actuator.log 2>/dev/null | tail -2 | sed 's/^/    /'
start localization ros2 launch car_localization localization.launch.py
sleep 8
if [ "$NO_EKF" != "1" ]; then
  start ekf ros2 launch dzactuator ekf.launch.py
  sleep 6
  echo "    （官方 EKF 已起：/odometry/filtered，用作 yaw 的第二路对照）"
fi
echo
echo "  -- 定位话题验活 --"
for t in /odom/localization /odometry/filtered; do
  r="$(timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" | sed 's/^/      /')"
  [ -n "$r" ] && echo "    [OK] $t $r" || echo "    [--] $t 没有数据"
done

# 采样：车静止时的 yaw
hr; echo "## 2. 静息 yaw 观测（${IDLE_S}s，车不动）"
echo "  判据 A：车不动时 yaw 若自己单调变化 → 定位朝向在漂，画圆就是它造成的"
timeout "$IDLE_S" ros2 topic echo /odom/localization --field pose.pose.orientation \
  > "$OUT/yaw_ours.txt" 2>&1 &
P1=$!
if [ "$NO_EKF" != "1" ]; then
  timeout "$IDLE_S" ros2 topic echo /odometry/filtered --field pose.pose.orientation \
    > "$OUT/yaw_ekf.txt" 2>&1 &
  P2=$!
fi
# 同时记录车是不是真的没动（位移必须 ≈ 0，否则"静息"不成立，判据作废）
timeout "$IDLE_S" ros2 topic echo /odom/localization --field pose.pose.position \
  > "$OUT/pos_idle.txt" 2>&1 &
P3=$!
wait $P1 2>/dev/null; [ -n "${P2:-}" ] && wait $P2 2>/dev/null; wait $P3 2>/dev/null

if [ "$SEND" != "1" ]; then
  hr; echo "## 3. 结论（未发命令，只看 yaw）"
else
  hr; echo "## 3. 发命令 '$WP'（${RUN_S}s）—— 看转向指令与 dist 变化"
  start executor python3 -u "$PE"
  sleep 6
  # 监听 /pursuitAngle（执行器真正下发给底盘的转向/速度）
  timeout "$RUN_S" ros2 topic echo /pursuitAngle > "$OUT/pursuit.txt" 2>&1 &
  PA=$!
  timeout "$RUN_S" ros2 topic echo /odom/localization --field pose.pose.position \
    > "$OUT/pos_run.txt" 2>&1 &
  PR=$!
  # 转向期间的 yaw：用来判断"指令让往左转、车是不是真的往左转"（判据 B2）
  timeout "$RUN_S" ros2 topic echo /odom/localization --field pose.pose.orientation \
    > "$OUT/yaw_run.txt" 2>&1 &
  PY=$!
  sleep 2
  timeout 5 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:N10,y'}" 2>&1 | sed 's/^/    /'
  sleep 2
  timeout 5 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: '$WP'}" 2>&1 | sed 's/^/    /'
  echo "    已发：$WP"
  wait $PA 2>/dev/null; wait $PR 2>/dev/null; wait $PY 2>/dev/null
  cp -f /tmp/ag_circle_executor.log "$OUT/" 2>/dev/null
  echo
  hr; echo "## 4. 结论"
fi

python3 - "$OUT" "$SEND" <<'PY'
import math, os, re, sys

out, send = sys.argv[1], sys.argv[2]

def quats(path):
    """从 `ros2 topic echo --field pose.pose.orientation` 的输出里提取 yaw(度) 序列。"""
    txt = open(path, encoding="utf-8", errors="replace").read()
    zs = [float(v) for v in re.findall(r"^\s*z:\s*(-?[\d.eE+-]+)", txt, re.M)]
    ws = [float(v) for v in re.findall(r"^\s*w:\s*(-?[\d.eE+-]+)", txt, re.M)]
    n = min(len(zs), len(ws))
    return [math.degrees(2 * math.atan2(zs[i], ws[i])) for i in range(n)]

def pts(path):
    txt = open(path, encoding="utf-8", errors="replace").read()
    xs = [float(v) for v in re.findall(r"^\s*x:\s*(-?[\d.eE+-]+)", txt, re.M)]
    ys = [float(v) for v in re.findall(r"^\s*y:\s*(-?[\d.eE+-]+)", txt, re.M)]
    return list(zip(xs, ys))

def drift(yaws):
    """单调漂移量：把角度展开后看首尾差（比逐点差分更抗噪）。"""
    if len(yaws) < 2:
        return 0.0
    unwrap, prev = [yaws[0]], yaws[0]
    for y in yaws[1:]:
        d = y - prev
        while d > 180: d -= 360
        while d < -180: d += 360
        prev = prev + d
        unwrap.append(prev)
    return unwrap[-1] - unwrap[0]

print("  [判据 A] 定位 yaw 漂不漂（车静止时测的，最要紧的一条）")
p0 = pts(os.path.join(out, "pos_idle.txt"))
if len(p0) > 1:
    # ⚠️ 坐标是 block 单位（不是米）：0.15 block ≈ 5 cm。
    move = max(math.dist(p0[0], p) for p in p0)
    print(f"    静息位移: {move:.3f} block ≈ {move*0.35:.2f} m  "
          f"{'(车确实没动，判据成立)' if move < 0.15 else '(⚠️ 车动了！判据作废，重跑)'}")
y = quats(os.path.join(out, "yaw_ours.txt"))
print(f"    /odom/localization: {len(y)} 个样本")
if len(y) > 2:
    d = drift(y)
    print(f"      静息 {len(y)} 个样本里 yaw 单调漂移 {d:+.1f}°")
    print(f"      → {'★yaw 在漂★ 这就是画圆的根因：换用官方 EKF 的朝向，或修 serial_reader 的 yaw' if abs(d) > 5 else 'yaw 稳（<5°），A 可排除'}")
ye = quats(os.path.join(out, "yaw_ekf.txt"))
if len(ye) > 2 and len(y) > 2:
    de = drift(ye)
    print(f"    /odometry/filtered : 静息漂移 {de:+.1f}°（官方 EKF 那一路）")
    print(f"      两路朝向差 {ye[0] - y[0]:+.1f}° → {'两路不一致，先确定该信哪一路' if abs(ye[0] - y[0]) > 10 else '两路一致'}")

if send == "1":
    print()
    print("  [判据 B] 转向指令是不是恒定同号（画圆的直接表现）")
    txt = open(os.path.join(out, "pursuit.txt"), encoding="utf-8", errors="replace").read()
    # ⚠️ 必须按帧拆开再取 `angular:` 段里的 z：整篇搜 `z:` 会把 **linear.z** 也抓进来
    #    （Twist 有 linear 和 angular 两段，各有一个 z），正负号统计就全乱了。
    ang = []
    for frame in re.split(r"^\s*---\s*$", txt, flags=re.M):
        m = re.search(r"angular:.*?z:\s*(-?[\d.eE+-]+)", frame, re.S)
        if m:
            ang.append(float(m.group(1)))
    if ang:
        pos = sum(1 for a in ang if a > 1e-6)
        neg = sum(1 for a in ang if a < -1e-6)
        mean = sum(ang) / len(ang)
        peak = max(abs(a) for a in ang)
        print(f"    /pursuitAngle angular.z: {len(ang)} 个，正 {pos} / 负 {neg}，均值 {mean:+.4f}")
        print(f"      前 8 个原值: {[round(a, 3) for a in ang[:8]]}")
        if pos == 0 or neg == 0:
            print("      → ★转向恒定同号★ = 车被持续往一边打方向 → 画圆")
        else:
            print("      → 转向有正有负，不是一直往一边打死")
        print()
        # ---- B1 单位：底盘 dzactuator 的约定是 TargetAngle = angular.z + 60（25~95 限幅） ----
        print("  [判据 B1] angular.z 的单位是弧度还是度")
        print(f"    绝对峰值 {peak:.3f}")
        if peak > 8:
            print("      → ★是度★（ROS 标准是弧度）。底盘 TargetAngle = z + 60 会被顶到 95/25")
            print("        上限 = 方向盘打死 → 车原地画圆。修：执行器输出前 ×π/180，")
            print("        或改底盘那侧的换算（后者动官方固件，优先改我们这边）")
        elif peak > 3:
            print("      → 偏大（3~8）：多半已经是度、只是幅度小，按上面那条处理")
        else:
            print("      → 弧度量级，单位没问题")
        # ---- B2 符号：指令方向 vs 车实际转向方向 ----
        print()
        print("  [判据 B2] 转向指令的符号接反了没（★正反馈会必然画圆★）")
        yr = quats(os.path.join(out, "yaw_run.txt"))
        if len(yr) > 2:
            d = drift(yr)
            print(f"    指令 angular.z 均值 {mean:+.4f}（>0 = 让车逆时针转）")
            print(f"    实际 yaw 变化 {d:+.1f}°（>0 = 车确实在逆时针转）")
            if abs(d) < 3:
                print("      → 车基本没转向，判据不成立（可能没收到命令）")
            elif mean * d > 0:
                print("      → 方向一致：让往左转、车确实往左转。符号没接反")
            else:
                print("      → ★符号反了★ 让往一边转、车往另一边转 = **正反馈**")
                print("        误差越转越大，车必然画圆且永不收敛 —— 这就是画圆的根因")
                print("        修：把执行器输出 angular.z 的符号取反（补丁里加一行取反即可）")
        else:
            print("    （没抓到转向期间的 yaw，判据 B2 跳过）")
    else:
        print("    （没抓到 /pursuitAngle —— 执行器没起，或话题名不同；先看 /tmp/ag_circle_executor.log）")
    print()
    print("  [判据 C] 目标点够不够得到（单位是否匹配）")
    p = pts(os.path.join(out, "pos_run.txt"))
    if len(p) > 1:
        # ⚠️ 坐标是 block 单位（不是米）：单格 = 1.0 block = 0.35 m
        walked = sum(math.dist(p[i], p[i+1]) for i in range(len(p)-1))
        netd = math.dist(p[0], p[-1])
        print(f"    走了 {walked:.2f} block（{walked*0.35:.2f} m），"
              f"净移动 {netd:.2f} block（{netd*0.35:.2f} m）；单格 = 1.0 block")
        # ⚠️ 判据必须是"净移动 / 走了多少"这个**效率**，不能只看净移动的绝对值：
        #    画圆时净移动也能有好几米（圈直径），光看绝对值会把画圆误判成"到了"。
        eff = netd / walked if walked > 1e-6 else 0.0
        print(f"    行进效率: {eff:.2f}  （净移动 / 走过的路；1.0 = 直着过去）")
        if eff > 0.5 and netd > 0.7:      # 0.7 block ≈ 0.25 m：单格命令"到了"的判据
            print("      → 到了（或接近到了）：单格命令能执行，问题在多格路线的转向衔接")
        elif walked > 1.0:                # 1.0 block = 一格 = 0.35 m
            print("      → ★走了很多却没到★ 目标点够不到：查前视距离 AG_LOOKAHEAD 与格/米单位")
        else:
            print("      → 基本没走：回到 car_move_dryrun.sh 那套排查（命令/执行器/底盘）")
    ex = os.path.join(out, "ag_circle_executor.log")
    if os.path.exists(ex):
        txt = open(ex, encoding="utf-8", errors="replace").read()
        dists = re.findall(r"dist[=: ]+([\d.]+)", txt)
        arrives = re.findall(r"ARRIVED.*", txt)
        print(f"    执行器日志: 抓到 {len(dists)} 个 dist、{len(arrives)} 条 ARRIVED")
        if dists:
            print(f"      dist 首 {dists[0]} → 末 {dists[-1]}"
                  f"  {'（在下降：目标点是对的）' if float(dists[-1]) < float(dists[0]) else '（★不下降★：目标点够不到，查单位/前视）'}")
        for a in arrives[:3]:
            print(f"      {a.strip()[:110]}")
print()
print("  下一步（按命中项走，别一次全改）：")
print("    A  命中（yaw 漂）→ 换/修定位朝向：用官方 EKF 的 yaw 顶替，或修 serial_reader 的解算")
print("    B1 命中（单位是度）→ 执行器输出前 ×π/180（底盘 TargetAngle = z + 60 只认度）")
print("    B2 命中（符号反了）→ 把 angular.z 取反。这一项是**正反馈**，改完立刻见效")
print("    B  命中但 B1/B2 都没中 → 调 AG_LOOKAHEAD（重跑 tools/car_patch_executor.sh）")
print("    C  命中（走了很多没到）→ 核对 path_executor 目标点是格还是米（map_utils 换算）")
print()
print("  ⚠️ 只改一项就重跑一次：同时改 yaw、单位、符号，画圆好了也不知道是哪个修好的，")
print("     下次再犯还得从头查一遍。")
PY

echo
echo "  证据目录: $OUT"
ls -1 "$OUT" 2>/dev/null | sed 's/^/      /'
echo "  取回：python tools/remote_run.py --host <IP> --user duzhong --get $OUT --to logs/car"
echo "==================== 结束（节点将自动停止） ===================="
