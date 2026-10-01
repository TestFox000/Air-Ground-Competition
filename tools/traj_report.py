#!/usr/bin/env python3
"""复盘一次真发车的轨迹：回答「车到底动没动、动得快不快、是不是在画圆」。

为什么要有这个工具：
  `car_move_dryrun.sh` 末尾只报一个数——「位移 1.741 m」。这个数**说不出车是怎么走的**：
    - 匀速把整条路线跑完了？
    - 一直在动，但在原地打转？
    - 冲一段、然后卡死？
  这三种位移可能一模一样，但对应的问题完全不同（速度太低 / 转向算错 / 命令没到）。
  轨迹文件里其实全都有答案，只是几千行坐标没有人会去读——所以交给脚本。

★ 2026-09-30 用它抓到的真因线索：
    真发车 60 s，位移 1.741 m 看着"动了"，但复盘发现：
      路径总长 12.68 m、净位移 1.74 m、**净转角 +936° 且转向方向零次反转**
      —— 车根本没跑路线，是在半径约 0.8 m 的圈里**单向画了 2.6 圈**。
  只看"位移 1.741 m"永远不会发现这件事。

用法：
    python tools/traj_report.py logs/car/ag_logs/run/traj.txt
    python tools/traj_report.py <traj.txt> --hz 50      # 定位实际频率（默认 50）
    python tools/traj_report.py <traj.txt> --win 2.0    # 每 2 秒一段统计

三个判据怎么读（这是本工具的核心）：
  - 转向反转次数 ≈ 0 且 |净转角| > 360°  → **单向画圆**（转圈），不是蛇形摆动
  - 转向反转次数很多、净转角 ≈ 0        → **蛇形摆动**（前视距离太短的 textbook 症状）
  - bbox 很小但路径很长                  → 车没离开过一小块地方（局部打转/卡死）
"""
import argparse
import math
import re
import sys

PAIR = re.compile(r"^\s*x:\s*(-?[\d.eE+-]+)\s*\n\s*y:\s*(-?[\d.eE+-]+)", re.M)
# traj.txt 里每一帧之间有一行 `---`，用它更准确地还原"帧边界"
SEP = re.compile(r"^\s*---\s*$", re.M)


def load(path):
    """返回 [(x, y), ...]。按 `---` 分帧，一帧取一对 x/y，容错没有分隔符的写法。"""
    txt = open(path, encoding="utf-8", errors="replace").read()
    frames = [f for f in SEP.split(txt) if "x:" in f]
    pts = []
    for f in frames:
        m = PAIR.search(f)
        if m:
            pts.append((float(m.group(1)), float(m.group(2))))
    return pts


def heading_stats(pts, hz, min_step=0.004):
    """算朝向序列：返回 (净转角deg, 累计转角deg, 转向反转次数, 朝向样本数)。

    min_step：位移小于它就认为算不出朝向（车停着或抖动），直接跳过。
    """
    step = max(1, int(hz / 10))          # 每 0.1 s 取一次朝向，压掉定位抖动
    heads = []
    for i in range(0, len(pts) - step, step):
        dx = pts[i + step][0] - pts[i][0]
        dy = pts[i + step][1] - pts[i][1]
        if math.hypot(dx, dy) > min_step:
            heads.append(math.atan2(dy, dx))
    turns = []
    for i in range(len(heads) - 1):
        d = math.degrees(heads[i + 1] - heads[i])
        while d > 180.0:
            d -= 360.0
        while d < -180.0:
            d += 360.0
        turns.append(d)
    if not turns:
        return 0.0, 0.0, 0, 0
    net = sum(turns)
    total = sum(abs(t) for t in turns)
    # 只认"明确转了"的窗口（>3°），否则噪声会淹没反转计数。
    # ⚠️ 必须**先滤掉不转的窗口再数反转**：由 -1 变到 +1 必然经过 0，
    #    若直接对相邻两项相乘，这一处的乘积是 0（不算反转），蛇形摆动会被
    #    误判成"零次反转"—— 那就和单向画圆混为一谈了（写测试时踩到）。
    signs = [1 if t > 3 else (-1 if t < -3 else 0) for t in turns]
    solid = [s for s in signs if s != 0]
    flips = sum(1 for i in range(len(solid) - 1) if solid[i] * solid[i + 1] < 0)
    return net, total, flips, len(heads)


def main():
    # Windows 控制台默认 GBK，中文报告一打印就 UnicodeEncodeError（而且报错发生在
    # 打印那一刻，工具会直接以非 0 退出）。强制按 UTF-8 输出；Linux（车端）本来就是
    # UTF-8，reconfigure 对它无副作用。
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    ap = argparse.ArgumentParser(description="轨迹复盘：把 traj.txt 变成人能读的结论")
    ap.add_argument("traj", help="traj.txt（ros2 topic echo --field pose.pose.position 的输出）")
    ap.add_argument("--hz", type=float, default=50.0, help="定位实际频率，默认 50 Hz")
    ap.add_argument("--win", type=float, default=1.0, help="分段统计窗口（秒），默认 1.0")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="坐标单位换算系数：车端 /odom/localization 是 block 单位"
                         "（不是米！），传 0.35 得到米制结论。2026-09-30 就因为这个"
                         "把半径 0.34 m 误报成 0.78 m")
    args = ap.parse_args()

    pts = load(args.traj)
    if len(pts) < 2:
        print(f"[FAIL] {args.traj} 里只解析出 {len(pts)} 个点，没法复盘")
        return 1
    if args.scale != 1.0:
        # 在入口处统一换算，下游所有几何计算都在同一单位里做
        pts = [(x * args.scale, y * args.scale) for x, y in pts]

    dur = (len(pts) - 1) / args.hz
    # 路径总长：逐点累加，反映"车走过的路"
    path_len = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    # 直线距离：起终点连起来，反映"净位移"
    net = math.dist(pts[0], pts[-1])
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    bbox = (max(xs) - min(xs), max(ys) - min(ys))
    net_turn, abs_turn, flips, n_head = heading_stats(pts, args.hz)

    print("==================== 轨迹复盘 ====================")
    print(f"  文件: {args.traj}")
    print(f"  样本: {len(pts)} 个   按 {args.hz:g} Hz 折算时长 ≈ {dur:.1f} s")
    if args.scale != 1.0:
        print(f"  单位: 已按 ×{args.scale} 换算（block→米）；下面的 m 是真米")
    print(f"  起点: x={pts[0][0]:.3f} y={pts[0][1]:.3f}")
    print(f"  终点: x={pts[-1][0]:.3f} y={pts[-1][1]:.3f}")
    print()
    print(f"  路径总长: {path_len:.3f} m   （车实际走过的路）")
    print(f"  直线距离: {net:.3f} m   （起点到终点的净位移）")
    print(f"  活动范围: {bbox[0]:.3f} × {bbox[1]:.3f} m   （bbox，车总共就在这块地方）")
    print(f"  全程均速: {path_len / dur:.3f} m/s")
    print()
    print("  -- 转向形状（★最关键的一组数）--")
    print(f"    朝向样本: {n_head} 个")
    print(f"    净转角  : {net_turn:+.1f}°   （正=逆时针；>360° 说明转了整圈以上）")
    print(f"    累计转角: {abs_turn:.1f}°   （净≈累计 = 一直朝同一边转）")
    print(f"    转向反转: {flips} 次   （0 次 = 单向画圆；很多次 = 蛇形左右摆）")
    if abs(net_turn) > 360 and flips <= 2:
        # 周长 ≈ 路径总长，反推半径
        r = path_len / (2 * math.pi * abs(net_turn) / 360.0)
        print(f"    → 单向画圆：约 {abs(net_turn) / 360:.1f} 圈，估算半径 {r:.2f} m")
    print()

    # 分段统计：每 win 秒走了多远
    per = max(1, int(args.win * args.hz))
    print(f"  -- 每 {args.win:g} 秒位移 --")
    rows = []
    for s in range(0, len(pts) - 1, per):
        seg = pts[s:s + per + 1]
        d = sum(math.dist(seg[i], seg[i + 1]) for i in range(len(seg) - 1))
        t = s / args.hz
        rows.append((t, d, d / args.win))
        bar = "#" * min(40, int(d * 20))
        print(f"    t={t:5.1f}s  位移 {d:6.3f} m  均速 {d / args.win:5.3f} m/s  {bar}")

    # 最长停顿：连续位移低于阈值的窗口
    stall_min = 0.002  # 2 mm/窗口，低于此视为没动
    best_len, best_at, cur_len, cur_at = 0, 0.0, 0, 0.0
    for t, d, _ in rows:
        if d < stall_min:
            if cur_len == 0:
                cur_at = t
            cur_len += 1
            if cur_len > best_len:
                best_len, best_at = cur_len, cur_at
        else:
            cur_len = 0
    stalled = sum(1 for _, d, _ in rows if d < stall_min)
    print()
    print(f"  没动的窗口: {stalled}/{len(rows)}  （占 {100 * stalled / max(1, len(rows)):.0f}%）")
    if best_len:
        print(f"  最长停顿 : {best_len * args.win:.1f} s（从 t={best_at:.1f}s 开始）")
    else:
        print("  最长停顿 : 无（全程都在动）")

    print()
    print("  -- 怎么读这份报告 --")
    if path_len < 0.15:
        print("    ① 车基本没动：去看 waypoint.txt（命令发了没）→ ag_move_executor.log（执行器活着没）")
    elif abs(net_turn) > 360 and flips <= 2:
        print("    ② ★单向画圆★：车没在跑路线，是在原地转圈。三个候选根因按这个顺序验：")
        print("       a) 定位 yaw 在漂 —— 车其实走直线，是上报的朝向在转（看 /odometry/filtered 与")
        print("          /odom/localization 的 yaw 是否一致；不一致就换用官方 EKF 的朝向）")
        print("       b) 前视距离 lookahead_dist 不对 —— 0.35 m 的格距配 0.40 m 前视会一直\"够不到\"目标点")
        print("       c) 单位不匹配（格 vs 米）—— 目标点永远在够不到的地方，车就绕着它转")
        print("       判据脚本：tools/car_probe_circle.sh（车上线后一键抓这三项）")
    elif flips > 10 and abs(net_turn) < 180:
        print("    ③ 蛇形左右摆：前视距离**太短**的经典症状，调大 AG_LOOKAHEAD（官方 0.15 → 试 0.40）")
    elif stalled > len(rows) * 0.5:
        print("    ④ 走走停停：到达判据太严/定位跳变，执行器反复认为还没到。看执行器日志的 dist-to-goal")
    elif path_len / dur < 0.3:
        print("    ⑤ 一直在动但很慢：速度上限太低，调 AG_LINEAR_SPEED（官方 dztrack 是 0.8）")
    else:
        print("    ⑥ 运动本身没问题：往下核它有没有真的走到该去的格子（对照 waypoint.txt 与路线）")
    print()
    # 注意：这里别用 ⚠️ 这类 emoji —— Windows 控制台是 GBK，打印它会直接
    # UnicodeEncodeError 把整个工具搞崩（写测试时踩到，报错还只在子进程里）。
    print("  注意：别只盯\"净位移\"：本项目的路线是**回到起点旁边**（N10 出发 → N11 结束），")
    print("     跑完全程的净位移本来也就一格（0.35 m），净位移小不等于没跑完。")
    print("==================== 结束 ====================")
    return 0


if __name__ == "__main__":
    sys.exit(main())
