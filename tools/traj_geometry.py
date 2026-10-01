#!/usr/bin/env python3
"""轨迹几何判定：车到底有没有朝路线的方向走。

为什么需要（2026-09-30 复盘教训）：
  轨迹复盘只看"画没画圆"还不够 —— 圆的位置说明问题的性质：
    - 圆心在路点附近 = 纯追踪够不到（前视/到达判据问题）
    - 圆心在场地外、且一开始就朝目标反方向跑 = **转向环从 t=0 就发散**
      （符号接反一类的正反馈，不是参数没调好）
  这两种的修法完全不同，必须区分。

  另一个教训：/odom/localization 是 **block 单位**（N10 = (1.5, 0.5)，与
  serial_reader 日志 "pos: (1.50, 0.50) | block: N10" 吻合），不是米。
  拿它当米看，会把半径/速度放大 2.86 倍（1/0.35），得出"半径 0.78 m"这种
  看着合理其实错误的数 —— 真实半径 0.34 m，是舵机打死的最小转弯半径。

用法：
    python tools/traj_geometry.py logs/car/ag_logs/run/traj.txt --first K9
    # 起点不是 N10 时用 --from-cell N10 指定 reset 的格号
"""
import argparse
import math
import re
import sys

PAIR = re.compile(r"^\s*x:\s*(-?[\d.eE+-]+)\s*\n\s*y:\s*(-?[\d.eE+-]+)", re.M)
SEP = re.compile(r"^\s*---\s*$", re.M)

# 车端 block_to_meters 坐标约定（与 path_executor 源码逐条核对过，
# 见 src/ag_car/ag_car/route_builder.py 头注）。注意：x 沿行号方向、
# y 沿列号方向，与直觉相反，且都是"数字越大越靠前/越靠 A"。
CELL_RE = re.compile(r"^([A-N])(\d{1,2})$")


def load(path):
    txt = open(path, encoding="utf-8", errors="replace").read()
    frames = [f for f in SEP.split(txt) if "x:" in f]
    pts = []
    for f in frames:
        m = PAIR.search(f)
        if m:
            pts.append((float(m.group(1)), float(m.group(2))))
    return pts


def cell_to_block(cell):
    """格号 -> /odom/localization 的 block 坐标（不乘 0.35）。"""
    m = CELL_RE.match(cell.strip().upper())
    if not m:
        raise SystemExit(f"[FAIL] 非法格号: {cell!r}（应形如 K9）")
    col = ord(m.group(1)) - ord("A")
    row = int(m.group(2)) - 1
    return (10.5 - row, 13.5 - col)


def main():
    # Windows 下子进程 stdout 默认按 GBK 编码，调用方按 UTF-8 解码会得到满屏问号
    # （traj_report.py 踩过同款坑）。Linux（车端）本来就是 UTF-8，无副作用。
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    ap = argparse.ArgumentParser(description="判定车是否朝路线方向行驶（block 坐标系）")
    ap.add_argument("traj", help="traj.txt（/odom/localization 的采样）")
    ap.add_argument("--first", required=True, help="路线的第一个路点格号，如 K9")
    ap.add_argument("--from-cell", default="N10", help="reset 锚定的起点格号（默认 N10）")
    ap.add_argument("--cell-size", type=float, default=0.35, help="格边长（米），默认 0.35")
    args = ap.parse_args()

    pts = load(args.traj)
    if len(pts) < 2:
        print(f"[FAIL] {args.traj} 只解析出 {len(pts)} 个点")
        return 1

    s = cell_to_block(args.from_cell)
    t = cell_to_block(args.first)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    to_target = (t[0] - s[0], t[1] - s[1])
    moved = (pts[0][0] - s[0], pts[0][1] - s[1])
    dot = moved[0] * to_target[0] + moved[1] * to_target[1]

    print("==================== 轨迹几何判定 ====================")
    print(f"  起点 {args.from_cell} = ({s[0]:.1f}, {s[1]:.1f}) block")
    print(f"  第一路点 {args.first} = ({t[0]:.1f}, {t[1]:.1f}) block"
          f"（方向 ({to_target[0]:+.1f}, {to_target[1]:+.1f})）")
    print(f"  开录时位置 ({pts[0][0]:.3f}, {pts[0][1]:.3f})，"
          f"离起点 {math.dist(s, pts[0]):.2f} block（{math.dist(s, pts[0])*args.cell_size:.2f} m）")
    print(f"  轨迹 bbox: x [{min(xs):.2f}, {max(xs):.2f}]  y [{min(ys):.2f}, {max(ys):.2f}] block")
    print()
    in_box = min(xs) <= t[0] <= max(xs) and min(ys) <= t[1] <= max(ys)
    print(f"  [1] 第一路点在轨迹范围内? {'在' if in_box else '★不在——全程未接近★'}")
    print(f"  [2] 开录前的位移与目标方向的点积: {dot:+.2f}"
          f"  （{'同向' if dot > 0 else '★反向——一上来就朝目标相反方向开★'}）")
    d_end = math.dist(pts[-1], t)
    print(f"  [3] 结束时离第一路点 {d_end:.2f} block（{d_end*args.cell_size:.2f} m）")
    print()
    print("  判读：")
    if not in_box and dot < 0:
        print("    ★转向环从发车那一刻就发散★：既没朝目标走、也从未进入它的邻域。")
        print("      这不是前视距离/到达判据的调参问题，是**符号或坐标架错了**")
        print("      （angular.z 符号接反 / yaw 反号 / 格-米混用之一）。")
        print("      判定脚本：tools/car_probe_circle.sh（AG_SEND=1，20 s 出结论）")
    elif not in_box:
        print("    车没到过第一路点，但方向大体对：查前视距离/到达判据/中途被打断。")
    else:
        print("    车到过第一路点附近：第一段没问题，问题在后续段（用 --first 逐段查）。")
    print("==================== 结束 ====================")
    return 0


if __name__ == "__main__":
    sys.exit(main())
