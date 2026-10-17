# -*- coding: utf-8 -*-
"""t3 收尾：终点停位复算（修正 offset 分支）+ I6 压雷几何窗口 + 挡板判据的两套口径对比。"""
import csv, io, math, os, sys

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
L, W = 0.28, 0.135
BARX, BARLO, BARHI = 1.75, 1.740, 1.760


def corners(px, py, yaw, off):
    cx = px + off * math.cos(yaw)
    cy = py + off * math.sin(yaw)
    hl, hw = L / 2, W / 2
    return [(cx + a*math.cos(yaw) - b*math.sin(yaw), cy + a*math.sin(yaw) + b*math.cos(yaw))
            for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


rows = list(csv.DictReader(io.open(os.path.join(
    ROOT, "artifacts", "sim", "t2_baseline_44tok", "v035", "traj.csv"), encoding="utf-8")))
print("末拍:", {k: rows[-1][k] for k in ("t", "truth_x", "truth_y", "truth_yaw")})
for off in (0.0, 0.075):
    mn = min(c[1] for c in corners(float(rows[-1]["truth_x"]), float(rows[-1]["truth_y"]),
                                   float(rows[-1]["truth_yaw"]), off))
    print("  offset=%.3f → 车体最小 y = %+.4f m（南缘 y=0；%s）"
          % (off, mn, "越过" if mn < 0 else "余量"))
# 全轨迹里最小的车体 y（含行进段）
for off in (0.0, 0.075):
    mn = 9e9; at = None
    for r in rows:
        v = min(c[1] for c in corners(float(r["truth_x"]), float(r["truth_y"]),
                                      float(r["truth_yaw"]), off))
        if v < mn:
            mn = v; at = r["t"]
    print("  全轨迹 offset=%.3f → 最小车体 y = %+.4f m @ t=%s s" % (off, mn, at))

# ---- I6 压雷 vs 挡板：x 向可行窗口 ----
print("\nI6 压雷窗口（pad 半 0.06 @ x=1.925，压雷判据=车心 0.035 圆 ∩ 垫片；朝向 -x）:")
for x in (1.925, 1.96, 1.99, 2.00, 2.02):
    gap_pad = max(0.0, abs(x - 1.925) - 0.06)
    nose = x - (L/2 + 0.075)
    print("  车心 x=%.3f → 垫片间隙 %.4f %s ; 车头 x=%.4f vs 挡板近面 %.3f → %s"
          % (x, gap_pad, "压到" if gap_pad <= 0.035 + 1e-9 else "压不到",
             nose, BARLO, "碰撞 %.1f mm" % ((BARLO - nose)*1000) if nose < BARLO else "不碰"))
