# -*- coding: utf-8 -*-
"""t3：用 SAT（与 t3_sat_barrier.py 同法）比较"落盘 44 token 路线"与"planner 当前产物"的挡板侵入。

SAT 是正确的判据（车体旋转矩形 ∩ 挡板细条）；标量 xmin 会把挡板以西的整个半场误算进来。
"""
import csv, io, math, os, sys

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
L, W, HT = 0.28, 0.135, 0.01
BARX, BARY0, BARY1 = 1.75, 0.0, 3.85
strip = [(BARX-HT, BARY0), (BARX+HT, BARY0), (BARX+HT, BARY1), (BARX-HT, BARY1)]


def corners(px, py, yaw, off):
    cx, cy = px + off*math.cos(yaw), py + off*math.sin(yaw)
    hl, hw = L/2, W/2
    return [(cx + a*math.cos(yaw) - b*math.sin(yaw), cy + a*math.sin(yaw) + b*math.cos(yaw))
            for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def sat(a, b):
    best = float("inf")
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            x0, y0 = poly[i]; x1, y1 = poly[(i+1) % n]
            nx, ny = -(y1-y0), (x1-x0)
            ln = math.hypot(nx, ny)
            if ln == 0:
                continue
            nx, ny = nx/ln, ny/ln
            pa = [p[0]*nx + p[1]*ny for p in a]
            pb = [p[0]*nx + p[1]*ny for p in b]
            ov = min(max(pa), max(pb)) - max(min(pa), min(pb))
            if ov <= 0:
                return (False, 0.0)
            best = min(best, ov)
    return (True, best)


for tag, path in (("落盘 44t (t2基线 v035)", os.path.join(ROOT, "artifacts", "sim",
                                                   "t2_baseline_44tok", "v035", "traj.csv")),
                  ("planner 当前产物 (含 M9) @0.35",
                   r"D:\User\Desktop\比赛存档\v1\_tmp_session\t3_resim_new\traj_new035.csv")):
    rows = list(csv.DictReader(io.open(path, encoding="utf-8")))
    print("\n==", tag, "(%d 拍)" % len(rows))
    for off in (0.0, 0.075):
        hits = []
        for r in rows:
            ok, dep = sat(corners(float(r["truth_x"]), float(r["truth_y"]),
                                  float(r["truth_yaw"]), off), strip)
            if ok:
                hits.append((float(r["t"]), dep, r["truth_seg_index"]))
        if hits:
            deep = sum(1 for h in hits if h[1] >= 0.019)
            print("  offset=%.3f: %d 拍重叠 / 持续 %.2f s / 最大 SAT 穿透 %.2f mm / "
                  "穿透≥19mm 的拍 %d / segs %s"
                  % (off, len(hits), len(hits)*0.02, max(h[1] for h in hits)*1000,
                     deep, sorted({h[2] for h in hits}, key=int)))
        else:
            print("  offset=%.3f: 无重叠" % off)
