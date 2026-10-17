# -*- coding: utf-8 -*-
"""t3 复核（只读）：v035 轨迹上"车体 ∩ 挡板细条"的细节 + SAT 穿透深度。"""
import csv, math, io, os, json

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
L, W = 0.28, 0.135
BAR_X, BAR_Y0, BAR_Y1 = 1.75, 0.0, 3.85
HT = 0.01


def corners(px, py, yaw, offset):
    cx = px + offset * math.cos(yaw)
    cy = py + offset * math.sin(yaw)
    hl, hw = L / 2.0, W / 2.0
    return [(cx + a * math.cos(yaw) - b * math.sin(yaw),
             cy + a * math.sin(yaw) + b * math.cos(yaw))
            for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def sat_penetration(polyA, polyB):
    """返回 (是否重叠, 最小穿透深度)。用 SAT：两凸多边形各边的法向上投影重叠最小值。"""
    best = float("inf")
    overlap_any = True
    for poly in (polyA, polyB):
        n = len(poly)
        for i in range(n):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % n]
            nx, ny = -(by - ay), (bx - ax)
            ln = math.hypot(nx, ny)
            if ln == 0:
                continue
            nx, ny = nx / ln, ny / ln
            pa = [p[0] * nx + p[1] * ny for p in polyA]
            pb = [p[0] * nx + p[1] * ny for p in polyB]
            o = min(max(pa), max(pb)) - max(min(pa), min(pb))
            if o <= 0:
                return (False, 0.0)
            best = min(best, o)
    return (overlap_any, best)


strip = [(BAR_X - HT, BAR_Y0), (BAR_X + HT, BAR_Y0), (BAR_X + HT, BAR_Y1), (BAR_X - HT, BAR_Y1)]

path = os.path.join(ROOT, "artifacts", "sim", "t2_baseline_44tok", "v035", "traj.csv")
with io.open(path, encoding="utf-8", newline="") as f:
    rows = list(csv.DictReader(f))

hits = []
for r in rows:
    tx, ty, tyaw = float(r["truth_x"]), float(r["truth_y"]), float(r["truth_yaw"])
    for off in (0.0, 0.075):
        poly = corners(tx, ty, tyaw, off)
        ov, dep = sat_penetration(poly, strip)
        if ov:
            hits.append((off, float(r["t"]), tx, ty, tyaw, dep, r["truth_seg_index"], float(r["v"])))

for off in (0.0, 0.075):
    h = [x for x in hits if x[0] == off]
    if not h:
        print("offset=%.3f 无重叠" % off)
        continue
    print("offset=%.3f 重叠拍数=%d  t∈[%.2f,%.2f]  最大SAT穿透=%.2f mm" %
          (off, len(h), h[0][1], h[-1][1], max(x[5] for x in h) * 1000))
    print("   x∈[%.4f,%.4f] y∈[%.4f,%.4f] yaw∈[%.2f°,%.2f°] v∈[%.3f,%.3f] segs=%s" %
          (min(x[2] for x in h), max(x[2] for x in h),
           min(x[3] for x in h), max(x[3] for x in h),
           math.degrees(min(x[4] for x in h)), math.degrees(max(x[4] for x in h)),
           min(x[7] for x in h), max(x[7] for x in h),
           sorted(set(x[6] for x in h), key=lambda s: int(s))))
    # 最深拍
    worst = max(h, key=lambda x: x[5])
    print("   最深拍: t=%.2f x=%.4f y=%.4f yaw=%.1f° pen=%.2f mm seg=%s v=%.3f" %
          (worst[1], worst[2], worst[3], math.degrees(worst[4]), worst[5] * 1000, worst[6], worst[7]))
    # 穿透 >=19mm 的拍
    deep = [x for x in h if x[5] >= 0.019]
    print("   穿透≥19mm 的拍数 = %d（=挡板被车体完全吞没）" % len(deep))
    # 该段耗时
    print("   重叠持续 = %.2f s（相邻拍 0.02s）" % (0.02 * len(h)))
print()
print("整车几何：车体 x 向半展（正对挡板时）= 前伸 0.075+0.14 = %.3f m > 半格 0.175 m" % (0.075 + 0.14))
