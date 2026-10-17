# -*- coding: utf-8 -*-
"""t3 独立复核（只读）：用 t2 的轨迹 CSV 复算"挡板触线"三种口径。

不复用 route/physics 的判定代码路径（只借用 body_corners 的几何公式自实现，
避免"用被测实现测被测实现"）。同时给出 t2 声称的 B/C 两列计数。
"""
import csv, math, sys, os, json, io

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
sys.path.insert(0, os.path.join(ROOT, "src", "dzbrain"))

L, W = 0.28, 0.135           # robot.yaml: dzactuator.vehicle.length_m / width_m
BAR_X, BAR_Y0, BAR_Y1 = 1.75, 0.0, 3.85
BAR_HALF_THICK = 0.01
NEED_CENTER = BAR_HALF_THICK + W / 2.0   # 0.0775


def corners(px, py, yaw, offset):
    """自实现：车体四角（后轴中心 p + 前向偏移 offset）。"""
    cx = px + offset * math.cos(yaw)
    cy = py + offset * math.sin(yaw)
    hl, hw = L / 2.0, W / 2.0
    out = []
    for sx in (+1, +1, -1, -1):
        for sy in (+1, -1):
            out.append((cx + sx * hl * math.cos(yaw) - sy * hw * math.sin(yaw),
                        cy + sx * hl * math.sin(yaw) + sy * hw * math.cos(yaw)))
    return out[:4] if False else [(cx + a * math.cos(yaw) - b * math.sin(yaw),
                                   cy + a * math.sin(yaw) + b * math.cos(yaw))
                                  for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def seg_seg_dist(p1, p2, p3, p4):
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    def on(o, a, b):
        return (min(o[0], a[0])-1e-12 <= b[0] <= max(o[0], a[0])+1e-12 and
                min(o[1], a[1])-1e-12 <= b[1] <= max(o[1], a[1])+1e-12)
    d1, d2 = cross(p3, p4, p1), cross(p3, p4, p2)
    d3, d4 = cross(p1, p2, p3), cross(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    if abs(d1) < 1e-12 and on(p3, p4, p1): return 0.0
    if abs(d2) < 1e-12 and on(p3, p4, p2): return 0.0
    if abs(d3) < 1e-12 and on(p1, p2, p3): return 0.0
    if abs(d4) < 1e-12 and on(p1, p2, p4): return 0.0
    def pt_seg(px, py, ax, ay, bx, by):
        dx, dy = bx-ax, by-ay
        dd = dx*dx + dy*dy
        t = 0.0 if dd == 0 else max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy)/dd))
        return math.hypot(px-(ax+t*dx), py-(ay+t*dy))
    return min(pt_seg(*p1, *p3, *p4), pt_seg(*p2, *p3, *p4),
               pt_seg(*p3, *p1, *p2), pt_seg(*p4, *p1, *p2))


def point_in_poly(px, py, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]; x1, y1 = poly[(i+1) % n]
        if (y0 > py) != (y1 > py):
            x_at = x0 + (py-y0)*(x1-x0)/(y1-y0)
            if x_at > px: inside = not inside
    return inside


def body_strip_clearance(px, py, yaw, offset):
    """旋转车体矩形 ↔ 挡板细条（x∈[1.74,1.76], y∈[0,3.85]）最近距离；重叠=0。"""
    poly = corners(px, py, yaw, offset)
    strip_corners = [(BAR_X-BAR_HALF_THICK, BAR_Y0), (BAR_X+BAR_HALF_THICK, BAR_Y0),
                     (BAR_X+BAR_HALF_THICK, BAR_Y1), (BAR_X-BAR_HALF_THICK, BAR_Y1)]
    # 包含检测（t4 同款口径）
    for sp in strip_corners:
        if point_in_poly(sp[0], sp[1], poly):
            return 0.0
    if point_in_poly((BAR_X), (BAR_Y0+BAR_Y1)/2.0, poly):
        return 0.0
    best = float("inf")
    for i in range(4):
        a = strip_corners[i]; b = strip_corners[(i+1) % 4]
        for j in range(4):
            c = poly[j]; d = poly[(j+1) % 4]
            d_ = seg_seg_dist(a, b, c, d)
            if d_ < best: best = d_
    return best


rows = {}
for v in ("v015", "v035", "v040"):
    path = os.path.join(ROOT, "artifacts", "sim", "t2_baseline_44tok", v, "traj.csv")
    with io.open(path, encoding="utf-8", newline="") as f:
        r = list(csv.DictReader(f))
    n = len(r)
    A = B = C = 0
    minB = minC = float("inf")
    worst = {}
    for row in r:
        tx, ty, tyaw = float(row["truth_x"]), float(row["truth_y"]), float(row["truth_yaw"])
        # A 弱口径：车心(后轴点)到中心线 < 0.0775
        if abs(tx-BAR_X) < NEED_CENTER and BAR_Y0 <= ty <= BAR_Y1:
            A += 1
        cb = body_strip_clearance(tx, ty, tyaw, 0.0)
        cc = body_strip_clearance(tx, ty, tyaw, 0.075)
        if cb <= 0.0: B += 1
        if cc <= 0.0:
            C += 1
            if cc <= 0.0 and "worst" not in worst:
                worst["first_breach_t"] = float(row["t"])
        minB = min(minB, cb); minC = min(minC, cc)
    rows[v] = dict(rows=n, A_weak=A, B_offset0=B, C_offset0075=C,
                   min_clear_offset0=round(minB, 6), min_clear_offset0075=round(minC, 6),
                   first_breach=worst.get("first_breach_t"))

print(json.dumps(rows, ensure_ascii=False, indent=1))

# ---- 也核对 t2 说"命中的是 G6/H6/I6/J6 段"：找 20.00mm 穿透时刻 ----
path = os.path.join(ROOT, "artifacts", "sim", "t2_baseline_44tok", "v035", "traj.csv")
with io.open(path, encoding="utf-8", newline="") as f:
    r = list(csv.DictReader(f))
lows = []
for row in r:
    tx, ty, tyaw = float(row["truth_x"]), float(row["truth_y"]), float(row["truth_yaw"])
    cc = body_strip_clearance(tx, ty, tyaw, 0.075)
    if cc <= 1e-9:
        lows.append((float(row["t"]), tx, ty, tyaw, row["truth_seg_index"]))
if lows:
    print("v035 offset0.075 命中拍数 =", len(lows), " 首拍 t=%.2f x=%.4f y=%.4f yaw=%.4f seg=%s"
          % lows[0])
    print("末拍 t=%.2f seg=%s" % (lows[-1][0], lows[-1][4]))
# 最小角点 x（越过细条近侧 1.740 的实证）
mincorner = 9e9; at = None
for row in r:
    tx, ty, tyaw = float(row["truth_x"]), float(row["truth_y"]), float(row["truth_yaw"])
    for (cxp, cyp) in corners(tx, ty, tyaw, 0.075):
        if BAR_Y0 <= cyp <= BAR_Y1 and cxp < mincorner:
            mincorner = cxp; at = (float(row["t"]), tx, ty, tyaw)
print("v035 最靠西的车体角点 x = %.4f @ t=%.2f (穿过细条近侧 1.740 的深度 %.2f mm)"
      % (mincorner, at[0], (1.740-mincorner)*1000))
