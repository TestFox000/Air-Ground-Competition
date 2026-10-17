# -*- coding: utf-8 -*-
"""t3：把"挡板侵入"量化到可判读的口径（只读 t2 轨迹）。

定义（严格、与项目 physics 同源）：
  车体 = 后轴参考点 + ref_offset(0.075) 处的 0.28×0.135 旋转矩形；
  挡板 = x∈[1.740,1.760]、y∈[0,3.85] 的细条（robot.yaml:783 barrier_width_m=0.02，
         格网真源 x=1.75）。
  侵入深度 = max(0, 1.740 − min{ 车体上 y∈[0,3.85] 的点的 x })   ← 车从 +x 侧逼近挡板
  另给"车体是否把整条 2cm 细条包进自身足迹"（= SAT 里报 20mm 的那种情形）。
"""
import csv, io, math, os

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
L, W = 0.28, 0.135
BAR_LO, BAR_HI, BARY0, BARY1 = 1.740, 1.760, 0.0, 3.85


def poly(px, py, yaw, off):
    cx, cy = px + off * math.cos(yaw), py + off * math.sin(yaw)
    hl, hw = L / 2, W / 2
    return [(cx + a*math.cos(yaw) - b*math.sin(yaw), cy + a*math.sin(yaw) + b*math.cos(yaw))
            for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def xmin_in_band(p, y0=BARY0, y1=BARY1):
    """车体多边形上 y∈[y0,y1] 部分的最小 x（无覆盖 ⇒ None）。"""
    xs = []
    for i in range(4):
        a, b = p[i], p[(i+1) % 4]
        if y0 <= a[1] <= y1:
            xs.append(a[0])
        if (a[1] - y0) * (b[1] - y0) < 0:
            t = (y0 - a[1]) / (b[1] - a[1]); xs.append(a[0] + t*(b[0]-a[0]))
        if (a[1] - y1) * (b[1] - y1) < 0:
            t = (y1 - a[1]) / (b[1] - a[1]); xs.append(a[0] + t*(b[0]-a[0]))
    return min(xs) if xs else None


for v in ("v015", "v035", "v040"):
    rows = list(csv.DictReader(io.open(os.path.join(
        ROOT, "artifacts", "sim", "t2_baseline_44tok", v, "traj.csv"), encoding="utf-8")))
    for off in (0.0, 0.075):
        worst = 0.0; wt = None; nyaw = None; nsteps = 0
        for r in rows:
            tx, ty, tyaw = float(r["truth_x"]), float(r["truth_y"]), float(r["truth_yaw"])
            m = xmin_in_band(poly(tx, ty, tyaw, off))
            if m is None:
                continue
            d = BAR_LO - m
            if d > 0:
                nsteps += 1
                if d > worst:
                    worst, wt, nyaw = d, float(r["t"]), math.degrees(tyaw)
        print("%s offset=%.3f 侵入挡板近面: %d 拍(%.2f s) 最深 %+.2f mm @ t=%s yaw=%s"
              % (v, off, nsteps, nsteps*0.02, worst*1000,
                 ("%.2f" % wt) if wt is not None else "-",
                 ("%.1f°" % nyaw) if nyaw is not None else "-"))

print()
print("几何判据（参考点在 row6 格心、车头朝 -x 压向挡板）:")
off, hl = 0.075, L/2
print("  row6 格心 x=1.925 ; 挡板中心线 1.750 半厚 0.01 ⇒ 近面 1.760")
print("  参考点→车头 = %.3f m ; 车头 x = %.3f ⇒ 越过近面 %+.1f mm、越过远面 %+.1f mm"
      % (off+hl, 1.925-(off+hl), (1.760-(1.925-(off+hl)))*1000, (1.750-(1.925-(off+hl)))*1000))
print("  允许的最近转弯点（车头刚好贴到近面）= x = %.3f m ; 路线航点却给到 1.925 m ⇒ 差 %.1f mm"
      % (1.760 + off + hl, (1.925 - (1.760 + off + hl))*1000))
print("  行 6 走廊内车身侧向（沿 -y 行驶）静态余量 = (1.925-1.760) - 0.0675 = %.4f m"
      % (0.165 - W/2))
