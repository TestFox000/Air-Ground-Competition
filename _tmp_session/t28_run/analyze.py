# -*- coding: utf-8 -*-
import csv, math

def load(p):
    rows=[]
    with open(p, newline='') as f:
        r=csv.DictReader(f)
        for d in r:
            try:
                rows.append((float(d['t_s']), float(d['x']), float(d['y']), float(d['yaw_deg']), float(d['vx']), float(d['wz'])))
            except Exception:
                pass
    return rows

base = r"D:/User/Desktop/比赛存档/v1/_tmp_session/t28_run/"
od = load(base+"run28_odom.csv")
lo = load(base+"run28_loc.csv")
print("odom rows", len(od), "loc rows", len(lo))
for name, rows in (("odom", od), ("loc", lo)):
    t0 = rows[0][0]
    # jumps
    jumps=[]
    for i in range(1,len(rows)):
        dx=rows[i][1]-rows[i-1][1]; dy=rows[i][2]-rows[i-1][2]
        d=math.hypot(dx,dy)
        if d>0.05:
            jumps.append((rows[i][0]-t0, round(d,3), round(rows[i-1][1],3), round(rows[i-1][2],3), round(rows[i][1],3), round(rows[i][2],3)))
    # total path length
    L=0
    for i in range(1,len(rows)):
        L+=math.hypot(rows[i][1]-rows[i-1][1], rows[i][2]-rows[i-1][2])
    print(f"--- {name}: t {rows[0][0]:.1f}..{rows[-1][0]:.1f}  path_len={L:.2f} m  start=({rows[0][1]:.3f},{rows[0][2]:.3f}) yaw={rows[0][3]:.1f}  end=({rows[-1][1]:.3f},{rows[-1][2]:.3f}) yaw={rows[-1][3]:.1f}")
    print(f"    jumps>0.05m: {len(jumps)}")
    for j in jumps[:12]:
        print("     ", j)
    # frozen spans: consecutive identical (x,y)
    frozen=0; maxf=0; start=None
    for i in range(1,len(rows)):
        if rows[i][1]==rows[i-1][1] and rows[i][2]==rows[i-1][2]:
            frozen+=1
            if start is None: start=rows[i][0]-t0
            maxf=max(maxf,frozen)
        else:
            frozen=0; start=None
    print(f"    max identical-pose span (samples)={maxf}")
