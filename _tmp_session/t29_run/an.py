# -*- coding: utf-8 -*-
import csv, math
base = r"D:/User/Desktop/比赛存档/v1/_tmp_session/t29_run/"
def load(n):
    rows=[]
    with open(base+f"run29_{n}.csv", newline='') as f:
        for d in csv.DictReader(f):
            try: rows.append((float(d['t_s']),float(d['x']),float(d['y']),float(d['yaw_deg']),float(d['vx'])))
            except: pass
    return rows
for n in ("odom","loc"):
    r=load(n); t0=r[0][0]
    mv=[x for x in r if x[4]!=0]
    print(n, len(r), "t0=%.1f"%t0, "motion samples", len(mv))
    if mv:
        print("  motion start t=%.1f (%.2f,%.2f) yaw %.0f"%(mv[0][0]-t0, mv[0][1], mv[0][2], mv[0][3]))
        print("  motion end   t=%.1f (%.2f,%.2f) yaw %.0f"%(mv[-1][0]-t0, mv[-1][1], mv[-1][2], mv[-1][3]))
    # jumps
    js=[]
    for i in range(1,len(r)):
        d=math.hypot(r[i][1]-r[i-1][1], r[i][2]-r[i-1][2])
        if d>0.3: js.append((r[i][0]-t0, d, r[i-1][1],r[i-1][2],r[i][1],r[i][2]))
    print("  jumps>0.3:", len(js), js[:5])
