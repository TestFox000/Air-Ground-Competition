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
for name in ("odom","loc"):
    rows = load(base+f"run28_{name}.csv")
    print(f"===== {name}: {len(rows)} rows, t0={rows[0][0]:.2f}, t1={rows[-1][0]:.2f}")
    # transitions (position change > 1e-4)
    trans=[]
    for i in range(1,len(rows)):
        if abs(rows[i][1]-rows[i-1][1])>1e-4 or abs(rows[i][2]-rows[i-1][2])>1e-4:
            trans.append((rows[i][0]-rows[0][0], rows[i][1], rows[i][2], rows[i][3], rows[i][4]))
    print(f"  motion transitions: {len(trans)}")
    for tr in trans[:8]:
        print("   first:", tuple(round(x,3) for x in tr))
    for tr in trans[-8:]:
        print("   last :", tuple(round(x,3) for x in tr))
    if trans:
        print("   transition t-range:", round(trans[0][0],1), "..", round(trans[-1][0],1))
    # sample every 2000 rows
    print("  samples:")
    for i in range(0, len(rows), max(1,len(rows)//12)):
        t,x,y,yaw,vx,wz = rows[i]
        print(f"   t={t-rows[0][0]:7.1f}  ({x:7.3f},{y:7.3f}) yaw={yaw:7.1f} vx={vx:.3f}")
