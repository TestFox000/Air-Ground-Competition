# -*- coding: utf-8 -*-
import csv
base = r"D:/User/Desktop/比赛存档/v1/_tmp_session/t28_run/"
for name in ("odom","loc"):
    rows=[]
    with open(base+f"run28_{name}.csv", newline='') as f:
        for d in csv.DictReader(f):
            try: rows.append((float(d['t_s']), float(d['x']), float(d['y']), float(d['yaw_deg'])))
            except: pass
    gaps=[]
    for i in range(1,len(rows)):
        dt = rows[i][0]-rows[i-1][0]
        if dt > 0.5:
            gaps.append((rows[i-1][0], dt, rows[i-1][1], rows[i-1][2], rows[i][1], rows[i][2]))
    print(name, "rows", len(rows), "gaps>0.5s:", len(gaps))
    for g in gaps[:12]:
        print("   gap at t=%.1f dt=%.2fs  (%.3f,%.3f)->(%.3f,%.3f)" % g)
    # motion window
    mv=[r for r in rows if r[0]>330]
    if mv:
        print("   motion window t=%.1f..%.1f  (%.2f,%.2f)->(%.2f,%.2f)" % (mv[0][0], mv[-1][0], mv[0][1], mv[0][2], mv[-1][1], mv[-1][2]))
