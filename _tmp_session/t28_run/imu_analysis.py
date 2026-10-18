# -*- coding: utf-8 -*-
import csv, math
p = r"D:/User/Desktop/比赛存档/v1/_tmp_session/t28_run/run28_imu.csv"
rows=[]
with open(p, newline='') as f:
    for d in csv.DictReader(f):
        try:
            rows.append((float(d['t_s']), float(d['yaw_deg']), float(d['gz']), float(d['ax']), float(d['ay']), float(d['az'])))
        except: pass
print("rows", len(rows), "t0", rows[0][0], "t1", rows[-1][0])
# sample every 500
for i in range(0, len(rows), max(1,len(rows)//20)):
    t,yaw,gz,ax,ay,az = rows[i]
    print(f"t={t:8.2f} yaw={yaw:8.1f} gz={gz:8.4f} ax={ax:7.3f} ay={ay:7.3f} az={az:7.3f}")
# detect yaw wrap events (jump > 90 deg in one sample)
print("--- yaw jumps ---")
prev=None
for t,yaw,gz,ax,ay,az in rows:
    if prev is not None and abs(yaw-prev[1])>90:
        print(f"JUMP t={t:.2f} {prev[1]:.1f} -> {yaw:.1f}")
    prev=(t,yaw)
# gz stats during motion
import statistics
gz=[r[2] for r in rows if r[0]>330]
print("gz mean/max/min during motion:", round(statistics.mean(gz),4), round(max(gz),4), round(min(gz),4))
