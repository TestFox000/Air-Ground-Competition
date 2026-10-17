# -*- coding: utf-8 -*-
"""t3：planner 挡板判据 + sim 是否使用精确车体口径（只读计数）。"""
import io, os, sys
ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"


def show(path, lo, hi):
    L = io.open(os.path.join(ROOT, path), encoding="utf-8").read().splitlines()
    print("=====", path, "lines", len(L))
    for i in range(lo, min(hi, len(L)) + 1):
        print("%5d| %s" % (i, L[i-1].rstrip()[:165]))


show(r"src\dzbrain\dzbrain\route\planner.py", 2965, 3000)
show(r"src\dzbrain\dzbrain\route\planner.py", 3090, 3120)

sim = io.open(os.path.join(ROOT, r"src\dzbrain\dzbrain\tools\sim.py"), encoding="utf-8").read()
print()
for fn in ("body_barrier_clearance_m", "body_bridge_wall_clearance_m",
           "barrier_distance_point_m", "barrier_distance_segment_m",
           "required_barrier_distance_m", "barrier_breach_steps"):
    print("%-32s sim.py 出现 %d 次" % (fn, sim.count(fn)))
