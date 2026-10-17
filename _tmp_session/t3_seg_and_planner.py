# -*- coding: utf-8 -*-
"""t3：① route_check 的"挡板不触线"对应哪一航段；② planner 的挡板判据用的是哪套口径。"""
import io, os, re, sys

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
sys.path.insert(0, os.path.join(ROOT, "src", "dzbrain"))
os.chdir(ROOT)
from dzbrain.core.config import Config
cfg = Config.load(os.path.join(ROOT, "config", "robot.yaml"))
toks = [str(t) for t in cfg.get("route.tokens")]
for i in (30, 31, 32, 33, 34, 35, 36, 37, 38):
    print("段#%-3d = %s -> %s" % (i, toks[i-1], toks[i]))

print()
p = io.open(os.path.join(ROOT, "src/dzbrain/dzbrain/route/planner.py"), encoding="utf-8").read().splitlines()
for i, l in enumerate(p[3075:3110], 3076):
    print("%5d| %s" % (i, l.rstrip()[:150]))
