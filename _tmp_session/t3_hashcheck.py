# -*- coding: utf-8 -*-
import subprocess, hashlib, os
ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
paths = ["config/robot.yaml", "config/map.yaml", "src/dzbrain/dzbrain/tools/sim.py",
         "tools/route_check.py", "src/dzbrain/dzbrain/route/physics.py",
         "src/dzbrain/dzbrain/route/planner.py"]
for p in paths:
    b = subprocess.run(["git", "-C", ROOT, "cat-file", "blob", "HEAD:" + p],
                       capture_output=True).stdout
    f = open(os.path.join(ROOT, p.replace("/", os.sep)), "rb").read()
    same = "SAME" if hashlib.sha256(b).digest() == hashlib.sha256(f).digest() else "DIFF"
    print("%-6s HEAD=%s WT=%s  %s" % (same, hashlib.sha256(b).hexdigest().upper()[:16],
                                      hashlib.sha256(f).hexdigest().upper()[:16], p))
d = open(os.path.join(ROOT, "docs", "仿真复验_第五张图44token基线_t2.md"), "rb").read()
print("t2 doc sha256:", hashlib.sha256(d).hexdigest().upper()[:16], len(d), "bytes")
