# -*- coding: utf-8 -*-
"""t3：对"当前工作树"再跑一次关键门禁与套件（并发写入者仍在改，故同时记录哈希快照）。"""
import hashlib, io, os, subprocess, sys, time

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
env = dict(os.environ, PYTHONIOENCODING="utf-8")


def snap():
    out = {}
    for p in ("config/robot.yaml", "config/map.yaml", "src/dzbrain/dzbrain/tools/sim.py",
              "src/dzbrain/dzbrain/route/physics.py", "src/dzbrain/dzbrain/route/planner.py",
              "src/dzbrain/dzbrain/core/config.py", "tools/route_check.py",
              "src/dzbrain/dzbrain/tasks/mission.py"):
        b = open(os.path.join(ROOT, p.replace("/", os.sep)), "rb").read()
        out[p] = hashlib.sha256(b).hexdigest().upper()[:16]
    return out


before = snap()
print("== 快照(前) ==")
for k, v in before.items():
    print("   %-46s %s" % (k, v))

print()
for cmd, tag in (([sys.executable, "tools/config_selfcheck.py"], "config_selfcheck"),
                 ([sys.executable, "tools/route_check.py", "--config", "config/robot.yaml"],
                  "route_check")):
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, env=env)
    out = r.stdout.decode("utf-8", "replace")
    keep = [l.strip() for l in out.splitlines()
            if "结论" in l or "全部通过" in l or "error" in l.lower()][-3:]
    print("%-18s rc=%d  %s" % (tag, r.returncode, " | ".join(keep)))

for t in ("test_planner", "test_physics", "test_core", "test_route", "test_deploy",
          "test_sim", "test_sim_guard", "test_corner_cut", "test_bridge_gate"):
    r = subprocess.run([sys.executable, os.path.join("src", "dzbrain", "tests", t + ".py")],
                       cwd=ROOT, capture_output=True, env=env)
    out = (r.stdout.decode("utf-8", "replace") + r.stderr.decode("utf-8", "replace"))
    tail = [l for l in out.splitlines() if l.strip()][-1][:100]
    print("%-18s rc=%d  %s" % (t, r.returncode, tail))

after = snap()
print()
print("== 快照(后) ==")
drift = [k for k in before if before[k] != after[k]]
print("复核期间发生变化的文件:", drift if drift else "无（快照稳定）")
for k in drift:
    print("   %-46s %s -> %s" % (k, before[k], after[k]))
