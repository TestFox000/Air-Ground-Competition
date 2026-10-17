# -*- coding: utf-8 -*-
"""t3 收尾：① t6 新惩罚上线后 tracker-safe 产物是否仍与落盘 route.tokens 逐 token 恒等；
② SHOOT 窗口几何（F3→板心）；③ 记录当次复核快照哈希。"""
import hashlib, io, os, subprocess, sys, yaml

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
sys.path.insert(0, os.path.join(ROOT, "src", "dzbrain"))
os.chdir(ROOT)
from dzbrain.core.config import Config
from dzbrain.core import geometry as geo

cfg = Config.load(os.path.join(ROOT, "config", "robot.yaml"))
tokens = [str(t) for t in cfg.get("route.tokens")]
raw = yaml.safe_load(io.open(os.path.join(ROOT, "config", "robot.yaml"), encoding="utf-8"))
pe = [str(t) for t in raw["path_executor"]["ros__parameters"]["mission_blocks"]]
print("route.tokens (%d) == mission_blocks (%d): %s" % (len(tokens), len(pe), tokens == pe))

env = dict(os.environ, PYTHONIOENCODING="utf-8")
r = subprocess.run([sys.executable, "tools/plan_route.py", "--config", "config/robot.yaml",
                    "--engine", "tracker-safe", "--emit", "tokens", "--quiet"],
                   cwd=ROOT, capture_output=True, env=env)
out = r.stdout.decode("utf-8", "replace").strip().splitlines()
# 找到含 44 个 token 的那一行
gen = None
for line in out:
    toks = line.replace(",", " ").split()
    if len(toks) >= 40 and toks[0].startswith("N10"):
        gen = toks
        break
print("planner 重算:", "EQUAL" if gen == tokens else ("DIFFER: %r" % (gen,)))
if gen != tokens:
    print("落盘:", tokens)

print()
print("t6 新键 planner.bridge_approach_align_penalty_m =",
      cfg.get("planner.bridge_approach_align_penalty_m"))

print()
E5 = geo.block_to_meters("E5"); F5 = geo.block_to_meters("F5"); F3 = geo.block_to_meters("F3")
bc = ((E5[0]+F5[0])/2, (E5[1]+F5[1])/2)
import math
print("E5=%s F5=%s 板心=%s F3=%s" % (E5, F5, bc, F3))
print("F3→板心 = %.4f m ; F3→E5 = %.4f ; F3→F5 = %.4f"
      % (math.hypot(F3[0]-bc[0], F3[1]-bc[1]),
         math.hypot(F3[0]-E5[0], F3[1]-E5[1]),
         math.hypot(F3[0]-F5[0], F3[1]-F5[1])))
print("窗口 route.shoot_distance_m=%.2f ± %.2f ⇒ [%.2f, %.2f]"
      % (cfg.get("route.shoot_distance_m"), cfg.get("route.shoot_distance_tol_m"),
         cfg.get("route.shoot_distance_m") - cfg.get("route.shoot_distance_tol_m"),
         cfg.get("route.shoot_distance_m") + cfg.get("route.shoot_distance_tol_m")))

print()
print("== 快照哈希（本次复核）==")
for p in ("config/robot.yaml", "config/map.yaml", "src/dzbrain/dzbrain/tools/sim.py",
          "src/dzbrain/dzbrain/route/physics.py", "src/dzbrain/dzbrain/route/planner.py",
          "tools/route_check.py", "src/dzbrain/dzbrain/core/config.py",
          "tools/config_selfcheck.py"):
    b = open(os.path.join(ROOT, p.replace("/", os.sep)), "rb").read()
    print("  %s  %s  (%d B)" % (p, hashlib.sha256(b).hexdigest().upper()[:16], len(b)))
print("  git HEAD =", subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                                    capture_output=True).stdout.decode().strip())
print("  auto_start_delay: legacy =", pe and raw["path_executor"]["ros__parameters"].get("auto_start_delay"),
      "; mission.auto_start_delay_s =", cfg.get("mission.auto_start_delay_s"))
