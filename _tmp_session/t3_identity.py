# -*- coding: utf-8 -*-
"""t3：严格比较 planner 重算产物 vs 落盘 route.tokens（修正分列解析）。"""
import io, os, re, subprocess, sys

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
sys.path.insert(0, os.path.join(ROOT, "src", "dzbrain"))
os.chdir(ROOT)
from dzbrain.core.config import Config

cfg = Config.load(os.path.join(ROOT, "config", "robot.yaml"))
tokens = [str(t) for t in cfg.get("route.tokens")]

env = dict(os.environ, PYTHONIOENCODING="utf-8")
for extra in ([], ["--set", "planner.bridge_approach_align_penalty_m=0.0"]):
    r = subprocess.run([sys.executable, "tools/plan_route.py", "--config", "config/robot.yaml",
                        "--engine", "tracker-safe", "--check", "--quiet"] + extra,
                       cwd=ROOT, capture_output=True, env=env)
    out = r.stdout.decode("utf-8", "replace")
    # planner 会把路线以空格分隔打印连续两行（plan_route.py 的 report）
    gen = None
    for line in out.splitlines():
        s = line.strip()
        if s.startswith("N10 ") and "N5" in s:
            gen = s.replace(",", " ").split()
            if gen and gen[0] == "N10":
                pass
    # 更稳：用 --emit tokens
    r2 = subprocess.run([sys.executable, "tools/plan_route.py", "--config", "config/robot.yaml",
                         "--engine", "tracker-safe", "--emit", "tokens", "--quiet"] + extra,
                        cwd=ROOT, capture_output=True, env=env)
    raw = r2.stdout.decode("utf-8", "replace").strip()
    gen = [t for t in re.split(r"[\s,]+", raw) if t]
    print("extra=%s rc=%d" % (extra, r.returncode))
    print("  生成 %d token: %s" % (len(gen), " ".join(gen)))
    print("  落盘 %d token: %s" % (len(tokens), " ".join(tokens)))
    print("  EQUAL =", gen == tokens)
    if gen != tokens:
        for i, (a, b) in enumerate(zip(gen, tokens)):
            if a != b:
                print("  首个分歧 #%d：生成 %s vs 落盘 %s" % (i, a, b))
                break
    print("  route_check:", [l.strip()[:60] for l in out.splitlines() if "route_check" in l])
