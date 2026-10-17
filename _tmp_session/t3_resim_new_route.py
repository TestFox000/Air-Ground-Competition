# -*- coding: utf-8 -*-
"""t3：对 planner 新产物（t6 上线后）做一次独立仿真 + 独立挡板复算（只读，产物落 _tmp_session）。

目的：回答"t6 新路线是否仍是 t2 基线那条 / 是否解决 20mm 挡板侵入"。
"""
import csv, hashlib, io, math, os, re, subprocess, sys, json

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
sys.path.insert(0, os.path.join(ROOT, "src", "dzbrain"))
os.chdir(ROOT)
from dzbrain.core.config import Config
from dzbrain.route.parser import parse_mission
from dzbrain.tools import sim as S

env = dict(os.environ, PYTHONIOENCODING="utf-8")
r = subprocess.run([sys.executable, "tools/plan_route.py", "--config", "config/robot.yaml",
                    "--engine", "tracker-safe", "--emit", "tokens", "--quiet"],
                   cwd=ROOT, capture_output=True, env=env)
gen_raw = r.stdout.decode("utf-8", "replace").strip()
gen = [t for t in re.split(r"[\s,]+", gen_raw) if t]
# 修回 N5,y 形式
gen = [t if t != "y" else "N5,y" for t in gen]
if gen and gen[-1] == "y":
    gen = gen[:-1] + ["N5,y"]
gen = [g for g in gen if g != "y"]

cfg = Config.load(os.path.join(ROOT, "config", "robot.yaml"))
committed = [str(t) for t in cfg.get("route.tokens")]
print("planner 产物 (%d): %s" % (len(gen), " ".join(gen)))
print("落盘 tokens (%d): %s" % (len(committed), " ".join(committed)))
print("EQUAL =", gen == committed)
print("差异位置:", [(i, a, b) for i, (a, b) in enumerate(zip(gen, committed)) if a != b])

L, W = 0.28, 0.135
BAR_LO, BAR_HI, BARY0, BARY1 = 1.740, 1.760, 0.0, 3.85


def corners(px, py, yaw, off):
    cx, cy = px + off * math.cos(yaw), py + off * math.sin(yaw)
    hl, hw = L/2, W/2
    return [(cx + a*math.cos(yaw) - b*math.sin(yaw), cy + a*math.sin(yaw) + b*math.cos(yaw))
            for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def xmin_band(p):
    xs = []
    for i in range(4):
        a, b = p[i], p[(i+1) % 4]
        if BARY0 <= a[1] <= BARY1:
            xs.append(a[0])
        for yy in (BARY0, BARY1):
            if (a[1]-yy)*(b[1]-yy) < 0:
                t = (yy-a[1])/(b[1]-a[1]); xs.append(a[0]+t*(b[0]-a[0]))
    return min(xs) if xs else None


out = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t3_resim_new"
os.makedirs(out, exist_ok=True)
route = parse_mission(list(gen), name="planner-new")
cfg35 = cfg.override("pursuit.cruise_speed_mps", 0.35)
res = S.run_sim(route, cfg35, S.SimConfig())
p = S.write_csv(res, os.path.join(out, "traj_new035.csv"))
h = hashlib.sha256(open(p, "rb").read()).hexdigest().upper()
print("\n新路线 @0.35 仿真: 完成=%s 时长=%.2f s 横向RMS=%.4f 越界=0?  sha256=%s.."
      % (res.metrics.get("completed"), res.metrics["time_s"], res.metrics["cross_track_rms_m"], h[:8]))
print("  与 t2 基线的 0.35 traj sha256 DB951683.. 比对:", h[:8] == "DB951683")

rows = list(csv.DictReader(io.open(p, encoding="utf-8")))
for off in (0.0, 0.075):
    n = 0; worst = 0.0; wt = None
    for row in rows:
        m = xmin_band(corners(float(row["truth_x"]), float(row["truth_y"]),
                              float(row["truth_yaw"]), off))
        if m is None:
            continue
        d = BAR_LO - m
        if d > 0:
            n += 1
            if d > worst:
                worst, wt = d, float(row["t"])
    print("  独立挡板复算 offset=%.3f: 侵入 %d 拍，最深 %+.2f mm @ t=%s"
          % (off, n, worst*1000, wt))
