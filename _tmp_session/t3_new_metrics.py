# -*- coding: utf-8 -*-
"""t3：给 t6 新路线的安全指标（与 t2 基线对照），并确认 G6/J6 两处拐角的几何冲突。"""
import io, json, math, os, sys

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
sys.path.insert(0, os.path.join(ROOT, "src", "dzbrain"))
os.chdir(ROOT)
from dzbrain.core import geometry as geo

base = json.load(io.open(os.path.join(ROOT, "artifacts", "sim", "t2_baseline_44tok",
                                      "baseline_summary.json"), encoding="utf-8"))
print("t2 基线 @0.35:", {k: base["0.35"][k] for k in
      ("completed", "time_s", "cross_track_rms_m", "mine_pressed_count", "obstacle_hit_steps",
       "board_overlap_steps", "k_seal_violation_steps", "out_of_bounds_steps")})

# 新路线指标
from dzbrain.core.config import Config
from dzbrain.route.parser import parse_mission
from dzbrain.tools import sim as S
import re, subprocess
env = dict(os.environ, PYTHONIOENCODING="utf-8")
r = subprocess.run([sys.executable, "tools/plan_route.py", "--config", "config/robot.yaml",
                    "--engine", "tracker-safe", "--emit", "tokens", "--quiet"],
                   cwd=ROOT, capture_output=True, env=env)
toks = [t for t in re.split(r"[\s,]+", r.stdout.decode("utf-8", "replace").strip()) if t]
toks = [t for t in toks if t != "y"]
toks[-1] = "N5,y"
cfg = Config.load(os.path.join(ROOT, "config", "robot.yaml")).override(
    "pursuit.cruise_speed_mps", 0.35)
res = S.run_sim(parse_mission(toks, name="new"), cfg, S.SimConfig())
print("t6 新路线 @0.35:", {k: res.metrics.get(k) for k in
      ("completed", "time_s", "cross_track_rms_m", "mine_pressed_count", "obstacle_hit_steps",
       "board_overlap_steps", "k_seal_violation_steps", "out_of_bounds_steps")})
print("  路径长:", res.metrics.get("path_length_m"), " 雷总数:", res.metrics.get("mine_total"))

print()
print("== 拐角 vs 挡板的几何（x 轴=行，y 轴=列；挡板 x=1.75 ± 0.01，y∈[0,3.85]）==")
for b in ("M10", "M9", "L10", "L9", "K9", "G5", "G6", "H6", "I6", "J6", "J5", "K5"):
    x, y = geo.block_to_meters(b)
    print("  %-4s x=%.3f y=%.3f  |x-1.75|=%.3f  端部扫掠半长=0.215 → %s"
          % (b, x, y, abs(x - 1.75),
             "侵入挡板 %.1f mm" % ((0.215 + 0.01 - abs(x - 1.75)) * 1000)
             if abs(x - 1.75) < 0.225 and 0.0 <= y <= 3.85 else "安全"))
