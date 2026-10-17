# -*- coding: utf-8 -*-
"""t3 复核：几何与挡板侵入的独立确认（只读）。

① block_to_meters / barrier_centerline 的坐标口径；
② 用项目自身的 footprint_diagnostic（折线口径 + ref_offset）复算 44 点路线；
③ 自写"沿折线 + offset 的车体角点极值"，验证"必要条件 ref_x ≥ 0.175+0.215=0.39?"。
"""
import io, os, sys, math, json

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
sys.path.insert(0, os.path.join(ROOT, "src", "dzbrain"))
os.chdir(ROOT)

from dzbrain.core import geometry as geo
from dzbrain.route import physics as ph
from dzbrain.core.config import Config

cfg = Config.load(os.path.join(ROOT, "config", "robot.yaml"))
tokens = cfg.get("route.tokens")

for b in ("G5", "G6", "H6", "I6", "J6", "B6", "B7", "F3", "K5", "N5"):
    print("%-4s -> %s" % (b, tuple(round(v, 4) for v in geo.block_to_meters(b))))
print("barrier_centerline =", ph.barrier_centerline())
print("required_barrier_distance_m =", ph.required_barrier_distance_m())
print("body length/width/offset =", cfg.get("vehicle.length_m"), cfg.get("vehicle.width_m"),
      cfg.get("vehicle.ref_offset_m"))
forward = float(cfg.get("vehicle.length_m")) / 2.0 + float(cfg.get("vehicle.ref_offset_m"))
print("参考点→车头沿 = %.4f m ; 行 6 格心到挡板中心线 = %.4f m ; 是否越线 = %s"
      % (forward, abs(geo.block_to_meters("G6")[0] - 1.75), forward > 0.175))

# ---- ② 项目自身的折线足迹诊断 ----
import yaml
from dzbrain.tasks.mission import footprint_diagnostic
map_cfg = yaml.safe_load(io.open(os.path.join(ROOT, "config", "map.yaml"), encoding="utf-8"))
rep = footprint_diagnostic(tokens, map_cfg, cfg)
print()
print("footprint_diagnostic: 采样 %d / offset %.3f / min_barrier %.6f @ %s"
      % (rep.samples, rep.offset_m, rep.min_barrier_m, rep.min_barrier_where))
print("  min_obstacle %.6f @ %s / min_target %.6f @ %s"
      % (rep.min_obstacle_m, rep.min_obstacle_where, rep.min_target_m, rep.min_target_where))
for r in rep.risks:
    print("  RISK:", r)
