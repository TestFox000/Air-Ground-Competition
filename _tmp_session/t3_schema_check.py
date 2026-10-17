# -*- coding: utf-8 -*-
"""t3：核对 SCHEMA 是否登记了 planner./vehicle./field. 等键（决定"未知配置段"warn 的性质）。"""
import os, sys, io

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
sys.path.insert(0, os.path.join(ROOT, "src", "dzbrain"))
from dzbrain.core.config import SCHEMA, SECTION_PREFIX

keys = ["planner.engine", "planner.tracker_max_seconds", "planner.auto_plan_enabled",
        "planner.footprint_gate", "planner.map_path", "planner.fallback_to_tokens",
        "planner.allow_reverse", "planner.max_seconds", "planner.pivot_penalty",
        "planner.mine_turn_penalty", "planner.obstacle_margin_m",
        "vehicle.length_m", "vehicle.width_m", "vehicle.coverage_radius_m",
        "vehicle.ref_offset_m", "field.obstacle_radius_m", "field.mine_pad_m",
        "field.barrier_width_m", "field.barrier_cols", "field.bridge_entry_redundancy_m"]
for k in keys:
    print("%-34s in SCHEMA = %s" % (k, k in SCHEMA))

print()
from dzbrain.core.config import Config
cfg = Config.load(os.path.join(ROOT, "config", "robot.yaml"))
iss = cfg.validate(strict=False)
warns = [i for i in iss if i.level == "warn"]
errs = [i for i in iss if i.level == "error"]
print("validate(strict=False): error=%d warn=%d info=%d" % (len(errs), len(warns),
                                                            len([i for i in iss if i.level == 'info'])))
for i in warns:
    print("   WARN:", i)
print("known_prefixes 缺的前缀（对比实际出现的顶层段）:")
import yaml
raw = yaml.safe_load(io.open(os.path.join(ROOT, "config", "robot.yaml"), encoding="utf-8"))
flat = sorted(set(k.split(".")[0] + "." for k in cfg.keys()))
print("   实际段:", flat)
