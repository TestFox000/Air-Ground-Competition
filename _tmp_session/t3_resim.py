# -*- coding: utf-8 -*-
"""t3 复核：在当前工作树上重跑 sim_cli（0.15/0.35/0.40），与 t2 产物 traj.csv 逐字节比对。

只读：产物写 _tmp_session/t3_resim/**，不碰 artifacts/** 与 config/**。
"""
import os, sys, json, hashlib, io, contextlib

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
sys.path.insert(0, os.path.join(ROOT, "src", "dzbrain"))
os.chdir(ROOT)

from dzbrain.tools import sim as S
print("SimConfig 默认：speed_scale_err=%r seed=%r anchor=%r dt=%r latency=%r"
      % (S.SimConfig.speed_scale_err, S.SimConfig.seed, S.SimConfig.anchor_on_arrival,
         S.SimConfig.dt, S.SimConfig.cmd_latency_s))
from dzbrain.tools.sim import run_sim, write_csv
from dzbrain.core.config import Config
from dzbrain.route.parser import parse_mission

cfg = Config.load(os.path.join(ROOT, "config", "robot.yaml"))
tokens = cfg.get("route.tokens")
print("route.tokens 长度 =", len(tokens), " 停站 =", [t for t in tokens if t.endswith("*")])

want = {"0.15": "9839BA29", "0.35": "DB951683", "0.40": "053172DA"}
out = os.path.join(r"D:\User\Desktop\比赛存档\v1\_tmp_session", "t3_resim")
os.makedirs(out, exist_ok=True)

for v in ("0.15", "0.35", "0.40"):
    route = parse_mission(list(tokens), name="cfg:route.tokens")
    sim = S.SimConfig(cruise_speed_mps=float(v)) if "cruise_speed_mps" in S.SimConfig.__dataclass_fields__ \
        else None
    ov = cfg.override("pursuit.cruise_speed_mps", float(v))
    r = run_sim(route, ov, S.SimConfig())
    p = write_csv(r, os.path.join(out, "traj_%s.csv" % v.replace(".", "")))
    h = hashlib.sha256(open(p, "rb").read()).hexdigest().upper()
    print("%-5s sim重跑 sha256=%s..  t2宣称=%s..  %s" %
          (v, h[:8], want[v], "MATCH" if h.startswith(want[v]) else "MISMATCH"))
