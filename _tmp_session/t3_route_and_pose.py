# -*- coding: utf-8 -*-
"""t3 复核：路线双真源一致性 + 终点停位南墙几何 + 配置消费点（只读）。"""
import io, os, sys, math, csv, yaml

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
sys.path.insert(0, os.path.join(ROOT, "src", "dzbrain"))
os.chdir(ROOT)
from dzbrain.core.config import Config
from dzbrain.route.parser import parse_mission

cfg = Config.load(os.path.join(ROOT, "config", "robot.yaml"))
tokens = list(cfg.get("route.tokens") or [])
raw = yaml.safe_load(io.open(os.path.join(ROOT, "config", "robot.yaml"), encoding="utf-8"))
pe = raw["path_executor"]["ros__parameters"]["mission_blocks"]
print("route.tokens      : %d 点" % len(tokens))
print("mission_blocks    : %d 点" % len(pe))
print("逐 token 恒等      :", [str(t) for t in tokens] == [str(t) for t in pe])
if [str(t) for t in tokens] != [str(t) for t in pe]:
    for i, (a, b) in enumerate(zip(tokens, pe)):
        if str(a) != str(b):
            print("   首个分歧 #%d: route.tokens=%r  mission_blocks=%r" % (i, a, b))
            break

# ---- 终点停位南墙几何（t2 §6 低风险观察②）----
rows = list(csv.DictReader(io.open(os.path.join(
    ROOT, "artifacts", "sim", "t2_baseline_44tok", "v035", "traj.csv"), encoding="utf-8")))
L, W, OFF = 0.28, 0.135, 0.075
last = rows[-1]
tx, ty, yaw = float(last["truth_x"]), float(last["truth_y"]), float(last["truth_yaw"])
cx = tx + OFF * math.cos(yaw)
cy = ty + OFF * math.sin(yaw)
cs = [(cx + a * math.cos(yaw) - b * math.sin(yaw), cy + a * math.sin(yaw) + b * math.cos(yaw))
      for a, b in ((L/2, W/2), (L/2, -W/2), (-L/2, -W/2), (-L/2, W/2))]
print("\n终点真值位姿: x=%.4f y=%.4f yaw=%.2f°" % (tx, ty, math.degrees(yaw)))
print("  车体角点 y 最小值 = %.4f m（南墙 y=0 ⇒ 余量 %+.4f m）"
      % (min(c[1] for c in cs), min(c[1] for c in cs)))
print("  offset=0 口径最小 y = %.4f" % min(
    [(cy + a*math.sin(yaw) + b*math.cos(yaw)) for a, b in ((L/2, W/2), (L/2, -W/2), (-L/2, -W/2), (-L/2, W/2))]))
print("  N5 格心 (2.275, 0.175) 距南墙 0.175 m ; 半长+偏移 = %.3f m" % (L/2 + OFF))

# ---- 部署配置 _deploy_cfg.yaml 是否被引用 ----
import subprocess
r = subprocess.run(["git", "-C", ROOT, "grep", "-l", "_deploy_cfg", "HEAD"],
                   capture_output=True)
print("\nHEAD 中引用 _deploy_cfg 的文件:", r.stdout.decode("utf-8", "replace").strip() or "(无)")
