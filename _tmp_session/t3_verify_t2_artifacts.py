# -*- coding: utf-8 -*-
"""t3 复核：对 w2 交付物做独立哈希/指标核对（只读）。"""
import hashlib, io, os, json

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
A = os.path.join(ROOT, "artifacts", "sim", "t2_baseline_44tok")
claim = {"v015": "9839BA29", "v035": "DB951683", "v040": "053172DA"}
for v, want in claim.items():
    p = os.path.join(A, v, "traj.csv")
    h = hashlib.sha256(open(p, "rb").read()).hexdigest().upper()
    # 复跑目录
    r = os.path.join(A, "repeat", v, "traj.csv")
    h2 = hashlib.sha256(open(r, "rb").read()).hexdigest().upper()
    print("%s  w2宣称=%s.. 实测=%s..  %s   复跑目录=%s..  %s"
          % (v, want, h[:8], "MATCH" if h.startswith(want) else "MISMATCH",
             h2[:8], "IDENTICAL" if h == h2 else "DIFFERENT"))

print()
s = json.load(io.open(os.path.join(A, "baseline_summary.json"), encoding="utf-8"))
for v in ("0.15", "0.35", "0.40"):
    d = s[v]
    print("%-4s time=%.2f rms=%.4f completed=%s mines=%g/%g obs_hit=%g board_overlap=%g "
          "barrier_breach=%g strictB=%g strictC=%g"
          % (v, d["time_s"], d["cross_track_rms_m"], d["completed"],
             d["mine_pressed_count"], d["mine_total"], d["obstacle_hit_steps"],
             d["board_overlap_steps"], d["barrier_breach_steps"],
             d["barrier_strict_offset0_steps"], d["barrier_strict_offset0075_steps"]))

# t2 报告 sha256
d = open(os.path.join(ROOT, "docs", "仿真复验_第五张图44token基线_t2.md"), "rb").read()
print("\nt2 报告 sha256 =", hashlib.sha256(d).hexdigest().upper()[:8], "字节", len(d))
