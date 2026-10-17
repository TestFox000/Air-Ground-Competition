# -*- coding: utf-8 -*-
"""t3 复核：route_check 的物理几何是否真"随配置走"（只读；用 _tmp_session 下的探针副本）。

两种注入位置：
  A. 注入 /dzactuator 段（真源位置）—— core/config 迁移链能读到；
  B. 注入 /dzbrain 段（route_check.py:1260 的读法）。
观察 route_check 打印的判据阈值是否随之变化。
"""
import io, os, subprocess, sys

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
src = os.path.join(ROOT, "config", "robot.yaml")
txt = io.open(src, encoding="utf-8").read()

# A：改 /dzactuator 段的真源值
a = txt.replace("      length_m: 0.28", "      length_m: 0.99").replace(
    "      width_m: 0.135", "      width_m: 0.50")
assert a != txt
pa = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t3_pA_robot.yaml"
io.open(pa, "w", encoding="utf-8").write(a)

# B：在 /dzbrain 段里插入 vehicle:/field: 块（route_check 的读法）
anchor = "    pursuit:\n      wheelbase_m: 0.157"
b = txt.replace(anchor, "    vehicle:\n      length_m: 0.99\n      width_m: 0.50\n"
                        "      coverage_radius_m: 0.035\n"
                        "    field:\n      obstacle_radius_m: 0.30\n      mine_pad_m: 0.12\n"
                        "      barrier_width_m: 0.02\n      barrier_cols: \"D..N\"\n" + anchor,
                1)
assert b != txt
pb = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t3_pB_robot.yaml"
io.open(pb, "w", encoding="utf-8").write(b)

env = dict(os.environ, PYTHONIOENCODING="utf-8")
for tag, cfgp in (("真源", src), ("A: 注入 /dzactuator（真源位置）", pa),
                  ("B: 注入 /dzbrain（route_check 的读法）", pb)):
    r = subprocess.run([sys.executable, "tools/route_check.py", "--config", cfgp],
                       cwd=ROOT, capture_output=True, env=env)
    out = r.stdout.decode("utf-8", "replace")
    keep = [l.strip() for l in out.splitlines()
            if "障碍净空" in l or "挡板不触线" in l or "车体不压靶板" in l]
    print("\n[%s] rc=%d" % (tag, r.returncode))
    for l in keep:
        print("   ", l)
