# -*- coding: utf-8 -*-
"""t3 收尾证据：① HEAD 干净检出可导入？② route_check 的物理几何是否真"随配置走"？

只读：不修改仓库任何文件；临时检出/副本都放 _tmp_session。
"""
import os, subprocess, sys, io, shutil, yaml, tempfile

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
TMP = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t3_head_checkout"
shutil.rmtree(TMP, ignore_errors=True)
os.makedirs(TMP)
# 只取 HEAD 的两份 route 源码，放到能 import 的临时树里
def blob(path, dest):
    out = subprocess.run(["git", "-C", ROOT, "show", "HEAD:" + path], capture_output=True).stdout
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    open(dest, "wb").write(out)
    return len(out)

for p in ("src/dzbrain/dzbrain/route/physics.py", "src/dzbrain/dzbrain/route/planner.py",
          "src/dzbrain/dzbrain/route/__init__.py", "src/dzbrain/dzbrain/__init__.py",
          "src/dzbrain/dzbrain/core/__init__.py", "src/dzbrain/dzbrain/core/geometry.py",
          "src/dzbrain/dzbrain/core/config.py"):
    dest = os.path.join(TMP, p.replace("/", os.sep))
    print("HEAD blob %-52s %6d B" % (p, blob(p, dest)))

r = subprocess.run([sys.executable, "-c",
                    "import sys; sys.path.insert(0, r'%s'); import dzbrain.route.physics as p; "
                    "print('HEAD physics import OK'); print('BRIDGE_ENTRY_LOWER_ROW_IDX =', "
                    "p.BRIDGE_ENTRY_LOWER_ROW_IDX); print('has body_bridge_wall_clearance_m =', "
                    "hasattr(p, 'body_bridge_wall_clearance_m'))" % os.path.join(TMP, "src", "dzbrain")],
                   capture_output=True, cwd=TMP)
print(r.stdout.decode("utf-8", "replace"), r.stderr.decode("utf-8", "replace")[-400:])

# ---- ② route_check 物理几何：注入哨兵后看门禁阈值是否变化 ----
src = os.path.join(ROOT, "config", "robot.yaml")
txt = io.open(src, encoding="utf-8").read()
sent = txt.replace("      length_m: 0.28", "      length_m: 0.99").replace(
    "      width_m: 0.135", "      width_m: 0.50")
probe = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t3_probe2_robot.yaml"
io.open(probe, "w", encoding="utf-8").write(sent)
env = dict(os.environ, PYTHONIOENCODING="utf-8")
for tag, cfgp in (("真源", src), ("哨兵(长0.99/宽0.50 注入 /dzactuator)", probe)):
    r = subprocess.run([sys.executable, "tools/route_check.py", "--config", cfgp],
                       cwd=ROOT, capture_output=True, env=env)
    out = r.stdout.decode("utf-8", "replace")
    keep = [l.strip() for l in out.splitlines()
            if "障碍净空" in l or "挡板不触线" in l or "车体不压靶板" in l]
    print("\n[%s] rc=%d" % (tag, r.returncode))
    for l in keep:
        print("   ", l)
