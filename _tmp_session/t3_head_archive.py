# -*- coding: utf-8 -*-
"""t3：用 `git archive HEAD` 导出整棵 src/dzbrain 到临时目录，验证 HEAD 提交能否 import + 跑测试。

只读仓库（archive 不改工作树、不建 worktree、不打 tag）。
"""
import os, subprocess, sys, shutil

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
TMP = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t3_head_tree"
shutil.rmtree(TMP, ignore_errors=True)
os.makedirs(TMP)

with open(os.path.join(TMP, "src.tar"), "wb") as fh:
    p = subprocess.Popen(["git", "-C", ROOT, "archive", "HEAD", "src/dzbrain", "config", "tools"],
                         stdout=fh, stderr=subprocess.PIPE)
    p.wait()
subprocess.run(["tar", "-xf", os.path.join(TMP, "src.tar")], cwd=TMP, check=True)
os.remove(os.path.join(TMP, "src.tar"))
print("HEAD 导出完成:", os.listdir(TMP))

env = dict(os.environ, PYTHONIOENCODING="utf-8")
r = subprocess.run([sys.executable, "-c",
                    "import sys; sys.path.insert(0,'src/dzbrain');"
                    "import dzbrain.route.physics as ph;"
                    "print('HEAD physics import OK, DEFAULT_REF_OFFSET_M =', ph.DEFAULT_REF_OFFSET_M);"
                    "print('has body_bridge_wall_clearance_m =', hasattr(ph,'body_bridge_wall_clearance_m'))"],
                   cwd=TMP, capture_output=True, env=env)
print("--- import ---")
print(r.stdout.decode("utf-8", "replace"))
print(r.stderr.decode("utf-8", "replace")[-800:])

for t in ("test_physics.py", "test_planner.py", "test_core.py"):
    r = subprocess.run([sys.executable, os.path.join("src", "dzbrain", "tests", t)],
                       cwd=TMP, capture_output=True, env=env)
    out = (r.stdout.decode("utf-8", "replace") + r.stderr.decode("utf-8", "replace"))
    tail = [l for l in out.splitlines() if l.strip()][-3:]
    print("--- %s rc=%d" % (t, r.returncode))
    for l in tail:
        print("     ", l[:140])
