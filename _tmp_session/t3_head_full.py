# -*- coding: utf-8 -*-
"""t3：① HEAD 干净树上 test_corner_cut 是否全绿（证明工作树 5 红是新增）；
② t6 键的"缺省=关闭"注释 vs 实际兜底常量 2.0 的自相矛盾取证。
"""
import io, os, re, shutil, subprocess, sys

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
TMP = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t3_head_full"
env = dict(os.environ, PYTHONIOENCODING="utf-8")

shutil.rmtree(TMP, ignore_errors=True)
os.makedirs(TMP)
with open(os.path.join(TMP, "t.tar"), "wb") as fh:
    subprocess.run(["git", "-C", ROOT, "archive", "HEAD", "src/dzbrain", "config", "tools"],
                   stdout=fh, check=True)
subprocess.run(["tar", "-xf", os.path.join(TMP, "t.tar")], cwd=TMP, check=True)
os.remove(os.path.join(TMP, "t.tar"))

for t in ("test_corner_cut", "test_planner", "test_physics", "test_sim", "test_sim_guard"):
    r = subprocess.run([sys.executable, os.path.join("src", "dzbrain", "tests", t + ".py")],
                       cwd=TMP, capture_output=True, env=env)
    out = (r.stdout.decode("utf-8", "replace") + r.stderr.decode("utf-8", "replace"))
    tail = [l for l in out.splitlines() if l.strip()][-1][:110]
    print("HEAD %-16s rc=%d  %s" % (t, r.returncode, tail))

print()
print("== t6 键注释 vs 兜底常量 ==")
for path, pat in ((os.path.join(ROOT, "config/robot.yaml"), r"bridge_approach_align_penalty_m"),
                  (os.path.join(ROOT, "src/dzbrain/dzbrain/route/planner.py"),
                   r"BRIDGE_APPROACH_ALIGN_PENALTY_M\s*=")):
    L = io.open(path, encoding="utf-8").read().splitlines()
    for i, l in enumerate(L, 1):
        if re.search(pat, l):
            print("  %s:%d  %s" % (os.path.basename(path), i, l.strip()[:150]))
    # 打印"缺省 0.0"声称的上下文
    for i, l in enumerate(L, 1):
        if "缺省 0.0" in l or "0.0 = 关闭" in l:
            print("  %s:%d  %s" % (os.path.basename(path), i, l.strip()[:150]))
