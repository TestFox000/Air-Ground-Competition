# -*- coding: utf-8 -*-
"""t3：确认 planner 产物漂移的成因 = t6 新键 planner.bridge_approach_align_penalty_m=2.0。

只读：把 robot.yaml 复制到 _tmp_session 做探针（不改仓库文件）。
"""
import io, os, re, subprocess, sys

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
sys.path.insert(0, os.path.join(ROOT, "src", "dzbrain"))
os.chdir(ROOT)
from dzbrain.core.config import Config

src = os.path.join(ROOT, "config", "robot.yaml")
txt = io.open(src, encoding="utf-8").read()
key = "      bridge_approach_align_penalty_m: 2.0"
assert key in txt, "未找到 t6 新键"

probes = {
    "t6键=2.0（当前工作树）": txt,
    "t6键=0.0（关闭 t6）": txt.replace(key, "      bridge_approach_align_penalty_m: 0.0"),
    "t6键整行删除": txt.replace(key + "\n", ""),
}
# 也去掉 t4 冗余键做交叉对照
key4 = "      bridge_entry_redundancy_m: 0.0875"
probes["t6=2.0 且 t4冗余=0.0"] = txt.replace(key4, "      bridge_entry_redundancy_m: 0.0")

cfg0 = Config.load(src)
committed = [str(t) for t in cfg0.get("route.tokens")]
print("落盘 route.tokens (%d):" % len(committed), " ".join(committed))
print()

env = dict(os.environ, PYTHONIOENCODING="utf-8")
for idx, (tag, content) in enumerate(probes.items()):
    p = os.path.join(r"D:\User\Desktop\比赛存档\v1\_tmp_session",
                     "t3_probe_%d.yaml" % idx)
    io.open(p, "w", encoding="utf-8").write(content)
    r = subprocess.run([sys.executable, "tools/plan_route.py", "--config", p,
                        "--engine", "tracker-safe", "--check", "--quiet"],
                       cwd=ROOT, capture_output=True, env=env)
    out = r.stdout.decode("utf-8", "replace")
    gen = None
    for line in out.splitlines():
        s = line.strip()
        if s.startswith("N10 ") and "N5" in s and len(s.split()) > 20:
            gen = s.replace(",", " ").split()
            break
    same = (gen == committed)
    print("[%s] rc=%d  EQUAL=%s" % (tag, r.returncode, same))
    if gen:
        print("     生成 %d token: %s" % (len(gen), " ".join(gen)))
    else:
        print("     未解析到路线；输出尾部:", out.strip().splitlines()[-2:])
    os.remove(p)
