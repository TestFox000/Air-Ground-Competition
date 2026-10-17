# -*- coding: utf-8 -*-
"""t58 开工侦察：依赖状态 + 在册 known-red + 工作树 + 套件数。"""
import json
import pathlib
import subprocess

ROOT = pathlib.Path(r"D:\User\Desktop\比赛存档\v1")
d = json.loads((ROOT / ".agent-teams" / "air-ground-v1" / "team.json").read_text(encoding="utf-8"))


def walk(o):
    if isinstance(o, dict):
        if {"id", "status"} <= set(o):
            yield o
        for v in o.values():
            yield from walk(v)
    elif isinstance(o, list):
        for v in o:
            yield from walk(v)


tasks = {str(t.get("id")): t for t in walk(d)}
print("=== t58 依赖与状态 ===")
t58 = tasks.get("t58", {})
print("  status =", t58.get("status"), " assignee =", t58.get("assignee"))
for k in ("deps", "dependsOn", "dependencies"):
    if k in t58:
        print(f"  {k} =", t58[k])
print()
print("=== 相关卡终态检查（t50/t51/t53/t57/t58）===")
for tid in ("t50", "t51", "t53", "t57", "t58"):
    t = tasks.get(tid, {})
    print(f"  {tid}: status={t.get('status'):11} assignee={str(t.get('assignee')):16} attempt={t.get('attempt')}")

print()
print("=== 在册 known-red（docs/回归守护_known_reds.md 现行口径段）===")
kr = ROOT / "dzrobot_v2" / "docs" / "回归守护_known_reds.md"
txt = kr.read_text(encoding="utf-8", errors="replace")
print(f"  文件 {len(txt.splitlines())} 行 / {kr.stat().st_size} B")
for i, ln in enumerate(txt.splitlines(), 1):
    if any(k in ln for k in ("在册", "名单", "5.6", "7.4", "3.5b", "3.6", "4.3", "4.4", "4.5", "新增红")):
        print(f"    {i:4}: {ln.strip()[:150]}")

print()
print("=== 工作树状态（t53 应只剩 M planner.py + ?? test_corner_cut.py）===")
r = subprocess.run(["git", "-C", str(ROOT / "dzrobot_v2"), "status", "--short"],
                   capture_output=True, text=True, encoding="utf-8", errors="replace")
lines = [l for l in (r.stdout or "").splitlines() if l.strip()]
print(f"  脏路径总数 = {len(lines)}")
for l in lines:
    print("   ", l)

print()
print("=== 套件清单 ===")
suites = sorted(p.name for p in (ROOT / "dzrobot_v2" / "src" / "dzbrain" / "tests").glob("test_*.py"))
print(f"  test_*.py 数量 = {len(suites)}")
print("  ", ", ".join(suites))
