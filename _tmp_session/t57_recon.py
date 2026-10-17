# -*- coding: utf-8 -*-
"""t57 开工前侦察：我的在办卡 + t57 实现痕迹 + 现役停站点 token。"""
import json
import pathlib

ROOT = pathlib.Path(r"D:\User\Desktop\比赛存档\v1")
TEAM = ROOT / ".agent-teams" / "air-ground-v1" / "team.json"
d = json.loads(TEAM.read_text(encoding="utf-8"))


def walk(o):
    if isinstance(o, dict):
        if {"id", "status", "subject"} <= set(o):
            yield o
        for v in o.values():
            yield from walk(v)
    elif isinstance(o, list):
        for v in o:
            yield from walk(v)


print("=== 我的在办卡（assignee=reviewer 且未结束）===")
rows = [t for t in walk(d) if t.get("assignee") == "reviewer"
        and t.get("status") not in ("completed", "cancelled", "failed")]
for t in sorted(rows, key=lambda r: str(r.get("id"))):
    print(f"  {t.get('id'):6} [{t.get('status')}] attempt={t.get('attempt')}  {str(t.get('subject'))[:64]}")
if not rows:
    print("  （无）")

print()
print("=== 团队所有未结束卡（看谁在写什么）===")
for t in sorted(walk(d), key=lambda r: str(r.get("id"))):
    if t.get("status") in ("claimed", "in_progress"):
        print(f"  {t.get('id'):6} [{t.get('status'):11}] {str(t.get('assignee')):16} {str(t.get('subject'))[:58]}")

print()
print("=== t57 实现痕迹（shoot_distance / F3* / E4*）===")
for rel, pats in (("dzrobot_v2/src/dzbrain/dzbrain/core/config.py", ("shoot_distance",)),
                  ("dzrobot_v2/src/dzbrain/dzbrain/route/planner.py", ("shoot_distance", "F3", "E4")),
                  ("dzrobot_v2/config/robot.yaml", ("shoot_distance", "E4*", "F3*"))):
    p = ROOT / rel
    txt = p.read_text(encoding="utf-8", errors="replace")
    hits = [(i, ln.strip()) for i, ln in enumerate(txt.splitlines(), 1)
            if any(k in ln for k in pats)]
    print(f"  {rel}: {len(hits)} 命中")
    for i, ln in hits[:6]:
        print(f"     {i:5}: {ln[:115]}")
