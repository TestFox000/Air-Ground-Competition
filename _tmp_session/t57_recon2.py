# -*- coding: utf-8 -*-
"""t40/t57 归属确认。"""
import json
import pathlib

d = json.loads((pathlib.Path(r"D:\User\Desktop\比赛存档\v1\.agent-teams\air-ground-v1\team.json")
                ).read_text(encoding="utf-8"))


def walk(o):
    if isinstance(o, dict):
        if {"id", "status"} <= set(o):
            yield o
        for v in o.values():
            yield from walk(v)
    elif isinstance(o, list):
        for v in o:
            yield from walk(v)


for t in sorted(walk(d), key=lambda r: str(r.get("id"))):
    if str(t.get("id")) in ("t40", "t57", "t39", "t50"):
        print(f"{t.get('id'):5} [{t.get('status'):12}] assignee={str(t.get('assignee')):16} "
              f"attempt={t.get('attempt')}  attemptId={str(t.get('attemptId'))[:12]}")
        print(f"      subject: {str(t.get('subject'))[:90]}")
