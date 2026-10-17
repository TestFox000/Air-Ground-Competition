# -*- coding: utf-8 -*-
"""t57 开工前：t50 的 output/实际改动面 + 当前射击窗口现状。"""
import json
import pathlib

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


for t in walk(d):
    if str(t.get("id")) == "t50":
        print(f"### t50 status={t.get('status')} attempt={t.get('attempt')} assignee={t.get('assignee')}")
        print("objective:", str(t.get("objective"))[:400])
        out = str(t.get("output") or "")
        print(f"--- output ({len(out)} 字) 前 2000 ---")
        print(out[:2000])
        cp = t.get("changedPaths")
        print("changedPaths:", json.dumps(cp, ensure_ascii=False)[:500] if cp else None)
        break
