# -*- coding: utf-8 -*-
"""Scan captain inbox jsonl files for team-lesson signals."""
import json, io, os, glob, re

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\.agent-teams"
OUT = r"D:\User\Desktop\比赛存档\v1\_tmp_session\heavy-digest\_inbox.txt"
files = sorted(glob.glob(os.path.join(ROOT, "**", "inbox", "*.jsonl"), recursive=True))
files += sorted(glob.glob(r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src_v1\.agent-teams\**\*.jsonl", recursive=True))

KEY = re.compile(r"(卡死|残留|冲突|重复|重复派|并发|假绿|静默|覆盖|未提交|返工|作废|撤回|超时|阻塞|踩坑|教训|注意|警告|风险|口径|竞态|抢锁|撞车|白跑|白做|重做|失效|错|误|泄露|丢失|覆盖写|半截|残留引用)")

o = io.open(OUT, "w", encoding="utf-8")
for p in files:
    rel = p[len(ROOT)+1:]
    o.write("\n" + "#"*100 + "\nFILE %s\n" % rel)
    try:
        lines = io.open(p, encoding="utf-8").read().splitlines()
    except Exception as e:
        o.write("  ERR %s\n" % e); continue
    o.write("  LINES: %d\n" % len(lines))
    for i, ln in enumerate(lines):
        ln = ln.strip()
        if not ln:
            continue
        try:
            d = json.loads(ln)
        except Exception:
            continue
        content = d.get("content") or d.get("message") or ""
        if isinstance(content, list):
            content = " ".join(str(c.get("text", c)) if isinstance(c, dict) else str(c) for c in content)
        if not isinstance(content, str):
            content = str(content)
        if KEY.search(content):
            head = re.sub(r"\s+", " ", content)
            o.write("\n  [%s] from=%s to=%s ts=%s\n    %s\n" % (
                i, str(d.get("from"))[:24], str(d.get("to"))[:20],
                str(d.get("createdAt") or d.get("ts") or "")[:16], head[:1400]))
o.close()
print("ok", os.path.getsize(OUT), len(files))