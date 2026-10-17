# -*- coding: utf-8 -*-
"""One-line-per-task glossary for all teams."""
import json, io, os, glob, datetime, re
from collections import Counter

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\.agent-teams"
SRCV1 = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src_v1\.agent-teams"
OUT = r"D:\User\Desktop\比赛存档\v1\_tmp_session\heavy-digest\_gloss.txt"

paths = sorted(glob.glob(os.path.join(ROOT, "archive", "*", "team.json")))
paths += sorted(glob.glob(os.path.join(ROOT, "camview-ann-team", "team.json")))
paths += sorted(glob.glob(os.path.join(SRCV1, "archive", "*", "team.json")))

def ts(ms):
    try:
        return datetime.datetime.fromtimestamp(ms/1000, datetime.timezone.utc).strftime("%m-%d %H:%M")
    except Exception:
        return "?"

def first_sentences(text, n=3, width=230):
    if not text:
        return ""
    t = re.sub(r"\s+", " ", text).strip()
    parts = re.split(r"(?<=。)|(?<=\n)", t)
    out = []
    for p in parts:
        p = p.strip()
        if not p:
            continue
        if p.startswith("【") or p.startswith("■") or re.match(r"^\d+[\.\)、]", p):
            pass
        out.append(p)
        if len(out) >= n:
            break
    s = " ".join(out)
    return s[:width]

o = io.open(OUT, "w", encoding="utf-8")
for p in paths:
    d = json.load(io.open(p, encoding="utf-8"))
    name = d.get("name")
    tasks = d.get("tasks") or []
    st = Counter(t.get("status") for t in tasks if isinstance(t, dict))
    o.write("\n" + "="*110 + "\nTEAM %s | 成员 %d | 任务 %d | %s | %s→%s\n" % (
        name, len(d.get("members") or []), len(tasks), dict(st),
        ts(min([t.get("createdAt", 0) for t in tasks if isinstance(t, dict)] or [0])),
        ts(max([t.get("updatedAt", 0) for t in tasks if isinstance(t, dict)] or [0]))))
    for t in tasks:
        if not isinstance(t, dict):
            continue
        o.write("  %-4s %-10s %-22s %s\n" % (
            t.get("id"), t.get("status"), str(t.get("assignee"))[:22],
            first_sentences(t.get("output"), 3)))
        if t.get("findings"):
            for f in t["findings"]:
                if isinstance(f, dict):
                    o.write("        FINDING %s/%s: %s\n" % (f.get("id"), f.get("severity"),
                                                          re.sub(r"\s+", " ", str(f.get("problem")))[:300]))
o.close()
print("ok", os.path.getsize(OUT))