# -*- coding: utf-8 -*-
"""Quantify churn: retries, assignee switches, task-card path errors."""
import json, io, os, glob, re
from collections import Counter, defaultdict

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\.agent-teams"
SRCV1 = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src_v1\.agent-teams"
OUT = r"D:\User\Desktop\比赛存档\v1\_tmp_session\heavy-digest\_churn.txt"

paths = sorted(glob.glob(os.path.join(ROOT, "archive", "*", "team.json")))
paths += sorted(glob.glob(os.path.join(ROOT, "camview-ann-team", "team.json")))
paths += sorted(glob.glob(os.path.join(SRCV1, "archive", "*", "team.json")))

o = io.open(OUT, "w", encoding="utf-8")
grand_mem = 0
grand_task = 0
for p in paths:
    d = json.load(io.open(p, encoding="utf-8"))
    name = d.get("name")
    tasks = [t for t in (d.get("tasks") or []) if isinstance(t, dict)]
    ms = d.get("members") or []
    grand_mem += len(ms); grand_task += len(tasks)
    o.write("\n" + "="*100 + "\nTEAM %s | members=%d tasks=%d\n" % (name, len(ms), len(tasks)))
    # distinct assignees actually used
    used = Counter(str(t.get("assignee")) for t in tasks if t.get("assignee"))
    o.write("  assignees used: %d  %s\n" % (len(used), dict(used)))
    o.write("  members never assigned: %d  %s\n" % (
        len(ms) - len([m for m in ms if m.get("name") in used]),
        [m.get("name") for m in ms if m.get("name") not in used]))
    # role suffixes (versioned names indicate re-created roles)
    o.write("  model routes: %s\n" % dict(Counter(str(m.get("model")) for m in ms if isinstance(m, dict))))
    o.write("  providers: %s\n" % dict(Counter(str(m.get("provider")) for m in ms if isinstance(m, dict))))
    # findings summary
    sev = Counter()
    ftexts = []
    for t in tasks:
        for f in (t.get("findings") or []):
            if isinstance(f, dict):
                sev[f.get("severity")] += 1
                ftexts.append((t.get("id"), f.get("id"), f.get("severity"), re.sub(r"\s+", " ", str(f.get("problem")))[:400]))
    if sev:
        o.write("  FINDINGS severity: %s\n" % dict(sev))
        for tid, fid, s, pr in ftexts:
            o.write("    %s %s [%s] %s\n" % (tid, fid, s, pr))
    # path-error mentions in outputs
    for t in tasks:
        out = t.get("output") or ""
        for m in re.finditer(r"[^\s，。；]{0,40}(路径更正|任务卡写错|真文件在|不存在)[^\s，。；]{0,60}", out):
            o.write("    PATHERR %s: %s\n" % (t.get("id"), m.group(0)[:160]))
    # acceptance failures on completed tasks (partial credit / 恒假门禁)
    for t in tasks:
        ar = t.get("acceptanceResults") or []
        if t.get("status") == "completed" and isinstance(ar, list):
            bad = [a for a in ar if isinstance(a, dict) and a.get("status") != "passed"]
            if bad:
                o.write("    PARTIAL-CREDIT %s [%s] failed-accept: %s\n" % (
                    t.get("id"), t.get("assignee"), " | ".join(str(a.get("criterion"))[:90] for a in bad)))
o.write("\n\nGRAND TOTAL: teams=%d members(sum,含改名重复)=%d tasks=%d\n" % (len(paths), grand_mem, grand_task))
o.close()
print("ok", os.path.getsize(OUT))