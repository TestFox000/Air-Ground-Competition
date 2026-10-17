# -*- coding: utf-8 -*-
"""Build a compact per-team index from team.json archives."""
import json, io, os, glob, datetime

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\.agent-teams"
SRCV1 = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src_v1\.agent-teams"
OUT = r"D:\User\Desktop\比赛存档\v1\_tmp_session\heavy-digest\_index.txt"

paths = sorted(glob.glob(os.path.join(ROOT, "archive", "*", "team.json")))
paths += sorted(glob.glob(os.path.join(ROOT, "camview-ann-team", "team.json")))
paths += sorted(glob.glob(os.path.join(SRCV1, "archive", "*", "team.json")))

def ts(ms):
    try:
        return datetime.datetime.utcfromtimestamp(ms/1000).strftime("%Y-%m-%d %H:%M")
    except Exception:
        return str(ms)

o = io.open(OUT, "w", encoding="utf-8")
for p in paths:
    d = json.load(io.open(p, encoding="utf-8"))
    o.write("\n" + "#"*110 + "\n")
    o.write("TEAM %s | id=%s | createdAt=%s | phase=%s | taskSeq=%s\n" % (
        d.get("name"), d.get("id"), ts(d.get("createdAt", 0)), d.get("phase"), d.get("taskSeq")))
    o.write("CAPTAIN session=%s\n" % d.get("captainSessionId"))
    o.write("DESCRIPTION: %s\n" % (d.get("description") or "")[:2000])
    ms = d.get("members") or []
    o.write("\nMEMBERS (%d):\n" % len(ms))
    for m in ms:
        if isinstance(m, dict):
            o.write("  %-22s role=%s | model=%s | effort=%s | status=%s\n" % (
                m.get("name"), m.get("role"), m.get("model"), m.get("reasoningEffort"), m.get("status")))
    ts_ = d.get("tasks") or []
    o.write("\nTASKS (%d)\n" % len(ts_))
    from collections import Counter
    st = Counter((t.get("status") if isinstance(t, dict) else "?") for t in ts_)
    o.write("  STATUS COUNTS: %s\n" % dict(st))
    for t in ts_:
        if not isinstance(t, dict):
            o.write("  * (non-dict) %r\n" % t); continue
        o.write("\n--- %s [%s] assignee=%s kind=%s created=%s updated=%s\n" % (
            t.get("id"), t.get("status"), t.get("assignee"), t.get("kind"),
            ts(t.get("createdAt", 0)), ts(t.get("updatedAt", 0))))
        for k in ("title", "objective", "inScope", "outOfScope", "acceptance", "verify", "dependsOn", "dependency"):
            v = t.get(k)
            if v not in (None, "", [], {}):
                s = v if isinstance(v, str) else json.dumps(v, ensure_ascii=False)
                o.write("  %s: %s\n" % (k, s[:900]))
        v = t.get("output")
        if v:
            o.write("  OUTPUT[:1400]: %s\n" % v[:1400])
        ar = t.get("acceptanceResults")
        if isinstance(ar, list) and ar:
            o.write("  ACCEPT: %s\n" % ("; ".join(
                "%s=%s" % (a.get("criterion", "")[:70], a.get("status")) for a in ar if isinstance(a, dict))))
        cr = t.get("commandsRun")
        if isinstance(cr, list) and cr:
            o.write("  CMDS(%d): %s\n" % (len(cr), " | ".join(
                "%s->%s(exit %s)" % ((c.get("command","")[:70]), c.get("status"), c.get("exitCode"))
                for c in cr if isinstance(c, dict))))
        cp = t.get("changedPaths")
        if isinstance(cp, list) and cp:
            o.write("  CHANGED(%d): %s\n" % (len(cp), ", ".join(str(x) for x in cp[:14])))
        fs = t.get("findings")
        if fs:
            o.write("  FINDINGS: %s\n" % json.dumps(fs, ensure_ascii=False)[:1200])
        if t.get("verdict"):
            o.write("  VERDICT: %s\n" % t.get("verdict"))
o.close()
print("written", os.path.getsize(OUT))