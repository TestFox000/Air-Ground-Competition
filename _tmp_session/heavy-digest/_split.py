# -*- coding: utf-8 -*-
"""Split team archives into per-team readable text files (full task text)."""
import json, io, os, glob, datetime

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\.agent-teams"
SRCV1 = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src_v1\.agent-teams"
OUTDIR = r"D:\User\Desktop\比赛存档\v1\_tmp_session\heavy-digest\teams"
os.makedirs(OUTDIR, exist_ok=True)

paths = sorted(glob.glob(os.path.join(ROOT, "archive", "*", "team.json")))
paths += sorted(glob.glob(os.path.join(ROOT, "camview-ann-team", "team.json")))
paths += sorted(glob.glob(os.path.join(SRCV1, "archive", "*", "team.json")))

def ts(ms):
    try:
        return datetime.datetime.fromtimestamp(ms/1000, datetime.timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    except Exception:
        return str(ms)

def j(v, n=6000):
    s = v if isinstance(v, str) else json.dumps(v, ensure_ascii=False)
    return s[:n]

for p in paths:
    d = json.load(io.open(p, encoding="utf-8"))
    name = d.get("name") or "unnamed"
    out = io.open(os.path.join(OUTDIR, name + ".txt"), "w", encoding="utf-8")
    out.write("TEAM %s\nid=%s\ncreatedAt=%s (%s)\nphase=%s\nplanReviewState=%s\ncaptain=%s\ndescription=%s\n\n" % (
        name, d.get("id"), d.get("createdAt"), ts(d.get("createdAt", 0)), d.get("phase"),
        d.get("planReviewState"), d.get("captainSessionId"), d.get("description")))
    out.write("MEMBERS (%d)\n" % len(d.get("members") or []))
    for m in (d.get("members") or []):
        if isinstance(m, dict):
            out.write("  %-24s | role=%s | provider=%s model=%s effort=%s | status=%s | joined=%s\n" % (
                m.get("name"), m.get("role"), m.get("provider"), m.get("model"),
                m.get("reasoningEffort"), m.get("status"), ts(m.get("joinedAt", 0))))
    tasks = d.get("tasks") or []
    out.write("\n\nTASKS (%d)\n" % len(tasks))
    for t in tasks:
        if not isinstance(t, dict):
            out.write("\n=== (non-dict task) %r\n" % (t,)); continue
        out.write("\n\n" + "="*100 + "\n")
        out.write("TASK %s | status=%s | assignee=%s | kind=%s\ncreated=%s updated=%s\n" % (
            t.get("id"), t.get("status"), t.get("assignee"), t.get("kind"),
            ts(t.get("createdAt", 0)), ts(t.get("updatedAt", 0))))
        for k in ("title", "objective", "inScope", "outOfScope", "acceptance", "verify",
                  "dependsOn", "dependencyResults", "kind", "verdict", "evidence_note"):
            if t.get(k) not in (None, "", [], {}):
                out.write("%s: %s\n" % (k, j(t.get(k), 4000)))
        if t.get("output"):
            out.write("\n--OUTPUT--\n%s\n" % t["output"])
        ar = t.get("acceptanceResults")
        if isinstance(ar, list) and ar:
            out.write("\n--ACCEPTANCE--\n")
            for a in ar:
                if isinstance(a, dict):
                    out.write("  [%s] %s\n      ev: %s\n" % (
                        a.get("status"), a.get("criterion"), j(a.get("evidence"), 1800)))
        cr = t.get("commandsRun")
        if isinstance(cr, list) and cr:
            out.write("\n--COMMANDS--\n")
            for c in cr:
                if isinstance(c, dict):
                    out.write("  [%s exit=%s] %s\n      ev: %s\n" % (
                        c.get("status"), c.get("exitCode"), c.get("command"), j(c.get("evidence"), 1200)))
        fs = t.get("findings")
        if fs:
            out.write("\n--FINDINGS--\n%s\n" % json.dumps(fs, ensure_ascii=False, indent=1)[:6000])
        cp = t.get("changedPaths")
        if cp:
            out.write("\n--CHANGED--\n%s\n" % "\n".join(str(x) for x in cp))
    out.close()
    print(name, os.path.getsize(os.path.join(OUTDIR, name + ".txt")))