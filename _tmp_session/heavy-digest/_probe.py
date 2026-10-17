# -*- coding: utf-8 -*-
import json, io, os, sys, glob

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\.agent-teams"
teams = sorted(glob.glob(os.path.join(ROOT, "archive", "*", "team.json"))) + \
        sorted(glob.glob(os.path.join(ROOT, "camview-ann-team", "team.json"))) + \
        sorted(glob.glob(os.path.join(ROOT, "..", "src_v1", ".agent-teams", "archive", "*", "team.json")))

out = io.open(r"D:\User\Desktop\比赛存档\v1\_tmp_session\heavy-digest\_probe.txt", "w", encoding="utf-8")
for p in teams:
    d = json.load(io.open(p, encoding="utf-8"))
    out.write("=" * 100 + "\n")
    out.write("FILE: %s\n" % p)
    out.write("name=%s id=%s createdAt=%s phase=%s planReviewState=%s taskSeq=%s\n" % (
        d.get("name"), d.get("id"), d.get("createdAt"), d.get("phase"), d.get("planReviewState"), d.get("taskSeq")))
    out.write("description=%r\n" % (d.get("description"),))
    ms = d.get("members") or []
    out.write("MEMBERS (%d):\n" % len(ms))
    for m in ms:
        if isinstance(m, dict):
            out.write("  - %s\n" % json.dumps(m, ensure_ascii=False)[:900])
        else:
            out.write("  - %r\n" % m)
    ts = d.get("tasks") or []
    out.write("TASKS (%d):\n" % len(ts))
    for t in ts:
        if isinstance(t, dict):
            out.write("  * id=%s status=%s assignee=%s kind=%s\n" % (
                t.get("id"), t.get("status"), t.get("assignee"), t.get("kind")))
            for k in ("title", "objective", "output", "verdict", "findings", "dependencyResults",
                      "acceptanceResults", "commandsRun", "evidence_note", "inScope", "outOfScope",
                      "acceptance", "verify", "dependsOn", "changedPaths", "attempt_id", "createdAt", "updatedAt"):
                v = t.get(k)
                if v not in (None, "", [], {}):
                    s = json.dumps(v, ensure_ascii=False) if not isinstance(v, str) else v
                    out.write("      %s: %s\n" % (k, s[:2500]))
        else:
            out.write("  * %r\n" % t)
    out.write("\n")

# key structures of one member / task for schema reference
d = json.load(io.open(teams[0], encoding="utf-8"))
out.write("SAMPLE member keys: %s\n" % list((d.get('members') or [{}])[0].keys()))
out.write("SAMPLE task keys: %s\n" % list((d.get('tasks') or [{}])[0].keys()))
out.close()
print("done", len(teams))