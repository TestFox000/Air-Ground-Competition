
# -*- coding: utf-8 -*-
import re, os, sys
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
files = ["dzrobot_v2/docs/TEAM_STATUS.md", "dzrobot_v2/docs/决策台账.md"]
DRAFT = "../../docs/_archive/snapshots/process_drafts_20261022.tar.gz"
R3 = "../../docs/_archive/snapshots/process_TRACK-FIELD-R3_20261022.tar.gz"
pat = re.compile(r"\]\((\.\./\.\./_tmp_session/[^)]+)\)")
for rel in files:
    p = os.path.join(ROOT, rel)
    s = open(p, encoding="utf-8").read()
    def rep(m):
        target = m.group(1)
        return "](" + (R3 if "TRACK-FIELD-R3" in target else DRAFT) + ")"
    s2, n = pat.subn(rep, s)
    if s2 != s:
        open(p, "w", encoding="utf-8", newline="\n").write(s2)
    print(rel, "replaced", n)
