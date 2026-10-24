
# -*- coding: utf-8 -*-
import csv, os, sys
sys.stdout.reconfigure(encoding="utf-8")
WD = r"D:\User\Desktop\比赛存档\v1\docs\_work"
led = list(csv.DictReader(open(os.path.join(WD,"cleanup_ledger.csv"), encoding="utf-8-sig")))
keys = ["agent_overview","aim_audit","audit_20261018","cleanup_20261018","d13_verify"]
for r in led:
    if any(k in r["rel_path"] for k in keys):
        print("LEDGER", r["phase"], r["rel_path"], "->", r["kept_path"])
print("ledger total:", len(led))
