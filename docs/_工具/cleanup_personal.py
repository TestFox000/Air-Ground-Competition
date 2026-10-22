
# -*- coding: utf-8 -*-
import os, csv, sys
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
files = [".zhiyuan/README.md", ".zhiyuan/工作日志.md", ".zhiyuan/纸鸢迁移档案.md",
         "memory/README.md", "memory/digest_20261018.md"]
led = os.path.join(ROOT, "docs", "_work", "cleanup_ledger.csv")
new = not os.path.exists(led)
with open(led, "a", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    if new: w.writerow(["phase","rel_path","sha256","bytes","class","kept_path","note"])
    for rel in files:
        p = os.path.join(ROOT, rel)
        if not os.path.isfile(p):
            print("missing", rel); continue
        w.writerow(["P6", rel, "", os.path.getsize(p), "E",
                    "docs/00_项目总览/06_本机值守与守护器.md + docs/20_资料索引库/历史/03_自主工作期_20261015_20261018.md",
                    "个人域：项目事实已提炼入库，个人内容不保留（用户决定）"])
        os.remove(p)
        print("removed", rel)
