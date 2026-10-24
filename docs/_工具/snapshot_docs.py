# -*- coding: utf-8 -*-
"""文档快照：把 docs/（不含 history_cache）打成带日期的 zip，供误删/被覆盖时回滚。

用法：
    python docs/_工具/snapshot_docs.py            # 打一个快照
    python docs/_工具/snapshot_docs.py --list     # 看已有快照
落点：<v1>/_archive/docs_snapshots/docs-YYYYMMDD-HHMM.zip（保留最近 12 个）
"""
import os, sys, zipfile
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.dirname(HERE)
WS = os.path.dirname(DOCS)
OUT = os.path.join(WS, "_archive", "docs_snapshots")
KEEP = 12

def main():
    os.makedirs(OUT, exist_ok=True)
    if "--list" in sys.argv:
        for n in sorted(os.listdir(OUT)):
            print(n, os.path.getsize(os.path.join(OUT, n)), "bytes")
        return 0
    stamp = datetime.now().strftime("%Y%m%d-%H%M")
    dst = os.path.join(OUT, "docs-" + stamp + ".zip")
    n = 0
    with zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as z:
        for dp, dns, fns in os.walk(DOCS):
            dns[:] = [d for d in dns if d != "history_cache"]
            for f in fns:
                full = os.path.join(dp, f)
                z.write(full, os.path.relpath(full, WS).replace("\\", "/"))
                n += 1
    snaps = sorted(x for x in os.listdir(OUT) if x.startswith("docs-") and x.endswith(".zip"))
    for old in snaps[:-KEEP]:
        os.remove(os.path.join(OUT, old))
    print("[快照] %s  文件 %d  大小 %.1f KB  保留 %d 个" % (dst, n, os.path.getsize(dst) / 1024.0, min(len(snaps), KEEP)))
    return 0

if __name__ == "__main__":
    sys.stdout.reconfigure(errors="replace")
    sys.exit(main())
