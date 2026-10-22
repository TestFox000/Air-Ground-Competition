
# -*- coding: utf-8 -*-
import os, tarfile, sys, gzip
sys.stdout.reconfigure(encoding="utf-8")
p = r"D:\User\Desktop\比赛存档\v1\docs\_archive\snapshots\process_drafts_20261022.tar.gz"
print("size", os.path.getsize(p))
# 1) gzip 流完整性
try:
    n = 0
    with gzip.open(p, "rb") as f:
        while True:
            b = f.read(1 << 20)
            if not b: break
            n += len(b)
    print("gzip decompress OK, bytes:", n)
except Exception as e:
    print("gzip ERR:", type(e).__name__, e)
    # 手工解压到出错点
    n = 0
    with open(p, "rb") as raw:
        d = gzip.GzipFile(fileobj=raw)
        while True:
            try:
                b = d.read(1 << 20)
            except Exception as e2:
                print("stopped at", n, "bytes:", type(e2).__name__, e2)
                break
            if not b: break
            n += len(b)
# 2) 可读成员数
try:
    cnt = 0; last = None
    with tarfile.open(p, "r|gz") as tf:
        for m in tf:
            cnt += 1; last = m.name
    print("members", cnt, "last", last)
except Exception as e:
    print("tar read stopped:", type(e).__name__, e, "| members so far:", cnt if 'cnt' in dir() else '?', "| last:", last)
