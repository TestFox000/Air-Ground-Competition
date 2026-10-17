# -*- coding: utf-8 -*-
"""t3: 以 UTF-8 读回归台账关键段（PowerShell 控制台 GBK 会把中文显示成乱码）。"""
import io, re, sys
p = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\docs\回归守护_known_reds.md"
txt = io.open(p, encoding="utf-8").read()
lines = txt.splitlines()
for i, l in enumerate(lines, 1):
    if re.search(r"^\s*#|^##|^\|.*(套|濂)|洁|清|转绿|重钉|洗绿|§1b|§1c|名单外", l):
        pass
# 打印全文（分段，避免过长）
out = io.open(r"D:\User\Desktop\比赛存档\v1\_tmp_session\known_reds_utf8.txt", "w", encoding="utf-8")
out.write("\n".join("%4d| %s" % (i, l) for i, l in enumerate(lines, 1)))
out.close()
print("lines", len(lines))
print("\n".join("%4d| %s" % (i, l) for i, l in enumerate(lines[:70], 1)))
