# -*- coding: utf-8 -*-
"""给知识库(00/10/20)补 front matter status；--apply 才写盘。"""
import os, sys
WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCOPE = ["docs/00_项目总览", "docs/10_agent提示词", "docs/20_资料索引库"]
changed, skipped = [], []
for root in SCOPE:
    base = os.path.join(WS, root)
    for dp, dns, fns in os.walk(base):
        for n in fns:
            if not n.endswith(".md"):
                continue
            full = os.path.join(dp, n)
            rel = os.path.relpath(full, WS).replace("\\", "/")
            text = open(full, encoding="utf-8", errors="replace").read()
            if not text.startswith("---"):
                skipped.append((rel, "无 front matter")); continue
            end = text.find("\n---", 3)
            fm = text[3:end]
            if "\nstatus:" in "\n" + fm or fm.startswith("status:"):
                continue
            val = "历史" if "/历史/" in rel else "现行"
            new = text[:end] + "\nstatus: " + val + text[end:]
            if "--apply" in sys.argv:
                open(full, "w", encoding="utf-8", newline="").write(new)
            changed.append((rel, val))
print("待补 status：%d 篇（其中无 front matter：%d）" % (len(changed), len(skipped)))
for rel, v in changed[:80]:
    print("  %s  -> %s" % (rel, v))
for rel, why in skipped:
    print("  [跳过] %s  %s" % (rel, why))
