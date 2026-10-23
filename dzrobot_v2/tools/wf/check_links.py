#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""文档链接健康检查（收口门禁用）

扫描 docs/**/*.md 与 src/history/INDEX.md（以及仓库内/工作区根的 AGENTS.md），
检查 markdown 相对链接指向的文件是否存在。断链 = 文档与真源漂移，直接判红。

用法：
    cd dzrobot_v2 && python tools/wf/check_links.py
退出码：0 无断链；1 有断链。
"""
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
LINK = re.compile(r"\]\(([^)\n]+)\)")

SKIP_PREFIXES = ("#", "/", "mailto:")
SKIP_SUBSTR = ("://",)


def check() -> tuple[int, list[tuple[str, str]]]:
    files = list((REPO / "docs").rglob("*.md"))
    files += [REPO / "src" / "history" / "INDEX.md", REPO / "AGENTS.md", REPO.parent / "AGENTS.md"]
    bad: list[tuple[str, str]] = []
    seen = set()
    for md in files:
        if not md.exists() or md in seen:
            continue
        seen.add(md)
        text = md.read_text(encoding="utf-8", errors="replace")
        for m in LINK.finditer(text):
            target = m.group(1).strip().strip("<>")
            if not target or target.startswith(SKIP_PREFIXES) or any(s in target for s in SKIP_SUBSTR):
                continue
            t2 = target.split("#", 1)[0].split("?", 1)[0]
            if not t2:
                continue
            if not (md.parent / t2).exists():
                try:
                    rel = str(md.relative_to(REPO))
                except ValueError:
                    rel = str(md)
                bad.append((rel, target))
    return len(seen), bad


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass
    n, bad = check()
    print("checked files: %d broken: %d" % (n, len(bad)))
    for src, target in bad[:40]:
        print("  ", src, "->", target)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
