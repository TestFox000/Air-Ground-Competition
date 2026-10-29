#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""复用门禁：新造的轮子必须登记

规则（2026-10-18 起）：
  1. **新增代码文件**（git status 里 ?? 或 A 的 .py，位于 tools/ 或 src/dzbrain/dzbrain/）
     必须在 docs/复用清单.md 或 docs/能力索引.md 里出现文件名 → 否则 **红**；
  2. 既有 tools/*.py 未登记 → **只警告**（历史存量，不阻断；谁清理谁收口）。
  3. 排除 __pycache__、tests/、_tmp_session/、backup/。

为什么：项目反复"造轮子"的物理入口就是"新建一个脚本没人拦"。这条把拦截点放在新文件出现的那一刻。

用法：cd dzrobot_v2 && python tools/wf/check_reuse.py
退出码：0 通过；1 有未登记的新文件。
"""
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
DOCS = [REPO / "docs" / "复用清单.md", REPO / "docs" / "能力索引.md"]
SCOPES = ("tools/", "src/dzbrain/dzbrain/")
EXCLUDE = ("__pycache__", "/tests/", "_tmp_session/", "backup/", "/legacy/")


def registered_text() -> str:
    parts = []
    for d in DOCS:
        if d.exists():
            parts.append(d.read_text(encoding="utf-8", errors="replace"))
    return "\n".join(parts)


def new_code_files() -> list[str]:
    # ★2026-10-29（t4 发现）：默认 -u 会把全新未跟踪目录折叠成 "?? dir/"，
    #   整目录新文件绕过复用门禁 ⇒ 必须 -uall 展开到文件粒度。
    r = subprocess.run(
        ["git", "-C", str(REPO), "-c", "core.quotepath=false", "status",
         "--porcelain", "--untracked-files=all"],
        capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=60,
    )
    out = []
    for line in (r.stdout or "").splitlines():
        if len(line) < 4:
            continue
        status, path = line[:2].strip(), line[3:].strip().strip('"')
        if " -> " in path:
            path = path.split(" -> ")[-1]
        posix = path.replace("\\", "/")
        if not posix.endswith(".py") or status not in ("??", "A", "AM"):
            continue
        if any(x in posix for x in EXCLUDE):
            continue
        if not any(posix.startswith(s) for s in SCOPES):
            continue
        out.append(posix)
    return out


def existing_tools(text: str) -> list[str]:
    missing = []
    for f in sorted((REPO / "tools").rglob("*.py")):
        rel = f.relative_to(REPO).as_posix()
        if any(x in rel for x in EXCLUDE):
            continue
        if f.name not in text and rel not in text:
            missing.append(rel)
    return missing


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass
    text = registered_text()
    news = new_code_files()
    bad = [p for p in news if Path(p).name not in text and p not in text]
    legacy = existing_tools(text)
    print("新文件: %d  未登记: %d  存量 tools 未登记(warn): %d"
          % (len(news), len(bad), len(legacy)))
    for p in bad:
        print("  [RED ] 新增未登记: %s" % p)
    for p in legacy[:10]:
        print("  [WARN] 存量未登记: %s" % p)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
