# -*- coding: utf-8 -*-
"""docs 目录索引生成器 —— 从各篇 front matter 与 TL;DR 自动生成索引，避免索引腐烂。

用法（在工作区根执行）：
    python docs/_工具/gen_index.py

产出：
    docs/目录索引.md     人看的
    docs/目录索引.json   机器读的
不纳入：_archive（历史原文）、_工具、_work（临时）
"""
import json
import os
import re
from datetime import datetime

DOCS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKIP_DIRS = {"_archive", "_工具", "_work"}
FM_KEYS = ("来源", "取证", "更新", "取代", "status", "摘要")


def read_head(path, limit=4000):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read(limit)
    except OSError:
        return ""


def parse(path):
    text = read_head(path)
    fm = {}
    body = text
    if text.startswith("---"):
        end = text.find("\n---", 3)
        if end > 0:
            for line in text[3:end].splitlines():
                if ":" in line:
                    k, v = line.split(":", 1)
                    k = k.strip()
                    if k in FM_KEYS:
                        fm[k] = v.strip()
            body = text[end + 4:]

    title = ""
    for line in body.splitlines():
        s = line.strip()
        if s.startswith("# "):
            title = s[2:].strip()
            break

    summary = fm.get("摘要", "")
    if not summary:
        in_tldr = False
        for line in body.splitlines():
            s = line.strip()
            if s.startswith("## ") and "TL;DR" in s:
                in_tldr = True
                continue
            if in_tldr:
                if s.startswith("## "):
                    break
                if s.startswith(("- ", "* ")):
                    summary = s[2:].strip()
                    break
    if not summary:
        for line in body.splitlines():
            s = line.strip()
            if s and not s.startswith(("#", ">", "|", "---", "```")):
                summary = s
                break
    summary = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", summary)  # strip_md_links: 索引里只留文字，避免相对链接失效
    summary = re.sub(r"\s+", " ", summary).strip()
    if len(summary) > 90:
        summary = summary[:88] + "…"
    return title, summary, fm


def main():
    rows = []
    for dirpath, dirnames, filenames in os.walk(DOCS):
        dirnames[:] = sorted(d for d in dirnames if d not in SKIP_DIRS and not d.startswith("."))
        for name in sorted(filenames):
            if not name.endswith(".md"):
                continue
            full = os.path.join(dirpath, name)
            rel = os.path.relpath(full, DOCS).replace("\\", "/")
            if rel in ("目录索引.md", "README.md"):
                continue
            title, summary, fm = parse(full)
            rows.append({
                "path": rel,
                "dir": os.path.dirname(rel) or ".",
                "title": title or name[:-3],
                "summary": summary,
                "from": fm.get("来源", ""),
                "level": fm.get("取证", ""),
                "updated": fm.get("更新", ""),
                "status": fm.get("status", ""),
                "lines": sum(1 for _ in open(full, encoding="utf-8", errors="replace")),
            })

    rows.sort(key=lambda r: (r["dir"] != ".", r["dir"], r["path"]))
    now = datetime.now().strftime("%Y-%m-%d %H:%M")

    out = []
    out.append("---")
    out.append("来源: 由 docs/_工具/gen_index.py 自动生成，勿手改")
    out.append("取证: 实")
    out.append("更新: " + now[:10])
    out.append("取代: 无")
    out.append("---")
    out.append("")
    out.append("# 目录索引（全库文件一览）")
    out.append("")
    out.append("> 自动生成，勿手改。重新生成：python docs/_工具/gen_index.py")
    out.append("> 生成时间 " + now + " ｜ 收录 " + str(len(rows)) + " 篇（不含 _archive 历史原文）")
    out.append("> 每篇的一句话取自其 TL;DR 首条或 front matter 的 摘要 字段。")
    out.append("")
    out.append("## 按目录")
    cur = None
    for r in rows:
        if r["dir"] != cur:
            cur = r["dir"]
            out.append("")
            out.append("### " + (cur if cur != "." else ".") + "/")
            out.append("")
            out.append("| 文件 | 一句话 | 取证 | 更新 | 行数 |")
            out.append("|---|---|---|---|---|")
        out.append("| [" + r["title"].replace("|", "/") + "](" + r["path"].replace(" ", "%20") + ") | " +
                   (r["summary"] or "—").replace("|", "/") + " | " + (r["level"] or "—") + " | " +
                   (r["updated"] or "—") + " | " + str(r["lines"]) + " |")
    out.append("")
    out.append("## 最近更新 20 篇")
    out.append("")
    out.append("| 更新 | 文件 | 一句话 |")
    out.append("|---|---|---|")
    recent = sorted([x for x in rows if x["updated"]], key=lambda x: x["updated"], reverse=True)[:20]
    for r in recent:
        out.append("| " + r["updated"] + " | [" + r["title"].replace("|", "/") + "](" +
                   r["path"].replace(" ", "%20") + ") | " + (r["summary"] or "—").replace("|", "/") + " |")
    out.append("")

    with open(os.path.join(DOCS, "目录索引.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out))
    with open(os.path.join(DOCS, "目录索引.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump({"generated": now, "count": len(rows), "files": rows}, f, ensure_ascii=False, indent=1)

    by_dir = {}
    for r in rows:
        by_dir[r["dir"]] = by_dir.get(r["dir"], 0) + 1
    print("OK  " + str(len(rows)) + " files")
    for d in sorted(by_dir):
        print("    " + d.ljust(28) + " " + str(by_dir[d]))


if __name__ == "__main__":
    main()
