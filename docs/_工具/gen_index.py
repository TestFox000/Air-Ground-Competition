# -*- coding: utf-8 -*-
"""docs 目录索引生成器 —— 从各篇 front matter 与 TL;DR 自动生成索引，避免索引腐烂。

用法（在工作区根执行）：
    python docs/_工具/gen_index.py                      # 全量重生成（旧行为）
    python docs/_工具/gen_index.py --ns lead-1023       # 只重生成该 ns 的卡（其余行原样保留）
    python docs/_工具/gen_index.py --only 30_任务卡库/在办/lead-1023/,docs/_work/   # 显式前缀

为什么有增量模式（2026-10-23）：
    全量重生成会把「别人还没入库的新文件」一起写进索引，导致一个会话无法提交索引
    （索引指向了未提交的文件 = 幽灵条目）。增量模式只重扫自己写域，别人的行原样保留。

产出：
    docs/目录索引.md     人看的
    docs/目录索引.json   机器读的
不纳入：history_cache（历史留存）、_工具、_work（临时）
"""
import json
import os
import re
import sys
from datetime import datetime

DOCS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKIP_DIRS = {"history_cache", "_工具", "_work"}
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


def scan_rows(prefixes=None):
    """prefixes=None 表示全量；否则只收这些前缀（相对 docs/）下的文件。"""
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
            if prefixes is not None and not any(rel.startswith(p) for p in prefixes):
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
    return rows


def parse_scope():
    """-> prefixes(list) 或 None（全量）"""
    args = sys.argv[1:]
    only, ns = [], ""
    for i, a in enumerate(args):
        if a == "--only" and i + 1 < len(args):
            only += [x.strip() for x in args[i + 1].split(",") if x.strip()]
        elif a == "--ns" and i + 1 < len(args):
            ns = args[i + 1].strip()
    if only:
        return only
    if ns:
        return ["30_任务卡库/在办/%s/" % ns, "30_任务卡库/归档/"]
    return None


def main():
    prefixes = parse_scope()
    fresh = scan_rows(prefixes)
    if prefixes is None:
        rows = fresh
    else:
        idx = os.path.join(DOCS, "目录索引.json")
        kept = []
        if os.path.exists(idx):
            with open(idx, "r", encoding="utf-8") as f:
                old = json.load(f).get("files", [])
            kept = [r for r in old if not any(r.get("path", "").startswith(p) for p in prefixes)]
        merge = dict((r["path"], r) for r in kept)
        for r in fresh:
            merge[r["path"]] = r
        rows = list(merge.values())
        print("[增量] 重扫 %d 条（%s），保留外部 %d 条" % (len(fresh), ", ".join(prefixes), len(kept)))
    rows.sort(key=lambda r: (r["dir"] != ".", r["dir"], r["path"]))
    now = datetime.now().strftime("%Y-%m-%d %H:%M")

    out = []
    out.append("---")
    out.append("来源: 由 docs/_工具/gen_index.py 自动生成，勿手改")
    out.append("取证: 实")
    out.append("status: 现行")   # 2026-10-26 补：生成物也是现行索引，缺 status 会被读者当沿革
    out.append("更新: " + now[:10])
    out.append("取代: 无")
    out.append("---")
    out.append("")
    out.append("# 目录索引（全库文件一览）")
    out.append("")
    out.append("> 自动生成，勿手改。重新生成：python docs/_工具/gen_index.py")
    out.append("> 生成时间 " + now + " ｜ 收录 " + str(len(rows)) + " 篇（不含 history_cache 历史留存）")
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
            # 2026-10-26 增「状态」列：status 早就从 front matter 读进 rows 了，
            # 但生成表格时被丢掉 —— 读者打开索引看不出哪篇是现行真源、哪篇已降级成沿革，
            # 只能一篇篇点进去看。这一列是索引可用性的关键，缺它索引就只是文件名清单。
            out.append("| 文件 | 一句话 | 状态 | 取证 | 更新 | 行数 |")
            out.append("|---|---|---|---|---|---|")
        # 状态列按库别渲染：知识库（00/10/20）= 现行|计划|历史|过程；
        # 任务卡库（30）= pending|in_progress|已归档（细则 11.7 本就分两套词表）。
        # 不区分库别混列，读者会以为"8 种状态乱用"，实为两套合法词表并存。
        _raw_st = (r["status"] or "").strip()
        if not _raw_st:
            _st = "—"
        elif r["dir"].startswith("30_任务卡库"):
            _st = {"pending": "待领", "in_progress": "在办", "已归档": "已归档"}.get(_raw_st, _raw_st)
        else:
            _st = {"现行": "**现行**"}.get(_raw_st, _raw_st)
        _st = _st.replace("|", "/")
        out.append("| [" + r["title"].replace("|", "/") + "](" + r["path"].replace(" ", "%20") + ") | " +
                   (r["summary"] or "—").replace("|", "/") + " | " + _st + " | " + (r["level"] or "—") + " | " +
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
