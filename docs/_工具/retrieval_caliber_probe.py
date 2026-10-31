#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""口径对照探针（LIB-01 修正）：检验「top-1 是否是正确答案」到底该怎么测。

问题来源（2026-10-31 实测）：
  原口径 M1 = 「top-1 返回的文件本身必须是权威分册篇」。
  但实测发现：当 top-1 返回 01_症状索引_52条.md 时，它的**命中行里已经写着**
  `症-09 | 规划器一半的图无解 | tour 引擎受结构约束 | 车端/03 §2/§4 | …`
  即用户一次查询**已经拿到答案**（机制 + 位置 + 处置命令）。

  AGENTS.md 本身就规定「只知道现象 → 先查症状索引」。症状索引答得住症状查询，
  是它的设计目的，不是缺陷。因此 M1 把「正确答案」判成「未命中」，是口径错。

  本探针不替代 retrieval_eval.py，只用来量化两种口径的差距，供 Lead 裁决。

M1：top-1 文件的路径含期望片段（= retrieval_eval.py 现口径）
M2：top-1 结果的**命中行文本**含期望片段（= 「一次查询是否拿到去向」）
M3：top-1 命中行含期望片段，或 top-1 文件本身就是权威篇（M1 ∪ M2）

用 python docs/_工具/retrieval_caliber_probe.py
"""
import io
import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SYM = os.path.join("docs", "20_资料索引库", "运维与排障", "01_症状索引_52条.md")
SEARCH = os.path.join("docs", "_工具", "doc_search.py")

BOOK_PREFIXES = ("车端", "视觉与云台", "无人机", "空地链路", "赛事与治理",
                 "运维与排障", "外部资料", "历史")


def read_rows():
    text = io.open(os.path.join(ROOT, SYM), encoding="utf-8", errors="replace").read()
    rows = []
    for line in text.splitlines():
        m = re.match(r"\|\s*(症-\d+)\s*\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|", line)
        if m:
            rows.append((m.group(1), m.group(2).strip(), m.group(4).strip()))
    return rows


def frags_of(locator):
    """返回 (path_frags, text_frags) 两种形态。

    ★2026-10-31 修正：初版只造了 path 形态（`车端/03_`，带下划线，用于匹配文件名
    `20_资料索引库/车端/03_规划器与自动规划.md`），却拿它去匹配**正文文本**。
    但正文里写的是 `车端/03 §2`（无下划线）⇒ 正文匹配恒为假，M2 被严重低估（实测
    3.9%，与手工观察矛盾）。教训：同一份 ground truth 用于两种匹配时，必须显式造两种形态，
    不能复用一套字符串。
    """
    path_out, text_out = [], []
    for part in re.split(r"[；;]", locator):
        part = part.strip()
        m = re.match(r"(%s)/([0-9][0-9])" % "|".join(BOOK_PREFIXES), part)
        if m:
            path_out.append("%s/%s_" % (m.group(1), m.group(2)))   # 文件名形态
            text_out.append("%s/%s" % (m.group(1), m.group(2)))   # 正文形态
            continue
        m = re.search(r"(docs/[^\s，；]+\.md)", part)
        if m:
            p = m.group(1).replace("docs/", "", 1)
            path_out.append(p)
            text_out.append(os.path.basename(p))
            continue
        m = re.match(r"([A-Z][A-Z0-9\-]{2,})\s*卡", part)
        if m:
            path_out.append(m.group(1))
            text_out.append(m.group(1))
    return path_out, text_out


def query_for(symptom):
    q = re.sub(r"[「」“”\"`]", "", symptom)
    q = re.sub(r"\s*（.*?）", "", q).strip()
    q = re.sub(r"[、，。].*$", "", q).strip()
    return q


def run(q, top=5):
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    r = subprocess.run([sys.executable, SEARCH, q, "-k", str(top), "--json", "--live-only", "--no-timing"],
                       cwd=ROOT, capture_output=True, timeout=240, env=env)
    try:
        return json.loads(r.stdout.decode("utf-8", "replace"))
    except Exception:
        return None


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    rows = read_rows()
    m1 = m2 = m3 = 0
    scored = 0
    detail = []
    for no, sym, loc in rows:
        path_fr, text_fr = frags_of(loc)
        q = query_for(sym)
        if len(q) < 3 or not path_fr:
            continue
        d = run(q)
        if not d or not d.get("results"):
            continue
        scored += 1
        top1 = d["results"][0]
        path_hit = any(f in top1["rel"] for f in path_fr)
        row_hit = any(any(f in (h.get("text") or "") for f in text_fr)
                      for h in top1.get("hits", []))
        if path_hit:
            m1 += 1
        if row_hit:
            m2 += 1
        if path_hit or row_hit:
            m3 += 1
        detail.append({
            "no": no, "query": q, "frags": path_fr,
            "top1": top1["rel"], "path_hit": path_hit, "row_hit": row_hit,
        })
    n = scored or 1
    print("=" * 70)
    print("检索口径对照（ground truth = 症状索引 52 条的位置列）")
    print("=" * 70)
    print("  计分 %d 条" % scored)
    print("  M1 top-1 文件本身是权威篇        : %d/%d = %.1f%%" % (m1, n, 100.0 * m1 / n))
    print("  M2 top-1 命中行含去向（一次查询即可定位）: %d/%d = %.1f%%" % (m2, n, 100.0 * m2 / n))
    print("  M3 两者并集                      : %d/%d = %.1f%%" % (m3, n, 100.0 * m3 / n))
    print()
    print("-- 仅 M2 成立（文件不是权威篇但命中行给了去向）的条目 --")
    k = 0
    for x in detail:
        if x["row_hit"] and not x["path_hit"]:
            print("  %s  %-26s top1=%s" % (x["no"], x["query"][:26],
                                           x["top1"].replace("docs/20_资料索引库/", "")))
            k += 1
            if k >= 12:
                print("  … 共 %d 条" % sum(1 for y in detail if y["row_hit"] and not y["path_hit"]))
                break
    print()
    print("-- M3 仍未命中的条目（真·答不出）--")
    for x in detail:
        if not (x["row_hit"] or x["path_hit"]):
            print("  %s  %-26s frags=%s top1=%s" % (x["no"], x["query"][:26], x["frags"],
                                                    x["top1"].replace("docs/20_资料索引库/", "")))
    io.open(os.path.join(ROOT, "_tmp_session", "_caliber.json"), "w", encoding="utf-8").write(
        json.dumps({"m1": m1, "m2": m2, "m3": m3, "scored": scored, "detail": detail},
                   ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
