# -*- coding: utf-8 -*-
"""scan_gaps.py —— t5 热词缺口扫描（标题 vs 现行库）· doc-indexer 写域

输入：docs/history_cache/99_未消化清单.md。
    · 「待提炼 242」那批的标题列 -> 归一化 -> 聚簇 -> 反查现行库（这就是 captain
      指定的缺口清单输入）。
    · 全部 489 篇的标题 -> 归一化 -> 词频（用来给同缺口数的词做热词次序，
      否则 173 个关键词里 89 个缺口数都是 1，排序等于字母序，没有信息量）。
做法：用 docs/_工具/doc_search.py 的同一 tokenize/search 口径，在
      现行库 docs/20_资料索引库/** + docs/00_项目总览/编年/** 里反查归一化标题。
      搜到 >=1 篇 = 已覆盖；0 篇 = 缺口。
输出：artifacts/wordcloud/热词_2026W43.csv（词 / 频次 / 出现在哪几篇 / 库内是否已有条目）
      + stdout 打印 top-30 缺口（供写 热词缺口清单_20261024.md）。

排序（两键 + 两 tie-break）：缺口数量降序 -> 489 篇历史词频降序 -> 关键词长度降序 -> 字母序。
为什么用 doc_search 的口径而不是另造匹配：
    「已覆盖」的定义必须和库内检索器一致，否则本篇的「搜不到」和人用
    doc_search 搜不到不是同一件事，清单就没有可复跑的意义。
零第三方依赖；只读，不写库。
"""
import csv
import importlib.util
import os
import re
from collections import Counter, OrderedDict

HERE = os.path.dirname(os.path.abspath(__file__))          # artifacts/wordcloud
WS = os.path.dirname(os.path.dirname(HERE))                # 工作区根
DOCS = os.path.join(WS, "docs")
MANIFEST = os.path.join(DOCS, "history_cache", "99_未消化清单.md")
OUT_DIR = os.path.join(WS, "artifacts", "wordcloud")
CSV_OUT = os.path.join(OUT_DIR, "热词_2026W43.csv")

_spec = importlib.util.spec_from_file_location("doc_search", os.path.join(DOCS, "_工具", "doc_search.py"))
ds = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ds)


def norm_title(s):
    """标题归一化：去扩展名 / README_ 前缀 / 开头编号 / 日期尾巴 / 分隔符，转小写。"""
    s = os.path.basename((s or "").strip())
    s = re.sub(r"\.md$", "", s, flags=re.I)
    s = re.sub(r"^readme[_-]?", "", s, flags=re.I)
    s = re.sub(r"^0*\d{1,2}[_-]", "", s)                              # 01_ / 11_ / 03_
    s = re.sub(r"[-_ ]*\d{4}[-_]?\d{2}[-_]?\d{2}[-_ ]*", " ", s)       # 20261011 / 2026-10-11
    s = re.sub(r"[-_ ]+", "", s)
    s = s.lower()
    return s.strip()


def load_manifest():
    """返回全部 489 行的 [(原路径, 标题, 分类)]。"""
    rows = []
    with open(MANIFEST, "r", encoding="utf-8") as f:
        for line in f:
            if not line.lstrip().startswith("|") or "|" not in line:
                continue
            parts = [p.strip() for p in line.strip().strip("|").split("|")]
            if len(parts) < 9:
                continue
            rows.append((parts[0], parts[6], parts[7]))
    return rows


def main():
    allrows = load_manifest()
    print("清单行数：%d" % len(allrows))

    freq_all = Counter()
    for _p, t, _c in allrows:
        k = norm_title(t)
        if k:
            freq_all[k] += 1
    print("489 篇归一化标题去重后：%d 个关键词" % len(freq_all))

    pending = [(p, t) for p, t, c in allrows if c == "待提炼"]
    print("待提炼标题行数：%d" % len(pending))

    clusters = OrderedDict()
    for path, title in pending:
        kw = norm_title(title)
        if kw:
            clusters.setdefault(kw, []).append((path, title))

    files = ds.collect_files(["20_资料索引库", "00_项目总览/编年"])
    corpus = []
    for full, rel in files:
        lines = ds.read_lines(full)
        if lines:
            corpus.append((rel, lines))
    corpus_low = "\n".join("\n".join(ls) for _, ls in corpus).lower()
    print("现行库语料：%d 篇 / %d 行" % (len(corpus), corpus_low.count("\n") + 1))

    covered, gaps = [], []
    for kw in clusters:
        terms = ds.tokenize(kw)
        if not terms:
            continue
        bre_map, sub_mode = {}, {}
        for t, kind in terms:
            if kind != "w":
                continue
            rx = ds.boundary_re(t)
            bre_map[t] = rx
            if not rx.search(corpus_low):
                sub_mode[t] = True
        res = ds.search(terms, corpus, sub_mode, bre_map, 1, True)
        item = {"词": kw, "频次": len(clusters[kw]), "热词频次": freq_all.get(kw, 0),
                "原题": clusters[kw], "已覆盖": len(res) > 0}
        (covered if item["已覆盖"] else gaps).append(item)

    key = lambda x: (-x["频次"], -x["热词频次"], -len(x["词"]), x["词"])
    all_items = sorted(covered + gaps, key=key)
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(CSV_OUT, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["词", "频次", "出现在哪几篇", "库内是否已有条目"])
        for it in all_items:
            sample = " / ".join(t for _, t in it["原题"][:3])
            w.writerow([it["词"], it["频次"],
                        "%d 篇；例：%s" % (it["频次"], sample),
                        "是" if it["已覆盖"] else "否"])
    print("CSV 写入 %s（%d 行）" % (os.path.relpath(CSV_OUT, WS), len(all_items)))
    print("已覆盖 %d / 缺口 %d（缺口行 %d）" % (
        len(covered), len(gaps), sum(1 for i in all_items if not i["已覆盖"])))

    # 机器可读 top-30（供写 热词缺口清单_20261024.md）
    import json as _json
    top = sorted(gaps, key=key)[:30]
    with open(os.path.join(OUT_DIR, "top30_gaps.json"), "w", encoding="utf-8") as f:
        _json.dump([{"词": it["词"], "缺口数": it["频次"], "489词频": it["热词频次"],
                     "已覆盖": False, "原题": [t for _p, t in it["原题"]]} for it in top],
                   f, ensure_ascii=False, indent=1)
    # 分类口径自检
    cat = Counter(c for _p, _t, c in allrows)
    print("清单行分类自检：%s（合计 %d）" % (dict(cat), sum(cat.values())))
    head = [r for r in allrows if r[2] in ("分类", "标题", "")][:3]
    print("疑似非数据行示例：%s" % head)

    print("\n=== TOP-30 缺口（缺口数量 -> 489篇词频 -> 长度 -> 字母）===")
    for i, it in enumerate(top, 1):
        print("%2d. %-38s 缺口数=%-2d 489词频=%-3d 例：%s" % (
            i, it["词"], it["频次"], it["热词频次"],
            " / ".join(t for _, t in it["原题"][:3])))


if __name__ == "__main__":
    main()
