#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多口径对照探针（LIB-09）：一次查询到底有没有拿到「去向」？

为什么有这个工具（2026-10-31，LIB-03 被判 failed 之后）：
  原来只有一个量尺（retrieval_eval.py 的 top-1 文件是否是权威篇）。它把
  「top-1 返回 01_症状索引_52条.md」判成未命中 —— 但症状索引的命中行里
  明明写着机制 + 位置 + 处置命令，用户一次查询**已经拿到了答案**。
  于是「改好了」和「改坏了」两个结论都能自圆其说，因为量尺本身就错。

本工具把三个口径摆在同一份真实语料上同时算，让分歧无处躲藏：

  M1  top-1 的**文件路径**含期望片段（文件形态 `车端/05_`）
      = 「检索器把权威篇排第一了吗」
  M2  top-1 的**命中行文本**含去向（正文形态 `车端/05`，**无下划线**）
      = 「一次查询的直接输出里有没有'该去哪篇'」
  M4  M1 或 M2 或「期望篇出现在 top-5 里」
      = 「一次查询能不能到达权威结论」，含两条合法路径（直接命中 / 经路由命中）

  三者不是精度关系，是**问题定义**关系。M4 才是「可用性」，M1/M2 各测一段。

另外输出两张表，用来区分「召回失败」和「排序失败」：
  ① top-1 分类表：top-1 是症状索引 vs 其它文档，各自 M2 命中率。
     实测会发现 M2 的分数几乎全由「症状索引坐 top-1」贡献 —— 这直接说明
     「抬 M1 会把 M2 压下去」，两条口径在排序层互相拆台。
  ② 期望篇是否进 top-5：没进 top-5 的那批是**召回**问题，排序层治不了。

★踩过的坑（写进来免得下次重踩）：文件形态片段带下划线（`车端/05_`），
  正文形态不带（`车端/05`）。初版拿文件形态去匹配正文文本，M2 假到 3.9%，
  与手工抽样完全矛盾。**凡量尺结果与手工观察矛盾，先怀疑量尺。**

只读工具：不改任何真实文件，不落缓存，不写 JSON 快照。
用法：
    python docs/_工具/retrieval_multi_caliber_probe.py
    python docs/_工具/retrieval_multi_caliber_probe.py --json   # 机器读
退出码：0 正常产出；1 症状索引解析为空 / 检索器缺失；2 用法错误。
"""
import json
import os
import re
import subprocess
import sys

# 与 retrieval_eval.py 同口径取数：症状索引 52 条的位置列就是 ground truth。
# 直接 import 而不是抄一份，是为了保证行集与分母（51）与官方评测器完全一致。
# dont_write_bytecode 置位：只读工具不留 __pycache__ 副作用。
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import retrieval_eval as RE   # noqa: E402

SELF_FRAGMENT = RE.SELF_FRAGMENT          # 运维与排障/01_症状索引
HUB_FRAGMENT = RE.HUB_FRAGMENT            # 20_资料索引库/README


def text_fragments(locator):
    """位置列 → 正文形态片段（`车端/05`，无下划线）。

    为什么单独造而不复用 RE.locator_to_fragments()：那个函数只造文件形态
    （`车端/05_`，供路径匹配）。拿文件形态去匹配正文恒为假，M2 会被严重低估。
    同一份 ground truth 用于两种匹配时，必须显式造两种形态。
    """
    out = []
    for part in re.split(r"[；;]", locator):
        part = part.strip()
        m = re.match(r"(%s)/([0-9][0-9])" % "|".join(RE.BOOK_PREFIXES), part)
        if m:
            out.append("%s/%s" % (m.group(1), m.group(2)))
            continue
        m = re.search(r"(docs/[^\s，；]+\.md)", part)
        if m:
            out.append(os.path.basename(m.group(1)))
            continue
        m = re.match(r"([A-Z][A-Z0-9\-]{2,})\s*卡", part)
        if m:
            out.append(m.group(1))
    return out


def query_for(symptom):
    """症状原话 → 查询串。

    规则与 retrieval_eval.py 的 evaluate() 完全一致（去引号、去全角括号内容、
    只取顿号/逗号/句号前的前半句）。那条规则在那边是内联在循环里的，没有抽成
    函数，所以这里只能照抄同一规则 —— 有意为之的重复，改任一处必须同步改另一处。
    """
    q = re.sub(r"[「」“”\"`]", "", symptom)
    q = re.sub(r"\s*（.*?）", "", q).strip()
    q = re.sub(r"[、，。].*$", "", q).strip()
    return q


def run_search(query, top=5):
    """只读调用唯一检索真源；返回解析后的 JSON 或 None。"""
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    cmd = [sys.executable, RE.SEARCH, query, "-k", str(top),
           "--json", "--live-only", "--no-timing"]
    r = subprocess.run(cmd, cwd=RE.ROOT, capture_output=True, timeout=240, env=env)
    try:
        return json.loads(r.stdout.decode("utf-8", "replace"))
    except Exception:
        return None


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    if not os.path.exists(os.path.join(RE.ROOT, RE.SEARCH)):
        print("找不到检索器 %s" % RE.SEARCH, file=sys.stderr)
        return 1
    rows = RE.read_symptom_rows()          # (编号, 症状, 位置列, 文件形态片段)
    if not rows:
        print("症状索引解析为空，ground truth 不可用", file=sys.stderr)
        return 1

    # 分类桶：top-1 是症状索引 / 是 hub / 其它文档
    def new_bucket():
        return {"n": 0, "m2": 0}

    buckets = {"症状索引": new_bucket(), "hub路由页": new_bucket(), "其它文档": new_bucket()}
    m1 = m2 = m4 = scored = 0
    top5_in = top5_out = 0          # 全库口径：期望篇在/不在 top-5
    other_in = other_out = 0        # 「其它文档当 top-1」这一组里的在/不在 top-5
    detail = []

    for no, symptom, locator, path_frags in rows:
        query = query_for(symptom)
        text_frags = text_fragments(locator)
        if len(query) < 3 or not path_frags:
            detail.append({"no": no, "query": query, "skip": True,
                           "reason": "无查询串或无位置列"})
            continue
        data = run_search(query, top=5)
        if not data or not data.get("results"):
            detail.append({"no": no, "query": query, "skip": True,
                           "reason": "检索无结果"})
            continue

        scored += 1
        res = data["results"]
        top1 = res[0]
        path_hit = any(f in top1["rel"] for f in path_frags)
        row_hit = any(any(f in (h.get("text") or "") for f in text_frags)
                      for h in top1.get("hits", []))
        in_top5 = any(any(f in x["rel"] for f in path_frags) for x in res)

        if path_hit:
            m1 += 1
        if row_hit:
            m2 += 1
        if path_hit or row_hit or in_top5:
            m4 += 1

        # 分类（用症状索引/hub 的路径特征判 top-1 身份）
        if SELF_FRAGMENT in top1["rel"]:
            key = "症状索引"
        elif HUB_FRAGMENT in top1["rel"]:
            key = "hub路由页"
        else:
            key = "其它文档"
        buckets[key]["n"] += 1
        if row_hit:
            buckets[key]["m2"] += 1
        if key != "症状索引":
            if in_top5:
                other_in += 1
            else:
                other_out += 1

        if in_top5:
            top5_in += 1
        else:
            top5_out += 1

        detail.append({
            "no": no, "query": query, "locator": locator,
            "path_frags": path_frags, "text_frags": text_frags,
            "top1": top1["rel"], "m1": path_hit, "m2": row_hit, "in_top5": in_top5,
        })

    n = scored or 1
    report = {
        "scored": scored,
        "skipped": len(rows) - scored,
        "m1": m1, "m1_pct": round(100.0 * m1 / n, 1),
        "m2": m2, "m2_pct": round(100.0 * m2 / n, 1),
        "m4": m4, "m4_pct": round(100.0 * m4 / n, 1),
        "top5_in": top5_in, "top5_out": top5_out,
        "other_top1_in_top5": other_in, "other_top1_out_top5": other_out,
        "buckets": {k: dict(v) for k, v in buckets.items()},
        "detail": detail,
    }
    if "--json" in sys.argv[1:]:
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0

    print("=" * 78)
    print("多口径对照（ground truth = 症状索引 52 条的位置列；只读，不改真库）")
    print("=" * 78)
    print("  计分 %d 条 / 跳过 %d 条" % (scored, len(rows) - scored))
    print()
    print("  M1 top-1 文件本身是权威篇            : %2d/%d = %5.1f%%" % (m1, n, 100.0 * m1 / n))
    print("  M2 top-1 命中行含去向（一次查询定位） : %2d/%d = %5.1f%%" % (m2, n, 100.0 * m2 / n))
    print("  M4 M1 或 M2 或期望篇在 top-5         : %2d/%d = %5.1f%%   ← 可用性" % (m4, n, 100.0 * m4 / n))
    print()
    print("-- ① top-1 分类表：M2 的分数是谁贡献的 --")
    print("  %-14s %5s %8s %10s" % ("top-1 是", "条数", "M2命中", "命中率"))
    for key in ("症状索引", "hub路由页", "其它文档"):
        b = buckets[key]
        pct = 100.0 * b["m2"] / b["n"] if b["n"] else 0.0
        print("  %-14s %5d %8d %9.1f%%" % (key, b["n"], b["m2"], pct))
    print()
    print("-- ② 期望篇是否进 top-5：区分「召回失败」vs「排序失败」--")
    print("  全库口径      在 top-5 %2d 条 / 不在 %2d 条" % (top5_in, top5_out))
    print("  仅『其它文档当 top-1』那组（%d 条）:" % (other_in + other_out))
    print("                 在 top-5 %2d 条 / 不在 %2d 条  ← 不在的这批，排序层治不了"
          % (other_in, other_out))
    print()
    fails = [d for d in detail if not d.get("skip") and not (d["m1"] or d["m2"] or d["in_top5"])]
    print("-- M4 仍未命中（真·一次查询到不了权威篇）%d 条 --" % len(fails))
    for x in fails:
        print("  %s  %-26s frags=%s" % (x["no"], x["query"][:26], x["path_frags"]))
        print("      top1=%s  期望篇在 top-5? %s" % (x["top1"], "是" if x["in_top5"] else "否"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
