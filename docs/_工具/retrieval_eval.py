#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检索质量评测器（LIB-01 / W0 基线冻结）

背景：资料库的首要痛点是「查一次拿不到答案」。实测 top-1 命中权威篇 = 0%，
路由页（20_资料索引库/README.md）抢走 64% 的 top-1。没有可复跑的量尺，
任何「修好了」都不可证。

做法：拿 症状索引 52 条自身的「症状 → 位置」两列当 ground truth ——
每条的「位置」列是人工标注的权威篇，因此可以机器判分：
    top-1 命中权威篇 = 一次查询就拿对了
    top-5 命中权威篇 = 五次尝试里能拿到
ground truth 用「路径片段匹配」而不做别名解析：位置列写「车端/05」，
则任何路径含 `车端/05_` 的结果都算命中；多篇用「；」分隔，命中任一即算对。

用法：
    python docs/_工具/retrieval_eval.py                 # 人读报告
    python docs/_工具/retrieval_eval.py --json          # 机器读（含逐条明细）
    python docs/_工具/retrieval_eval.py --record        # 记录为新基线
    python docs/_工具/retrieval_eval.py --check         # 与已记录基线比对门槛

退出码：0 = 达标（--check 模式）/ 正常产出（其它模式）；1 = 未达标。

为什么不用 --json 之外的接口：doc_search.py 是唯一检索真源（AGENTS §3
「同一能力不建第二份」），本工具只**调用**它，不复制它的打分逻辑。
"""
import argparse
import io
import json
import os
import re
import subprocess
import sys
import time

# docs/_工具/retrieval_eval.py → 上三级 = 项目根 v1
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SYM = os.path.join("docs", "20_资料索引库", "运维与排障", "01_症状索引_52条.md")
SEARCH = os.path.join("docs", "_工具", "doc_search.py")
BASELINE = os.path.join("docs", "_工具", "retrieval_baseline.json")

HUB_FRAGMENT = "20_资料索引库/README"
SELF_FRAGMENT = "运维与排障/01_症状索引"

# 位置列里允许出现的分册前缀（用于切分「；」分隔的多篇）
BOOK_PREFIXES = ("车端", "视觉与云台", "无人机", "空地链路", "赛事与治理",
                 "运维与排障", "外部资料", "历史")


def read_symptom_rows():
    """解析症状索引表格 → [(编号, 症状原话, 位置列, [期望路径片段...])]"""
    path = os.path.join(ROOT, SYM)
    text = io.open(path, encoding="utf-8", errors="replace").read()
    rows = []
    for line in text.splitlines():
        m = re.match(r"\|\s*(症-\d+)\s*\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|", line)
        if not m:
            continue
        no, symptom, locator = m.group(1), m.group(2).strip(), m.group(4).strip()
        rows.append((no, symptom, locator, locator_to_fragments(locator)))
    return rows


def locator_to_fragments(locator):
    """把「车端/05 §5 坑1；start_all.sh …」切成可比对的路径片段列表。

    只保留「分册/编号」形状与显式 docs 路径；丢弃 §章节 与行号（检索器不按章节排序）。

    ★2026-10-31 修正（LIB-08）：本函数同时产出**正文形态**片段。
    起因：初版只造 `车端/05_`（带下划线，用于匹配文件名），却用它去匹配正文文本。
    但正文写的是 `车端/05 §2`（无下划线）⇒ 正文匹配恒为假，M2 被低估到 3.9%，
    与手工观察矛盾。教训：同一份 ground truth 用于「路径匹配」和「文本匹配」两种用途时，
    必须显式造两套形态，不能复用同一串。
    """
    frags = []
    for part in re.split(r"[；;]", locator):
        part = part.strip()
        # 形状一：分册/编号（如 车端/05、视觉与云台/01）
        m = re.match(r"(%s)/([0-9][0-9])" % "|".join(BOOK_PREFIXES), part)
        if m:
            frags.append("%s/%s_" % (m.group(1), m.group(2)))
            continue
        # 形状二：显式 docs 路径（如 docs/00_项目总览/现状快照.md）
        m = re.search(r"(docs/[^\s，；]+\.md)", part)
        if m:
            frags.append(m.group(1).replace("docs/", "", 1))
            continue
        # 形状三：任务卡号（如 TMP-GC-20261023、PLUGIN-PREFLIGHT）
        m = re.match(r"([A-Z][A-Z0-9\-]{2,})\s*卡", part)
        if m:
            frags.append(m.group(1))
            continue
    return frags


def text_fragments(locator):
    """与 locator_to_fragments 同源，但产出**正文形态**（去掉文件名用的下划线）。

    M2 口径专用：判断「top-1 的命中行文本里有没有写出应该去哪」。
    """
    out = []
    for part in re.split(r"[；;]", locator):
        part = part.strip()
        m = re.match(r"(%s)/([0-9][0-9])" % "|".join(BOOK_PREFIXES), part)
        if m:
            out.append("%s/%s" % (m.group(1), m.group(2)))   # 正文形态：无下划线
            continue
        m = re.search(r"(docs/[^\s，；]+\.md)", part)
        if m:
            out.append(os.path.basename(m.group(1)))
            continue
        m = re.match(r"([A-Z][A-Z0-9\-]{2,})\s*卡", part)
        if m:
            out.append(m.group(1))
            continue
    return out


def run_search(query, top=5):
    """调用唯一检索真源，返回 (paths, results, elapsed_ms, corpus_files, err)。"""
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    cmd = [sys.executable, SEARCH, query, "-k", str(top), "--json", "--live-only", "--no-timing"]
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, timeout=180, env=env)
    out = r.stdout.decode("utf-8", errors="replace")
    try:
        data = json.loads(out)
    except Exception:
        return [], [], 0.0, 0, out.strip().splitlines()[-1:] if out else ["(无输出)"]
    results = data.get("results", [])
    paths = [x["rel"] for x in results]
    return paths, results, data.get("elapsed_ms", 0.0), data.get("corpus", {}).get("files", 0), []


def evaluate(rows, top=5):
    detail = []
    t0 = time.time()
    for no, symptom, locator, frags in rows:
        q = re.sub(r"[「」“”\"`]", "", symptom)
        q = re.sub(r"\s*（.*?）", "", q).strip()
        q = re.sub(r"[、，。].*$", "", q).strip()  # 只取症状前半句，避免查询过长
        if len(q) < 3 or not frags:
            detail.append({"no": no, "query": symptom, "skip": True,
                           "reason": "无查询串或无定位符"})
            continue
        paths, results, ms, corpus, err = run_search(q, top)
        if err:
            detail.append({"no": no, "query": q, "skip": True, "reason": "检索失败: %s" % err})
            continue

        def hit(rank_list, pats=None):
            pats = pats or frags
            for p in rank_list:
                if any(f in p for f in pats):
                    return p
            return None

        t_frags = text_fragments(locator)
        top1_path_hit = hit(paths[:1])
        top1_res = results[0] if results else {}
        # M2：top-1 的命中行文本里有没有写出「该去哪」
        row_hit = False
        if t_frags:
            for h in top1_res.get("hits", []):
                txt = h.get("text") or ""
                if any(f in txt for f in t_frags):
                    row_hit = True
                    break

        detail.append({
            "no": no, "query": q, "locator": locator,
            "frags": frags, "text_frags": t_frags,
            "top1_path": paths[0] if paths else None,
            "top1_hit": top1_path_hit is not None,     # M1
            "row_hit": row_hit,                        # M2
            "either_hit": (top1_path_hit is not None) or row_hit,   # M3 = M1∪M2
            "top5_hit": hit(paths[:top]) is not None,
            # M4 = 一次查询能到达权威结论（三条合法路径：直接命中 / 经索引中转 / 在 top-5 内）
            # ★必须与 retrieval_multi_caliber_probe.py 的 M4 同定义，否则两个工具会互相矛盾。
            "m4_hit": ((top1_path_hit is not None) or row_hit
                       or hit(paths[:top]) is not None),
            "top5_hit_path": hit(paths[:top]),
            "hub_top1": bool(paths) and HUB_FRAGMENT in paths[0],
            "self_top1": bool(paths) and SELF_FRAGMENT in paths[0],
            "elapsed_ms": ms,
        })
    valid = [d for d in detail if not d.get("skip")]
    n = len(valid) or 1

    def cnt(k):
        return sum(1 for d in valid if d.get(k))

    def pct(k):
        return round(100.0 * cnt(k) / n, 1)

    agg = {
        "queries_parsed": len(rows),
        "queries_scored": len(valid),
        "queries_skipped": len(detail) - len(valid),
        "top1_hits": cnt("top1_hit"),
        "row_hits": cnt("row_hit"),
        "either_hits": cnt("either_hit"),
        "m4_hits": cnt("m4_hit"),
        "top5_hits": cnt("top5_hit"),
        "hub_top1": cnt("hub_top1"),
        "self_top1": cnt("self_top1"),
        "top1_pct": pct("top1_hit"),
        "row_pct": pct("row_hit"),
        "either_pct": pct("either_hit"),
        "m4_pct": pct("m4_hit"),
        "top5_pct": pct("top5_hit"),
        "hub_top1_pct": pct("hub_top1"),
        "self_top1_pct": pct("self_top1"),
        "wall_s": round(time.time() - t0, 1),
    }
    return agg, detail


# --- 门槛 ---
# 2026-10-31 口径演进史（别删，后来者需要知道为什么变了三次）：
#   v1（11:51）主口径 = top1_pct（top-1 文件本身是权威分册篇），目标 ≥60%。
#        实测基线 0.0% ⇒ 曾误判为「检索完全没用」。
#   v2（13:13）改主口径 = row_pct（M2：top-1 命中行是否写出该去的篇），目标 ≥85%。
#        但被独立验收 + 成员探针共同证明有设计缺陷：M2 的分数几乎全部来自
#        「症状索引坐 top-1」（26 条全中 vs 其它文档 1/25），而排序改动的目标恰是
#        把路由型文档赶出 top-1 ⇒ 两条指标在排序层互相拆台。
#   v3（13:30）改主口径 = either_pct+M4（M1 ∪ M2 ∪ 期望篇在 top-5），目标 ≥90%。
#        这是「一次查询能不能到达权威结论」的完整定义，含三条合法路径：
#          A 直接命中权威篇（M1）
#          B 经症状索引/路由页中转，且命中行写出该去的篇（M2）
#          C 权威篇虽非 top-1 但在 top-5 内（用户翻两三条即可到达）
#        M1/M2 降为辅助指标（各测一段，不单独作为成败依据）。
#
# 多口径复跑工具：docs/_工具/retrieval_multi_caliber_probe.py（只读，不改真库）。
# 实测（2026-10-31 13:26）：M1=9.8% / M2=52.9% / M4=68.6%，与 eval 一致。
#
# 已知剩余病灶（M4 未命中的 16 条全部「期望篇不在 top-5」⇒ 召回失败，排序治不了）：
#   权威篇写的���「机制」（如「命令发出后无人订阅」），查询是「症状原话」（如「激光不亮」），
#   2-gram 字面命中率不足。解法不是排序，是**给权威篇补「症状原话→本篇」的字面锚点**。
GATES = {
    "m4_pct": (78.4, 90.0),        # M4 主口径：**真实基线 78.4%**（旧状态实影，见下）→ 目标 ≥90%
    "row_pct": (76.5, 85.0),       # M2 辅助：一次查询命中行给出去向（基线 76.5%）
    "top1_pct": (0.0, 40.0),       # M1 辅助：top-1 直接就是权威篇（不要求高，只要不退化）
    "hub_top1_pct": (56.9, 10.0),  # 路由页（纯转述型 README/00_索引）抢位：≤10%
}


def cmd_record(agg, detail):
    payload = {
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S+08:00"),
        "ground_truth": SYM,
        "agg": agg,
        "gates": GATES,
        "detail": detail,
    }
    path = os.path.join(ROOT, BASELINE)
    io.open(path, "w", encoding="utf-8").write(
        json.dumps(payload, ensure_ascii=False, indent=2))
    print("已记录基线 → %s" % BASELINE)
    print_summary(agg)
    return 0


def cmd_check(agg):
    print_summary(agg)
    ok = True
    print("\n门槛判定：")
    for key, (was, target) in GATES.items():
        cur = agg[key]
        if key == "hub_top1_pct":
            passed = cur <= target
            arrow = "≤"
        else:
            passed = cur >= target
            arrow = "≥"
        flag = "PASS" if passed else "FAIL"
        print("  [%s] %-14s 目标 %s %-5.1f  实测 %-5.1f  (基线 %.1f)"
              % (flag, key, arrow, target, cur, was))
        ok = ok and passed
    print("\n结论: %s" % ("达标" if ok else "未达标"))
    return 0 if ok else 1


def print_summary(agg):
    print("=" * 66)
    print("检索评测（ground truth = 症状索引 52 条的位置列）")
    print("=" * 66)
    print("  解析 %d 条 / 计分 %d 条 / 跳过 %d 条"
          % (agg["queries_parsed"], agg["queries_scored"], agg["queries_skipped"]))
    print()
    print("  【主口径·可用性】M4 一次查询能到达权威结论")
    print("            （三条合法路径：top-1 直接命中 / 经索引中转给出去向 / 权威篇在 top-5 内）")
    print("            %d/%d = %.1f%%"
          % (agg["m4_hits"], agg["queries_scored"], agg["m4_pct"]))
    print()
    print("  【分项】  M2 经索引中转让出去向 %d/%d = %.1f%%"
          % (agg["row_hits"], agg["queries_scored"], agg["row_pct"]))
    print("            M3 M1∪M2（不含 top-5）  %d/%d = %.1f%%"
          % (agg["either_hits"], agg["queries_scored"], agg["either_pct"]))
    print()
    print("  【辅助】  M1 top-1 文件本身是权威篇（不用点进索引页）")
    print("            %d/%d = %.1f%%"
          % (agg["top1_hits"], agg["queries_scored"], agg["top1_pct"]))
    print("            top-5 命中权威篇 %d/%d = %.1f%%"
          % (agg["top5_hits"], agg["queries_scored"], agg["top5_pct"]))
    print()
    print("  路由页抢 top-1   : %d/%d = %.1f%%"
          % (agg["hub_top1"], agg["queries_scored"], agg["hub_top1_pct"]))
    print("  症状索引自吸     : %d/%d = %.1f%%"
          % (agg["self_top1"], agg["queries_scored"], agg["self_top1_pct"]))
    print("  墙钟 %.1f s" % agg["wall_s"])


def print_detail(detail):
    misses = [d for d in detail if not d.get("skip") and not d["either_hit"]]
    print("\n-- M3 未命中（真·答不出）(%d) --" % len(misses))
    for d in misses[:40]:
        got = d["top1_path"] or "(无命中)"
        print("  %s  %-26s 期望=%s" % (d["no"], d["query"][:26], d["frags"]))
        print("      → top1=%s" % got.replace("docs/20_资料索引库/", "").replace("docs/", ""))
    if len(misses) > 40:
        print("  … 另 %d 条见 --json" % (len(misses) - 40))
    only_m2 = [d for d in detail if not d.get("skip") and d["row_hit"] and not d["top1_hit"]]
    print("\n-- M2 成立但 M1 不成立（索引页给出去向、文件非权威篇）(%d) --" % len(only_m2))
    for d in only_m2[:20]:
        print("  %s  %-26s top1=%s" % (d["no"], d["query"][:26],
                                       (d["top1_path"] or "").replace("docs/20_资料索引库/", "")))
    if len(only_m2) > 20:
        print("  … 另 %d 条见 --json" % (len(only_m2) - 20))


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="检索质量评测器（LIB-01 W0 基线冻结）")
    ap.add_argument("--json", action="store_true", help="机器读（输出完整明细 JSON）")
    ap.add_argument("--record", action="store_true", help="记录为新基线")
    ap.add_argument("--check", action="store_true", help="与内置门槛比对（未达标 exit1）")
    ap.add_argument("--detail", action="store_true", help="人读模式附逐条明细")
    ap.add_argument("-k", "--top", type=int, default=5, help="取前 N 篇（默认 5）")
    args = ap.parse_args()

    if not os.path.exists(os.path.join(ROOT, SEARCH)):
        print("找不到检索器 %s" % SEARCH, file=sys.stderr)
        return 2
    rows = read_symptom_rows()
    if not rows:
        print("症状索引解析为空，ground truth 不可用", file=sys.stderr)
        return 2

    agg, detail = evaluate(rows, top=args.top)

    if args.json:
        payload = {"agg": agg, "gates": GATES, "detail": detail}
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0
    if args.record:
        return cmd_record(agg, detail)
    if args.check:
        return cmd_check(agg)

    print_summary(agg)
    if args.detail:
        print_detail(detail)
    return 0


if __name__ == "__main__":
    sys.exit(main())
