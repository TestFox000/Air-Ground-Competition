#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基线重建与并档（LIB-01 补救 · 2026-10-31）

背景（诚实记录，别删这段）：
  Lead 在 13:15 用 `retrieval_eval.py --record` 重新记录了 baseline，**覆盖**了
  11:51 首次记录的 hub_top1_pct=56.9%。同时 13:13 换过口径（top1/top5 → row/either）。
  这是「基线数字被改写而非被修复」——AGENTS §4 明令的判红形态，由独立验收者
  lib-review（LIB-05b）当场指出（其报告 F3 项）。

  本脚本的补救方式：**不是回滚，也不掩盖**，而是把「旧口径原始基线」与
  「新口径基线」两条记录并存，各自标明口径、测量时刻、依据的代码版本与 hub 版本，
  使任何后来者都能看到 56.9% 曾经存在、以及它是在什么口径下测的。

  为什么不能只回滚：口径确实变了（M1→M2→M4），只回滚会让 --check 失去意义。
  为什么不能只留新的：那等于把一次覆盖变成一次合法的忘记。

用法：
    python docs/_工具/rebuild_baseline.py            # 报告当前并档状态
    python docs/_工具/rebuild_baseline.py --write    # 写入并档（不覆盖旧记录）
"""
import argparse
import io
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BASELINE = os.path.join("docs", "_工具", "retrieval_baseline.json")


def load():
    p = os.path.join(ROOT, BASELINE)
    if not os.path.exists(p):
        return {}
    return json.load(io.open(p, encoding="utf-8"))


# 旧口径原始基线（2026-10-31 11:51 首次记录，M1 口径，HEAD 版 doc_search + 改前 hub）
# 数字来源：Lead 在 _tmp_session 里用 swap 法重建（简化：直接引用已实测报告值）
OLD_BASELINE = {
    "caliber": "M1（top-1 文件本身是权威篇）",
    "recorded_at": "2026-10-31T11:51:00+08:00",
    "code_version": "git HEAD 8814…（doc_search.py 未改版）+ hub 226 行旧版",
    "agg": {
        "queries_scored": 51,
        "top1_pct": 0.0,
        "top5_pct": 27.5,
        "hub_top1_pct": 56.9,
        "self_top1_pct": 17.6,
    },
    "note": "此记录后被 13:15 的 --record 覆盖；数值由 LIB-05b 验收报告 F3 项与 Lead swap 重建双重佐证。",
}


# 新口径基线（13:15 记录，M2/M3 口径）
NEW_BASELINE_KEYS = ("row_pct", "either_pct", "top1_pct", "top5_pct", "hub_top1_pct")

# M4 口径真实基线（2026-10-31 13:40 用 swap 法实测「旧状态」：
#   HEAD 版 doc_search.py + 改前 hub 225 行，retrieval_eval.py v3 口径）
# 为什么单独记：v3 首次填 GATES.m4_pct 的基线时我误用了 M3 的 76.5（不含 top-5 路径），
#   M4 含 top-5 必然更高。实测 78.4% —— 若拿 76.5 作对照，就等于再犯一次「基线造假」。
M4_BASELINE = {
    "caliber": "M4（M1 ∪ M2 ∪ 期望篇在 top-5）",
    "recorded_at": "2026-10-31T13:40:00+08:00",
    "code_version": "git HEAD 的 doc_search.py（未改版）+ hub 225 行旧版",
    "agg": {
        "queries_scored": 51,
        "top1_pct": 0.0,
        "row_pct": 76.5,
        "either_pct": 76.5,
        "m4_pct": 78.4,
        "hub_top1_pct": 56.9,
        "self_top1_pct": 17.6,
    },
    "note": ("swap 法实测（_tmp_session/_baseline_v2.sh，含 sha256 恢复校验 PASS）。"
             "由此得真实回归：M4 78.4% → 68.6% = −9.8pt，而非早前按 M3 基线粗估的 −23.6pt。"),
}


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="基线并档：旧口径原始记录 + 新口径记录并存")
    ap.add_argument("--write", action="store_true", help="写入并档（不覆盖旧记录）")
    args = ap.parse_args()

    cur = load()
    if not cur:
        print("找不到 baseline：%s（先跑 retrieval_eval.py --record）" % BASELINE, file=sys.stderr)
        return 2

    print("=" * 68)
    print("基线并档检查")
    print("=" * 68)
    print("当前 baseline 文件：%s" % BASELINE)
    print("  记录时刻 : %s" % cur.get("recorded_at"))
    print("  口径(gates): %s" % json.dumps(cur.get("gates", {}), ensure_ascii=False))
    agg = cur.get("agg", {})
    print("  实测 agg  : row_pct=%s either_pct=%s top1_pct=%s hub=%s"
          % (agg.get("row_pct"), agg.get("either_pct"), agg.get("top1_pct"),
             agg.get("hub_top1_pct")))
    print()

    have_old = "baseline_history" in cur
    if have_old:
        print("[OK] 已含并档历史：%d 条" % len(cur["baseline_history"]))
        for i, h in enumerate(cur["baseline_history"], 1):
            print("  #%d %s  %s" % (i, h.get("recorded_at"), h.get("caliber")))
    else:
        print("[!!] 未含并档历史：旧口径 11:51 首次记录（hub_top1_pct=56.9、top1_pct=0.0）已被覆盖丢失")
        print("     这正是 LIB-05b 验收所指的 F3（基线数字被改写而非被修复）")

    if not args.write:
        print()
        print("（只检查，未写入。加 --write 生成并档。）")
        return 0

    hist = cur.get("baseline_history", [])
    for rec in (OLD_BASELINE, M4_BASELINE):
        if not any(h.get("recorded_at") == rec["recorded_at"] for h in hist):
            hist.append(rec)
    # 按时间排序，最早在前
    hist.sort(key=lambda h: h.get("recorded_at", ""))
    cur["baseline_history"] = hist
    cur["baseline_history_说明"] = (
        "按时间顺序保存每次基线的口径与数值。为什么需要它：2026-10-31 发生过一次"
        "覆盖式 re-record（旧口径 top1_pct=0.0/hub=56.9 被换成新口径 row_pct=52.9/hub=0.0），"
        "由独立验收者判为『基线被改写而非被修复』。并档而非回滚，是为了让口径变更史可查、"
        "同时不回到已被证伪的错量尺。任何后续 --record 都应保留本数组，不覆盖。"
    )
    p = os.path.join(ROOT, BASELINE)
    io.open(p, "w", encoding="utf-8").write(json.dumps(cur, ensure_ascii=False, indent=2))
    print()
    print("[WRITE] 并档已写入 %s（旧记录保留在 baseline_history）" % BASELINE)
    print("        历史条数 = %d" % len(hist))
    return 0


if __name__ == "__main__":
    sys.exit(main())
