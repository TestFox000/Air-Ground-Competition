#!/usr/bin/env python3
"""离线回放情报帧，调 car 端投票参数（min_votes / min_ratio / ttl_s）。

输入是 JSONL，每行一帧（由机载端落盘或链路抓包得到）：
    {"seq": 1, "mines": ["C3"], "obstacles": [], "bridge_a": "K4", "bridge_b": null,
     "target_cells": [], "target_face": null, "ts": 0.0}

两种用法：
    # 单组参数回放，看收敛过程
    python tools/replay_intel.py --input intel.jsonl --min-votes 2 --ratio 0.5

    # 参数扫描：找出「最早收敛且不引入误报」的组合
    python tools/replay_intel.py --input intel.jsonl --sweep

判定基准（--truth 未给出时）：出现率 >= 50% 的格号视为真实目标。
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
# 每个包目录都要入 path（src/<pkg>/<pkg>/ 两层布局），与 tests/conftest.py 一致
for _p in [SRC] + sorted(p for p in SRC.iterdir() if p.is_dir()):
    sys.path.insert(0, str(_p))

from ag_core.link_protocol import FieldIntel  # noqa: E402
from ag_mission.field_fusion import IntelVoteConfig, IntelVoter  # noqa: E402


def load_frames(path: str) -> List[FieldIntel]:
    frames: List[FieldIntel] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            frames.append(FieldIntel.from_dict(json.loads(line)))
    if not frames:
        raise SystemExit(f"文件里没有有效帧: {path}")
    return frames


def dominant_cells(frames: List[FieldIntel], ratio: float = 0.5) -> Set[str]:
    c: Counter = Counter()
    for fr in frames:
        for m in fr.mines:
            c[m] += 1
    return {cell for cell, n in c.items() if n / max(1, len(frames)) >= ratio}


def replay(frames: List[FieldIntel], min_votes: int, ratio: float, ttl: float,
           truth: Set[str], verbose: bool) -> Tuple[Optional[int], List[str]]:
    """返回 (首次收敛帧号, 误报格号列表)。收敛 = 稳定情报首次等于真值集合。"""
    v = IntelVoter(IntelVoteConfig(min_votes=min_votes, min_ratio=ratio, ttl_s=ttl))
    converged_at: Optional[int] = None
    false_cells: Set[str] = set()
    for i, fr in enumerate(frames, start=1):
        v.push(fr, now=float(fr.ts or i))
        st = v.stable
        cur = set(st.mines) if st else set()
        if verbose:
            mark = "  <== 已收敛到真值" if (converged_at is None and cur == truth) else ""
            print(f"帧 {i:3d} seq={fr.seq:3d} 稳定雷区={sorted(cur)}{mark}")
        if converged_at is None and cur == truth:
            converged_at = i
        false_cells |= (cur - truth)
    return converged_at, sorted(false_cells)


def sweep(frames: List[FieldIntel], truth: Set[str]) -> List[dict]:
    results: List[dict] = []
    for min_votes in (1, 2, 3):
        for ratio in (0.3, 0.5, 0.7):
            for ttl in (10.0, 20.0, 40.0):
                v = IntelVoter(IntelVoteConfig(min_votes=min_votes, min_ratio=ratio, ttl_s=ttl))
                converged_at: Optional[int] = None
                false_cells: Set[str] = set()
                for i, fr in enumerate(frames, start=1):
                    v.push(fr, now=float(fr.ts or i))
                    st = v.stable
                    cur = set(st.mines) if st else set()
                    if converged_at is None and cur == truth:
                        converged_at = i
                    false_cells |= (cur - truth)
                results.append(
                    {
                        "min_votes": min_votes,
                        "ratio": ratio,
                        "ttl": ttl,
                        "converged_at": converged_at,
                        "false": sorted(false_cells),
                    }
                )
    return results


def main() -> int:
    ap = argparse.ArgumentParser(description="情报帧离线回放与投票参数扫描")
    ap.add_argument("--input", required=True, help="JSONL 情报帧文件")
    ap.add_argument("--min-votes", type=int, default=2)
    ap.add_argument("--ratio", type=float, default=0.5)
    ap.add_argument("--ttl", type=float, default=20.0)
    # ⚠️ argparse 的 help 会做 % 格式化，字面百分号必须写成 %%
    # （写成 50% 会让 `--help` 直接抛 ValueError）
    ap.add_argument("--truth", default="", help="真实雷区格号，逗号分隔；缺省用出现率 50%% 推断")
    ap.add_argument("--sweep", action="store_true", help="扫描参数组合")
    args = ap.parse_args()

    frames = load_frames(args.input)
    truth = {s.strip().upper() for s in args.truth.split(",") if s.strip()} or dominant_cells(frames)
    print(f"载入 {len(frames)} 帧；真值雷区 = {sorted(truth)}")

    if args.sweep:
        results = sweep(frames, truth)
        ok = [r for r in results if r["converged_at"] is not None and not r["false"]]
        print("\n参数扫描（要求：能收敛 且 无误报）")
        print(f"{'min_votes':>9} {'ratio':>6} {'ttl':>6} {'收敛帧':>7}  误报")
        for r in sorted(results, key=lambda x: ((x["converged_at"] or 9999), len(x["false"]))):
            print(f"{r['min_votes']:>9} {r['ratio']:>6.1f} {r['ttl']:>6.0f} "
                  f"{str(r['converged_at'] or '-'):>7}  {','.join(r['false']) or '-'}")
        if ok:
            best = min(ok, key=lambda x: x["converged_at"])
            print(f"\n推荐: min_votes={best['min_votes']} min_ratio={best['ratio']} "
                  f"ttl_s={best['ttl']}（第 {best['converged_at']} 帧收敛，无误报）")
        else:
            print("\n没有组合能同时满足「收敛 + 无误报」，优先保证无误报，再调 min_votes")
        return 0

    print()
    converged_at, false_cells = replay(frames, args.min_votes, args.ratio, args.ttl, truth, True)
    print(f"\n首次收敛帧: {converged_at or '未收敛'}")
    print(f"误报格号: {', '.join(false_cells) or '无'}")
    if converged_at is None:
        print("未收敛：可降低 min_ratio / min_votes，或用 --sweep 扫描")
        return 1
    if false_cells:
        print("存在误报：可提高 min_votes / min_ratio")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
