# -*- coding: utf-8 -*-
"""64 图双引擎规划头对头基准（2026-10-26，[H2H-PLAN-20261026]）。

口径
----
- 图集：本脚本 ``generate_maps()``（seed=20261026）生成 64 张随机图——
  雷 8 / 障 8 / 打击区相邻两格（移动靶轨道）/ 桥洞 K5+K7 双开 / 起 N10 终 N5；
  雷/障/靶不入 K 列（K 列整列是桥）、不压起终点与桥格。
  *为什么不是对方压测的 seed=20261018 原图集*：其生成器 stressbench.generateMap
  在 gitignore 的 _tmp_session 里，未入库已丢，无法逐字节复刻；改为同规则重生成，
  双引擎在同一图集上跑，公平性由“同图同权”保证（对方原口径见
  docs/压测_规划器64图_20261018.md，其 tour 基线 28/64=43.75%）。
- A 方（车端现役）：../air_ground_car @2582d1b 的 tools/plan_route.py
  ``--emit json --check --quiet --engine tour``，子进程隔离，判定用他们自己的
  route_check（exit 0）+ failures 空。
- B 方（我方）：ag_integrate.mission_router.plan_route，增强全开
  （turn90_guard/solid_barriers/reverse/heading_prop），起终点 N10→N5。
- 成功口径（全任务）：雷 8/8 覆盖 + 桥洞 2/2 到访 + 靶旁停站 ≥1；
  里程=格心折线（0.35 m/格，对角 ×√2）；秒数=route_time.estimate_seconds。

用法
----
    python tools/h2h_planner_bench.py --n 8      # 试点
    python tools/h2h_planner_bench.py            # 全量 64
产物：_h2h_20261026/{maps/*.yaml, results.json, summary.txt}
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import subprocess
import sys
import warnings

HERE = os.path.dirname(os.path.abspath(__file__))
OURS_ROOT = os.path.dirname(HERE)
THEIR_WT = os.path.abspath(os.path.join(OURS_ROOT, "..", "air_ground_car"))
OUT_DIR = os.path.join(OURS_ROOT, "_h2h_20261026")

COLS = "ABCDEFGHIJKLMN"          # 14 列
N_ROWS = 11                      # 11 行
START, END = "N10", "N5"
BRIDGES = ("K5", "K7")           # 双开（同对方现役第六图）
CELL = 0.35                      # m
SEED = 20261026

_MOTION_ON = dict(turn90_guard=True, solid_barriers=True,
                  reverse=True, heading_prop=True)


# ---------------------------------------------------------------- 图集生成
def _neighbors4(name: str):
    ci, ri = COLS.index(name[0]), int(name[1:]) - 1
    for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        c, r = ci + dc, ri + dr
        if 0 <= c < len(COLS) and 0 <= r < N_ROWS:
            yield COLS[c] + str(r + 1)


def _k_col() -> set[str]:
    return {c + str(r) for c in ("K",) for r in range(1, N_ROWS + 1)}


def generate_maps(n: int = 64, seed: int = SEED) -> list[dict]:
    rng = random.Random(seed)
    banned_static = ({START, END} | set(BRIDGES) | _k_col())
    maps: list[dict] = []
    for idx in range(n):
        while True:  # 打击区：相邻两格（轨道），非 K 列、非固定占用
            base = rng.choice([c + str(r) for c in COLS[:10] for r in range(1, N_ROWS + 1)])
            if base in banned_static:
                continue
            horiz = rng.random() < 0.5
            a_ci, a_ri = COLS.index(base[0]), int(base[1:]) - 1
            b_ci, b_ri = (a_ci + 1, a_ri) if horiz else (a_ci, a_ri + 1)
            if not (0 <= b_ci < len(COLS)) or not (0 <= b_ri < N_ROWS):
                continue
            pair = (base, COLS[b_ci] + str(b_ri + 1))
            if pair[1] in banned_static:
                continue
            break
        reserved = banned_static | set(pair)
        pool = [c + str(r) for c in COLS for r in range(1, N_ROWS + 1)
                if c + str(r) not in reserved]
        mines = rng.sample(pool, 8)
        rest = [c for c in pool if c not in mines]
        obstacles = rng.sample(rest, 8)
        # 蓝面朝向追加抽取（不扰动上面的随机序列）：取首个"两端候选停站
        # 均在界内且不在障碍/桥墙/轨道上"的一侧；全无效 = 该靶物理不可射
        # （双方都应无解）。依据 _shoot_candidates_moving：候选=轨道两端在
        # 蓝面侧的正交邻格。
        occupied = set(obstacles) | set(mines) | set(pair) | set(BRIDGES) | _k_col()
        vertical = pair[0][0] == pair[1][0]

        def _cand_ok(c: str) -> bool:
            return (c[0] in COLS and 1 <= int(c[1:]) <= N_ROWS
                    and c not in occupied)

        if vertical:
            col = pair[0][0]
            faces = ("E", "W")
            cand = {"E": [COLS[COLS.index(col) + 1] + pair[0][1:],
                          COLS[COLS.index(col) + 1] + pair[1][1:]],
                    "W": [COLS[COLS.index(col) - 1] + pair[0][1:],
                          COLS[COLS.index(col) - 1] + pair[1][1:]]}
        else:
            row = pair[0][1:]
            faces = ("S", "N")
            cand = {"S": [pair[0][0] + str(int(row) + 1), pair[1][0] + str(int(row) + 1)],
                    "N": [pair[0][0] + str(int(row) - 1), pair[1][0] + str(int(row) - 1)]}
        blue_face = next((f for f in faces
                          if all(_cand_ok(c) for c in cand[f])), None)
        maps.append(dict(idx=idx, start=START, end=END, bridges=list(BRIDGES),
                         targets=list(pair), mines=mines, obstacles=obstacles,
                         blue_face=blue_face))
    return maps


def write_map_yaml(m: dict) -> str:
    path = os.path.join(OUT_DIR, "maps", f"map_{m['idx']:02d}.yaml")
    with open(path, "w", encoding="utf-8") as f:
        f.write(
            "map:\n"
            f"  name: \"H2H-20261026 idx{m['idx']:02d}\"\n"
            f"  start: \"{m['start']}\"\n"
            f"  end: \"{m['end']}\"\n"
            "  barrier_col_from: \"D\"\n"
            "  barrier_row_hi: 6\n"
            "  barrier_row_lo: 7\n"
            f"  mines:      {m['mines']}\n"
            f"  obstacles:  {m['obstacles']}\n"
            f"  targets:    {m['targets']}\n"
            f"  bridges:    {m['bridges']}\n")
    return path


# ---------------------------------------------------------------- A 方（tour）
def run_tour(map_path: str) -> dict:
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1")
    try:
        p = subprocess.run(
            [sys.executable, "tools/plan_route.py", "--map", map_path,
             "--emit", "json", "--check", "--quiet", "--engine", "tour"],
            cwd=THEIR_WT, capture_output=True, text=True,
            encoding="utf-8", timeout=90, env=env)
    except subprocess.TimeoutExpired:
        return dict(ok=False, reason="timeout90s")
    out = p.stdout.strip()
    if not out.startswith("{"):
        return dict(ok=False, reason=f"cli_rc={p.returncode}: "
                     + (p.stderr or out)[:160].replace("\n", " "))
    try:
        d = json.loads(out)
    except json.JSONDecodeError as e:
        return dict(ok=False, reason=f"json_err:{e.msg}")
    if d.get("failures"):
        r = str(d["failures"][0])[:160]
        return dict(ok=False, reason=r)
    plans = d.get("plans") or []
    if not plans:
        return dict(ok=False, reason="no_plan")
    plan = plans[0]
    rc = d.get("route_check") or [{}]
    rc_exit = rc[0].get("exit", 1) if rc else 1
    ok = rc_exit == 0
    return dict(
        ok=ok,
        reason="" if ok else "route_check_exit_" + str(rc_exit),
        tokens=plan.get("token_count"),
        length_m=plan.get("polylength_m"),
        pivot_count=plan.get("pivot_count"),
        cost=plan.get("cost"),
        shoot_stops=plan.get("shoot_stops"),
        elapsed_ms=plan.get("elapsed_ms"),
        rc_lines=rc[0].get("lines", []) if rc else [],
    )


# ---------------------------------------------------------------- B 方（我方）
def _load_ours():
    sys.path.insert(0, os.path.join(OURS_ROOT, "src", "ag_integrate"))
    from ag_integrate.field_frame import CellRef, LETTERS
    from ag_integrate.world_model import WorldSnapshot, TargetRef
    from ag_integrate.mission_router import plan_route
    from ag_integrate.route_time import estimate_seconds

    def cell(name: str) -> CellRef:
        return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)
    return cell, WorldSnapshot, TargetRef, plan_route, estimate_seconds


def _polyline_len(route) -> float:
    from ag_integrate.field_frame import LETTERS
    total = 0.0
    for a, b in zip(route, route[1:]):
        dc = abs(LETTERS.index(b[0]) - LETTERS.index(a[0]))
        dr = abs(int(b[1:]) - int(a[1:]))
        if dc and dr:
            total += CELL * math.sqrt(2)
        else:
            total += CELL * max(dc, dr)
    return round(total, 2)


def run_ours(m: dict, kit, permissive: bool = False) -> dict:
    cell, WorldSnapshot, TargetRef, plan_route, estimate_seconds = kit
    pair = m["targets"]
    vertical = pair[0][0] == pair[1][0]
    blue_face = m.get("blue_face") or ("E" if vertical else "S")
    snap = WorldSnapshot(
        timestamp_ms=1,
        mines=tuple(cell(n) for n in m["mines"]),
        passable=tuple(cell(n) for n in m["bridges"]),
        obstacles=tuple(cell(n) for n in m["obstacles"]),
        targets=(TargetRef(cell(pair[0]), True,
                           track=(cell(pair[0]), cell(pair[1])),
                           blue_face=blue_face),),
    )
    # 停站判定与规划器同源（_shoot_candidates_moving）：轨道两端蓝面侧邻格
    if vertical:
        ci = COLS.index(pair[0][0])
        dci = 1 if blue_face == "E" else -1
        stop_cells = {COLS[ci + dci] + pair[0][1:], COLS[ci + dci] + pair[1][1:]}
    else:
        ri = int(pair[0][1:])
        dri = 1 if blue_face == "S" else -1
        stop_cells = {pair[0][0] + str(ri + dri), pair[1][0] + str(ri + dri)}
    kw = dict(_MOTION_ON)
    if permissive:
        # 宽松口径：tunnel_dirs={} 关隧道轴向+进洞邻格硬规则（2026-10-24 实测
        # 蹭墙证据的保守策略），对齐对方引擎的宽松度——用于隔离"安全策略
        # 差异"与"搜索能力差异"；生产模式仍用严格口径。
        kw["tunnel_dirs"] = {}
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            route = plan_route(snap, cell(m["start"]), cell(m["end"]), **kw)
    except Exception as e:  # noqa: BLE001 —— 失败原因也是产出
        return dict(ok=False, reason=f"{type(e).__name__}: {str(e)[:150]}")
    if not route:
        return dict(ok=False, reason="empty_route")
    names = list(route)
    mines_cov = sum(1 for x in m["mines"] if x in names)
    bridge_cov = sum(1 for x in m["bridges"] if x in names)
    stops = [x for x in names if x in stop_cells]
    length_m = _polyline_len(names)
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            sec = float(estimate_seconds(
                tuple(names), mine_cells=set(m["mines"]),
                shoot_cells=set(stops) or frozenset()).seconds)
    except Exception:
        sec = None
    ok = mines_cov == 8 and bridge_cov == len(m["bridges"]) and stops
    return dict(ok=ok, reason="" if ok else
                f"mines {mines_cov}/8 bridges {bridge_cov}/2 stops {len(stops)}",
                tokens=len(names), length_m=length_m, seconds=sec,
                mines_cov=mines_cov, bridge_cov=bridge_cov,
                stop_count=len(stops), route=names)


# ---------------------------------------------------------------- 主流程
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=64)
    ap.add_argument("--seed", type=int, default=SEED)
    a = ap.parse_args()

    os.makedirs(os.path.join(OUT_DIR, "maps"), exist_ok=True)
    maps = generate_maps(a.n, a.seed)
    kit = _load_ours()

    rows = []
    for m in maps:
        mp = write_map_yaml(m)
        tour = run_tour(mp)
        ours = run_ours(m, kit, permissive=False)
        ours_p = run_ours(m, kit, permissive=True)
        rows.append(dict(idx=m["idx"], targets=m["targets"],
                         tour=tour, ours=ours, ours_p=ours_p))
        tag = ("A:" + ("ok" if tour["ok"] else "F") +
               " B:" + ("ok" if ours["ok"] else "F") +
               " Bp:" + ("ok" if ours_p["ok"] else "F"))
        print(f"[{m['idx']:02d}] {tag}  A_len={tour.get('length_m')} "
              f"B_len={ours.get('length_m')} B_s={ours.get('seconds')}",
              flush=True)

    ok_a = sum(1 for r in rows if r["tour"]["ok"])
    ok_b = sum(1 for r in rows if r["ours"]["ok"])
    ok_bp = sum(1 for r in rows if r["ours_p"]["ok"])
    both = [r for r in rows if r["tour"]["ok"] and r["ours"]["ok"]]
    deltas = [r["tour"]["length_m"] - r["ours"]["length_m"] for r in both
              if r["tour"].get("length_m") and r["ours"].get("length_m")]

    def _reasons(side):
        from collections import Counter
        c = Counter()
        for r in rows:
            if not r[side]["ok"]:
                key = r[side]["reason"]
                for pat in ("穿洞", "射击停站", "无解", "unreachable", "不可达",
                            "timeout", "route_check"):
                    if pat in key:
                        key = pat
                        break
                c[key[:60]] += 1
        return c

    summary = {
        "n": len(rows),
        "blue_face_none_unshootable_by_our_model": sum(
            1 for m in maps if m.get("blue_face") is None),
        "tour_ok": ok_a, "ours_ok_strict": ok_b, "ours_ok_permissive": ok_bp,
        "policy_gap_strict_vs_permissive": ok_bp - ok_b,
        "both_ok": len(both),
        "ours_only": sum(1 for r in rows if r["ours"]["ok"] and not r["tour"]["ok"]),
        "tour_only": sum(1 for r in rows if r["tour"]["ok"] and not r["ours"]["ok"]),
        "tour_only_vs_permissive": sum(
            1 for r in rows if r["tour"]["ok"] and not r["ours_p"]["ok"]),
        "permissive_only_vs_tour": sum(
            1 for r in rows if r["ours_p"]["ok"] and not r["tour"]["ok"]),
        "len_delta_tour_minus_ours": {
            "mean": round(sum(deltas) / len(deltas), 2) if deltas else None,
            "n": len(deltas),
            "ours_shorter": sum(1 for d in deltas if d > 0),
            "tour_shorter": sum(1 for d in deltas if d < 0)},
        "tour_fail_reasons": dict(_reasons("tour")),
        "ours_fail_reasons": dict(_reasons("ours")),
        "ours_seconds_both_solved": {
            "mean": round(sum(r["ours"]["seconds"] for r in both
                             if r["ours"].get("seconds")) /
                          max(1, sum(1 for r in both if r["ours"].get("seconds"))), 1)},
    }
    with open(os.path.join(OUT_DIR, "results.json"), "w", encoding="utf-8") as f:
        json.dump(dict(summary=summary, rows=rows), f, ensure_ascii=False,
                  indent=1, default=str)
    with open(os.path.join(OUT_DIR, "summary.txt"), "w", encoding="utf-8") as f:
        f.write(json.dumps(summary, ensure_ascii=False, indent=2))
    print("\n==== SUMMARY ====")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
