# -*- coding: utf-8 -*-
"""t40 官方图本地仿真（核心臂 A1/A2/B6）+ 与车端自规划（plan_route_from_config）对账。

口径（队长 2026-10-14 识读定稿，取代现役第五张图 config/map.yaml）：
  雷 C1/A3/E3/G4/I6/B7/D9/H10；障 A1/E2/H4/D8/I8/G9/B11/F11；
  靶 E5+F5（跨格板）；桥 K5(上)/K9(下)；起 N10 终 N5；无人机 N8→M2。

边界：
  - 只读 dzrobot_v2/config/robot.yaml 与 src/dzbrain（import）；官方图走内存 Config 覆盖，
    绝不写 dzrobot_v2/config（不改 config、不改车侧代码）。
  - 只写本目录（map_official.yaml）与 --out-dir（_tmp_session/sim_ag/t40_official_<ts>/）。
  - 固定种子 = sim_airground 内置 20261007（run_sim + 链路）；任务书"20261012"系 --seed-base
    默认值口径，非臂级 seed——此处如实沿用 20261007，复跑 sha256 逐字节一致即验收。

三臂设计（官方图）：
  A1 无无人机基线 0.35：cfg=官方(K9/K5)，truth=官方，无无人机 → 完赛（验证官方路线）。
  A2 理想揭示：同上 + 理想无人机 → 完赛（信息领先契约）。
  B6 受限旗舰：cfg=现役**陈旧**(K10/K5，t39 尚未改车侧配置)，truth=官方(K9/K5)，
     无人机迟到 8s + 蓝红翻转误判 180° → 权威门+L10 桥前等待采纳 K9 → 完赛（红面扣分）。
     （等价原 B6 的"随机桥洞"语义：下半场真值 K9 ≠ 陈旧兜底 K10，靠无人机纠正。）

待确认（如实标注，仿真无法裁决）：
  - 靶蓝面朝向 φ 官方图未标：本仿真按 φ=0°（蓝面朝北）假设；φ 只影响射击账（蓝/红）与
    驻留时长，不影响路线长度/穿洞格/盖雷/越界。
"""
from __future__ import annotations

import json
import os
import sys
import time

_WS = r"D:\User\Desktop\比赛存档\v1"
_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(_WS, "dzrobot_v2", "src", "dzbrain"),
           os.path.join(_WS, "dzsmartscout", "tools")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from dzbrain.core.config import Config
from dzbrain.tools.sim import parse_map_yaml
from dzbrain.tasks.mission import plan_route_from_config

from sim_airground.runner import run_arm
from sim_airground.scenarios import (Scenario, TargetParams, _base_mission,
                                     _spec_with_bridges, derive_stop_candidates)
from sim_airground.geo import block_to_meters

MAP_OFFICIAL = os.path.join(_HERE, "map_official.yaml")


def official_cfg() -> Config:
    """内存构造官方图口径 Config：穿洞候选/兜底 = K9(下)/K5(上)，不落盘。"""
    cfg = Config.load(os.path.join(_WS, "dzrobot_v2", "config", "robot.yaml"))
    flat = dict(cfg._flat)
    flat["route.cross_lower_candidates"] = ["K9"]
    flat["route.cross_upper_candidates"] = ["K5"]
    flat["route.cross_lower_default"] = "K9"
    flat["route.cross_upper_default"] = "K5"
    return Config(flat, source=f"{cfg.source} + t40-official-bridges(K9/K5)")


def stale_cfg() -> Config:
    """现役（陈旧）Config：穿洞候选/兜底 = K10(下)/K5(上)，即当前 robot.yaml 未改口径。"""
    return Config.load(os.path.join(_WS, "dzrobot_v2", "config", "robot.yaml"))


def official_spec():
    with open(MAP_OFFICIAL, encoding="utf-8") as fh:
        return parse_map_yaml(fh.read(), source="map_official.yaml")


def build_scenario(name, cfg, spec, *, judge_error=0.0, authority=False, wait=False,
                   takeoff=0.0, belief_bridges=None, truth_bridges=None):
    track = tuple(spec.targets)
    truth = spec if truth_bridges is None else _spec_with_bridges(spec, truth_bridges)
    sc = Scenario(
        name=name,
        truth=truth,
        belief_bridges=tuple(spec.bridges) if belief_bridges is None else tuple(belief_bridges),
        candidates={"lower": tuple(cfg.get("route.cross_lower_candidates") or ()),
                    "upper": tuple(cfg.get("route.cross_upper_candidates") or ())},
        defaults={"lower": str(cfg.get("route.cross_lower_default")),
                  "upper": str(cfg.get("route.cross_upper_default"))},
        arrows={"K9": "W", "K5": "E"},
        target=TargetParams(track_cells=track, orientation_deg=0.0,
                            judge_error_deg=judge_error,
                            stop_candidates=derive_stop_candidates(track)),
        mission=_base_mission(cfg),
        authority_gate=authority,
        wait_at_bridge=wait,
    )
    sc.drone.takeoff_delay_s = takeoff
    return sc


# ---------- 路线几何（对账用，sim 与车端同一把尺子） ----------
def _base_cell(tok):
    t = str(tok).strip().upper()
    if t.startswith("R:"):
        t = t[2:]
    t = t.replace("*", "")
    if t.endswith(",Y"):
        t = t[:-2]
    return t.strip(",")


def route_cells(tokens):
    return [b for t in tokens if (b := _base_cell(t))]


def polyline_m(tokens):
    cells = route_cells(tokens)
    total = 0.0
    for a, b in zip(cells, cells[1:]):
        (xa, ya) = block_to_meters(a)
        (xb, yb) = block_to_meters(b)
        total += ((xb - xa) ** 2 + (yb - ya) ** 2) ** 0.5
    return round(total, 3)


def stop_cell(tokens):
    for t in tokens:
        if "*" in str(t).upper():
            return _base_cell(t)
    return None


def cross_cells(tokens):
    lower, upper = set(), set()
    for t in tokens:
        b = _base_cell(t)
        if b.startswith("K") and len(b) >= 2 and b[1:].isdigit():
            row = int(b[1:])
            (lower if 7 <= row <= 11 else upper).add(b)
    return sorted(lower), sorted(upper)


def main():
    out_root = os.path.join(_WS, "_tmp_session", "sim_ag",
                            "t40_official_" + time.strftime("%Y%m%d_%H%M%S"))
    cfg = official_cfg()
    spec = official_spec()
    baseline = [str(t) for t in (cfg.get("route.tokens") or [])]

    arms = [
        ("A1", build_scenario("官方-无无人机基线0.35", cfg, spec),
         dict(drone_on=False, stop_on_violation=False), "无无人机基线（0.35，官方图）"),
        ("A2", build_scenario("官方-理想揭示", cfg, spec),
         dict(drone_on=True, stop_on_violation=False), "理想揭示（官方图）"),
        ("B6", build_scenario("官方-受限旗舰", cfg, spec, judge_error=180.0,
                              authority=True, wait=True, takeoff=8.0,
                              truth_bridges=("K8", "K5")),
         dict(drone_on=True, stop_on_violation=True),
         "受限旗舰（真值下半场随机K8≠官方K9 + 迟到8s + 误判180° → 权威门纠正）"),
    ]

    results = []
    for arm_id, sc, kw, desc in arms:
        out_dir = os.path.join(out_root, arm_id.lower())
        res = run_arm(arm_id, sc, cfg, baseline, out_dir=out_dir, description=desc, **kw)
        results.append(res)
        c = res.car
        s = res.shoot
        print(f"{arm_id} 跑完={c['completed']} 时长={c['time_s']:.2f}s RMS={c['cross_track_rms_m']:.4f} "
              f"越界={c['out_of_bounds_steps']} 盖雷={res.field_metrics.get('mine_pressed_count')}/"
              f"{res.field_metrics.get('mine_total')} 障碰={res.field_metrics.get('obstacle_hit_steps')} "
              f"预算={res.budget['total_s']:.2f}s/{res.budget['budget_s']:.0f}s 改道={res.reroute_count} "
              f"蓝={s['blue_hits']} 红={s['red_hits']} 停站={s['chosen_stop']} 等待={res.wait_lower_s:.2f}s")

    # ---------- 车端自规划（plan_route_from_config，官方图 + K9/K5 口径） ----------
    plan = plan_route_from_config(cfg, map_path=MAP_OFFICIAL)
    car_tokens = [str(t) for t in (plan.tokens or [])]
    a1_tokens = list(results[0].executed_tokens)

    reco = {
        "car_plan_ok": bool(getattr(plan, "ok", False)),
        "car_plan_error": str(getattr(plan, "error", "")),
        "car_plan_map": str(getattr(plan, "map_path", "")),
        "sim_a1": {
            "n": len(a1_tokens), "len_m": polyline_m(a1_tokens),
            "stop": stop_cell(a1_tokens),
            "cross": {"lower": cross_cells(a1_tokens)[0], "upper": cross_cells(a1_tokens)[1]},
        },
        "car_plan": {
            "n": len(car_tokens), "len_m": polyline_m(car_tokens),
            "stop": stop_cell(car_tokens),
            "cross": {"lower": cross_cells(car_tokens)[0], "upper": cross_cells(car_tokens)[1]},
        },
        "tokens_identical": a1_tokens == car_tokens,
        "len_m_diff": round(polyline_m(a1_tokens) - polyline_m(car_tokens), 3),
        "sim_a1_tokens": a1_tokens,
        "car_plan_tokens": car_tokens,
    }

    print("\n=== 与车端自规划对账（plan_route_from_config，官方图） ===")
    print(f"  车端 plan ok={reco['car_plan_ok']} err={reco['car_plan_error']!r}")
    print(f"  simA1  n={reco['sim_a1']['n']} len={reco['sim_a1']['len_m']}m 停站={reco['sim_a1']['stop']} "
          f"穿洞下={reco['sim_a1']['cross']['lower']} 上={reco['sim_a1']['cross']['upper']}")
    print(f"  车端    n={reco['car_plan']['n']} len={reco['car_plan']['len_m']}m 停站={reco['car_plan']['stop']} "
          f"穿洞下={reco['car_plan']['cross']['lower']} 上={reco['car_plan']['cross']['upper']}")
    print(f"  tokens 逐位相同 = {reco['tokens_identical']}；长度差 = {reco['len_m_diff']}m")

    with open(os.path.join(out_root, "ledger.json"), "w", encoding="utf-8") as fh:
        json.dump({"arms": [r.__dict__ for r in results], "reconciliation": reco},
                  fh, ensure_ascii=False, indent=2, default=str)
    with open(os.path.join(out_root, "reconciliation.json"), "w", encoding="utf-8") as fh:
        json.dump(reco, fh, ensure_ascii=False, indent=2, default=str)
    print(f"\nout_root={out_root}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
