# -*- coding: utf-8 -*-
"""空地联合仿真编排：一条臂 = 无人机时间线 + 链路 + 车侧采纳门/任务流 + run_sim + 射击账 + 预算账。

台账纪律（交接包）：报数字不报形容词；每条结论附命令原文与 exit code；物理在环两口径
（盖雷几何 vs 车体足迹）分开写；sim 不建模 PIVOT / 射击驻留的边界如实标注。
"""

from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass, field

from dzbrain.core.config import Config

from .car_gate import MissionFlow, RouteVersion, approach_cell
from .carsim import gap_metrics, lead_table, run_car
from .drone_model import DroneTimeline, build_timeline, emit_messages, bridge_half
from .geo import meters_to_block
from .link import Link
from .plot import draw_map
from .scenarios import Scenario
from .shoot import evaluate_shooting


@dataclass
class ArmResult:
    arm_id: str
    world: str
    description: str = ""
    executed_tokens: list[str] = field(default_factory=list)
    versions: list[dict] = field(default_factory=list)
    car: dict = field(default_factory=dict)
    field_metrics: dict = field(default_factory=dict)
    commit_t: float = 0.0
    intel_complete_t: float | None = None
    wait_lower_s: float = 0.0
    wait_upper_s: float = 0.0
    reroute_count: int = 0
    adopted_changes: int = 0
    gate_events: list[dict] = field(default_factory=list)
    gap: dict = field(default_factory=dict)
    lead_rows: list[dict] = field(default_factory=list)
    shoot: dict = field(default_factory=dict)
    budget: dict = field(default_factory=dict)
    link: dict = field(default_factory=dict)
    drone: dict = field(default_factory=dict)
    artifacts: dict = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)


def _planned_stop(tokens: list[str]) -> str | None:
    for tok in tokens:
        text = str(tok).strip().upper()
        base = text[2:] if text.startswith("R:") else text
        if "*" in base:
            b = base.strip("*").rstrip(",y").strip(",")
            if b:
                return b
    return None


def _route_cells(tokens: list[str]) -> set[str]:
    out = set()
    for tok in tokens:
        text = str(tok).strip().upper()
        base = text[2:] if text.startswith("R:") else text
        base = base.strip("*").rstrip(",y").strip(",")
        if base:
            out.add(base)
    return out


def run_arm(arm_id: str, scenario: Scenario, cfg: Config, baseline_tokens: list[str],
            *, out_dir: str, drone_on: bool, stop_on_violation: bool,
            description: str = "") -> ArmResult:
    """跑一条对照臂（确定性；同参 → 同 traj.csv sha256）。"""
    os.makedirs(out_dir, exist_ok=True)
    arm = ArmResult(arm_id=arm_id, world=scenario.name, description=description)

    # ================= 无人机侧 =================
    timeline: DroneTimeline | None = None
    delivered = []
    link_stats = {"sent": 0, "delivered": 0, "dropped": 0}
    if drone_on:
        timeline = build_timeline(scenario)
        horizon = scenario.mission.budget_s + 60.0
        msgs = emit_messages(scenario, timeline, horizon)
        link = Link(scenario.link)
        delivered, link_stats = link.apply(msgs, scenario.drone.takeoff_delay_s)
        # ⑤ H1 判据：箭头垫片所在半场 → 该半场通行格；与 bridge_passable 交叉验证
        arrow_h1: dict[str, str] = {}
        for a in timeline.arrow_reads:
            half = bridge_half(a.cell, scenario)
            if half:
                arrow_h1[half] = a.cell
        truth_by_half = {}
        for c in scenario.truth.bridges:
            h = bridge_half(c, scenario)
            if h:
                truth_by_half[h] = c
        arm.drone = {
            "flight_len_m": round(timeline.total_len_m, 3),
            "flight_end_t": round(timeline.flight_end_t, 3),
            "recon_done_s": round(timeline.flight_end_t + scenario.drone.takeoff_delay_s, 3),
            "speed_mps": scenario.drone.speed_mps,
            "takeoff_delay_s": scenario.drone.takeoff_delay_s,
            "radius_cells": scenario.drone.reveal_radius_cells,
            "reveal_entrance": {c: timeline.reveal_time.get(c) for c in scenario.truth.bridges},
            "arrows": [{"cell": a.cell, "direction": a.direction, "t": a.t,
                        "confidence": a.confidence} for a in timeline.arrow_reads],
            "arrow_h1": arrow_h1,
            "arrow_h1_matches_bridge_passable": {
                h: (arrow_h1.get(h) == truth_by_half.get(h)) for h in truth_by_half},
        }
    arm.link = link_stats

    # ================= 车侧任务流（采纳门 + 发车窗口 + 路线装载） =================
    flow = MissionFlow(scenario, cfg, list(baseline_tokens))
    commit_t, snap = flow.commit_decision(delivered)
    v1 = flow.load_route(snap)
    flow.result.versions.append(v1)
    arm.commit_t = round(commit_t, 3)
    arm.intel_complete_t = flow.result.intel_complete_t

    # ================= 桥前等待（与现车 mission.py 桥前等待段同型） =================
    executed_tokens = list(v1.tokens)
    wait_lower = 0.0
    v1_observe = None
    if scenario.wait_at_bridge:
        # 先按 v1 观察轨迹拿穿洞入口格（approach = 穿洞三连前格，现役线 L7）到达时刻
        # （stop=False 不改变轨迹，t24 硬不变量）。入口格一律从路线现算（approach_cell），
        # 旧版硬编码 L10 在换线后静默失效（2026-10-24 审计）。
        v1_observe = run_car(f"{arm_id}.v1.observe", list(v1.tokens), cfg, scenario.truth,
                             stop_on_violation=False,
                             out_dir=os.path.join(out_dir, "v1_observe"))
        approach = approach_cell(list(v1.tokens), "lower")
        arrival_approach = v1_observe.arrival_sim.get(approach) if approach else None
        if arrival_approach is None:
            print(f"⚠️ [{arm_id}] 桥前等待：执行路线里找不到下半场穿洞三连（无 approach 格）"
                  f"→ 跳过等待重规划（检查 baseline 是否含 K 列穿洞）", file=sys.stderr)
        else:
            joint_approach = commit_t + arrival_approach
            v2 = flow.bridge_wait_decision(joint_approach, delivered)
            if v2 is not None:
                flow.result.versions.append(v2)
                executed_tokens = list(v2.tokens)
        wait_lower = flow.result.wait_lower_s
    arm.wait_lower_s = round(wait_lower, 3)
    arm.reroute_count = sum(1 for i in range(1, len(flow.result.versions))
                            if flow.result.versions[i].tokens
                            != flow.result.versions[i - 1].tokens)
    arm.adopted_changes = flow.gate.adopted_changes
    arm.gate_events = [{"t": round(e.t, 3), "half": e.half, "source": e.source,
                        "cell": e.cell, "default": e.default, "reason": e.reason}
                       for e in flow.gate.events]
    arm.versions = [{"kind": v.kind, "n": len(v.tokens), "crosses": v.crosses, "note": v.note,
                     "tokens": " ".join(v.tokens)}
                    for v in flow.result.versions]

    # ================= 车侧执行（run_sim，物理真值场） =================
    car = run_car(arm_id, executed_tokens, cfg, scenario.truth,
                  stop_on_violation=stop_on_violation, out_dir=out_dir)
    arm.executed_tokens = executed_tokens
    m = car.metrics
    arm.car = {k: m[k] for k in ("completed", "time_s", "cross_track_rms_m", "cross_track_max_m",
                                 "overshoot_max_m", "final_error_m", "out_of_bounds_steps",
                                 "anchors", "path_len_ratio", "field_stop_t", "field_stopped",
                                 "steps")}
    arm.field_metrics = {k: m[k] for k in (
        "mine_total", "mine_pressed_count", "mine_press_steps", "mine_worst_clearance_m",
        "obstacle_hit_steps", "obstacle_breach_steps", "barrier_min_distance_m",
        "barrier_breach_steps", "board_overlap_steps", "board_breach_steps",
        "k_column_steps", "k_seal_violation_steps", "k_solid_cell_steps") if k in m}

    # ================= 射击账（④ 双面移动靶，② 命中时扇区口径） =================
    stop_cell = _planned_stop(executed_tokens) or "C10"
    on_route = _route_cells(executed_tokens)
    # 两趟：先取停站点 → 用轨迹到达时刻做射击锚点（面板在动，命中有效性逐发按时刻算）
    probe = evaluate_shooting(scenario.target, stop_cell, on_route,
                              judge_available=drone_on, anchor_t=0.0)
    anchor = commit_t + car.arrival_sim.get(probe.stop,
                                            car.arrival_sim.get(stop_cell, 0.0)) + wait_lower
    outcome = evaluate_shooting(scenario.target, stop_cell, on_route,
                                judge_available=drone_on, anchor_t=anchor)
    arm.shoot = {
        "planned_stop": stop_cell, "chosen_stop": outcome.stop,
        "reached": bool(m["completed"]),
        "sector_fail": outcome.sector_fail, "blue_hits": outcome.blue_hits,
        "red_hits": outcome.red_hits, "misses": outcome.misses,
        "dwell_s": round(outcome.dwell_s, 3), "penalties": outcome.penalties,
        "score": outcome.score,
        "true_phi_deg": outcome.true_phi, "judged_phi_deg": outcome.judged_phi,
        "chosen_theta_deg": round(outcome.chosen_theta, 2),
        "sector_deg": scenario.target.sector_deg,
        "panel_speed_mps": scenario.target.speed_mps,
        "anchor_t": round(anchor, 2),
    }

    # ================= 信息领先契约（揭示 vs 到达） =================
    reveal_joint: dict[str, float] = {}
    recon_deadlines: dict[str, dict] = {}
    if timeline is not None:
        d = scenario.drone.takeoff_delay_s + scenario.link.latency_s
        for cell, t in timeline.reveal_time.items():
            if cell in scenario.truth.bridges:
                continue                        # 桥洞用"第一条送达报文的接收时刻"（含丢包）
            reveal_joint[cell] = t + d
        for msg in delivered:
            if msg.payload.get("type") != "bridge_passable":
                continue
            half = str(msg.payload.get("half") or "").lower()
            block = str(msg.payload.get("block") or "").upper()
            if block and block not in reveal_joint:
                reveal_joint[block] = msg.recv_t
        # ③E 侦察截止：车到穿洞入口格（approach = 穿洞三连前格，现役线 L7/J5）时刻
        #    vs 该半场桥洞情报送达时刻（入口格从路线现算，找不到 → deadline=None 如实报）
        for half in ("lower", "upper"):
            approach = approach_cell(executed_tokens, half)
            arr = car.arrival_sim.get(approach) if approach else None
            deadline = (commit_t + arr) if arr is not None else None
            cells = [c for c in scenario.truth.bridges if bridge_half(c, scenario) == half]
            got = min((reveal_joint.get(c, 9e9) for c in cells), default=9e9)
            recon_deadlines[half] = {
                "approach": approach, "deadline_s": round(deadline, 2) if deadline else None,
                "intel_recv_s": round(got, 2) if got < 9e9 else None,
                "ok": bool(deadline is not None and got <= deadline),
                "margin_s": round(deadline - got, 2) if (deadline is not None and got < 9e9) else None,
            }
        arm.drone["recon_deadlines"] = recon_deadlines
    key_cells = sorted(set(scenario.truth.mines) | set(scenario.truth.obstacles)
                       | set(scenario.truth.targets) | set(scenario.truth.bridges))
    arm.gap = gap_metrics(car, commit_t, reveal_joint, key_cells)
    arm.lead_rows = lead_table(car, commit_t, reveal_joint, key_cells)

    # ================= 预算账（② 3 分钟） =================
    dwell = outcome.dwell_s
    if not m["completed"]:
        total = commit_t + m["time_s"]
        within = total <= scenario.mission.budget_s
        arm.budget = {"drive_s": m["time_s"], "wait_lower_s": wait_lower, "dwell_s": 0.0,
                      "commit_s": commit_t, "total_s": round(total, 3),
                      "budget_s": scenario.mission.budget_s, "within": within,
                      "margin_s": round(scenario.mission.budget_s - total, 3),
                      "note": "未跑完（撞封死桥室/硬违规中断），驻留不计"}
    else:
        total = commit_t + m["time_s"] + wait_lower + dwell
        arm.budget = {"drive_s": m["time_s"], "wait_lower_s": wait_lower, "dwell_s": dwell,
                      "commit_s": commit_t, "total_s": round(total, 3),
                      "budget_s": scenario.mission.budget_s,
                      "within": total <= scenario.mission.budget_s,
                      "margin_s": round(scenario.mission.budget_s - total, 3),
                      "note": "commit+drive+wait+dwell"}

    # ================= 出图（t28：一张图 = 车路线+真值轨迹+60 个真值位姿 + 无人机航线） =================
    _pose_stride = max(1, (len(car.truth_pose) + 59) // 60)   # 位姿点数定死 ≤60（见交付说明②）
    png = draw_map(os.path.join(out_dir, "route_map.png"), title=f"{arm_id} — {scenario.name}",
                   truth=scenario.truth, car_tokens=executed_tokens,
                   car_cells=car.cell_series,
                   car_traj=[(p[0], p[1]) for p in car.truth_pose],
                   car_poses=car.truth_pose, pose_every=_pose_stride,
                   drone=timeline,
                   car_abort_t=(m["field_stop_t"] if not m["completed"] else None))
    arm.artifacts = {"traj_csv": car.traj_csv, "traj_sha256": car.sha256,
                     "route_map": png}
    if v1_observe is not None:
        arm.artifacts["v1_observe_sha256"] = v1_observe.sha256

    # ================= 台账落盘 =================
    with open(os.path.join(out_dir, "ledger.json"), "w", encoding="utf-8") as fh:
        json.dump(_arm_to_dict(arm), fh, ensure_ascii=False, indent=2, default=str)
    return arm


def _arm_to_dict(arm: ArmResult) -> dict:
    out = {k: v for k, v in arm.__dict__.items()}
    return out


# --------------------------------------------------------------------------- #
#  分析输出（不跑车）：失败概率 / 半径敏感度 / 速度敏感度 / 扇区敏感度
# --------------------------------------------------------------------------- #
def blind_failure_analysis(baseline_defaults: dict[str, str]) -> dict:
    """③(a) 无情报直冲的完赛失败概率（解析枚举 30 组合 + 结果表）。"""
    from .scenarios import ALL_BRIDGE_COMBOS
    lo_d, up_d = baseline_defaults["lower"], baseline_defaults["upper"]
    fails, oks = [], []
    for entrance, exit_cell in ALL_BRIDGE_COMBOS:
        if entrance == lo_d and exit_cell == up_d:
            oks.append((entrance, exit_cell))
        else:
            fails.append((entrance, exit_cell))
    return {
        "combos_total": len(ALL_BRIDGE_COMBOS),
        "combos_pass": len(oks), "combos_fail": len(fails),
        "fail_probability": round(len(fails) / len(ALL_BRIDGE_COMBOS), 4),
        "pass_probability": round(len(oks) / len(ALL_BRIDGE_COMBOS), 4),
        "note": "入口 K7~K11（5 格）× 出口 K1~K6（6 格）= 30 组合；盲冲默认 K10/K5 仅 1/30 命中"
                "（队长口径 1/25 对应两半场各 5 格的近似；本仿真按 5×6=30 精算）",
    }


def radius_sweep(scenario: Scenario, radii=(1.5, 1.75, 2.0, 2.25, 2.5, 3.0)) -> list[dict]:
    """③D 感知半径敏感度（无人机侧，不跑车）：关键格覆盖与揭示时刻。"""
    key_cells = sorted(set(scenario.truth.mines) | set(scenario.truth.obstacles)
                       | set(scenario.truth.targets) | set(scenario.truth.bridges))
    rows = []
    for r in radii:
        sc = _clone(scenario)
        sc.drone.reveal_radius_cells = r
        tl = build_timeline(sc)
        uncovered = [c for c in key_cells if c not in tl.reveal_time]
        rows.append({"radius_cells": r, "radius_m": round(r * 0.35, 3),
                     "key_cells": len(key_cells), "covered": len(key_cells) - len(uncovered),
                     "uncovered": uncovered,
                     "max_reveal_t": round(max(tl.reveal_time.values()), 3)
                     if tl.reveal_time else None})
    return rows


def speed_sweep(scenario: Scenario, speeds=(0.3, 0.5, 0.8, 1.5)) -> list[dict]:
    """③E 无人机速度档位：桥洞与全要素揭示时刻 vs 侦察截止时间。"""
    rows = []
    for v in speeds:
        sc = _clone(scenario)
        sc.drone.speed_mps = v
        tl = build_timeline(sc)
        rows.append({"speed_mps": v, "flight_end_t": round(tl.flight_end_t, 3),
                     "bridge_reveals": {c: tl.reveal_time.get(c) for c in sc.truth.bridges},
                     "all_key_revealed_by": round(max(tl.reveal_time.values()), 3)
                     if tl.reveal_time else None})
    return rows


def sector_sweep(scenario: Scenario, sectors=(30.0, 45.0, 60.0),
                 phis=(0.0, 90.0, -90.0, 180.0)) -> list[dict]:
    """④ 扇区敏感度：4 种蓝面朝向 × 3 档扇区 × 停站点选择与命中侧。"""
    rows = []
    for phi in phis:
        for sector in sectors:
            t = _clone_target(scenario.target)
            t.orientation_deg = phi
            t.judge_error_deg = 0.0
            t.sector_deg = sector
            from .shoot import pick_stop
            chosen, _err = pick_stop(t, phi)
            row = {"phi_deg": phi, "sector_deg": sector, "chosen_stop": chosen}
            if chosen is None:
                row["side"] = "无解(扇区内无停站点)"
                row["dwell_s"] = 30.0
            else:
                out = evaluate_shooting(t, "C10", {"A10", "B10", "C10", "D10"})
                row["side"] = ("蓝面(命中有效)" if out.blue_hits else
                               "红面(扣分)" if out.red_hits else "掠射(不命中)")
                row["dwell_s"] = round(out.dwell_s, 2)
            rows.append(row)
    return rows


def _clone(scenario: Scenario) -> Scenario:
    import copy
    return copy.deepcopy(scenario)


def _clone_target(t):
    import copy
    return copy.deepcopy(t)
