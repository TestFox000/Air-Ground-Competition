# -*- coding: utf-8 -*-
"""① 多场次随机抽取统计：桥洞 30 组合（入口 K7~K11 × 出口 K1~K6）× 靶面朝向 4 种。

## 方法（诚实标注：解析统计 + 少量全仿真验证）

- 无人机揭示时刻**只依赖航线与半径，与抽到哪个桥洞无关** → 预计算一次 K1~K11 揭示表。
- 车侧任务流（发车窗口 / 采纳门 / 桥前等待 / 重规划）是**纯确定性计算** → 逐场复用
  ``MissionFlow`` 的同一套逻辑（含 wait 上限与部分情报语义）。
- 每场的执行路线由真规划器 ``plan_route_tracker_safe`` 现算（30 场 × ~0.4 s）；
  行驶时长估计 = 基线实测 6.883 s/m × 规划路线长（t31 基线 20.65 m = 142.14 s 实测标定），
  抽样 3 场全仿真验证该标定（偏差进报告）。
- 靶面朝向 φ ∈ {0°, 90°, −90°, 180°} 每场随机；射击逐发按"命中时车→靶连线扇区"判
  （② 口径），锚点 = 车到停站点时刻（基线轨迹实测，跨场取同一值并标注 ±2 s 相位不确定）。
- 新指标 ④：``drone_recon_s``（无人机侦察耗时）与 ``budget_margin_s``（总时间预算余量）
  逐场输出并聚合。
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import yaml as _yaml

from dzbrain.core.config import Config
from dzbrain.route.planner import plan_route_tracker_safe
from dzbrain.tools.sim import FieldSpec

from .car_gate import MissionFlow
from .drone_model import build_timeline
from .link import Link
from .scenarios import ALL_BRIDGE_COMBOS, build_world_b
from .shoot import evaluate_shooting

ORIENTATIONS = (0.0, 90.0, -90.0, 180.0)


@dataclass
class MatchRow:
    entrance: str
    exit_cell: str
    phi: float
    adopted_lower: str
    adopted_upper: str
    wait_s: float
    complete: bool
    reason: str
    drive_est_s: float
    dwell_s: float
    budget_s: float | None          # None = 未跑完（判负原因 = 撞封死桥室，不是超时）
    within: bool | None
    margin_s: float | None
    blue: int
    red: int
    score: int
    stop: str


@dataclass
class PolicyStats:
    policy: str
    n: int = 0
    completion_rate: float = 0.0
    budget_pass_rate: float = 0.0
    blue_rate: float = 0.0
    red_rate: float = 0.0
    mean_score: float = 0.0
    mean_margin_s: float = 0.0
    worst_margin_s: float = 0.0
    worst_case: str = ""
    max_wait_s: float = 0.0
    mean_wait_s: float = 0.0
    rows: list[MatchRow] = field(default_factory=list)


def _cells(tokens) -> set[str]:
    out = set()
    for t in tokens:
        text = str(t).strip().upper()
        base = text[2:] if text.startswith("R:") else text
        base = base.strip("*").rstrip(",y").strip(",")
        if base:
            out.add(base)
    return out


def _adopt_for_combo(scenario, cfg, baseline, delivered, arrival_l10_joint, cap_s,
                     authority: bool, wait_enabled: bool) -> tuple[str, str, float, str]:
    """复用任务流逻辑算一场的采纳结果：(adopted_lower, adopted_upper, wait, 原因)。"""
    scenario.authority_gate = authority
    scenario.wait_at_bridge = wait_enabled
    scenario.mission.wait_cap_s = cap_s
    flow = MissionFlow(scenario, cfg, list(baseline))
    commit, snap = flow.commit_decision(delivered)
    v1 = flow.load_route(snap)
    flow.result.versions.append(v1)
    tokens = list(v1.tokens)
    wait = 0.0
    if wait_enabled and "L10" in _cells(v1.tokens):
        v2 = flow.bridge_wait_decision(arrival_l10_joint, delivered)
        if v2 is not None:
            flow.result.versions.append(v2)
            tokens = list(v2.tokens)
        wait = flow.result.wait_lower_s
    crosses = flow.result.versions[-1].crosses
    return crosses.get("lower", scenario.defaults["lower"]), \
        crosses.get("upper", scenario.defaults["upper"]), wait, \
        flow.result.versions[-1].note


def plan_for_bridges(cfg: Config, map_cfg: dict, plan_lo: str, plan_up: str,
                     *, targets: tuple[str, ...] | list[str] | None = None,
                     max_seconds: float = 30.0):
    """规划一条执行路线（权威模式内存扩池，不落盘）。返回 (tokens, polyline_m, shoot_stop)。

    t28 ③：``targets`` 覆盖规划地图里的靶格（只改内存 dict，不落 config/map.yaml）。
    """
    flat = dict(cfg._flat)
    for half, cell in (("lower", plan_lo), ("upper", plan_up)):
        flat[f"route.cross_{half}_candidates"] = sorted(
            set(flat.get(f"route.cross_{half}_candidates") or []) | {cell})
        if cell not in map_cfg["map"].get("bridges", []):
            map_cfg["map"]["bridges"] = list(map_cfg["map"].get("bridges", [])) + [cell]
    if targets:
        map_cfg["map"]["targets"] = sorted(set(str(t).upper() for t in targets))
    cfg2 = Config(flat, source="match_stats")
    res = plan_route_tracker_safe(map_cfg, cfg2, cross_lower=plan_lo, cross_upper=plan_up,
                                  max_seconds=max_seconds)
    return list(res.tokens), float(res.polyline_m), str(res.shoot_stop or "C10")


#: 向后兼容别名（run_match_stats 内部旧名）
_plan = plan_for_bridges


def run_match_stats(cfg: Config, baseline: list[str], arrival_ref: dict[str, float],
                    drive_s_per_m: float, commit_t: float, arrival_l10_sim: float,
                    delay_s: float = 8.0, cap_s: float = 12.0) -> dict:
    """30 组合 × 4 朝向 × 3 策略 的逐场统计 + 聚合。

    :param arrival_ref: 基线轨迹逐格到达时刻（sim 时间）——停站锚点与 L10 用。
    :param drive_s_per_m: 行驶时长标定（基线 142.14 s / 20.65 m）。
    """
    base_sc = build_world_b("K9", "K4")       # 只借它的航线/链路/预算默认参数
    budget_s = base_sc.mission.budget_s
    # ★ 关键：时间线的"关键格"必须包含全部 K1~K11，否则换组合后揭示时刻查不到
    base_sc.truth = FieldSpec(
        mines=base_sc.truth.mines, obstacles=base_sc.truth.obstacles,
        targets=base_sc.truth.targets,
        bridges=tuple(f"K{r}" for r in range(1, 12)),
        barrier_col_from=base_sc.truth.barrier_col_from,
        barrier_row_hi=base_sc.truth.barrier_row_hi,
        barrier_row_lo=base_sc.truth.barrier_row_lo,
        name="K1~K11 全列（揭示时刻真源）", source="内存构造（不落盘）")
    tl = build_timeline(base_sc)
    link = Link(base_sc.link)
    repo = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "..", "..", "dzrobot_v2"))
    horizon = budget_s + 60.0

    policies: dict[str, PolicyStats] = {p: PolicyStats(policy=p)
                                        for p in ("no_drone", "ideal_wait", "constrained")}
    arrival_l10_joint = commit_t + arrival_l10_sim
    for entrance, exit_cell in ALL_BRIDGE_COMBOS:
        # 该组合的报文时间线：桥洞首报 = 该格揭示时刻；之后每 reemit_s 重发续期；心跳 1 Hz
        msgs: list[tuple[float, dict]] = []
        for half, cell in (("lower", entrance), ("upper", exit_cell)):
            t0 = tl.reveal_time.get(cell)
            if t0 is None:
                continue
            t = t0
            first = True
            while t <= horizon:
                msgs.append((t, {"type": "bridge_passable", "half": half, "block": cell,
                                 "confidence": 1.0, "ts": round(t, 3)}))
                if first:
                    t = math.ceil(t0 / base_sc.drone.reemit_s + 1e-9) * base_sc.drone.reemit_s
                    first = False
                else:
                    t += base_sc.drone.reemit_s
        hb = 0.0
        while hb <= horizon:
            msgs.append((hb, {"type": "heartbeat", "ts": round(hb, 3)}))
            hb += 1.0 / base_sc.drone.heartbeat_hz
        msgs.sort(key=lambda m: m[0])
        delivered0, _ = Link(base_sc.link).apply(msgs, 0.0)          # 理想臂：0 起飞延迟
        delivered8, _ = Link(base_sc.link).apply(msgs, delay_s)      # 受限臂：起飞晚 delay_s

        for phi in ORIENTATIONS:
            for pname, (authority, wait_enabled, judge_err, delivered) in (
                    ("no_drone", (False, False, 0.0, [])),
                    ("ideal_wait", (True, True, 0.0, delivered0)),
                    ("constrained", (True, True, 180.0, delivered8))):
                st = policies[pname]
                sc = build_world_b(entrance, exit_cell)
                sc.target.orientation_deg = phi
                sc.target.judge_error_deg = judge_err
                if pname == "no_drone":
                    adopted = (sc.defaults["lower"], sc.defaults["upper"])
                    wait = 0.0
                    reason = "无无人机：默认格直冲"
                    complete = (entrance == adopted[0] and exit_cell == adopted[1])
                    plan_lo, plan_up = adopted
                else:
                    adopted = _adopt_for_combo(sc, cfg, baseline, delivered,
                                               arrival_l10_joint, cap_s, authority, wait_enabled)
                    wait = adopted[2]
                    complete = (adopted[0] == entrance and adopted[1] == exit_cell)
                    reason = ("完赛" if complete else
                              f"情报不足：采用 {adopted[0]}/{adopted[1]} ≠ 真值")
                    plan_lo, plan_up = adopted[0], adopted[1]
                try:
                    map_cfg = _yaml.safe_load(open(os.path.join(repo, "config", "map.yaml"),
                                                   encoding="utf-8"))
                    tokens, polyline, shoot_stop = _plan(cfg, map_cfg, plan_lo, plan_up)
                    drive_est = round(drive_s_per_m * polyline, 2)
                    route_cells = _cells(tokens)
                except Exception as exc:                     # noqa: BLE001
                    drive_est = float("nan")
                    route_cells = {"A10", "B10", "C10", "D10"}
                    shoot_stop = "C10"
                    reason = f"规划失败（{type(exc).__name__}）"
                    complete = False
                # 射击（逐发按命中时刻；锚点 = 车到停站点时刻 + 等待）
                probe = evaluate_shooting(sc.target, shoot_stop, route_cells,
                                          judge_available=(pname != "no_drone"), anchor_t=0.0)
                anchor = commit_t + arrival_ref.get(probe.stop, arrival_ref.get("C10", 0.0)) + wait
                shot = evaluate_shooting(sc.target, shoot_stop, route_cells,
                                         judge_available=(pname != "no_drone"), anchor_t=anchor)
                dwell = 0.0 if not complete else shot.dwell_s
                if complete:
                    budget = commit_t + (drive_est if not math.isnan(drive_est) else 0.0) \
                        + wait + dwell
                    st.rows.append(MatchRow(
                        entrance=entrance, exit_cell=exit_cell, phi=phi,
                        adopted_lower=adopted[0], adopted_upper=adopted[1],
                        wait_s=round(wait, 2), complete=complete, reason=reason,
                        drive_est_s=drive_est, dwell_s=round(dwell, 2),
                        budget_s=round(budget, 2), within=budget <= budget_s,
                        margin_s=round(budget_s - budget, 2),
                        blue=shot.blue_hits, red=shot.red_hits, score=shot.score,
                        stop=shot.stop))
                else:
                    # 未跑完：判负原因 = 撞封死桥室（非超时）；预算口径 = 中断时刻
                    abort_cell = ("K10" if entrance != plan_lo else "K5")
                    abort_sim = arrival_ref.get(abort_cell, 5.9)
                    st.rows.append(MatchRow(
                        entrance=entrance, exit_cell=exit_cell, phi=phi,
                        adopted_lower=adopted[0], adopted_upper=adopted[1],
                        wait_s=round(wait, 2), complete=complete, reason=reason,
                        drive_est_s=drive_est, dwell_s=0.0,
                        budget_s=round(commit_t + abort_sim, 2), within=None, margin_s=None,
                        blue=0, red=0, score=0, stop=shot.stop))
                st.n += 1

    out = {"drone_recon_s": round(tl.flight_end_t, 2),
           "budget_s": budget_s,
           "note": f"航速 {base_sc.drone.speed_mps:g} m/s；drone_recon_s 不含起飞延迟"
                   f"（受限臂 +{delay_s:g}s）；drive_est 标定 = {drive_s_per_m:.3f} s/m；"
                   f"L10 决策点到达时刻用基线轨迹实测 {arrival_l10_sim:.2f}s"
                   f"（改道后路线在 L10 前 0.4 m 内因前视预转向有 ±0.5 s 轨迹差——保守口径）"}
    for st in policies.values():
        rows = st.rows
        comp = [r for r in rows if r.complete]
        st.completion_rate = round(len(comp) / len(rows), 4)
        budget_ok = [r for r in comp if r.within]
        st.budget_pass_rate = round(len(budget_ok) / len(comp), 4) if comp else 0.0
        st.blue_rate = round(sum(1 for r in rows if r.blue > 0) / len(rows), 4)
        st.red_rate = round(sum(1 for r in rows if r.red > 0) / len(rows), 4)
        st.mean_score = round(sum(r.score for r in rows) / len(rows), 3)
        margins = [r.margin_s for r in comp if r.margin_s is not None]
        st.mean_margin_s = round(sum(margins) / len(margins), 2) if margins else 0.0
        worst = min(comp, key=lambda r: r.margin_s if r.margin_s is not None else 9e9)
        st.worst_margin_s = worst.margin_s if worst.margin_s is not None else 0.0
        st.worst_case = (f"{worst.entrance}/{worst.exit_cell} φ={worst.phi:.0f}° "
                         f"预算{worst.budget_s}s（{worst.reason}）")
        st.max_wait_s = max(r.wait_s for r in rows)
        st.mean_wait_s = round(sum(r.wait_s for r in rows) / len(rows), 2)
        out[st.policy] = st
    return out
