# -*- coding: utf-8 -*-
"""t28 ④ 地图调试功能：一键多种子生成 N 张随机地图 + 每张跑最佳路线 + 统计。

## 随机口径（按 t11 规则）

- 桥洞：入口 K7~K11（5 格）随机、出口 K1~K6（6 格）随机；
- 靶蓝面朝向 φ ∈ {0°, 90°, −90°, 180°} 随机（靶格默认 B11/C11，可用 ``targets`` 覆盖）；
- 每张图种子 = ``seed_base + i`` ⇒ 全程确定性，同参数复跑统计逐位相同（墙钟时间除外）。

## 统计定义（交付判据 ④：定义到分量级，失败/超时单列）

- **平均计算时间 ``avg_plan_s``** = 真规划器 ``plan_route_tracker_safe`` 单次墙钟秒，
  **全部 N 张图**的均值（规划在开车前必做，失败图也计入）。
- **平均完赛时间 ``avg_finish_s``** = **完赛样本**的 ``commit + drive + wait + dwell``
  总时长均值（``n_completed`` 样本数显式标注）。
- **失败/超时样本单列**：``abort_rows``（撞封死桥室/硬违规中断，附中断时刻）与
  ``over_budget_rows``（完赛但总时长 > 180 s 预算）**不进完赛均值**，各自成行。
- 每张图一条明细行（``rows``），控制台全量打印，JSON 落盘。

## 时间预算口径（如实告知）

- 每张图 = 规划(≈0.4~2 s) + 任务流规划复算 + 2 次 run_sim（v1 观察 + 最终执行，
  各 ≈1.5~3 s）+ 渲染一张 route_map.png（≈0.5 s）⇒ 实测单图墙钟见交付说明；
- 串行 50 张按实测单图 ×50 外推（交付说明给实测数与公式）。
"""

from __future__ import annotations

import json
import os
import random
import time
from dataclasses import asdict, dataclass

import yaml as _yaml

from dzbrain.core.config import Config

from .match_stats import plan_for_bridges
from .runner import run_arm
from .scenarios import apply_target_override, build_world_b

_ORIENTATIONS = (0.0, 90.0, -90.0, 180.0)


@dataclass
class RandomMapRow:
    map_id: str
    seed: int
    entrance: str
    exit_cell: str
    phi: float
    plan_s: float                      # 规划墙钟（计算时间分量）
    plan_tokens_n: int
    polyline_m: float
    completed: bool
    abort_t_s: float | None            # 中断时刻（未跑完）
    drive_s: float                     # run_sim 行驶时长
    wait_s: float                      # 桥前等待
    dwell_s: float                     # 射击驻留（未跑完 = 0）
    total_s: float                     # commit+drive+wait+dwell
    within_budget: bool | None         # None = 未跑完（判负原因非超时）
    mine_pressed: str                  # 盖雷 "7/8"
    blue: int
    red: int
    traj_sha256: str
    route_map: str


def _fmt_row(r: RandomMapRow) -> str:
    status = "完赛" if r.completed else f"中断@t={r.abort_t_s:.1f}s"
    budget = ("预算内" if r.within_budget is True else
              "超预算" if r.within_budget is False else "—")
    return (f"{r.map_id:<5} seed={r.seed:<10} 桥={r.entrance}/{r.exit_cell:<4} φ={r.phi:>4.0f}°  "
            f"规划={r.plan_s:>5.2f}s tokens={r.plan_tokens_n:>2} len={r.polyline_m:>6.2f}m  "
            f"{status:<12} drive={r.drive_s:>7.2f}s wait={r.wait_s:>4.2f}s dwell={r.dwell_s:>5.2f}s "
            f"总={r.total_s:>7.2f}s {budget}  盖雷={r.mine_pressed:<4} 蓝={r.blue} 红={r.red}  "
            f"traj={r.traj_sha256[:8]}")


def run_random_maps(cfg: Config, baseline: list[str], out_root: str, *,
                    n: int = 50, seed_base: int = 20261012,
                    targets: tuple[str, ...] | None = None,
                    orient: float | None = None,
                    repo_map_path: str | None = None) -> dict:
    """一键 N 张随机地图：每张规划最佳路线 → run_arm 全仿真 → 一张 route_map.png。

    :param targets: 覆盖靶格（两格相邻）；None = 默认 B11/C11。
    :param orient: 覆盖蓝面朝向（度）；None = 每场随机。
    :return: 统计 dict（rows 明细 + 分量级聚合；见模块 docstring 的统计定义）。
    """
    os.makedirs(out_root, exist_ok=True)
    repo = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "..", "..", "dzrobot_v2"))
    map_path = repo_map_path or os.path.join(repo, "config", "map.yaml")
    rows: list[RandomMapRow] = []
    for i in range(n):
        seed = seed_base + i
        rng = random.Random(seed)
        entrance = f"K{rng.randint(7, 11)}"
        exit_cell = f"K{rng.randint(1, 6)}"
        phi = float(rng.choice(_ORIENTATIONS))
        sc = build_world_b(entrance, exit_cell)
        sc.authority_gate = True
        sc.wait_at_bridge = True
        sc.mission.auto_plan_enabled = False     # 执行路线 = 下面这条已定时的规划产物
        if targets is not None:
            sc = apply_target_override(sc, targets, phi if orient is None else orient)
        else:
            # 无靶格覆盖：随机朝向（或显式覆盖）必须落到情景上
            sc.target.orientation_deg = phi if orient is None else float(orient) % 360.0
        sc.target.judge_error_deg = 0.0

        map_cfg = _yaml.safe_load(open(map_path, encoding="utf-8"))
        t0 = time.perf_counter()
        tokens, polyline, _shoot_stop = plan_for_bridges(
            cfg, map_cfg, entrance, exit_cell, targets=tuple(sc.truth.targets))
        plan_s = time.perf_counter() - t0

        map_dir = os.path.join(out_root, f"map_{i:02d}")
        arm = run_arm(f"m{i:02d}", sc, cfg, list(tokens), drone_on=True,
                      stop_on_violation=True, out_dir=map_dir,
                      description=f"随机地图 seed={seed}：桥 {entrance}/{exit_cell}，"
                                  f"φ={sc.target.orientation_deg:.0f}°（t28 ④ 批量）")
        m = arm.car
        b = arm.budget
        s = arm.shoot
        fm = arm.field_metrics
        rows.append(RandomMapRow(
            map_id=f"m{i:02d}", seed=seed, entrance=entrance, exit_cell=exit_cell,
            phi=sc.target.orientation_deg, plan_s=round(plan_s, 4),
            plan_tokens_n=len(tokens), polyline_m=round(polyline, 3),
            completed=bool(m["completed"]),
            abort_t_s=(round(m["field_stop_t"], 2) if not m["completed"] else None),
            drive_s=round(m["time_s"], 3), wait_s=round(arm.wait_lower_s, 3),
            dwell_s=round(s["dwell_s"], 3), total_s=round(b["total_s"], 3),
            within_budget=(bool(b["within"]) if m["completed"] else None),
            mine_pressed=f"{int(fm.get('mine_pressed_count', -1))}/{int(fm.get('mine_total', -1))}",
            blue=int(s["blue_hits"]), red=int(s["red_hits"]),
            traj_sha256=arm.artifacts["traj_sha256"], route_map=arm.artifacts["route_map"]))

    completed = [r for r in rows if r.completed]
    aborted = [r for r in rows if not r.completed]
    over = [r for r in completed if r.within_budget is False]
    stats = {
        "n_total": len(rows),
        "n_completed": len(completed),
        "n_aborted": len(aborted),
        "n_over_budget": len(over),
        "avg_plan_s": round(sum(r.plan_s for r in rows) / len(rows), 4)
        if rows else 0.0,                       # 计算时间：全 N 张（含失败）均值
        "max_plan_s": round(max(r.plan_s for r in rows), 4) if rows else 0.0,
        "avg_finish_s": round(sum(r.total_s for r in completed) / len(completed), 3)
        if completed else None,                 # 完赛时间：仅完赛样本均值（口径见 docstring）
        "avg_drive_s": round(sum(r.drive_s for r in completed) / len(completed), 3)
        if completed else None,
        "avg_wait_s": round(sum(r.wait_s for r in completed) / len(completed), 3)
        if completed else None,
        "avg_dwell_s": round(sum(r.dwell_s for r in completed) / len(completed), 3)
        if completed else None,
        "completion_rate": round(len(completed) / len(rows), 4) if rows else 0.0,
        "budget_pass_rate": round(
            len([r for r in completed if r.within_budget is not False and r.within_budget is not None])
            / len(completed), 4) if completed else 0.0,
        "abort_rows": [asdict(r) for r in aborted],      # 失败样本单列（不进完赛均值）
        "over_budget_rows": [asdict(r) for r in over],   # 超时样本单列（不进完赛均值）
        "rows": [asdict(r) for r in rows],
    }
    with open(os.path.join(out_root, "random_maps_summary.json"), "w", encoding="utf-8") as fh:
        json.dump(stats, fh, ensure_ascii=False, indent=2)
    return stats
