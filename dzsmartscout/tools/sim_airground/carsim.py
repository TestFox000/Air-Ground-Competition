# -*- coding: utf-8 -*-
"""车侧执行仿真包装：只读调用 ``dzbrain.tools.sim.run_sim``，采集完赛指标与逐格到达时刻。

★ 台账纪律（交接包）：
- 跑完/总时长/RMS/MAX/超调/吸附/越界 六项 + 物理在环两口径（盖雷几何
  ``mine_pressed_count`` vs 车体足迹 ``obstacle_hit_steps``/``board_overlap_steps``）
  全部取自 run_sim 契约指标（docs/P0接口约定.md §5），本模块不另写判据；
- 每次运行落盘 ``traj.csv``（sha256 进台账，复跑逐字节相同）；
- sim 不建模 PIVOT（sim.py:109-113 刻意不做）——凡涉及折返/PIVOT 的结论标注"仿真无法裁决"；
- sim 不建模射击停站驻留——射击/驻留/扣分账由 ``shoot.py`` 解析模型给出（局限标注）。
"""

from __future__ import annotations

import csv
import hashlib
import os
from dataclasses import dataclass, field

from dzbrain.core.config import Config
from dzbrain.core import geometry as geo
from dzbrain.route.parser import parse_mission
from dzbrain.tools.sim import FieldSpec, SimConfig, run_sim

from .geo import meters_to_block


@dataclass
class CarRunResult:
    """一次 run_sim 的落盘产物 + 指标 + 逐格到达时刻。"""

    arm: str
    tokens: list[str]
    metrics: dict = field(default_factory=dict)
    field_report: dict = field(default_factory=dict)
    traj_csv: str = ""
    sha256: str = ""
    arrival_sim: dict[str, float] = field(default_factory=dict)  # 格名 → 首到达时刻（sim 时间）
    visit_ticks: dict[str, int] = field(default_factory=dict)    # 格名 → 停留拍数
    t_series: list[float] = field(default_factory=list)
    cell_series: list[str] = field(default_factory=list)
    #: t28 ② 车位姿（**真值口径**：run_sim 的 truth_x/truth_y/truth_yaw，非信念位姿）
    truth_pose: list[tuple[float, float, float]] = field(default_factory=list)


def run_car(arm: str, tokens: list[str], cfg: Config, field: FieldSpec | None,
            *, stop_on_violation: bool = False, out_dir: str,
            seed: int = 20261007) -> CarRunResult:
    """跑一次车仿真并落盘。**纯确定性**：同参同种子 → 同 sha256。"""
    route = parse_mission([str(t) for t in tokens], name=arm)
    sim = SimConfig(stop_on_field_violation=stop_on_violation, seed=seed)
    res = run_sim(route, cfg, sim, field)
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "traj.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "yaw", "truth_x", "truth_y", "truth_yaw",
                    "seg", "s", "truth_seg", "truth_s", "steer", "steer_cmd", "anchored"])
        for i in range(len(res.t)):
            w.writerow([f"{res.t[i]:.4f}", f"{res.x[i]:.4f}", f"{res.y[i]:.4f}",
                        f"{res.yaw[i]:.4f}", f"{res.truth_x[i]:.4f}", f"{res.truth_y[i]:.4f}",
                        f"{res.truth_yaw[i]:.4f}", res.seg_index[i], f"{res.s[i]:.4f}",
                        res.truth_seg_index[i], f"{res.truth_s[i]:.4f}",
                        f"{res.steer[i]:.4f}", f"{res.steer_cmd[i]:.4f}",
                        int(res.anchored[i])])
    sha = hashlib.sha256(open(csv_path, "rb").read()).hexdigest()

    arrival: dict[str, float] = {}
    visits: dict[str, int] = {}
    cells: list[str] = []
    poses: list[tuple[float, float, float]] = []
    for i in range(len(res.t)):
        block = meters_to_block(res.truth_x[i], res.truth_y[i])
        cells.append(block)
        visits[block] = visits.get(block, 0) + 1
        if block not in arrival:
            arrival[block] = res.t[i]
        poses.append((float(res.truth_x[i]), float(res.truth_y[i]),
                      float(res.truth_yaw[i])))
    return CarRunResult(
        arm=arm, tokens=[str(t) for t in tokens], metrics=dict(res.metrics),
        field_report=dict(res.field_report), traj_csv=csv_path, sha256=sha,
        arrival_sim=arrival, visit_ticks=visits, t_series=list(res.t),
        cell_series=cells, truth_pose=poses,
    )


# --------------------------------------------------------------------------- #
#  信息领先契约（关键格揭示 vs 车到达；情报缺口拍数/缺口格数）
# --------------------------------------------------------------------------- #
def gap_metrics(car: CarRunResult, joint_commit_t: float,
                reveal_joint: dict[str, float], key_cells: list[str] | None = None) -> dict:
    """逐拍统计"车已在格内、该格尚未被无人机揭示"的缺口（队长补充 A）。

    :param reveal_joint: 格名 → 揭示时刻（joint 时间 = 航线揭示 + 起飞延迟 + 链路延迟；
        永不揭示的格不在表内）。
    :param key_cells: 关键格（雷/障/靶/桥洞）——**主指标**只统计关键格的缺口
        （"该段及其下一个关键格是否已被揭示"）；``gap_*`` 全口径、``gap_key_*`` 只算关键格。
    """
    gap_ticks = 0
    gap_cells: dict[str, int] = {}
    key_gap_ticks = 0
    key_gap_cells: dict[str, int] = {}
    key_set = set(key_cells or [])
    for t, block in zip(car.t_series, car.cell_series):
        joint_t = joint_commit_t + t
        rev = reveal_joint.get(block)
        if rev is None or joint_t < rev:
            gap_ticks += 1
            gap_cells[block] = gap_cells.get(block, 0) + 1
            if block in key_set:
                key_gap_ticks += 1
                key_gap_cells[block] = key_gap_cells.get(block, 0) + 1
    return {"gap_ticks": gap_ticks, "gap_cells": len(gap_cells),
            "gap_by_cell": gap_cells,
            "gap_key_ticks": key_gap_ticks, "gap_key_cells": len(key_gap_cells),
            "gap_key_by_cell": key_gap_cells}


def lead_table(car: CarRunResult, joint_commit_t: float, reveal_joint: dict[str, float],
               key_cells: list[str]) -> list[dict]:
    """每个关键格的"揭示提前量"：揭示时刻 vs 车到达时刻（到达 = 未到则 None）。"""
    rows = []
    for cell in key_cells:
        arr = car.arrival_sim.get(cell)
        arr_joint = (joint_commit_t + arr) if arr is not None else None
        rev = reveal_joint.get(cell)
        lead = None
        if arr_joint is not None and rev is not None:
            lead = round(arr_joint - rev, 3)
        elif arr_joint is not None:
            lead = "从未揭示"
        elif rev is not None:
            lead = "车未到"
        rows.append({"cell": cell, "reveal_joint": rev, "arrival_joint": arr_joint,
                     "lead_s": lead})
    return rows
