# -*- coding: utf-8 -*-
"""站距环带 + LOS 遮挡优选 + 按轨端覆盖的几何护栏（2026-10-26
[STANDOFF-ABSORB-20261026] 的多方向验证单测）。

口径来源：车端 route_check 第④层（射击点距离 0.78~1.02 m，整数格偏移
恰为 √5=0.783 / √8=0.990 m 两档；每轨端须有窗内停站）。
"""
from __future__ import annotations

import warnings

from ag_integrate.field_frame import CellRef, LETTERS
from ag_integrate.mission_router import (
    _los_clear,
    _shoot_candidates_moving,
    _standoff_serves,
    plan_route,
)
from ag_integrate.world_model import TargetRef, WorldSnapshot


def cell(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


def _moving(pair, blue_face):
    return TargetRef(cell(pair[0]), True,
                     track=(cell(pair[0]), cell(pair[1])), blue_face=blue_face)


# ---------------------------------------------------------------- 环带几何
def test_ring_offsets_exact_two_tiers():
    """所有候选对某轨端的偏移恰为 √5/√8 两档（0.783/0.990 m），无其它距离。"""
    for pair, face in ((("E7", "F7"), "S"), (("B2", "B3"), "E"),
                       (("H9", "I9"), "N")):
        cands = _shoot_candidates_moving(_moving(pair, face), frozenset())
        assert cands, (pair, face)
        for c in cands:
            assert any(_standoff_serves(c, cell(r)) for r in pair), (pair, c.name)


def test_ring_respects_blue_face_side():
    cands = _shoot_candidates_moving(_moving(("E7", "F7"), "S"), frozenset())
    assert cands and all(int(c.name[1:]) > 7 for c in cands)   # 全在南侧
    cands = _shoot_candidates_moving(_moving(("B2", "B3"), "E"), frozenset())
    assert cands and all(LETTERS.index(c.name[0]) > 1 for c in cands)  # 全在东侧


def test_ring_excludes_blocked_cells():
    blocked = frozenset({cell("E9"), cell("F9"), cell("H8"), cell("G9")})
    cands = _shoot_candidates_moving(_moving(("E7", "F7"), "S"), blocked)
    names = {c.name for c in cands}
    assert not (names & {"E9", "F9", "H8", "G9"})


def test_rail_at_field_edge_still_has_candidates():
    # 西缘 A 列垂直轨道：东侧（B..C）仍有环带候选
    cands = _shoot_candidates_moving(_moving(("A2", "A3"), "E"), frozenset())
    assert cands and all(c.name[0] in "BC" for c in cands)
    # 南缘水平轨道：北侧仍有候选
    cands = _shoot_candidates_moving(_moving(("H11", "I11"), "N"), frozenset())
    assert cands and all(int(c.name[1:]) <= 10 for c in cands)


# ---------------------------------------------------------------- LOS
def test_los_clear_detects_blocking_bucket():
    # 停 C7 → 轨 D5：(−2,1) 偏移；连线穿过 D6 与 C6 的格界，D6 放桶即挡
    assert not _los_clear(cell("C7"), cell("D5"),
                          frozenset({cell("D6")}))
    assert _los_clear(cell("C7"), cell("D5"),
                      frozenset({cell("H1")}))          # 远处桶不挡
    assert _los_clear(cell("C7"), cell("D5"), frozenset())


def test_los_endpoint_semantics_documented():
    # 线段终点=轨格心：若把轨格误入障碍集，t=1 采样必落其中而自挡——
    # 因此 plan_router 分组处显式剔除 target/track 格。本测锁死该语义：
    # 轨格在集合内时确为 False，防将来有人"顺手"把整个 blocked 传进来。
    assert not _los_clear(cell("C7"), cell("D5"), frozenset({cell("D5")}))


# ---------------------------------------------------------------- 按端覆盖
def test_plan_route_serves_both_rail_ends():
    snap = WorldSnapshot(
        timestamp_ms=1,
        mines=tuple(cell(n) for n in ("D2", "F3", "H2", "J4", "E8",
                                      "G9", "I8", "M3")),
        passable=(cell("K9"),),
        obstacles=tuple(cell(n) for n in ("C5", "F6", "H7")),
        targets=(_moving(("E7", "F7"), "S"),),
    )
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        route = plan_route(snap, cell("B9"), cell("B2"),
                           start_heading="y", final_heading="x",
                           turn90_guard=True, solid_barriers=True,
                           reverse=True, heading_prop=True)
    for rail in ("E7", "F7"):
        assert any(_standoff_serves(cell(x), cell(rail)) for x in route), rail
