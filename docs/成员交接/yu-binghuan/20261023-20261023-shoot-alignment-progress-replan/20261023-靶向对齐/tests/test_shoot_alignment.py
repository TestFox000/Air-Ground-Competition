# -*- coding: utf-8 -*-
"""靶向对齐（2026-10-23）专项测试。

需求（用户口径）：停靠射击格时**车头正对标靶**——一次到位（进段几何
直接对齐）优先，逐渐调整（停靠转身/云台兜底）只在物理不可行时降级，
且降级必须显式告警（操作手可见，不静默）。

背景：官方 path_executor 只跟折线、不知道靶在哪——开火瞬间的车头朝向
只能由"进段方向"保证。旧实现只把对齐当"更便宜"（朝向零转向准则），
实测路线出现斜向进停靠格（如 E9→D8*，NW 到达打北侧靶，残余 45°）。
"""

from __future__ import annotations

import warnings

import pytest

from ag_integrate.field_frame import CellRef
from ag_integrate.world_model import TargetRef, WorldSnapshot
from ag_integrate.mission_router import plan_route
from ag_integrate.car_adapter import compact_and_adapt

LETTERS = "ABCDEFGHIJKLMN"


def cell(n: str) -> CellRef:
    return CellRef(LETTERS.index(n[0]), int(n[1:]) - 1)


def snap(mines=(), passable=("K4", "K9"), obstacles=(), targets=()):
    return WorldSnapshot(0, tuple(cell(c) for c in mines),
                         tuple(cell(c) for c in passable),
                         tuple(cell(c) for c in obstacles),
                         tuple(targets))


def arrival_dir(route, idx: int) -> tuple[int, int]:
    """route[idx] 的进段方向（前一格 -> 本格，按符号取整；首格返回 (0,0)）。"""
    if idx <= 0:
        return (0, 0)
    a, b = cell(route[idx - 1]), cell(route[idx])
    dc = (b.col > a.col) - (b.col < a.col)
    dr = (b.row > a.row) - (b.row < a.row)
    return (dc, dr)


def face_dir(stop: str, target: str) -> tuple[int, int]:
    """停靠格 -> 靶格 的正对方向（符号化）。"""
    a, b = cell(stop), cell(target)
    dc = (b.col > a.col) - (b.col < a.col)
    dr = (b.row > a.row) - (b.row < a.row)
    return (dc, dr)


def route_index_of(cells, name: str) -> int:
    return list(cells).index(name)


# ------------------------------------------------------------- 一次到位

def test_fixed_target_stop_arrives_aligned():
    """固定蓝靶 D7：停靠格的进段方向必须正对靶心（一次到位）。"""
    s = snap(mines=("E3", "I2"), targets=(TargetRef(cell("D7"), True),))
    route = plan_route(s, cell("N10"), cell("N5"),
                       start_heading="y", final_heading="y")
    compact, cmds = compact_and_adapt(s, route, "N10", "y", "y")
    assert cmds.shoot_cells, "必须有射击停靠格"
    for stop in cmds.shoot_cells:
        i = route_index_of(compact, stop)
        assert arrival_dir(compact, i) == face_dir(stop, "D7"), (
            f"停靠格 {stop} 进段方向 {arrival_dir(compact, i)} 未正对靶心 "
            f"{face_dir(stop, 'D7')}（一次到位要求）")


def test_moving_target_stop_arrives_aligned():
    """移动靶 D7-E7（蓝面 S，停靠 D8/E8、面朝 N）：进段必须朝 N 到达。"""
    track = (cell("D7"), cell("E7"))
    t = TargetRef(cell("D7"), True, track=track, blue_face="S")
    s = snap(mines=("E3", "I2"), targets=(t,))
    route = plan_route(s, cell("N10"), cell("N5"),
                       start_heading="y", final_heading="y")
    compact, cmds = compact_and_adapt(s, route, "N10", "y", "y")
    assert cmds.shoot_cells, "必须有射击停靠格"
    for stop in cmds.shoot_cells:
        i = route_index_of(compact, stop)
        # 靶在停靠格正北（蓝面 S -> 车面 N），进段方向必须是 (0,-1)
        assert arrival_dir(compact, i) == (0, -1), (
            f"停靠格 {stop} 进段方向 {arrival_dir(compact, i)} 不是正北——"
            "移动靶停靠必须 N 向到达（车头正对轨道端点）")


def test_alignment_survives_compression():
    """对齐由压缩保真：进段是长直线时，压缩删中间格后方向不变。"""
    t = TargetRef(cell("D7"), True, track=(cell("D7"), cell("E7")), blue_face="S")
    s = snap(mines=("E3",), targets=(t,))
    route = plan_route(s, cell("N10"), cell("N5"),
                       start_heading="y", final_heading="y")
    compact, cmds = compact_and_adapt(s, route, "N10", "y", "y")
    for stop in cmds.shoot_cells:
        assert stop in compact
        i = route_index_of(compact, stop)
        assert arrival_dir(compact, i) == (0, -1)


# ------------------------------------------------- 逐渐调整（降级兜底）

def test_blocked_approach_degrades_with_warning():
    """对齐进段被堵（D9/E9 放障碍）：路线仍要出、停靠格仍在，但必须
    显式 RuntimeWarning 告警降级（逐渐调整），不许静默斜进。"""
    track = (cell("D7"), cell("E7"))
    t = TargetRef(cell("D7"), True, track=track, blue_face="S")
    s = snap(mines=("E3", "I2"), obstacles=("D9", "E9"), targets=(t,))
    with pytest.warns(RuntimeWarning, match="正对朝向"):
        route = plan_route(s, cell("N10"), cell("N5"),
                           start_heading="y", final_heading="y")
    compact, cmds = compact_and_adapt(s, route, "N10", "y", "y")
    assert cmds.shoot_cells, "降级不得丢靶：停靠格必须在"
    for stop in cmds.shoot_cells:
        assert stop in ("D8", "E8"), "移动靶停靠格只允许 D8/E8"


def test_no_warning_when_aligned_possible():
    """对齐可行时不得告警（告警只留给真降级，防狼来了）。"""
    t = TargetRef(cell("D7"), True, track=(cell("D7"), cell("E7")), blue_face="S")
    s = snap(mines=("E3", "I2"), targets=(t,))
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", RuntimeWarning)
        plan_route(s, cell("N10"), cell("N5"),
                   start_heading="y", final_heading="y")
    hit = [str(w.message) for w in caught if "正对朝向" in str(w.message)]
    assert not hit, f"对齐可行却告警: {hit}"


# ------------------------------------------------- 其他回归护栏

def test_red_target_never_stops():
    """红靶（友方）绝不产生停靠格（禁火铁律不因本次改动放松）。"""
    s = snap(mines=("E3",), targets=(TargetRef(cell("D7"), False),))
    route = plan_route(s, cell("N10"), cell("N5"),
                       start_heading="y", final_heading="y")
    compact, cmds = compact_and_adapt(s, route, "N10", "y", "y")
    assert not cmds.shoot_cells


def test_unknown_face_target_no_stop():
    """未辨靶（is_blue=None）不排停靠（两段式：先确认，确认后 replan 加位）。"""
    s = snap(mines=("E3",), targets=(TargetRef(cell("D7"), None),))
    route = plan_route(s, cell("N10"), cell("N5"),
                       start_heading="y", final_heading="y")
    compact, cmds = compact_and_adapt(s, route, "N10", "y", "y")
    assert not cmds.shoot_cells
