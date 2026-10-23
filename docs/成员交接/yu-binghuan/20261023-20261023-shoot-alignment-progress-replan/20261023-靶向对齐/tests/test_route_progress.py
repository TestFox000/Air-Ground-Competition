# -*- coding: utf-8 -*-
"""route_progress（重规划"基于最新进度"底座）单元测试。"""

from __future__ import annotations

import math

import pytest

from ag_integrate.field_frame import CellRef, cell_to_world
from ag_integrate.route_progress import (
    RouteProgress, filter_swept_mines, route_cells_of, route_progress,
)
from ag_integrate.world_model import WorldSnapshot

LETTERS = "ABCDEFGHIJKLMN"


def cell(n: str) -> CellRef:
    return CellRef(LETTERS.index(n[0]), int(n[1:]) - 1)


def world(n: str) -> tuple[float, float]:
    return cell_to_world(cell(n))


# ------------------------------------------------------------- 解析

def test_route_cells_of_parses_dialect():
    """车端方言：'*' 开火标记、末尾 ',x' 朝向 token 都要正确处理。"""
    assert route_cells_of("N10 K10 I10 H9* J3 ,x") == \
        ["N10", "K10", "I10", "H9", "J3"]
    assert route_cells_of("") == []


def test_route_cells_of_rejects_bad_token():
    """reset 命令串不是路线（路线命令与 reset 分开发布），必须整体拒绝。"""
    with pytest.raises(Exception):
        route_cells_of("reset:N10,x A1 B2")
    with pytest.raises(Exception):
        route_cells_of("Z99")


# ------------------------------------------------------------- 进度推断

def test_progress_at_exact_waypoint():
    x, y = world("M10")
    p = route_progress("N10 M10 K10 J10", x, y, math.pi / 2)
    assert p.cell.name == "M10"
    assert p.heading == "y"
    # 当前格不算已扫（保守：车可能才到一半）
    assert p.swept_cells == frozenset({"N10"})


def test_progress_between_waypoints_takes_nearest():
    kx, ky = world("K10")
    jx, jy = world("J10")
    # K10 -> J10 60% 处（更靠近 J10）：进度点取 J10，之前全算已扫
    x = kx + (jx - kx) * 0.6
    y = ky + (jy - ky) * 0.6
    p = route_progress("N10 M10 K10 J10 I10", x, y, 0.0)
    assert p.cell.name == "J10"
    assert p.swept_cells == frozenset({"N10", "M10", "K10"})
    assert p.heading == "x"


def test_progress_off_field_raises():
    with pytest.raises(ValueError):
        route_progress("A1 B1", -5.0, -5.0, 0.0)


def test_progress_far_from_center_raises():
    # 场外远点（哪怕在场地包围盒外一点）：world_to_cell 残差防线拒绝
    with pytest.raises(ValueError):
        route_progress("A1 B1", 0.0, 0.0, 0.0)   # 场地左下角外


def test_progress_first_waypoint_sweeps_nothing():
    x, y = world("N10")
    p = route_progress("N10 M10 K10", x, y, math.pi / 2)
    assert p.swept_cells == frozenset()
    assert p.cell.name == "N10"


# ------------------------------------------------------------- 已扫雷过滤

def _snap(mines):
    return WorldSnapshot(0, tuple(cell(c) for c in mines), (), (), ())


def test_filter_swept_mines_removes_only_swept():
    s = _snap(("A1", "B2", "C3"))
    out = filter_swept_mines(s, {"A1", "C3", "Z9"})
    assert [m.name for m in out.mines] == ["B2"]


def test_filter_swept_mines_empty_swept_returns_same():
    s = _snap(("A1",))
    assert filter_swept_mines(s, frozenset()) is s


def test_filter_keeps_other_fields():
    s = WorldSnapshot(0, (cell("A1"),), (cell("K4"),), (cell("D9"),), ())
    out = filter_swept_mines(s, {"A1"})
    assert [c.name for c in out.passable] == ["K4"]
    assert [c.name for c in out.obstacles] == ["D9"]
    assert out.mines == ()
