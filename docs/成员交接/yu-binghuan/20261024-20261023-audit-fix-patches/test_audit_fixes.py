# -*- coding: utf-8 -*-
"""审查复核修复项（2026-10-23）回归测试。

D1：移动靶 track 有值而 blue_face 缺/错 -> 显式 ValueError（点名靶格与
    非法值），不再是裸 KeyError 被吞成无信息日志。
D4：射击格只标首次出现 -> 回环重访不重复停车开火（弹量契约）。
"""

from __future__ import annotations

import pytest

from ag_integrate.field_frame import CellRef
from ag_integrate.world_model import TargetRef, WorldSnapshot
from ag_integrate.mission_router import plan_route
from ag_integrate.car_commands import RoutePlan, route_command

LETTERS = "ABCDEFGHIJKLMN"


def cell(n: str) -> CellRef:
    return CellRef(LETTERS.index(n[0]), int(n[1:]) - 1)


def snap(targets):
    return WorldSnapshot(0, (cell("E3"),), (cell("K4"), cell("K9")),
                         (), tuple(targets))


# ---------------------------------------------------------------- D1

@pytest.mark.parametrize("bad_face", [None, "X", "s", "", "NORTH"])
def test_d1_bad_blue_face_raises_actionable_error(bad_face):
    """track 有值 + blue_face 非法 -> ValueError 点名靶格与非法值。"""
    t = TargetRef(cell("D7"), True, track=(cell("D7"), cell("E7")),
                  blue_face=bad_face)
    with pytest.raises(ValueError, match="蓝面朝向非法"):
        plan_route(snap((t,)), cell("N10"), cell("N5"),
                   start_heading="y", final_heading="y")


def test_d1_error_names_the_target_and_value():
    t = TargetRef(cell("D7"), True, track=(cell("D7"), cell("E7")),
                  blue_face=None)
    with pytest.raises(ValueError) as ei:
        plan_route(snap((t,)), cell("N10"), cell("N5"))
    msg = str(ei.value)
    assert "D7" in msg, "报错必须点名靶格"
    assert "None" in msg, "报错必须带非法值本身"


def test_d1_valid_face_still_plans():
    """合法 blue_face 不受影响（回归护栏）。"""
    t = TargetRef(cell("D7"), True, track=(cell("D7"), cell("E7")),
                  blue_face="S")
    r = plan_route(snap((t,)), cell("N10"), cell("N5"),
                   start_heading="y", final_heading="y")
    assert "D8" in r or "E8" in r


# ---------------------------------------------------------------- D4

def test_d4_star_marks_first_occurrence_only():
    """回环重访同一射击格：'*' 只标首次。"""
    plan = RoutePlan(cells=("A1", "B1", "A1", "C1"), shoot_at={"A1"})
    out = route_command(plan)
    assert out.count("A1*") == 1, f"射击格重复标星: {out!r}"
    assert out == "A1* B1 A1 C1"


def test_d4_distinct_shoot_cells_all_marked():
    """不同射击格互不影响。"""
    plan = RoutePlan(cells=("A1", "B1", "C1"), shoot_at={"A1", "C1"})
    assert route_command(plan) == "A1* B1 C1*"


def test_d4_no_shoot_no_star():
    plan = RoutePlan(cells=("A1", "B1"))
    assert "*" not in route_command(plan)


def test_d4_final_heading_token_intact():
    plan = RoutePlan(cells=("A1", "B1", "A1"), shoot_at={"A1"},
                     final_heading="y")
    out = route_command(plan)
    assert out.endswith(",x") and out.count("A1*") == 1
