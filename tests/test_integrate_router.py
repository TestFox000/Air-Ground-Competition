"""ag_integrate mission_router 测试。"""

import pytest

from ag_integrate.field_frame import CellRef, cell_to_world
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.world_model import WorldModel
from ag_integrate.mission_router import plan_route


def make_snapshot(objects):
    return WorldModel().update(IntelFrame(1, tuple(objects)))


def obj(cell, t):
    x, y = cell_to_world(cell)
    return IntelObject(t, x, y, 0.35, 0.35)


def test_simple_open_route():
    start, goal = CellRef(13, 9), CellRef(13, 10)   # N10 -> N11
    snap = make_snapshot(
        (obj(CellRef(3, 10), ObjectType.MINE),       # D11
         obj(CellRef(1, 6), ObjectType.MINE),        # B7
         obj(CellRef(10, 8), ObjectType.ARROW),     # K9 passable
         obj(CellRef(5, 5), ObjectType.OBSTACLE),   # F6 blocked
         obj(CellRef(3, 1), ObjectType.TARGET_BLUE),  # D2 enemy
         )
    )
    route = plan_route(snap, start, goal)
    # 起点在首，终点在末；必经点都出现；障碍格不出现
    assert route[0] == "N10"
    assert route[-1] == "N11"
    for required in ("D11", "B7", "K9", "D2"):
        assert required in route
    assert "F6" not in route
    # 相邻不重复
    assert all(a != b for a, b in zip(route, route[1:]))


def test_unreachable_required_is_skipped_but_goal_reached():
    start, goal = CellRef(0, 0), CellRef(0, 2)     # A1 -> A3
    # 围住 C1：左 B1、右 D1、下 C2 均为障碍
    wall = [CellRef(1, 0), CellRef(3, 0), CellRef(2, 1)]
    objects = [obj(w, ObjectType.OBSTACLE) for w in wall]
    objects.append(obj(CellRef(2, 0), ObjectType.MINE))   # C1 被困，不可达
    snap = make_snapshot(objects)
    route = plan_route(snap, start, goal)
    assert "C1" not in route          # 不可达必经点被跳过
    assert route[-1] == "A3"          # 终点照常


def test_goal_unreachable_raises():
    start = CellRef(0, 0)             # A1
    goal = CellRef(1, 0)              # B1
    # 封死 A1 唯二的出路 B1 / A2
    wall = [obj(CellRef(1, 0), ObjectType.OBSTACLE),
            obj(CellRef(0, 1), ObjectType.OBSTACLE)]
    snap = make_snapshot(wall)
    with pytest.raises(ValueError):
        plan_route(snap, start, goal)


def test_no_required_straight_to_goal():
    start, goal = CellRef(0, 0), CellRef(0, 1)   # A1 -> A2
    snap = make_snapshot(())
    assert plan_route(snap, start, goal) == ("A1", "A2")


def test_cross_gates_without_gate_makes_goal_unreachable():
    # 空场地，起点/终点分处两个半场；启用约束但不给任何 gate -> 不可穿越
    start, goal = CellRef(0, 0), CellRef(0, 10)   # A1 -> A11
    snap = make_snapshot(())
    with pytest.raises(ValueError):
        plan_route(snap, start, goal, cross_gates=frozenset())


def test_cross_gates_route_crosses_only_at_gate():
    start, goal = CellRef(0, 0), CellRef(0, 10)   # A1 -> A11
    gate = CellRef(5, 5)                          # F6：跨入北半场的第一格
    snap = make_snapshot(())
    route = plan_route(snap, start, goal, cross_gates=frozenset({gate}))
    assert route[0] == "A1"
    assert route[-1] == "A11"
    assert "F6" in route
    # 全路线里跨越 行号5↔6 的相邻对只能有一处，且两格都在 F 列
    crossings = [
        (a, b)
        for a, b in zip(route, route[1:])
        if a[0] == b[0] and {int(a[1:]), int(b[1:])} == {5, 6}
    ]
    assert crossings == [("F5", "F6")]
