"""ag_integrate mission_router 测试。"""

import pytest

from ag_integrate.field_frame import CellRef, cell_to_world
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.world_model import WorldModel
from ag_integrate.mission_router import compress_route, plan_route


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
    # 起点在首，终点在末；雷必经；障碍格不出现
    assert route[0] == "N10"
    assert route[-1] == "N11"
    for required in ("D11", "B7"):
        assert required in route
    assert "F6" not in route
    # 蓝靶 D2 不可踩；其正交邻格中必有停靠射击点
    assert "D2" not in route
    assert any(n in route for n in ("C2", "E2", "D1", "D3"))
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
    start = CellRef(1, 1)             # B2
    goal = CellRef(0, 0)              # A1（角落格，唯二出路 A2/B1）
    # 封死 A1 的出路 A2 / B1（注意：直接给目标格标障碍已不触发不可达——
    # 起终点格对封锁免疫，那里的障碍报告按误检处理，见 test_router_robustness）
    wall = [obj(CellRef(0, 1), ObjectType.OBSTACLE),
            obj(CellRef(1, 0), ObjectType.OBSTACLE)]
    snap = make_snapshot(wall)
    with pytest.raises(ValueError):
        plan_route(snap, start, goal)


def test_no_required_straight_to_goal():
    start, goal = CellRef(0, 0), CellRef(0, 1)   # A1 -> A2
    snap = make_snapshot(())
    assert plan_route(snap, start, goal) == ("A1", "A2")


def test_moving_target_stops_on_blue_face_side():
    """移动靶（双面，轨道 D8-E8 水平，蓝面朝南）：
    轨道两端不可踩；必经蓝面侧端点正对格（D9/E9）停靠；红面可见时不安排停靠。"""
    from ag_integrate.world_model import TargetRef, WorldSnapshot

    start, goal = CellRef(0, 0), CellRef(0, 10)   # A1 -> A11
    a, b = CellRef(3, 7), CellRef(4, 7)          # D8, E8 轨道两端
    mine = CellRef(0, 5)                          # A6
    snap = WorldSnapshot(0, (mine,), (), (), (TargetRef(a, True, (a, b), "S"),))
    route = plan_route(snap, start, goal)
    assert "A6" in route                    # 雷照常
    assert "D8" not in route and "E8" not in route    # 轨道不可踩
    assert "D9" in route or "E9" in route   # 蓝面侧停靠必经

    # 红面可见（友方）：不安排停靠，路线只剩排雷
    snap_red = WorldSnapshot(0, (mine,), (), (), (TargetRef(a, False, (a, b), "S"),))
    route2 = plan_route(snap_red, start, goal)
    assert "A6" in route2
    assert "D9" not in route2 and "E9" not in route2


def test_cross_gates_without_gate_makes_goal_unreachable():
    # 空场地，起点/终点分处两个半场；启用约束但不给任何 gate -> 不可穿越
    start, goal = CellRef(0, 0), CellRef(0, 10)   # A1 -> A11
    snap = make_snapshot(())
    with pytest.raises(ValueError):
        plan_route(snap, start, goal, cross_gates=frozenset())


def test_cross_gates_route_crosses_only_at_gate():
    start, goal = CellRef(0, 0), CellRef(0, 10)   # A1 -> A11
    gate = CellRef(5, 5)                          # F6：紧贴挡板边界的南侧格
    snap = make_snapshot(())
    # mid_boundary=6 = 真实场地（白色挡板 D7..N7 在第 6/7 行之界）
    route = plan_route(snap, start, goal, cross_gates=frozenset({gate}), mid_boundary=6)
    assert route[0] == "A1"
    assert route[-1] == "A11"
    assert "F6" in route
    # 全路线里跨越 行号6↔7 的相邻对只能有一处，且两格都在 F 列
    crossings = [
        (a, b)
        for a, b in zip(route, route[1:])
        if a[0] == b[0] and {int(a[1:]), int(b[1:])} == {6, 7}
    ]
    assert crossings == [("F6", "F7")]


def test_compress_route_removes_collinear_middle_cells():
    # A1→B2→C3→D4 同向对角，E4→F4 同向水平：只留拐点
    route = ("A1", "B2", "C3", "D4", "E4", "F4")
    assert compress_route(route) == ("A1", "D4", "F4")


def test_compress_route_keeps_corners_and_waypoints():
    # 拐点（方向变化处）永不被删
    route = ("A1", "A2", "A3", "B4")
    assert compress_route(route) == ("A1", "A3", "B4")
    # keep 中的必经点即使共线也保留
    route = ("A1", "B2", "C3", "D4", "E4", "F4")
    assert compress_route(route, keep={"C3"}) == ("A1", "C3", "D4", "F4")


def test_compress_route_short_and_degenerate():
    assert compress_route(("A1", "A2")) == ("A1", "A2")
    assert compress_route(("A1",)) == ("A1",)
    # 折返点（进入/离开方向相反）是拐点，保留
    route = ("A1", "A2", "A1")
    assert compress_route(route) == ("A1", "A2", "A1")
