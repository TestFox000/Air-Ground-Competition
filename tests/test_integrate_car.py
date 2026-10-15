"""ag_integrate car_commands / fire_guard / car_adapter 测试。"""

import pytest

from ag_integrate.car_commands import (
    CommandError,
    RoutePlan,
    reset_command,
    route_command,
)
from ag_integrate.car_adapter import adapt
from ag_integrate.field_frame import CellRef, cell_to_world
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.world_model import WorldModel
from ag_integrate.fire_guard import decide_fire


# ---------- 命令编码 ----------

def test_reset_command():
    assert reset_command("n10", "y") == "reset:N10,y"
    with pytest.raises(CommandError):
        reset_command("N12", "y")
    with pytest.raises(CommandError):
        reset_command("N10", "z")


def test_route_command_basic():
    plan = RoutePlan(("g8", "a10", "d2", "e2"), frozenset({"D2"}), "y")
    assert route_command(plan) == "G8 A10 D2* E2,y"


def test_route_command_dedup_and_validation():
    assert route_command(RoutePlan(("d2", "D2", "e3"))) == "D2 E3"
    with pytest.raises(CommandError):
        route_command(RoutePlan(("Z9",)))
    with pytest.raises(CommandError):
        route_command(RoutePlan(()))
    with pytest.raises(CommandError):
        route_command(RoutePlan(("D2",), frozenset(), "z"))


def test_shoot_mark_only_for_present_cells():
    plan = RoutePlan(("D2",), frozenset({"E9"}), None)
    assert route_command(plan) == "D2"   # E9 不在路线，不加 *


# ---------- 禁火保险 ----------

def snapshot_with_targets(blue: CellRef, red: CellRef):
    def obj(cell, t):
        x, y = cell_to_world(cell)
        return IntelObject(t, x, y, 0.35, 0.35)

    frame = IntelFrame(
        1,
        (obj(blue, ObjectType.TARGET_BLUE), obj(red, ObjectType.TARGET_RED)),
    )
    return WorldModel().update(frame)


def test_fire_guard():
    blue, red = CellRef(3, 1), CellRef(7, 9)   # D2 蓝敌, H10 红友
    snap = snapshot_with_targets(blue, red)
    assert decide_fire(snap, "D3").allowed     # D3 正交邻 D2 蓝
    assert decide_fire(snap, "D2").denied      # 靶格本身无邻靶，不可停靠开火
    assert decide_fire(snap, "G10").denied      # G10 邻 H10 红友方
    assert decide_fire(snap, "K9").denied       # 附近无靶


def test_fire_guard_moving_target():
    """移动靶禁火判定：蓝面可见时只有蓝面侧正对端点的格允许开火；
    红面侧/顺路格一律禁；红面可见全场禁火。"""
    from ag_integrate.world_model import TargetRef, WorldSnapshot

    a, b = CellRef(3, 7), CellRef(4, 7)   # D8-E8 水平轨道，蓝面朝南
    blue = TargetRef(a, True, (a, b), "S")
    red = TargetRef(a, False, (a, b), "S")
    snap_blue = WorldSnapshot(0, (), (), (), (blue,))
    snap_red = WorldSnapshot(0, (), (), (), (red,))
    assert decide_fire(snap_blue, "D9").allowed    # 正对 D8 端点，蓝面侧
    assert decide_fire(snap_blue, "E9").allowed     # 正对 E8 端点
    assert decide_fire(snap_blue, "C8").denied     # 顺路邻轨道但非蓝面停靠位
    assert decide_fire(snap_blue, "D7").denied     # 红面侧（北），禁停
    assert decide_fire(snap_red, "D9").denied       # 红面可见：友方，全场禁火


def test_fire_guard_mixed_targets_risk():
    """红蓝双邻：停靠格同时正交邻到蓝靶和红靶时拒绝开火（误伤友方风险）。"""
    blue, red = CellRef(3, 1), CellRef(3, 3)   # D2 蓝敌, D4 红友
    snap = snapshot_with_targets(blue, red)
    # D3 同时邻 D2(蓝) 和 D4(红) → 混合风险，禁火
    d = decide_fire(snap, "D3")
    assert d.denied
    assert d.reason == "mixed_targets_risk"


def test_fire_guard_reason_codes():
    """各禁火/允火原因码正确。"""
    blue, red = CellRef(3, 1), CellRef(7, 9)   # D2 蓝, H10 红
    snap = snapshot_with_targets(blue, red)
    assert decide_fire(snap, "D3").reason == "enemy_blue_adjacent"
    assert decide_fire(snap, "G10").reason == "friendly_red_adjacent"
    assert decide_fire(snap, "K9").reason == "no_target_adjacent"


def test_fire_guard_boundary_cells():
    """边界/角落格的邻居计算正确，不会越界。"""
    from ag_integrate.fire_guard import neighbor_names
    # 角落 A1：只有 B1、A2 两个邻居
    assert set(neighbor_names("A1")) == {"B1", "A2"}
    # 角落 N11：只有 M11、N10
    assert set(neighbor_names("N11")) == {"M11", "N10"}
    # 顶边中部 F1：E1、G1、F2
    assert set(neighbor_names("F1")) == {"E1", "G1", "F2"}
    # 左边中部 A6：A5、A7、B6
    assert set(neighbor_names("A6")) == {"A5", "A7", "B6"}


def test_fire_guard_moving_target_vertical():
    """垂直轨道移动靶：蓝面朝东/西时仅东/西侧正对端点允许开火。"""
    from ag_integrate.world_model import TargetRef, WorldSnapshot

    a, b = CellRef(3, 7), CellRef(3, 8)   # D8-D9 垂直轨道，蓝面朝东
    blue_e = TargetRef(a, True, (a, b), "E")
    snap_e = WorldSnapshot(0, (), (), (), (blue_e,))
    assert decide_fire(snap_e, "E8").allowed    # 东侧正对 D8
    assert decide_fire(snap_e, "E9").allowed    # 东侧正对 D9
    assert decide_fire(snap_e, "C8").denied     # 西侧（红面侧），禁
    assert decide_fire(snap_e, "D7").denied     # 顺路端点外，非停靠位

    # 蓝面朝西
    blue_w = TargetRef(a, True, (a, b), "W")
    snap_w = WorldSnapshot(0, (), (), (), (blue_w,))
    assert decide_fire(snap_w, "C8").allowed    # 西侧正对 D8
    assert decide_fire(snap_w, "E8").denied     # 东侧（红面侧），禁


def test_fire_guard_moving_target_north_face():
    """水平轨道蓝面朝北时，北侧正对端点允许开火，南侧禁。"""
    from ag_integrate.world_model import TargetRef, WorldSnapshot

    a, b = CellRef(3, 7), CellRef(4, 7)   # D8-E8 水平轨道，蓝面朝北
    blue = TargetRef(a, True, (a, b), "N")
    snap = WorldSnapshot(0, (), (), (), (blue,))
    assert decide_fire(snap, "D7").allowed    # 北侧正对 D8
    assert decide_fire(snap, "E7").allowed    # 北侧正对 E8
    assert decide_fire(snap, "D9").denied     # 南侧（红面侧），禁


# ---------- 适配器端到端 ----------

def test_adapt_emits_reset_route_and_filters_shoot():
    blue, red = CellRef(3, 1), CellRef(7, 9)   # D2 蓝敌, H10 红友
    snap = snapshot_with_targets(blue, red)
    # 路线同时经过蓝靶停靠格（D3 邻 D2）与红靶旁格（I10 邻 H10）：只有前者保留 '*'
    cmds = adapt(
        snap,
        ("G8", "D3", "I10", "N11"),
        start_cell="N10",
        start_heading="y",
        final_heading="y",
    )
    assert cmds.reset == "reset:N10,y"
    assert cmds.route == "G8 D3* I10 N11,y"
    assert cmds.shoot_cells == ("D3",)


def test_parse_cell_normalizes_leading_zero():
    """前导零格号归一化：'A01' -> 'A1'，保证路线/命令里的格号字面量唯一。"""
    from ag_integrate.car_commands import parse_cell
    assert parse_cell("a01") == "A1"
    assert parse_cell(" D11* ") == "D11"
    with pytest.raises(CommandError):
        parse_cell("A12")


# ---------- 移动靶：命中判定用轨道几何（与 decide_fire 一致） ----------

def _moving_snap(cell: CellRef, is_blue: bool, blue_face: str = "S"):
    """轨道 D8-E8（水平）的移动靶快照，瞬时落点 cell。"""
    from ag_integrate.world_model import TargetRef, WorldSnapshot
    a, b = CellRef(3, 7), CellRef(4, 7)   # D8, E8
    return WorldSnapshot(0, (), (), (), (TargetRef(cell, is_blue, (a, b), blue_face),))


def test_adapt_moving_target_uses_track_geometry():
    """移动靶命中按轨道端点+蓝面朝向判定，不按瞬时落点的格名邻接：
    瞬时落点在 D8，E9 仍是合法开火格（正对 E8 端点、蓝面侧）。"""
    snap = _moving_snap(CellRef(3, 7), True)   # 瞬时落点 D8
    cmds = adapt(snap, ("C9", "E9", "N11"), start_cell="N10", start_heading="y")
    assert cmds.shoot_cells == ("E9",)
    assert "E9*" in cmds.route.split()
    assert "C9*" not in cmds.route.split()     # 非蓝面侧停靠位，不开火


def test_adapt_moving_target_served_once_by_identity():
    """served 以靶身份（轨道）为键：路线经过两个合法开火格只开一次火；
    瞬时落点换格不改变身份、不会重复记。"""
    snap = _moving_snap(CellRef(3, 7), True)
    cmds = adapt(snap, ("D9", "E9", "N11"), start_cell="N10", start_heading="y")
    assert cmds.shoot_cells == ("D9",)
    assert cmds.route == "D9* E9 N11"

    # 同一轨道、瞬时落点换到 E8：身份不变，语义一致
    snap2 = _moving_snap(CellRef(4, 7), True)
    cmds2 = adapt(snap2, ("D9", "E9", "N11"), start_cell="N10", start_heading="y")
    assert cmds2.shoot_cells == ("D9",)


def test_adapt_moving_red_never_fires():
    """移动靶红面可见（友方）：全场禁火，路线无任何 '*'。"""
    snap = _moving_snap(CellRef(3, 7), False)
    cmds = adapt(snap, ("D9", "E9", "N11"), start_cell="N10", start_heading="y")
    assert cmds.shoot_cells == ()
    assert "*" not in cmds.route


def test_sticky_red_consumed_by_fire_guard():
    """红靶消失帧恰好触发规划：粘性红靶仍否决其邻格开火（误击 −5 铁律）。"""
    model = WorldModel(red_veto_ttl_s=5.0)

    def obj(cell, t):
        x, y = cell_to_world(cell)
        return IntelObject(t, x, y, 0.35, 0.35)

    blue = obj(CellRef(3, 1), ObjectType.TARGET_BLUE)   # D2
    red = obj(CellRef(3, 3), ObjectType.TARGET_RED)     # D4
    model.update(IntelFrame(1000, (blue, red)))
    # 红靶消失帧（漏检/遮挡）：D3 邻 D2 蓝 + 邻 D4 粘性红 → 混合风险禁火
    snap = model.update(IntelFrame(2000, (blue,)))
    d = decide_fire(snap, "D3")
    assert d.denied and d.reason == "mixed_targets_risk"
    # TTL 后恢复：纯蓝邻，允许开火
    snap2 = model.update(IntelFrame(9000, (blue,)))
    assert decide_fire(snap2, "D3").allowed
