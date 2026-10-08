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
