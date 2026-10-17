"""规划硬化三项的单测：雷格过心 / 障碍多帧确认 / 发现态靶确认点。

对应 docs/执行层深优化-20261016.md 与 docs/分区规划与感知约束-20261015.md
的"车端规划"分派项。SIL 层（真 executor 轨迹过心 ≤6cm、交界障碍绕行验收）
见 tests/test_sim_tracking.py。
"""

import pytest

from ag_integrate.field_frame import CellRef, cell_to_world
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.world_model import TargetRef, WorldModel, WorldSnapshot
from ag_integrate.mission_router import ensure_mine_center_pass, plan_route
from ag_integrate.fire_guard import decide_fire


def obj(cell, t):
    x, y = cell_to_world(cell)
    return IntelObject(t, x, y, 0.35, 0.35)


def make_snapshot(objects, **kw):
    return WorldModel(**kw).update(IntelFrame(1, tuple(objects)))


def _snap(targets=(), mines=(), obstacles=()):
    """直接构造快照（发现态靶 is_blue=None 无法从协议帧构造）。"""
    return WorldSnapshot(
        1,
        tuple(mines),
        (),
        tuple(obstacles),
        tuple(targets),
    )


# ---------------------------------------------------------------- 雷格过心

def test_center_pass_right_angle_mine_gets_overshoot():
    """90° 拐点雷 B3（南进、东出）→ 沿进入方向补 B4 过心穿越点。"""
    route = ("B1", "B3", "C3")
    out = ensure_mine_center_pass(route, {"B3"}, frozenset())
    assert out == ("B1", "B3", "B4", "C3")


def test_center_pass_straight_mine_untouched():
    """直线途经的雷天然过心，不补点。"""
    route = ("A1", "B1", "C1", "D1")
    out = ensure_mine_center_pass(route, {"B1"}, frozenset())
    assert out == route


def test_center_pass_45deg_corner_untouched():
    """45° 拐点雷：拐角保护（lookahead 钳顶点）自然贴顶点过弯，不补点
    ——补点反而会把转向升级成 135° PIVOT（SIL 实测更糟）。"""
    route = ("A1", "B2", "C2", "D2")       # B2 处对角南进、正东出（45°）
    out = ensure_mine_center_pass(route, {"B2"}, frozenset())
    assert out == route


def test_center_pass_blocked_overshoot_skipped():
    """过心穿越点被堵 → 放弃补点（雷退化为拐点，尽力而为）。"""
    route = ("B1", "B3", "C3")
    blocked = frozenset({CellRef(1, 3)})          # B4
    out = ensure_mine_center_pass(route, {"B3"}, blocked)
    assert out == route


def test_center_pass_diagonal_side_cut_skipped():
    """对角补点腿两侧正交格有障碍（防切角）→ 放弃补点。"""
    route = ("A2", "B3", "A4")              # B3 处 90°，补点 C4，腿 B3→C4 对角
    blocked = frozenset({CellRef(2, 2)})          # C3：B3→C4 的切角侧格
    out = ensure_mine_center_pass(route, {"B3"}, blocked)
    assert out == route


def test_center_pass_chained_mines():
    """连环 90° 拐点雷：前一颗的补点是后一颗的前驱，逐颗各自处理。"""
    route = ("A3", "B3", "B4", "B5", "B6", "C6")
    out = ensure_mine_center_pass(route, {"B3", "B6"}, frozenset())
    # B3 东进南拐补 C3；B6 南进东拐补 B7
    assert out == ("A3", "B3", "C3", "B4", "B5", "B6", "B7", "C6")


def test_center_pass_endpoint_mines_untouched():
    """首/尾格雷无进入/离开方向，不动。"""
    route = ("B1", "C1", "D1")
    out = ensure_mine_center_pass(route, {"B1", "D1"}, frozenset())
    assert out == route


def test_center_pass_no_mines_noop():
    route = ("A1", "B2", "C2")
    assert ensure_mine_center_pass(route, set(), frozenset()) == route
    assert ensure_mine_center_pass(("A1",), {"A1"}, frozenset()) == ("A1",)


def test_center_pass_bridge_column_rejected():
    """补点落在 K 列（桥体 + 车端 BRIDGE_BLOCKS 强制直行保护）→ 拒绝。"""
    route = ("M8", "L8", "L7")      # L8 处西进北拐 90°，补点 K8 落在桥列
    out = ensure_mine_center_pass(route, {"L8"}, frozenset())
    assert out == route


def _corridor_gates():
    """西区走廊 gate 集（与 plan_route 自动推导口径一致）。"""
    return frozenset(CellRef(c, r) for c in range(3) for r in (5, 6))


def test_center_pass_barrier_crossing_rejected():
    """挡板列拐点雷 D7：向北补点 D6 跨挡板（D 列非 gate）→ 必须拒绝补点。

    （K 列的反例被桥列规则先行拦截，此用 D 列单独验证挡板跨界规则。）
    """
    route = ("D8", "D7", "E7")     # D8→D7 北行，D7 处东拐（不过界，合法路线）
    out = ensure_mine_center_pass(route, {"D7"}, frozenset(),
                                  mid_boundary=6,
                                  cross_gates=_corridor_gates())
    assert out == route


def test_center_pass_gate_crossing_allowed():
    """刚过走廊 gate 格的拐点雷 C6：向北补点 C5 不再跨界（同一侧）→ 允许。

    反例自证：若拐点在 C7（跨界那一刻）且下一格在挡板另一侧回拐，
    补点路线必含对角跨界边，会被 _leg_ok 拒掉——跨界补点只在
    "gate 竖向穿越后继续同向直行"的几何下成立。
    """
    route = ("C8", "C7", "C6", "B6")     # C7→C6 经 gate 竖向跨界（合法），C6 处西拐
    out = ensure_mine_center_pass(route, {"C6"}, frozenset(),
                                  mid_boundary=6,
                                  cross_gates=_corridor_gates())
    assert out == ("C8", "C7", "C6", "C5", "B6")


def test_center_pass_tunnel_diagonal_entry_rejected():
    """隧道邻格拐点雷：补点后对角斜进隧道格 K9 = 撞桥边 → 拒绝补点。"""
    route = ("J8", "J9", "K9")     # J9 处南进东拐，补点 J10 后 J10→K9 对角入洞
    out = ensure_mine_center_pass(route, {"J9"}, frozenset(),
                                  mid_boundary=6,
                                  cross_gates=_corridor_gates(),
                                  tunnel_names=frozenset({"K9"}))
    assert out == route


def test_center_pass_knight_geometry_rejected():
    """135° 拐点雷：补点→后继是骑士步（长弦）→ 拒绝补点。"""
    route = ("A3", "B2", "A2")     # d_in=(1,-1) 对角南进，d_out=(-1,0) 西出
    out = ensure_mine_center_pass(route, {"B2"}, frozenset())
    assert out == route


def test_plan_route_inserts_center_pass_for_corner_mine():
    """端到端：拐点雷的路线在雷后紧跟沿进入方向的过心穿越点。

    A1 东行到 C1（雷）后南拐去 C3：C1 处方向东→南，补点 = C1+(1,0) = D1。
    """
    snap = make_snapshot((obj(CellRef(2, 0), ObjectType.MINE),))  # C1
    route = plan_route(snap, CellRef(0, 0), CellRef(2, 2))
    i = route.index("C1")
    assert route[i + 1] == "D1"                          # 过心穿越点紧跟雷格


# ---------------------------------------------------------------- 障碍多帧确认

def test_obstacle_single_frame_ghost_never_confirms():
    """单帧幽灵（confirm=2）：连续 1 帧后消失 → 永不进态势。"""
    m = WorldModel(obstacle_confirm_frames=2)
    ghost = obj(CellRef(5, 5), ObjectType.OBSTACLE)      # F6
    m.update(IntelFrame(1, (ghost,)))
    assert m.snapshot.obstacles == ()
    m.update(IntelFrame(2, ()))                          # 幽灵消失
    assert m.snapshot.obstacles == ()


def test_obstacle_two_frames_confirms_and_hysteresis():
    """真障碍：第 2 帧进入；单帧漏检靠滞回保持；连续 2 帧消失才移出。"""
    m = WorldModel(obstacle_confirm_frames=2)
    real = obj(CellRef(5, 5), ObjectType.OBSTACLE)       # F6
    m.update(IntelFrame(1, (real,)))
    assert m.snapshot.obstacles == ()                    # 第 1 帧未确认
    m.update(IntelFrame(2, (real,)))
    assert [c.name for c in m.snapshot.obstacles] == ["F6"]
    m.update(IntelFrame(3, ()))                          # 漏检 1 帧：滞回保持
    assert [c.name for c in m.snapshot.obstacles] == ["F6"]
    m.update(IntelFrame(4, ()))                          # 连续 2 帧消失：移出
    assert m.snapshot.obstacles == ()


def test_obstacle_confirm_one_is_legacy():
    """confirm=1 精确退化为旧行为：本帧检出即在态势、消失即移出。"""
    m = WorldModel(obstacle_confirm_frames=1)
    real = obj(CellRef(5, 5), ObjectType.OBSTACLE)
    m.update(IntelFrame(1, (real,)))
    assert [c.name for c in m.snapshot.obstacles] == ["F6"]
    m.update(IntelFrame(2, ()))
    assert m.snapshot.obstacles == ()


def test_obstacle_flicker_breaks_consecutive_count():
    """检出→漏检→检出 不算连续（M3）：漏检打断检出计数——注释宣称"连续
    N 帧检出才确认"，旧实现是累计计数，N=2 时闪断序列第 3 帧即误确认。"""
    m = WorldModel(obstacle_confirm_frames=2)
    real = obj(CellRef(5, 5), ObjectType.OBSTACLE)       # F6
    m.update(IntelFrame(1, (real,)))
    m.update(IntelFrame(2, ()))                          # 漏检：断连续
    m.update(IntelFrame(3, (real,)))                     # 重新计 1，不得确认
    assert m.snapshot.obstacles == ()
    m.update(IntelFrame(4, (real,)))                     # 连续第 2 帧：确认
    assert [c.name for c in m.snapshot.obstacles] == ["F6"]


def test_obstacle_confirmed_survives_miss_then_redetect():
    """已确认障碍的滞回不回归（M3 边界）：漏 1 帧后再次检出不清空确认身份
    ——移出只看连续漏检计数，真障碍不会被"漏 1 + 检出 1"闪出态势。"""
    m = WorldModel(obstacle_confirm_frames=2)
    real = obj(CellRef(5, 5), ObjectType.OBSTACLE)
    m.update(IntelFrame(1, (real,)))
    m.update(IntelFrame(2, (real,)))                     # 连续 2 帧：确认
    assert [c.name for c in m.snapshot.obstacles] == ["F6"]
    m.update(IntelFrame(3, ()))                          # 漏检 1 帧：滞回保持
    m.update(IntelFrame(4, (real,)))                     # 再检出：仍在态势
    assert [c.name for c in m.snapshot.obstacles] == ["F6"]
    m.update(IntelFrame(5, ()))
    m.update(IntelFrame(6, ()))                          # 连续 2 帧消失：移出
    assert m.snapshot.obstacles == ()


# ---------------------------------------------------------------- 发现态靶

def test_unknown_target_never_allows_fire():
    """未辨靶（is_blue=None）邻格一律禁火（fail-safe）。"""
    blue = TargetRef(CellRef(3, 1), True)                # D2
    unknown = TargetRef(CellRef(3, 3), None)             # D4
    # 蓝靶 D2 的停靠位 D3 恰邻未辨靶 D4：混合风险 → 拒绝
    assert decide_fire(_snap(targets=(blue, unknown)), "D3").denied
    # 只有未辨靶：其邻格无蓝命中 → 拒绝
    assert decide_fire(_snap(targets=(unknown,)), "D3").denied
    # 对照组：无未辨邻靶时正常放行
    assert not decide_fire(_snap(targets=(blue,)), "D3").denied


def test_unknown_target_gets_confirm_point_in_route():
    """发现态靶的确认点（距靶 1 格正交邻格）成为必经点；靶格不可踩。"""
    unknown = TargetRef(CellRef(3, 1), None)              # D2
    snap = _snap(targets=(unknown,), mines=(CellRef(0, 0),))  # A1 雷
    route = plan_route(snap, CellRef(0, 3), CellRef(5, 3))
    assert "D2" not in route                             # 靶格不可踩
    assert any(n in route for n in ("C2", "E2", "D1", "D3"))  # 确认点必经


def test_resolved_target_replaces_confirm_with_shooting():
    """face 分辨后：蓝靶换停靠射击位（*），未辨靶的确认点只是途经、绝不开火。"""
    from ag_integrate.car_adapter import adapt
    mine = (CellRef(0, 0),)
    start, goal = CellRef(0, 3), CellRef(5, 3)
    snap_unknown = _snap(targets=(TargetRef(CellRef(3, 1), None),), mines=mine)
    snap_blue = _snap(targets=(TargetRef(CellRef(3, 1), True),), mines=mine)
    cmds_unknown = adapt(snap_unknown,
                         plan_route(snap_unknown, start, goal), "A4", "y")
    cmds_blue = adapt(snap_blue,
                      plan_route(snap_blue, start, goal), "A4", "y")
    assert cmds_blue.shoot_cells                        # 蓝靶：停靠射击
    assert not cmds_unknown.shoot_cells                 # 未辨：确认点只途经
    assert "D2" not in cmds_blue.route and "D2" not in cmds_unknown.route
