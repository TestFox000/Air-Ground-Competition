# -*- coding: utf-8 -*-
"""分区规划三特性回归（2026-10-15，docs/分区规划与感知约束-20261015.md）：

1. 隧道轴向约束：一格宽桥洞进出必须同行正交直行（阿克曼底盘斜进必撞桥边——
   随机图复盘 J3→K2 斜入教训）；
2. 隧道单向：规则通行方向=桥顶箭头指向，逆向入洞禁边；
3. 穿越惩罚：跨越挡板边界附加代价，促使"一区扫完再转场"；
4. 交界不可达降级告警：必经点被跳过时按功能分区标注（整区跳过≈唯一通道被堵）。
"""
from __future__ import annotations

import pytest

from ag_integrate.field_frame import CellRef, LETTERS
from ag_integrate.mission_router import plan_route
from ag_integrate.world_model import WorldSnapshot


def c(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


def snap(mines=(), passable=(), obstacles=()):
    return WorldSnapshot(0, tuple(mines), tuple(passable), tuple(obstacles), tuple())


def test_tunnel_axis_straight_through():
    """K 列桥体只留 K2 隧道：J2→L2 必须 J2→K2→L2 同行直穿。"""
    s = snap(passable=[c("K2")])
    route = plan_route(s, c("J2"), c("L2"), start_heading="x")
    i = route.index("K2")
    # 进出格必须是同行的 J2/L2（轴向直行），不允许 J3/L3 斜插
    assert {route[i - 1], route[i + 1]} == {"J2", "L2"}


def test_tunnel_direction_oneway():
    """隧道方向=E（西向东）：东向放行，西向逆向禁边。

    K 列整列桥体墙、只留 K2 一个口——逆向时无路可绕，应报不可达。
    """
    s = snap(passable=[c("K2")])
    r_e = plan_route(s, c("J2"), c("L2"), start_heading="x", tunnel_dirs={"K2": "E"})
    assert "K2" in r_e                      # 顺向直穿
    with pytest.raises(ValueError, match="unreachable"):
        plan_route(s, c("L2"), c("J2"), start_heading="x", tunnel_dirs={"K2": "E"})


def test_tunnel_direction_bad_value():
    s = snap()
    with pytest.raises(ValueError, match="tunnel_dirs"):
        plan_route(s, c("A1"), c("A2"), tunnel_dirs={"K2": "X"})


def test_crossing_penalty_reduces_zone_hops():
    """南北各两颗雷、起点终点在南区：惩罚越大，跨界次数越少（或不增）。

    机制：穿越惩罚让"扫完南区再过界"天然优于"南北反复横跳"。
    """
    south = [c("F5"), c("G6")]
    north = [c("F8"), c("G9")]
    s = snap(mines=south + north)

    def crossings(route: tuple[str, ...]) -> int:
        return sum(
            1 for a, b in zip(route, route[1:])
            if a[0] == b[0] and {int(a[1:]), int(b[1:])} == {6, 7}
        )

    r_low = plan_route(s, c("F4"), c("F4"), crossing_penalty=0.0)
    r_high = plan_route(s, c("F4"), c("F4"), crossing_penalty=50.0)
    assert crossings(r_high) <= crossings(r_low)


def test_unreachable_required_warns_with_zone():
    """N9（东北区）被障碍围死：跳过它时必须告警并标注分区。"""
    s = snap(mines=[c("N9")], obstacles=[c("M9"), c("N8"), c("N10")])
    with pytest.warns(RuntimeWarning, match="东北区"):
        route = plan_route(s, c("A1"), c("A2"))
    assert "N9" not in route          # 不可达雷被剔除，路线照常产出


def test_zone_helper_west_corridor():
    """西区走廊格（A~C 列任意行）不归东南/东北区。"""
    from ag_integrate.mission_router import _zone_of
    assert _zone_of(c("A7"), 6) == "西"
    assert _zone_of(c("C1"), 6) == "西"
    assert _zone_of(c("D6"), 6) == "东南"
    assert _zone_of(c("N7"), 6) == "东北"


# ---------------- 分区排序编排（plan_route_zoned） ----------------


def _referee_like():
    """2026-10-15 裁判随机图同款（要素已在渲染复盘中核过）。"""
    mines = [c(n) for n in ["B6", "B10", "D8", "E1", "F5", "H3", "I6", "I8"]]
    obstacles = [c(n) for n in ["B2", "B9", "C1", "D11", "E4", "F9", "H11", "I7"]]
    passable = [c("K2"), c("K8")]
    return snap(mines=mines, passable=passable, obstacles=obstacles)


def test_zoned_order_respected():
    """东北→西→东南：分区顺序必须被遵守，终点区自动收尾。"""
    from ag_integrate.mission_router import plan_route_zoned
    s = _referee_like()
    r = plan_route_zoned(s, c("N10"), c("N5"), ("东北", "西", "东南"),
                         start_heading="y", final_heading="y")
    assert r[0] == "N10" and r[-1] == "N5"
    # 东北区隧道 K8 先于西区雷 B10，西区雷先于东南区雷 E1
    assert r.index("K8") < r.index("B10") < r.index("E1")
    for n in ["B6", "B10", "D8", "E1", "F5", "H3", "I6", "I8", "K2", "K8"]:
        assert n in r          # 分区执行不漏一个必经点


def test_zoned_missing_zone_auto_appends():
    """zone_order 漏覆盖含必经点的区：自动追加到末尾并告警（不抛异常）。

    部分场域下无人机只侦察到 1/3 区域，配置的 zone_order 可能不全；
    自动追加保证不漏必经点，告警提示配置问题。
    """
    from ag_integrate.mission_router import plan_route_zoned
    import warnings
    s = snap(mines=[c("N9")])           # N9 在东北区
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        r = plan_route_zoned(s, c("A1"), c("A2"), ("东南",))
    assert any("自动追加" in str(x.message) for x in w)
    assert "N9" in r                    # 漏覆盖区的必经点仍被执行


def test_zoned_none_is_global():
    """zone_order=None 与全局 plan_route 逐格一致（向后兼容）。"""
    from ag_integrate.mission_router import plan_route_zoned
    s = _referee_like()
    a = plan_route_zoned(s, c("N10"), c("N5"), None, start_heading="y", final_heading="y")
    b = plan_route(s, c("N10"), c("N5"), start_heading="y", final_heading="y")
    assert a == b


# ---------------- 朝向零转向打击点 ----------------


def test_shoot_leg_pivot_priced_and_aligned():
    """朝向零转向机制（单元级锚定）：

    1. 给 face 朝向时，停靠段代价 = 路程 + 原地转身（G6→G5 朝西开火：
       直进 1 步 + 90° 转身 2×0.75 = 2.5）；
    2. 已有便宜正对路径时优先选正对到达（H5→G5 朝西：1 步、零转身、1.0）；
    3. 不给 face 朝向时同段不计转身（1.0）。
    """
    from ag_integrate.mission_router import _route_leg, _ALL_HEADINGS

    cells, h, cost = _route_leg(c("G6"), c("G5"), frozenset(), set(), None, 6,
                                _ALL_HEADINGS, None, 0.75, True, face_idx=4)
    assert cost == pytest.approx(2.5)          # 1 步 + 90° 转身
    assert h == 6                              # 最优到达是直进朝北（转身 90°）

    cells2, h2, cost2 = _route_leg(c("H5"), c("G5"), frozenset(), set(), None, 6,
                                   _ALL_HEADINGS, None, 0.75, True, face_idx=4)
    assert cost2 == pytest.approx(1.0)         # 朝西直达、零转身
    assert h2 == 4                             # 到达朝向即开火朝向

    cells3, h3, cost3 = _route_leg(c("G6"), c("G5"), frozenset(), set(), None, 6,
                                   _ALL_HEADINGS, None, 0.75, True)
    assert cost3 == pytest.approx(1.0)         # 无 face 不加转身


def test_shoot_stop_face_in_full_route():
    """整链路：起点 H5、蓝靶 F5、终点 N1——射击停靠 G5 应朝西正对到达。"""
    from ag_integrate.world_model import TargetRef
    s = WorldSnapshot(0, (), (), (), (TargetRef(c("F5"), True),))
    route = plan_route(s, c("H5"), c("N1"))
    i = route.index("G5")
    assert route[i - 1] == "H5"     # 前驱 H5，朝西正对（到达朝向=开火朝向）


# ---------------- 对角线与步数-姿态平衡（2026-10-17） ----------------


def _moves(route):
    """相邻格位移列表 [(dc, dr), ...]。"""
    return [
        (ord(b[0]) - ord(a[0]), int(b[1:]) - int(a[1:]))
        for a, b in zip(route, route[1:])
    ]


def test_long_offset_uses_max_diagonals():
    """长斜距走对角线：D2→N6（dx=+10, dy=+4）几何最优 = 4 对角 + 6 正交。

    对角步 √2 < 两正交步 2.0，任何距离最优路线对角步数已达几何上限；
    总步数同时取最小值 10（纯正交要 14 步）。
    """
    s = snap()
    route = plan_route(s, c("D2"), c("N6"))
    moves = _moves(route)
    diag = sum(1 for dc, dr in moves if dc and dr)
    assert diag == 4                       # 对角步数取到几何上限
    assert len(moves) == 10                # 步数最少（14-4 换对角）


def test_small_offset_prefers_diagonal():
    """1 格斜偏移（E2→F3，车头朝东起步）：一步对角直达。

    对角 = 1.41 + 45° 转向 0.75 = 2.16；正交绕行 = 2.0 + 90° 转向 1.5 = 3.5。
    即使 turn_45 拉满（姿态极端优先），45°+45°=90° 转向当量不劣于正交
    90° 且距离更短——对角线永远胜出。
    """
    s = snap()
    route = plan_route(s, c("E2"), c("F3"), start_heading="x")
    assert route == ("E2", "F3")
    # turn_45 极端放大（姿态权重拉满）也不改对角选择
    route_hot = plan_route(s, c("E2"), c("F3"), start_heading="x", turn_45=50.0)
    assert route_hot == ("E2", "F3")


def test_turn45_zero_picks_straightest_among_ties():
    """turn_45=0（纯最短路）时同代价形态全平局：必须取转向最少的最平直形态。

    D2→N6 无转向计费时，任何"4 对角 + 6 正交"的排列代价相同（含锯齿形态）；
    平局裁决要求总转向 = 1 个 45° 单位（先直线后斜线，不锯齿）。
    """
    from ag_integrate.mission_router import _route_shape
    s = snap()
    route = plan_route(s, c("D2"), c("N6"), turn_45=0.0)
    turns, steps = _route_shape(route)
    assert steps == 10                      # 步数最少
    assert turns == 1                       # 转向最少：一段直线 + 一段斜线


def test_allow_diagonal_false_forces_orthogonal():
    """allow_diagonal=False：退化为纯正交路线（对角追踪不稳时的安全开关）。"""
    s = snap()
    route = plan_route(s, c("E2"), c("G4"), allow_diagonal=False)
    moves = _moves(route)
    assert all((dc and not dr) or (dr and not dc) for dc, dr in moves)
    assert len(moves) == 4                  # 纯正交步数 = |dx|+|dy|


def test_route_shape_counts_turns_and_steps():
    """_route_shape 单元锚定：直线 0 转向；L 形 90° = 2 单位；对角锯齿最劣。"""
    from ag_integrate.mission_router import _route_shape
    straight = ("A1", "B1", "C1", "D1")
    l_shape = ("A1", "B1", "B2", "B3")
    zigzag = ("A1", "B2", "C1", "D2")       # NE→SE→NE：每次 90° 当量 = 2 单位
    assert _route_shape(straight) == (0, 3)
    assert _route_shape(l_shape) == (2, 3)
    assert _route_shape(zigzag) == (4, 3)   # 锯齿 4 单位 > L 形 2 单位：平局裁决弃锯齿
