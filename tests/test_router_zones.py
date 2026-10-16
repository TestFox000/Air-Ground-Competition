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


def test_zoned_missing_zone_raises():
    """zone_order 漏覆盖含必经点的区：必须报错而不是静默跳过。"""
    from ag_integrate.mission_router import plan_route_zoned
    s = snap(mines=[c("N9")])           # N9 在东北区
    with pytest.raises(ValueError, match="漏覆盖"):
        plan_route_zoned(s, c("A1"), c("A2"), ("东南",))


def test_zoned_none_is_global():
    """zone_order=None 与全局 plan_route 逐格一致（向后兼容）。"""
    from ag_integrate.mission_router import plan_route_zoned
    s = _referee_like()
    a = plan_route_zoned(s, c("N10"), c("N5"), None, start_heading="y", final_heading="y")
    b = plan_route(s, c("N10"), c("N5"), start_heading="y", final_heading="y")
    assert a == b
