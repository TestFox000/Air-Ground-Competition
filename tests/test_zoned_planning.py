"""分区规划硬化测试：部分场域适配 + 中转优化 + 可靠性兜底。

覆盖 2026-10-16 路径规划全面提升的四项要求：
1. 部分场域（无人机只给 1/3 区域）也能正常规划
2. 最小化折返（重复踩格 = 0）与绕弯
3. 高可靠性（缺区自动追加、不可达降级保底、不抛异常）
4. 每区单独可完赛（必经点全覆盖校验）

另含 2026-10-17 审查修复回归：
- S1 压缩删射击格（keep 集并入 decide_fire 放行格）
- S2 降级/补程空快照穿桥墙（降级快照保留 passable/靶体）
- S3 接缝雷过心（缝合后全量 ensure_mine_center_pass）
- M4 朝向只透传首程/终点区末程
- 所有分区路线套 _assert_zoned_route_legal 全路径合法性校验
"""

from __future__ import annotations

import warnings
from math import gcd

import pytest

from ag_integrate.field_frame import LETTERS, CellRef
from ag_integrate.mission_router import (
    _DIRS,
    _MID_BOUNDARY,
    _derive_cross_gates,
    _transit_blocked,
    _zone_of,
    plan_route,
    plan_route_zoned,
)
from ag_integrate.world_model import TargetRef, WorldSnapshot


def _cell(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


def _pos(name: str) -> tuple[int, int]:
    return LETTERS.index(name[0]), int(name[1:]) - 1


def _expand_legs(route: tuple[str, ...]) -> list[str]:
    """压缩路线 -> 途经格序列：逐对格按格线插值（gcd 归一步长）铺开。

    compress_route 只删同向共线中间格，物理轨迹即相邻格号的格心连线——
    展开后的每个单位边才是真实的逐格行驶轨迹。
    """
    out = [route[0]]
    for a, b in zip(route, route[1:]):
        ac, ar = _pos(a)
        bc, br = _pos(b)
        dc, dr = bc - ac, br - ar
        g = gcd(abs(dc), abs(dr)) or 1
        sc, sr = dc // g, dr // g
        assert (sc, sr) in _DIRS, f"航段 {a}->{b} 不沿 8 方向共线（压缩不变量被破坏）"
        cc, rr = ac, ar
        for _ in range(g):
            cc += sc
            rr += sr
            out.append(f"{LETTERS[cc]}{rr + 1}")
    return out


def _assert_zoned_route_legal(
    route: tuple[str, ...],
    snap: WorldSnapshot,
    mid_boundary: int = _MID_BOUNDARY,
) -> None:
    """分区路线全路径合法性校验（公共断言）：逐单位边检查。

    - 途经格不踩障碍 / 靶体（含移动靶轨道）/ 桥洞墙（起终点格免疫，
      与 plan_route 同口径——起终点上的"障碍"只能是误检）；
    - 跨挡板边界的边只许正交、且端点是合法穿越格（走廊/桥洞缺口）；
    - 桥洞隧道格只许东西向进出（竖向 = 撞桥边）。
    """
    blocked = {c.name for c in _transit_blocked(snap)}
    gates = {c.name for c in _derive_cross_gates(snap, mid_boundary)}
    tunnels = {c.name for c in snap.passable}
    cells = _expand_legs(route)
    immune = {route[0], route[-1]}
    for name in cells:
        assert name not in blocked or name in immune, \
            f"路线踩障碍/靶体/桥墙 {name}: {route}"
    for a, b in zip(cells, cells[1:]):
        ac, ar = _pos(a)
        bc, br = _pos(b)
        dc, dr = bc - ac, br - ar
        if dr and ((ar < mid_boundary <= br) or (br < mid_boundary <= ar)):
            assert dc == 0 and (a in gates or b in gates), \
                f"非法穿越挡板 {a}->{b}（非正交或无缺口）: {route}"
        if dr and (a in tunnels or b in tunnels):
            assert False, f"桥洞隧道格竖向进出 {a}->{b}: {route}"


# ---------- 1. 部分场域适配：单区已知也能规划 ----------

def test_single_zone_planning_completes():
    """无人机只侦察到 1 个区（东南），规划应覆盖该区雷并到达终点。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine = _cell("D3")          # 东南区
    snap = WorldSnapshot(0, (mine,), (), (), ())
    route = plan_route_zoned(snap, start, goal, ("东北", "西", "东南"),
                             start_heading="y", final_heading="y")
    assert route[0] == "N11"
    assert route[-1] == "N6"
    assert "D3" in route, f"雷 D3 未被访问: {route}"
    _assert_zoned_route_legal(route, snap)


def test_single_zone_planning_no_backtracking():
    """单区规划路线不应有折返（重复踩格）。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine = _cell("D3")
    snap = WorldSnapshot(0, (mine,), (), (), ())
    route = plan_route_zoned(snap, start, goal, ("东北", "西", "东南"),
                             start_heading="y", final_heading="y")
    dup = len(route) - len(set(route))
    assert dup == 0, f"路线有 {dup} 次折返: {route}"
    _assert_zoned_route_legal(route, snap)


# ---------- 2. 缺区自动追加：不抛 ValueError ----------

def test_missing_zone_auto_appended_no_error():
    """zone_order 未覆盖含任务的区时自动追加，不抛异常。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine = _cell("D3")          # 东南区
    snap = WorldSnapshot(0, (mine,), (), (), ())
    # zone_order 只列了东北，东南有雷但未列入
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        route = plan_route_zoned(snap, start, goal, ("东北",),
                                 start_heading="y", final_heading="y")
    assert any("自动追加" in str(x.message) for x in w), \
        f"应告警自动追加东南区: {[str(x.message) for x in w]}"
    assert "D3" in route
    assert route[-1] == "N6"
    _assert_zoned_route_legal(route, snap)


# ---------- 3. 空区跳过：无任务区不穿越 ----------

def test_empty_zone_skipped():
    """无任务的区（西）应被跳过，路线不白跑。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine_se = _cell("D3")       # 东南
    mine_ne = _cell("K9")       # 东北
    snap = WorldSnapshot(0, (mine_se, mine_ne), (), (), ())
    route = plan_route_zoned(snap, start, goal, ("东北", "西", "东南"),
                             start_heading="y", final_heading="y")
    assert "D3" in route
    assert "K9" in route
    assert route[-1] == "N6"
    # 无折返
    assert len(route) == len(set(route))
    _assert_zoned_route_legal(route, snap)


# ---------- 4. 跨区中转走西区走廊 ----------

def test_cross_zone_transit_uses_corridor():
    """东南↔东北 转场必须经西区走廊格（A/B/C 列 6/7 行）。"""
    start = _cell("N11")        # 东北
    goal = _cell("N6")          # 东北
    mine_se = _cell("D3")       # 东南
    snap = WorldSnapshot(0, (mine_se,), (), (), ())
    route = plan_route_zoned(snap, start, goal, ("东南", "东北"),
                             start_heading="y", final_heading="y")
    # 路线必须经过西区走廊格（A/B/C 列，第 6/7 行）
    corridor_cells = {"A6", "A7", "B6", "B7", "C6", "C7"}
    assert corridor_cells & set(route), \
        f"跨区转场未走西区走廊: {route}"
    _assert_zoned_route_legal(route, snap)


# ---------- 5. 每区完赛校验 ----------

def test_zone_completion_warns_on_missing():
    """区内存有可达必经点但路线遗漏时应告警（兜底校验）。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine = _cell("D3")
    snap = WorldSnapshot(0, (mine,), (), (), ())
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        route = plan_route_zoned(snap, start, goal, ("东南",),
                                 start_heading="y", final_heading="y")
    # 正常情况下 D3 可达，不应有完赛校验告警
    completion_warns = [x for x in w if "完赛校验" in str(x.message)]
    assert not completion_warns, \
        f"可达雷不应触发完赛校验告警: {[str(x.message) for x in completion_warns]}"
    _assert_zoned_route_legal(route, snap)


def test_zone_completion_warns_when_reachable_mine_dropped(monkeypatch):
    """完赛校验正面用例：可达雷被路线遗漏（规划 bug）必须告警——
    校验逻辑本身不能哑火（只会"不漏时不报"的校验等于没有）。"""
    import ag_integrate.mission_router as mr
    real = mr.plan_route

    def drop_mine(snapshot, start, goal, **kw):
        r = real(snapshot, start, goal, **kw)
        if "D3" in r and len(r) > 2:
            i = r.index("D3")
            if 0 < i < len(r) - 1:
                r = r[:i] + r[i + 1:]   # 伪造"可达但被遗漏"
        return r

    monkeypatch.setattr(mr, "plan_route", drop_mine)
    snap = WorldSnapshot(0, (_cell("D3"),), (), (), ())
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        plan_route_zoned(snap, _cell("N11"), _cell("N6"), ("东南",),
                         start_heading="y", final_heading="y")
    assert any("完赛校验" in str(x.message) and "D3" in str(x.message) for x in w), \
        f"可达雷被遗漏必须触发完赛校验告警: {[str(x.message) for x in w]}"


# ---------- 6. 不可达区降级保底 ----------

def test_unreachable_zone_degrades_without_crossing_walls():
    """某区规划失败时降级直奔终点；终点被桥墙真正隔断则返回部分路线。

    （S2 回归，审查实锤场景：mines=(D3) passable=(K6) N11→N6）K 列桥墙
    叠加挡板把 N11 所在象限与 N6 完全隔断，降级直奔物理上也不可达。
    旧实现的降级快照是空的（passable=() → 桥墙不建模、targets=() →
    靶体不建模），降级路线直接从 K 列墙里穿过去——本测试锁死：
    降级路线绝不踩桥墙/靶体，宁可返回部分路线等下拍重规划。
    """
    start = _cell("N11")
    goal = _cell("N6")
    mine = _cell("D3")
    # 用桥洞隧道 K6 隔断东南与东北（桥列墙使两区不连通）
    pas = _cell("K6")
    snap = WorldSnapshot(0, (mine,), (pas,), (), ())
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        route = plan_route_zoned(snap, start, goal, ("东南", "东北"),
                                 start_heading="y", final_heading="y")
    assert any("降级" in str(x.message) for x in w), \
        "应告警降级跳过不可达区"
    # 降级路线不穿桥墙（K 列除隧道格 K6 外全是桥体结构），
    # 全路径合法性由公共断言兜底（旧实现的穿墙路线过不了它）
    assert not any(
        n.startswith("K") and n != "K6" for n in _expand_legs(route)
    ), f"降级路线穿桥墙: {route}"
    _assert_zoned_route_legal(route, snap)


# ---------- 7. 终点不可达返回部分路线 ----------

def test_goal_unreachable_returns_partial():
    """终点完全不可达时返回已规划部分，不抛异常。"""
    start = _cell("N11")
    goal = _cell("A1")
    # 用障碍墙围死起点周围
    obstacles = [
        _cell("M11"), _cell("N10"), _cell("M10"),
    ]
    snap = WorldSnapshot(0, (), (), tuple(obstacles), ())
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        try:
            route = plan_route_zoned(snap, start, goal, ("西",),
                                     start_heading="y", final_heading="y")
        except ValueError:
            pytest.fail("终点不可达不应抛 ValueError，应返回部分路线")
    # 至少返回了起点
    assert route[0] == "N11"
    _assert_zoned_route_legal(route, snap)


# ---------- 8. 多雷多区全局最优（无折返） ----------

def test_multiple_mines_across_zones_no_backtrack():
    """多个雷分布在多个区，路线应全覆盖且无折返。"""
    start = _cell("N11")
    goal = _cell("N6")
    mines = [_cell("D3"), _cell("F4"), _cell("H8"), _cell("J9")]
    snap = WorldSnapshot(0, tuple(mines), (), (), ())
    route = plan_route_zoned(snap, start, goal, ("东南", "东北"),
                             start_heading="y", final_heading="y")
    for m in mines:
        assert m.name in route, f"雷 {m.name} 未被访问"
    assert route[-1] == "N6"
    # 无折返
    dup = len(route) - len(set(route))
    assert dup == 0, f"多雷路线有 {dup} 次折返"
    _assert_zoned_route_legal(route, snap)


# ---------- 9. zone_order=None 退化为全局 TSP ----------

def test_zone_order_none_falls_back_to_global():
    """zone_order=None 时退化为全局 plan_route（向后兼容）。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine = _cell("D3")
    snap = WorldSnapshot(0, (mine,), (), (), ())
    route = plan_route_zoned(snap, start, goal, None,
                             start_heading="y", final_heading="y")
    assert "D3" in route
    assert route[-1] == "N6"
    _assert_zoned_route_legal(route, snap)


# ---------- 10. 未知区名抛 ValueError ----------

def test_unknown_zone_name_raises():
    """zone_order 含未知区名时抛 ValueError（区分于漏覆盖自动追加）。"""
    start = _cell("N11")
    goal = _cell("N6")
    snap = WorldSnapshot(0, (), (), (), ())
    with pytest.raises(ValueError, match="未知区域名"):
        plan_route_zoned(snap, start, goal, ("火星",),
                         start_heading="y", final_heading="y")


# ---------- 11. 桥洞格作为必经点 ----------

def test_passable_cell_is_required():
    """可通行桥洞格（ARROW）应作为必经点被访问。"""
    start = _cell("N11")
    goal = _cell("N7")           # 东北区（与桥洞同区，避免桥列隔断）
    pas = _cell("K7")            # 东北区桥洞（挡板边界行，桥列唯一缺口）
    snap = WorldSnapshot(0, (), (pas,), (), ())
    route = plan_route_zoned(snap, start, goal, ("东北",),
                             start_heading="y", final_heading="y")
    assert "K7" in route, f"桥洞格 K7 未被访问: {route}"
    _assert_zoned_route_legal(route, snap)


# ---------- 12. 区域划分正确性 ----------

def test_zone_classification():
    """三区划分：西=A-C 通长，东南=D-N 第1-6行，东北=D-N 第7-11行。"""
    assert _zone_of(_cell("A1"), _MID_BOUNDARY) == "西"
    assert _zone_of(_cell("C11"), _MID_BOUNDARY) == "西"
    assert _zone_of(_cell("D6"), _MID_BOUNDARY) == "东南"   # row5 < 6
    assert _zone_of(_cell("D7"), _MID_BOUNDARY) == "东北"   # row6 >= 6
    assert _zone_of(_cell("N1"), _MID_BOUNDARY) == "东南"
    assert _zone_of(_cell("N11"), _MID_BOUNDARY) == "东北"


# ---------- 13. S1 回归：压缩不得删掉射击停靠格 ----------

def test_zoned_shoot_stop_survives_compression():
    """射击停靠格是共线中间格时，压缩必须保留——否则 '*' 丢失、打击分
    静默丢失（S1：keep 集并入 decide_fire 放行的路线格，与开火标记同口径）。

    蓝靶 F5 的停靠格 F4 在 A4→N4 的直行航线上：压缩前的路线逐格途经 F4，
    旧实现把它当普通共线格删掉，adapt 找不到停靠格 → 全程不开火。
    """
    from ag_integrate.car_adapter import adapt
    snap = WorldSnapshot(0, (_cell("J4"),), (), (), (TargetRef(_cell("F5"), True),))
    route = plan_route_zoned(snap, _cell("A4"), _cell("N4"), ("东南",),
                             start_heading="x", final_heading="x")
    _assert_zoned_route_legal(route, snap)
    assert "F5" not in route                       # 靶体不可踩
    cmds = adapt(snap, route, "A4", "x", "x")
    assert cmds.shoot_cells == ("F4",), f"射击停靠格被压缩吞掉: {route}"
    assert "F4*" in cmds.route, f"路线文本缺 '*': {cmds.route}"


def test_zoned_shoot_stop_survives_compression_moving_target():
    """移动靶同款（S1 的移动靶轨道几何分支）：轨道端点蓝面侧停靠格
    直行途经时被压缩吞掉 → 必须经 decide_fire 唯一判定口保留。"""
    from ag_integrate.car_adapter import adapt
    # 水平轨道 D5~F5，蓝面朝 S（row+1=row4 侧）：合法开火格 D4/F4
    track = (_cell("D5"), _cell("F5"))
    snap = WorldSnapshot(0, (_cell("J4"),), (), (),
                         (TargetRef(_cell("E5"), True, track=track, blue_face="S"),))
    route = plan_route_zoned(snap, _cell("A4"), _cell("N4"), ("东南",),
                             start_heading="x", final_heading="x")
    _assert_zoned_route_legal(route, snap)
    cmds = adapt(snap, route, "A4", "x", "x")
    assert cmds.shoot_cells, f"移动靶射击停靠格被压缩吞掉: {route}"
    for s in cmds.shoot_cells:
        assert f"{s}*" in cmds.route


# ---------- 14. S3 回归：接缝雷过心穿越点 ----------

def test_zoned_seam_mine_gets_center_pass():
    """接缝雷（上一段尾格 = 下一段首格）是两段路线各自的端点，逐区
    ensure_mine_center_pass 都看不到它——缝合后必须对 full route 再跑一次。

    mines=(B6 西, D3 东南)、A1→N6、('西','东南')：D3 恰为西段终点/东南段
    起点，缝合后 D3 处 90° 转向（D4→D3→E3）→ 沿进入方向补过心穿越点 D2，
    雷格变为直线段中间点（排雷判据：轮中心进入 12cm 圆）。
    """
    snap = WorldSnapshot(0, (_cell("B6"), _cell("D3")), (), (), ())
    route = plan_route_zoned(snap, _cell("A1"), _cell("N6"), ("西", "东南"),
                             start_heading="y", final_heading="y")
    _assert_zoned_route_legal(route, snap)
    assert "D3" in route and "B6" in route
    i = route.index("D3")
    assert route[i + 1] == "D2", \
        f"接缝雷 D3 后未补过心穿越点（沿进入方向的延长格）: {route}"


# ---------- 15. M4 回归：朝向只透传首程/终点区末程 ----------

def test_zoned_headings_only_on_first_and_goal_zone(monkeypatch):
    """start_heading 是车端 reset 断言，仅首程携带；final_heading 仅
    "最后一区且即终点区"的末程携带——中转格被强加终点朝向会逼出
    无谓绕转/朝向降级（M4）。"""
    import ag_integrate.mission_router as mr
    calls: list[dict] = []
    real = mr.plan_route

    def spy(snapshot, start, goal, **kw):
        calls.append(kw)
        return real(snapshot, start, goal, **kw)

    monkeypatch.setattr(mr, "plan_route", spy)
    mines = [_cell("B6"), _cell("D3"), _cell("K9")]   # 西/东南/东北 各一
    snap = WorldSnapshot(0, tuple(mines), (), (), ())
    route = plan_route_zoned(snap, _cell("A1"), _cell("N11"), ("西", "东南", "东北"),
                             start_heading="y", final_heading="y")
    _assert_zoned_route_legal(route, snap)
    assert len(calls) == 3, f"应逐区各一次规划: {len(calls)}"
    # start_heading 仅首程
    assert calls[0]["start_heading"] == "y"
    assert calls[1]["start_heading"] is None
    assert calls[2]["start_heading"] is None
    # final_heading 仅终点区（东北）末程
    assert calls[0]["final_heading"] is None
    assert calls[1]["final_heading"] is None
    assert calls[2]["final_heading"] == "y"


def test_zoned_single_zone_heading_chain_matches_global(monkeypatch):
    """同场景单区分区 == 全局规划的朝向链口径：唯一的区既是首程又是
    终点区末程，透传的朝向参数必须与全局 plan_route 调用一致（M4）。"""
    import ag_integrate.mission_router as mr
    calls: list[dict] = []
    real = mr.plan_route

    def spy(snapshot, start, goal, **kw):
        calls.append(kw)
        return real(snapshot, start, goal, **kw)

    monkeypatch.setattr(mr, "plan_route", spy)
    snap = WorldSnapshot(0, (_cell("D3"),), (), (), ())
    plan_route_zoned(snap, _cell("N11"), _cell("N6"), ("东南",),
                     start_heading="y", final_heading="y")
    assert len(calls) == 1
    assert calls[0]["start_heading"] == "y" and calls[0]["final_heading"] == "y"
