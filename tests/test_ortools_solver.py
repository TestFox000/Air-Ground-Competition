"""U1 联合求解器测试：排序+取舍联合最优、血线哲学、降级契约、真实场地 A/B。

覆盖（对齐 docs/全线升级方案-20261018.md U1 验收标准）：
  1. dp 后端核心语义：宽预算全保留 / 紧预算按策略舍弃 / 不可达必经不可行；
  2. 双后端一致性：cp-sat 健康时与 dp 同输入同（得分, 舍弃集, 耗时）；
  3. pipeline 集成（真实场地夹具，4 段视频侦察 field_state.json）：
     启发式 vs 求解器 A/B——耗时与预算内得分不劣；
  4. 血线哲学回归：shooting_only 不舍雷；纯排雷链超预算不自动砍雷；
     求解器关闭时行为与旧逻辑逐位一致。
"""

import json
import warnings
from dataclasses import replace
from pathlib import Path

import pytest

from ag_integrate.field_frame import LETTERS, CellRef, cell_to_world
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.mission_router import plan_route
from ag_integrate.ortools_solver import (
    STATUS_INFEASIBLE,
    STATUS_OPTIMAL,
    VisitTask,
    cp_sat_healthy,
    solve_visit_plan,
)
from ag_integrate.pipeline import build_mission
from ag_integrate.route_time import RouteTimeParams, estimate_seconds
from ag_integrate.world_model import WorldModel


# ---------------------------------------------------------------- 公共构造
def obj(cell, t):
    x, y = cell_to_world(cell)
    return IntelObject(t, x, y, 0.35, 0.35)


def snapshot(objects):
    return WorldModel().update(IntelFrame(1, tuple(objects)))


def cell(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


def _grid_leg(a: str, b: str) -> float:
    """合成几何：格距 ×10s（只给求解器单元测试用，可控可复现）。"""
    ca, cb = cell(a), cell(b)
    return (((cb.col - ca.col) ** 2 + (cb.row - ca.row) ** 2) ** 0.5) * 10.0


TASKS = (
    VisitTask("L2", "mine", 5.0, 1.0),
    VisitTask("K2", "mine", 5.0, 1.0),
    VisitTask("D8", "shoot", 5.0, 4.0),
)


# ---------------------------------------------------------------- dp 单元语义
def test_dp_keeps_all_when_generous():
    p = solve_visit_plan("N10", "N5", TASKS, leg_s=_grid_leg,
                         time_budget_s=600.0, backend="dp")
    assert p.status == STATUS_OPTIMAL
    assert set(p.order) == {"L2", "K2", "D8"} and not p.dropped
    assert p.est_score == 15.0


def test_dp_shoot_only_policy_keeps_mines():
    """血线哲学：shooting_only 下只有射击位可舍，雷全保留。"""
    p = solve_visit_plan("N10", "N5", TASKS, leg_s=_grid_leg,
                         time_budget_s=150.0, drop_policy="shooting_only",
                         backend="dp")
    assert p.status == STATUS_OPTIMAL
    assert p.dropped == ("D8",) and set(p.order) == {"L2", "K2"}
    assert p.est_seconds <= 150.0 and p.est_score == 10.0


def test_dp_any_task_drops_least_efficient():
    """V3 口径：any_task 下低效任务让位（本几何里射击位 D8 远离排雷线）。"""
    p = solve_visit_plan("N10", "N5", TASKS, leg_s=_grid_leg,
                         time_budget_s=150.0, drop_policy="any_task",
                         backend="dp")
    assert p.status == STATUS_OPTIMAL
    # 得分最大化的舍弃集（D8 与 L2 二选一时按效率取舍，期望舍 D8）
    assert "D8" in p.dropped and p.est_score >= 10.0


def test_dp_no_tasks_direct():
    p = solve_visit_plan("N10", "N5", (), leg_s=_grid_leg,
                         time_budget_s=100.0, backend="dp")
    assert p.status == STATUS_OPTIMAL and p.order == () and p.dropped == ()
    assert p.est_seconds == pytest.approx(_grid_leg("N10", "N5"))


def test_dp_mandatory_unreachable_infeasible():
    def leg_inf(a, b):
        return float("inf") if b == "L2" else _grid_leg(a, b)
    p = solve_visit_plan("N10", "N5", (VisitTask("L2", "mine", 5.0, 1.0),),
                         leg_s=leg_inf, time_budget_s=100.0, backend="dp")
    assert p.status == STATUS_INFEASIBLE


def test_drop_policy_validation():
    with pytest.raises(ValueError):
        solve_visit_plan("N10", "N5", TASKS, leg_s=_grid_leg,
                         time_budget_s=100.0, drop_policy="whatever")


# ---------------------------------------------------------------- 双后端一致性
@pytest.mark.skipif(not cp_sat_healthy(), reason="ortools 不可用/不健康（探测不过）")
@pytest.mark.parametrize("budget,policy", [(600.0, "shooting_only"),
                                           (150.0, "shooting_only"),
                                           (150.0, "any_task")])
def test_backends_agree(budget, policy):
    """cp-sat 健康时：与 dp 同（得分, 舍弃集），耗时一致（同精确解）。"""
    dp = solve_visit_plan("N10", "N5", TASKS, leg_s=_grid_leg,
                          time_budget_s=budget, drop_policy=policy, backend="dp")
    cs = solve_visit_plan("N10", "N5", TASKS, leg_s=_grid_leg,
                          time_budget_s=budget, drop_policy=policy, backend="cp-sat")
    assert cs.status == STATUS_OPTIMAL and dp.status == STATUS_OPTIMAL
    assert cs.est_score == dp.est_score
    assert set(cs.dropped) == set(dp.dropped)
    assert cs.est_seconds == pytest.approx(dp.est_seconds, abs=0.02)


# ---------------------------------------------------------------- pipeline 集成
FIXTURE_DIR = (Path(__file__).resolve().parent.parent / "docs" / "成员交接"
               / "yu-binghuan" / "20261018-uav-video-recon" / "视频侦察_20261018")

# 代表性全场雷布（跨两场）+ 双桥门 K4/K9 + 蓝靶——与 tests/test_route_time.py
# test_full_match_route_within_soft_cap 同款已验证配方（桥墙沿 K 列纵向切场，
# 上下段各需一门才连通；单门时 N5 所在右上区不可达属预期诊断）。
_MATCH_MINES = ((0, 1), (3, 10), (6, 0), (1, 8), (12, 2), (13, 5), (9, 7), (5, 9))
_GATES = (CellRef(10, 3), CellRef(10, 8))   # K4 / K9


def _match_snapshot():
    objs = [obj(CellRef(*c), ObjectType.PAPER) for c in _MATCH_MINES]
    objs += [obj(g, ObjectType.PAPER) for g in _GATES]
    objs += [obj(CellRef(3, 1), ObjectType.TARGET_BLUE)]   # D2 敌靶
    return snapshot(tuple(objs))


def _capped_cap(snap, frac: float, **extra) -> float:
    """按满额估计的 frac 收紧软预算，确定性触发超预算路径。"""
    base = build_mission(snap, "N10", "N5", "y", "y", soft_cap_s=10_000.0, **extra)
    return base.est_seconds * frac


def _fixture_snapshots():
    """真实场地夹具（4 段视频侦察 field_state.json）-> WorldSnapshot 列表。

    补充假设第二桥门 K4：单目视频常只拍到一侧箭头（夹具里只有 K10 下段门，
    N5 所在右上区会不可达）——这是测试假设不是侦察结论，A/B 只关心门控行为。
    """
    out = []
    for fs in sorted(FIXTURE_DIR.glob("cam_*/field_state.json")):
        d = json.loads(fs.read_text(encoding="utf-8"))
        objs = [obj(cell(n), ObjectType.PAPER) for n in d.get("mines_video", [])]
        objs += [obj(cell(n), ObjectType.PAPER) for n in d.get("passable_video", [])]
        objs += [obj(CellRef(10, 3), ObjectType.PAPER)]   # K4 假设门
        objs += [obj(cell(n), ObjectType.OBSTACLE) for n in d.get("obstacles_video", [])]
        objs += [obj(cell(n), ObjectType.TARGET_BLUE)
                 for n in d.get("targets_video", {}).get("blue", [])]
        out.append((fs.parent.name, snapshot(tuple(objs))))
    return out


def _score_of(cmds, mine_names):
    """路线口径得分：踩到的雷 ×5 + 射击位 ×5（与 route_time 计时口径一致）。"""
    tokens = set(cmds.route.replace(",", " ").replace("*", " ").split())
    return 5 * (len(tokens & mine_names) + len(cmds.shoot_cells))


@pytest.mark.skipif(not _fixture_snapshots(), reason="场地夹具缺失")
@pytest.mark.parametrize("name, snap", _fixture_snapshots())
def test_solver_ab_real_fields(name, snap):
    """A/B 验收：同预算下求解器耗时与预算内得分均不劣于启发式。"""
    mine_names = {m.name for m in snap.mines}
    try:
        cap = _capped_cap(snap, 0.6)
    except ValueError as exc:
        # 真实侦察噪声（幻影障碍堵死走廊等）导致规划不可达：规划器诊断正确，
        # 本用例只关心门控行为，跳过该夹具。
        pytest.skip(f"夹具规划不可达（{exc}）")
    if _capped_cap(snap, 1.0) <= cap:
        pytest.skip("该夹具未超预算")

    heur = build_mission(snap, "N10", "N5", "y", "y", soft_cap_s=cap)
    solved = build_mission(snap, "N10", "N5", "y", "y", soft_cap_s=cap,
                           use_ortools_solver=True, ortools_backend="dp")
    # 耗时不劣
    assert solved.est_seconds <= heur.est_seconds + 1e-9
    # 预算内得分不劣：都在预算内比得分；启发式爆预算而求解器压进预算 = 求解器胜
    if heur.est_seconds <= cap:
        assert _score_of(solved, mine_names) >= _score_of(heur, mine_names)
    else:
        assert solved.est_seconds <= heur.est_seconds
    # 血线：shooting_only 下不舍雷
    dropped_mines = set(solved.dropped_tasks) & mine_names
    assert not dropped_mines, f"shooting_only 误舍雷：{dropped_mines}"


def test_switch_off_bit_identical_to_legacy():
    """开关关闭 = 旧逻辑逐位一致（dropped_tasks 恒 ()，est 同值）。"""
    s = _match_snapshot()
    cap = _capped_cap(s, 0.6)
    a = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=cap)
    b = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=cap,
                      use_ortools_solver=False)
    assert a == b and a.dropped_tasks == ()
    # 该场景超预算且有射击位：启发式应已砍射击位（回归现行语义）
    assert a.shed_shooting is True


def test_any_task_policy_beats_heuristic_when_mines_are_remote():
    """V3 口径（显式 opt-in）：远雷让位近雷——紧预算下联合求解严格优于启发式。

    预算窗：direct N10→N5 的路由地板（约 45s，场地强制绕行）之上、
    "砍完射击位仍超预算"（约 79s）之下——取满额估计的 65%。

    ⚠ 本用例原先只断言时间。在 leg 代价模型通胀 5.7× 的时期它一直绿着：
    求解器弃 7/8 颗雷 + 射击位、把时间压到 40.9s，"严格优于启发式"。
    这个 cap 下弃 7 雷恰好是真值，所以看不出问题；真正的 25 分损失发生在
    中等预算（cap=0.9，真值只需弃 2 雷）——由下面那条用例守住。
    """
    s = _match_snapshot()
    cap = _capped_cap(s, 0.65)
    mine_names = {m.name for m in s.mines}
    heur = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=cap)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)   # 模型乐观 -> 收紧重解
        solved = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=cap,
                               use_ortools_solver=True, ortools_backend="dp",
                               ortools_drop_policy="any_task")
    assert heur.est_seconds > cap          # 前提：启发式确实压不进预算
    assert solved.est_seconds < heur.est_seconds   # 联合求解严格更优
    assert solved.dropped_tasks            # 确实舍了任务
    assert solved.est_seconds <= cap       # 且压进了预算
    # 这个 cap（0.65×满额 = 49.1s）本身就是极限压缩：真实规划器逐颗试砍的
    # 真值是"砍 7 雷才进预算、最多保 1 雷"，求解器保住了 2 雷（优于贪心真值）。
    # 所以此处只锁"没有比真值更差"；"不得过度弃雷"的判别性守卫在中等预算那条
    # （test_any_task_does_not_gut_the_mine_set_at_moderate_cap，cap=0.9）。
    kept = _mines_on_route(solved, mine_names)
    assert kept >= 1, f"any_task 在 cap={cap:.0f}s 下一颗雷都没保住（真值可保 1）"
    assert kept >= 2, f"实测应保住 2 雷，只剩 {kept}——模型或收紧逻辑回归"


def _mines_on_route(cmds, mine_names) -> int:
    toks = set(cmds.route.replace(",", " ").replace("*", " ").split())
    return len(toks & mine_names)


# ------------------------------------------------- leg 代价模型（F13 / P0）
def test_require_gates_decouples_score_waypoint_from_travel_cost():
    """leg 代价只测移动，不得被"桥门是计分项"绑架。

    B9->A2 两格同在无挡板的 A/B 列，直线 8 格。桥门格（passable）既是
    桥墙开口又是计分必经点（"桥洞通道协同穿梭"每段 10 分），若 leg 计价
    也强制穿门，这一腿要绕到 37 格：东行到 K9 折返、穿挡板、再到 K4 折返。
    """
    s = _match_snapshot()
    fld = replace(s, mines=(), targets=())
    free = plan_route(fld, cell("B9"), cell("A2"), require_gates=False)
    forced = plan_route(fld, cell("B9"), cell("A2"))
    assert len(free) < 12, f"去掉计分绕行后应接近直线 8 格，实得 {len(free)}"
    assert len(forced) > 2 * len(free), (
        f"前提校验：默认（穿门计分）确实绕远，{len(forced)} vs {len(free)}")
    assert "K9" not in free and "K4" not in free, "纯移动代价不该被迫穿门"


def test_require_gates_false_preserves_bridge_connectivity():
    """去掉"必经"语义不得连带去掉"开口"语义：桥墙仍只在门格可穿。

    require_gates=False 若被误实现成忽略 passable，K 列会整列成墙，
    东侧（N10）到西侧直接不可达 —— 那是比原 bug 更严重的回归。
    """
    s = _match_snapshot()
    fld = replace(s, mines=(), targets=())
    route = plan_route(fld, cell("N10"), cell("A2"), require_gates=False)
    assert route, "东->西必须仍可达"
    crossed = [c for c in route if c.startswith("K")]
    assert crossed and set(crossed) <= {"K4", "K9"}, (
        f"只能从门格穿桥，实得 {crossed}")


def test_leg_cost_model_tracks_real_route_within_tolerance():
    """P0 回归守卫：模型全保留估值必须贴着真实连续路线。

    实测（本夹具 8 雷 + 双桥门，N10→N5）：
        require_gates=True （旧）：模型 404s vs 真实 71.5s = 5.7×
        require_gates=False（新）：模型  69.9s vs 真实 71.5s = 0.98×
    真实 recon 夹具（视频侦察_20261018）实测 0.91~0.96×。
    模型偏乐观是已知且可接受的（build_mission 事后用实路由复核 + 收紧重解），
    但偏离超过 1.5× 就说明 leg 代价又被什么东西污染了。
    """
    s = _match_snapshot()
    real = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=10_000.0)
    fld = replace(s, mines=(), targets=())
    names = [m.name for m in s.mines]

    def leg(a, b):
        try:
            r = plan_route(fld, cell(a), cell(b), require_gates=False)
        except ValueError:
            return float("inf")
        return estimate_seconds(tuple(r), params=RouteTimeParams()).seconds

    tasks = [VisitTask(m, "mine", 5.0, RouteTimeParams().mine_slow_s) for m in names]
    tasks += [VisitTask(c, "shoot", 5.0, RouteTimeParams().shoot_stop_s)
              for c in real.shoot_cells]
    p = solve_visit_plan("N10", "N5", tasks, leg_s=leg,
                         time_budget_s=1e9, backend="dp")
    assert p.status == STATUS_OPTIMAL and not p.dropped, "全保留必须是可行解"
    ratio = p.est_seconds / real.est_seconds
    assert 1 / 1.5 <= ratio <= 1.5, (
        f"leg 代价模型偏离真实连续路线 {ratio:.2f}× "
        f"（模型 {p.est_seconds:.0f}s / 真实 {real.est_seconds:.0f}s）")


def test_any_task_does_not_gut_the_mine_set_at_moderate_cap():
    """25 分回归守卫：中等收紧的预算下不得把雷弃光。

    真值（真实规划器逐颗试砍）：cap=0.9×满额 时砍 2 雷即可进预算、保 6 雷。
    旧模型下求解器砍 7 雷 + 射击位，只保 1 雷 = 5 分（应得 30+）。
    """
    s = _match_snapshot()
    mine_names = {m.name for m in s.mines}
    cap = _capped_cap(s, 0.9)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        solved = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=cap,
                               use_ortools_solver=True, ortools_backend="dp",
                               ortools_drop_policy="any_task")
    kept = _mines_on_route(solved, mine_names)
    assert kept >= 6, f"cap={cap:.0f}s 下只保 {kept}/8 雷——过度弃雷（真值可保 6~7）"


# ------------------------------------------------- 记录 == 执行（F6）
def _two_blue_snapshot(collide: bool):
    """两个蓝靶 -> 两个独立射击位。collide=True 时 D10 的停靠格 D11 压着雷。"""
    objs = [obj(CellRef(*c), ObjectType.PAPER) for c in _MATCH_MINES]
    objs += [obj(g, ObjectType.PAPER) for g in _GATES]
    objs += [obj(cell("D2"), ObjectType.TARGET_BLUE)]
    objs += [obj(cell("D10") if collide else cell("B4"), ObjectType.TARGET_BLUE)]
    return snapshot(tuple(objs))


def test_partial_shoot_drop_is_executed_not_just_reported():
    """求解器按站弃射击时，被弃的站必须真的没有 '*'。

    旧实现 shoot_kept = any(...)：只要还剩一个站，snapshot.targets 原样保留，
    adapt 按几何把**全部**站重新标出来 -> 被弃的站照样停靠（时间没省下），
    而 dropped_tasks 又报告说弃了 —— 黑匣子记的是没发生的事。
    """
    s = _two_blue_snapshot(collide=False)
    full = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=10_000.0)
    assert len(full.shoot_cells) == 2, f"前提：需要两个射击位，实得 {full.shoot_cells}"

    cap = full.est_seconds * 0.97          # 只够弃一个站
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        c = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=cap,
                          use_ortools_solver=True, ortools_backend="dp")
    assert c.dropped_tasks, f"前提：cap={cap:.1f} 应触发按站舍弃"
    stars = {t.rstrip("*") for t in c.route.split() if t.endswith("*")}
    assert stars == set(c.shoot_cells), (
        f"命令串 '*'={sorted(stars)} 与 shoot_cells={c.shoot_cells} 不一致")
    for d in c.dropped_tasks:
        assert d not in stars, f"{d} 已报告舍弃却仍带 '*' 停靠"


def test_dropped_tasks_never_claims_an_executed_stop():
    """同格既是雷又是射击位：任务身份按格名塌缩，"弃射击留排雷"弃不掉。

    D11 同时是雷 (3,10) 与 D10 蓝靶的停靠格。求解器弃掉 D11 的射击任务，
    但车反正要去 D11 排雷、停靠照旧 -> 那条舍弃没有效果。记录里必须摘掉，
    否则复盘时"已弃 2 项"对不上实际耗时。此处只锁"记录不得说谎"，
    真正按 (格,任务类型) 区分身份是 ortools_solver 的契约改动，另议。
    """
    s = _two_blue_snapshot(collide=True)
    assert "D11" in {m.name for m in s.mines}, "前提：D11 是雷"
    full = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=10_000.0)
    assert "D11" in full.shoot_cells, f"前提：D11 也是射击位，实得 {full.shoot_cells}"

    cap = full.est_seconds * 0.92
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        c = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=cap,
                          use_ortools_solver=True, ortools_backend="dp")
    stars = {t.rstrip("*") for t in c.route.split() if t.endswith("*")}
    assert stars == set(c.shoot_cells), "'*' 标记必须与 shoot_cells 一致"
    for d in c.dropped_tasks:
        assert d not in stars, f"{d} 报告已弃却仍带 '*'——记录与执行不符"
    if any("另有保留任务" in str(w.message) for w in caught):
        # 塌缩告警出现时，被塌缩的格不得混进 dropped_tasks
        assert not (set(c.dropped_tasks) & stars)


def test_worst_case_solver_planning_speed():
    """P0 回归守卫：求解器路径（含收紧重解）最坏密度单次 build_mission < 6s（x86）。

    与 test_route_time.test_worst_case_planning_speed 同一理由：规划跑在
    180s 比赛计时里、且在 ARM 车端放大。非求解器路径 2026-10-14 曾退化到
    33.7s（x86）→ 车端分钟级。本路径比它多两处开销：leg_s 的 O(n²) 次
    plan_route，以及模型乐观时的第二轮求解 + 第二次重规划。
    leg_s 已按 (a,b) 记忆化，重解复用缓存，所以第二轮只多一次求解与规划。
    实测 x86：dp 2.7s / cp-sat 3.3s（8雷+双桥门+2蓝靶，cap=0.8×满额）。
    """
    import time

    mines = [CellRef(*c) for c in
             ((7, 5), (0, 5), (13, 7), (2, 9), (13, 4), (0, 8), (0, 2), (13, 3))]
    objs = [obj(m, ObjectType.PAPER) for m in mines]
    objs += [obj(CellRef(10, 4), ObjectType.PAPER), obj(CellRef(10, 6), ObjectType.PAPER)]
    objs += [obj(CellRef(2, 6), ObjectType.TARGET_BLUE),
             obj(CellRef(5, 2), ObjectType.TARGET_BLUE)]
    s = snapshot(tuple(objs))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        full = build_mission(s, "N10", "N11", "y", "y", soft_cap_s=10_000.0)
        cap = full.est_seconds * 0.8
        t0 = time.monotonic()
        c = build_mission(s, "N10", "N11", "y", "y", soft_cap_s=cap,
                          use_ortools_solver=True, ortools_backend="dp",
                          ortools_drop_policy="any_task")
        dt = time.monotonic() - t0
    assert c.est_seconds <= cap + 1e-9 or c.dropped_tasks, "要么进预算要么确实弃了任务"
    assert dt < 6.0, f"求解器路径 {dt:.1f}s——性能退化（P0 阈值 6s，x86）"


def test_pure_mine_chain_never_auto_sheds_anything():
    """纯排雷链超预算：不舍任何任务（血线：少排雷是人的决策）。"""
    objs = [obj(CellRef(*c), ObjectType.PAPER) for c in _MATCH_MINES]
    objs += [obj(g, ObjectType.PAPER) for g in _GATES]
    s = snapshot(tuple(objs))
    cap = _capped_cap(s, 0.5)
    cmds = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=cap,
                         use_ortools_solver=True, ortools_backend="dp",
                         ortools_drop_policy="shooting_only")
    assert cmds.dropped_tasks == () and not cmds.shed_shooting
