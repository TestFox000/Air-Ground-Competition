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
from pathlib import Path

import pytest

from ag_integrate.field_frame import LETTERS, CellRef, cell_to_world
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.ortools_solver import (
    STATUS_INFEASIBLE,
    STATUS_OPTIMAL,
    VisitTask,
    cp_sat_healthy,
    solve_visit_plan,
)
from ag_integrate.pipeline import build_mission
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
    "砍完射击位仍超预算"（约 79s）之下——取满额估计的 65%。"""
    s = _match_snapshot()
    cap = _capped_cap(s, 0.65)
    heur = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=cap)
    solved = build_mission(s, "N10", "N5", "y", "y", soft_cap_s=cap,
                           use_ortools_solver=True, ortools_backend="dp",
                           ortools_drop_policy="any_task")
    assert heur.est_seconds > cap          # 前提：启发式确实压不进预算
    assert solved.est_seconds < heur.est_seconds   # 联合求解严格更优
    assert solved.dropped_tasks            # 确实舍了任务
    assert solved.est_seconds <= cap       # 且压进了预算


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
