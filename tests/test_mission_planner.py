import pytest

from ag_core.grid_map import CellID, FieldGrid
from ag_core.mission_planner import MissionPlanner, TaskType


def _planner() -> MissionPlanner:
    grid = FieldGrid()
    return MissionPlanner(grid, CellID.parse("A1"), CellID.parse("N11"), speed_mps=0.55)


def test_full_plan_has_endpoints():
    p = _planner()
    p.set_intel(mines=["C3", "G8", "M9"], obstacles=["E5"], bridge_a="K4", bridge_b="K9", target_cell="D2")
    plan = p.full_plan()
    assert plan[0].type == TaskType.LEAVE_START
    assert plan[-1].type == TaskType.GOAL
    assert plan[-1].cell.name == "N11"


def test_plan_respects_time_budget():
    p = _planner()
    p.set_intel(
        mines=["B2", "C3", "D4", "E5", "F6", "G7", "H8", "M9"],
        obstacles=[],
        bridge_a="K4",
        bridge_b="K9",
        target_cell="D2",
    )
    plan = p.full_plan()
    assert p.estimate_time(plan) <= p.budget_s
    assert p.estimate_score(plan) > 0


def test_all_eight_mines_scheduled_when_time_allows():
    p = _planner()
    p.set_intel(mines=["B2", "C3", "D4", "E5", "F6", "G7", "H8", "I9"], bridge_a="K4")
    plan = p.plan()
    mines = [t for t in plan if t.type == TaskType.MINE]
    assert len(mines) == 8


def test_unreachable_task_is_dropped():
    p = _planner()
    # 封死 A1 的两个唯一出口，使其与外界隔离
    p.grid.set_blocked(CellID(1, 0))
    p.grid.set_blocked(CellID(0, 1))
    p.set_intel(mines=["J12"], bridge_a="K4")
    assert p.plan() == []


def test_obstacles_marked_blocked():
    p = _planner()
    p.set_intel(obstacles=["C3", "H9"])
    assert sorted(p.grid.blocked_cells()) == ["C3", "H9"]


def test_bridge_tasks_carry_ten_points():
    p = _planner()
    p.set_intel(bridge_a="K4", bridge_b="K9")
    plan = p.plan()
    bridges = {t.type: t.score for t in plan if t.type in (TaskType.BRIDGE_A, TaskType.BRIDGE_B)}
    assert bridges == {TaskType.BRIDGE_A: 10, TaskType.BRIDGE_B: 10}


def test_tasks_ordered_by_score_over_time():
    p = _planner()
    p.set_intel(mines=["A2", "B2", "C3"], bridge_a="K4", bridge_b="K9", target_cell="D2")
    plan = p.plan()
    assert len(plan) >= 2
    # 贪心不变式：第一步一定是「从起点看收益率最高」的那个任务
    start_ratios = []
    for t in p._build_candidates():
        cand = p._cost_to(p.start, t)
        if cand is not None:
            start_ratios.append(cand.ratio)
    assert start_ratios
    assert pytest.approx(plan[0].ratio) == max(start_ratios)


def test_path_avoids_obstacles():
    p = _planner()
    p.set_intel(mines=["B2"], obstacles=["A2"])
    plan = p.plan()
    path = plan[0].path
    assert CellID.parse("A2") not in path


def test_task_serialization():
    p = _planner()
    p.set_intel(mines=["B2"], bridge_a="K4")
    d = p.full_plan()[1].to_dict()
    assert set(d) == {"type", "cell", "score", "path", "travel_s", "cost_s"}
