"""任务编排：把无人机侦察情报转成地面车的执行顺序。

分值模型（《15空地协同侦排挑战赛》评分表）：
    驶离起点 5 / 回到终点 5 / 排雷 5 每颗（8 颗）/ 桥洞 10 每处（2 处）
    打击敌方 +5 每次计数（最多 3 次 = 15）/ 误击友方 -5（最多 -15）
    单局限时 180 s

策略：以「分值 / 耗时」为收益率的贪心排序，并始终预留回终点的余量。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional

from .grid_map import CellID, FieldGrid

__all__ = ["MissionPlanner", "Task", "TaskType"]


class TaskType(str, Enum):
    LEAVE_START = "leave_start"
    MINE = "mine"
    BRIDGE_A = "bridge_a"
    BRIDGE_B = "bridge_b"
    TARGET = "target"
    GOAL = "goal"


# 各项固定分值与执行开销（秒）
TASK_SCORE: Dict[TaskType, int] = {
    TaskType.LEAVE_START: 5,
    TaskType.MINE: 5,
    TaskType.BRIDGE_A: 10,
    TaskType.BRIDGE_B: 10,
    TaskType.TARGET: 15,
    TaskType.GOAL: 5,
}

TASK_COST_S: Dict[TaskType, float] = {
    TaskType.LEAVE_START: 1.0,
    TaskType.MINE: 2.0,
    TaskType.BRIDGE_A: 5.0,
    TaskType.BRIDGE_B: 5.0,
    TaskType.TARGET: 8.0,
    TaskType.GOAL: 1.0,
}


@dataclass
class Task:
    type: TaskType
    cell: Optional[CellID]
    score: int
    path: List[CellID] = field(default_factory=list)
    travel_s: float = 0.0
    cost_s: float = 0.0

    @property
    def total_s(self) -> float:
        return self.travel_s + self.cost_s

    @property
    def ratio(self) -> float:
        return self.score / max(1e-6, self.total_s)

    def to_dict(self) -> dict:
        return {
            "type": self.type.value,
            "cell": self.cell.name if self.cell else None,
            "score": self.score,
            "path": [c.name for c in self.path],
            "travel_s": round(self.travel_s, 2),
            "cost_s": round(self.cost_s, 2),
        }


@dataclass
class MissionPlanner:
    grid: FieldGrid
    start: CellID
    goal: CellID
    speed_mps: float = 0.55          # 保守速度（最高 1 m/s，含转弯减速）
    budget_s: float = 180.0
    goal_margin_s: float = 12.0      # 预留回终点的安全余量

    mines: List[CellID] = field(default_factory=list)
    bridges: Dict[TaskType, CellID] = field(default_factory=dict)
    target_cell: Optional[CellID] = None

    def set_intel(
        self,
        mines: Optional[List[str]] = None,
        obstacles: Optional[List[str]] = None,
        bridge_a: Optional[str] = None,
        bridge_b: Optional[str] = None,
        target_cell: Optional[str] = None,
    ) -> None:
        """接受无人机下发的情报（格号字符串），更新占据栅格与任务点。"""
        if obstacles:
            self.grid.apply_blocked_names(obstacles)
        self.mines = [CellID.parse(m) for m in (mines or [])]
        self.bridges.clear()
        if bridge_a:
            self.bridges[TaskType.BRIDGE_A] = CellID.parse(bridge_a)
        if bridge_b:
            self.bridges[TaskType.BRIDGE_B] = CellID.parse(bridge_b)
        self.target_cell = CellID.parse(target_cell) if target_cell else None

    # ---------------- 规划 ----------------
    def _build_candidates(self) -> List[Task]:
        tasks: List[Task] = []
        for m in self.mines:
            tasks.append(Task(TaskType.MINE, m, TASK_SCORE[TaskType.MINE]))
        for ttype, cell in self.bridges.items():
            tasks.append(Task(ttype, cell, TASK_SCORE[ttype]))
        if self.target_cell is not None:
            tasks.append(Task(TaskType.TARGET, self.target_cell, TASK_SCORE[TaskType.TARGET]))
        return tasks

    def _cost_to(self, frm: CellID, task: Task) -> Optional[Task]:
        if task.cell is None:
            return task
        path = self.grid.bfs_path(frm, task.cell)
        if path is None:
            return None
        task.path = path
        task.travel_s = self.grid.path_length_m(path) / max(0.05, self.speed_mps)
        task.cost_s = TASK_COST_S[task.type]
        return task

    def _goal_cost(self, frm: CellID) -> float:
        p = self.grid.bfs_path(frm, self.goal)
        if p is None:
            return float("inf")
        return self.grid.path_length_m(p) / max(0.05, self.speed_mps) + TASK_COST_S[TaskType.GOAL]

    def plan(self) -> List[Task]:
        """返回按执行顺序排列的任务列表（不含 LEAVE_START / GOAL 两个端点任务）。"""
        remaining = list(self._build_candidates())
        cur = self.start
        # 出发本身也有开销（起步加速）；不算它，预算会比实际乐观 1 秒
        used = TASK_COST_S[TaskType.LEAVE_START]
        ordered: List[Task] = []

        while remaining:
            feasible: List[Task] = []
            for t in remaining:
                cand = self._cost_to(cur, t)
                if cand is None:
                    continue
                back = self._goal_cost(t.cell or cur)
                if used + cand.total_s + back + self.goal_margin_s > self.budget_s:
                    continue
                feasible.append(cand)
            if not feasible:
                break
            feasible.sort(key=lambda x: (-x.ratio, x.total_s))
            best = feasible[0]
            ordered.append(best)
            used += best.total_s
            cur = best.cell or cur
            remaining = [t for t in remaining if not (t.type == best.type and t.cell == best.cell)]

        return ordered

    def full_plan(self) -> List[Task]:
        """含首尾：LEAVE_START -> ... -> GOAL。"""
        mid = self.plan()
        out = [Task(TaskType.LEAVE_START, self.start, TASK_SCORE[TaskType.LEAVE_START],
                    [self.start], cost_s=TASK_COST_S[TaskType.LEAVE_START])]
        out.extend(mid)
        last = mid[-1].cell if mid else self.start
        gp = self.grid.bfs_path(last, self.goal) or [last, self.goal]
        out.append(
            Task(
                TaskType.GOAL,
                self.goal,
                TASK_SCORE[TaskType.GOAL],
                gp,
                travel_s=self.grid.path_length_m(gp) / max(0.05, self.speed_mps),
                cost_s=TASK_COST_S[TaskType.GOAL],
            )
        )
        return out

    def estimate_score(self, tasks: Optional[List[Task]] = None) -> int:
        tasks = tasks if tasks is not None else self.full_plan()
        return sum(t.score for t in tasks)

    def estimate_time(self, tasks: Optional[List[Task]] = None) -> float:
        tasks = tasks if tasks is not None else self.full_plan()
        return sum(t.total_s for t in tasks)
