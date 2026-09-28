"""底盘行进状态机（无 ROS 依赖，可单测）。

仿真模式：按任务预估耗时 eta_s 计时推进，time_scale 用于快进
         （time_scale=10 表示 10 倍速，方便在笔记本上几秒跑完一局）。
实机模式：start() 时把路径下发给底盘，tick() 轮询底盘是否到位。
         底盘协议尚未对接，实机模式下先落日志并回退为计时推进，
         拿到协议后只需实现 _send_path / _poll_arrived。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

__all__ = ["DriveTask", "ChassisLink", "ChassisState"]


@dataclass
class DriveTask:
    """一段待执行的行程。"""

    type: str = ""
    cell: str = ""
    path: List[str] = field(default_factory=list)
    eta_s: float = 0.0
    index: int = 0


@dataclass
class ChassisState:
    status: str = "idle"          # idle | moving | arrived | aborted
    cell: str = ""
    progress: float = 0.0         # 0~1
    elapsed_s: float = 0.0
    note: str = ""


class ChassisLink:
    def __init__(self, time_scale: float = 1.0, simulate: bool = True) -> None:
        self.time_scale = max(1e-3, float(time_scale))
        self.simulate = bool(simulate)
        self.task: Optional[DriveTask] = None
        self.state = ChassisState()

    # ---------------- 控制 ----------------
    def start(self, task: DriveTask) -> bool:
        if self.task is not None and self.state.status == "moving":
            return False  # 正在执行，忽略新的路点
        self.task = task
        self.state = ChassisState(status="moving", cell=task.cell, progress=0.0)
        if not self.simulate:
            self._send_path(task.path)
        return True

    def abort(self, note: str = "aborted") -> None:
        self.task = None
        self.state = ChassisState(status="aborted", note=note)

    # ---------------- 推进 ----------------
    def tick(self, dt: float) -> Optional[str]:
        """推进 dt 秒；本段走完返回目标格号，否则返回 None。

        eta_s <= 0 表示「本来就在位」（比如起点 A1），第一次 tick 即完成，
        否则这类任务会卡住不动，整个计划推进不下去。
        """
        if self.task is None or self.state.status != "moving":
            return None
        self.state.elapsed_s += max(0.0, float(dt)) * self.time_scale
        total = max(1e-6, float(self.task.eta_s))
        self.state.progress = min(1.0, self.state.elapsed_s / total)

        if not self.simulate and not self._poll_arrived():
            return None  # 实机以底盘回报为准
        if self.state.progress < 1.0 and self.task.eta_s > 0:
            return None

        cell = self.task.cell
        self.task = None
        self.state = ChassisState(status="arrived", cell=cell, progress=1.0)
        return cell

    # ---------------- 查询 ----------------
    @property
    def busy(self) -> bool:
        return self.state.status == "moving"

    # ---------------- 实机对接位 ----------------
    def _send_path(self, path: List[str]) -> None:
        """TODO(实机): 把格号路径翻译成底盘指令下发（串口 / CAN / SDK）。

        底盘协议尚未对接，这里只记录一条提示，不影响仿真链路跑通。
        """
        raise NotImplementedError(
            "底盘下发协议未对接：请在 _send_path 中实现，或先用 simulate:=true 跑仿真"
        )

    def _poll_arrived(self) -> bool:
        """TODO(实机): 查询底盘是否已到位。"""
        return False
