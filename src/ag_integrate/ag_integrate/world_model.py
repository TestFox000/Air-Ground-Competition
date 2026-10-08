"""世界模型：把解析出的 dzuav 目标按类别汇总为一帧场地态势。

输入 intel_protocol.IntelFrame（世界米制），输出不可变 WorldSnapshot：
    mines     地雷所在格
    passable  可通行桥洞格（ARROW）
    obstacles 障碍格
    targets   敌我靶（格 + 蓝/红面）
CAR / CELL_LABEL 只用于机上定位与展示，不进入态势。
"""

from __future__ import annotations

from dataclasses import dataclass

from .field_frame import CellRef, world_to_cell
from .intel_protocol import IntelFrame, ObjectType


@dataclass(frozen=True)
class TargetRef:
    cell: CellRef          # 检测落点（移动靶=该帧瞬时位置）
    is_blue: bool          # True=敌方蓝面可见 False=友方红面可见
    # 移动靶专用（全场仅一个）：track=轨道两端格，blue_face=蓝面朝向。
    # 轨道水平（两端同行）时 blue_face='S'(朝南)/'N'；垂直时 'E'/'W'。
    # 轨道两端由裁判系统地图确定（每局固定），蓝面朝向由无人机识别/现场标定。
    track: tuple[CellRef, CellRef] | None = None
    blue_face: str | None = None


@dataclass(frozen=True)
class WorldSnapshot:
    timestamp_ms: int
    mines: tuple[CellRef, ...]
    passable: tuple[CellRef, ...]
    obstacles: tuple[CellRef, ...]
    targets: tuple[TargetRef, ...]

    @property
    def has_enemy(self) -> bool:
        return any(t.is_blue for t in self.targets)


class WorldModel:
    """累积/替换最新态势。dzuav 每帧是全场结果，因此采用整帧替换。"""

    def __init__(self) -> None:
        self._snapshot: WorldSnapshot | None = None

    @property
    def snapshot(self) -> WorldSnapshot | None:
        return self._snapshot

    @property
    def ready(self) -> bool:
        return self._snapshot is not None

    def update(self, frame: IntelFrame) -> WorldSnapshot:
        mines: dict[str, CellRef] = {}
        passable: dict[str, CellRef] = {}
        obstacles: dict[str, CellRef] = {}
        targets: list[TargetRef] = []

        for obj in frame.objects:
            try:
                cell = world_to_cell(obj.x, obj.y)
            except ValueError:
                # 出界/无法吸附的点不纳入态势（CAR/CELL_LABEL 也走这条忽略）
                continue

            if obj.type is ObjectType.MINE:
                mines.setdefault(cell.name, cell)
            elif obj.type is ObjectType.ARROW:
                passable.setdefault(cell.name, cell)
            elif obj.type is ObjectType.OBSTACLE:
                obstacles.setdefault(cell.name, cell)
            elif obj.type is ObjectType.TARGET_BLUE:
                targets.append(TargetRef(cell, True))
            elif obj.type is ObjectType.TARGET_RED:
                targets.append(TargetRef(cell, False))
            # CAR / CELL_LABEL：忽略

        self._snapshot = WorldSnapshot(
            frame.timestamp_ms,
            tuple(mines.values()),
            tuple(passable.values()),
            tuple(obstacles.values()),
            tuple(targets),
        )
        return self._snapshot
