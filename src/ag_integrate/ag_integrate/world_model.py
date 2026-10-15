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
    """累积/替换最新态势。dzuav 每帧是全场结果，因此采用整帧替换。

    红靶粘性否决（禁火铁律 fail-safe）：红面（友方）靶单帧漏检/遮挡/
    落格残差过大被丢弃的那一帧，若恰好触发规划就会误击友方。因此见过
    红靶的格子在 red_veto_ttl_s 秒内持续以红靶身份留在态势里参与否决
    ——粘性只增加否决，绝不减少（TTL 用帧时间戳计量，帧停了粘性自然
    冻结在最后已知态势上，方向同样安全）。
    """

    def __init__(self, red_veto_ttl_s: float = 5.0) -> None:
        self._snapshot: WorldSnapshot | None = None
        self.red_veto_ttl_s = float(red_veto_ttl_s)
        # 红靶格 -> 最后一次见到的时刻（秒，取情报帧时间戳）
        self._red_seen: dict[CellRef, float] = {}

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

        # 红靶粘性否决：本帧红靶登记时刻；TTL 内见过但本帧消失的红靶格
        # 补回态势（fail-safe：只增加否决），过期的清出历史表。
        now_s = frame.timestamp_ms / 1000.0
        red_cells = {t.cell for t in targets if not t.is_blue}
        for c in red_cells:
            self._red_seen[c] = now_s
        expired = []
        for c, ts in self._red_seen.items():
            if now_s - ts > self.red_veto_ttl_s:
                expired.append(c)
            elif c not in red_cells:
                targets.append(TargetRef(c, False))
        for c in expired:
            del self._red_seen[c]

        self._snapshot = WorldSnapshot(
            frame.timestamp_ms,
            tuple(mines.values()),
            tuple(passable.values()),
            tuple(obstacles.values()),
            tuple(targets),
        )
        return self._snapshot
