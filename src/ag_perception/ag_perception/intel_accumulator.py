"""机载端情报累积：把逐帧「格号观测」累积成一份可下发的侦察情报。

与车端 ag_mission.field_fusion 的分工：
    * 本模块在 **机载端**：把一帧帧检测结果按格号累积（谁在哪个格）
    * field_fusion 在 **车端**：对已下发的整份情报再做一次多帧投票（防链路误码/偶发误报）

两者都要，是因为噪声来源不同：机载端是单帧视觉误检，车端是链路与融合口径。
"""

from __future__ import annotations

import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Optional

from ag_core.link_protocol import FieldIntel

__all__ = [
    "KIND",
    "IntelAccumulator",
    "DEFAULT_MAX_PER_KIND",
]

DEFAULT_MAX_PER_KIND = {
    "mine": 8,
    "obstacle": 8,
    "bridge": 2,
    "target_blue": 2,
    "target_red": 2,
}


class KIND:
    MINE = "mine"
    OBSTACLE = "obstacle"
    BRIDGE = "bridge"
    TARGET_BLUE = "target_blue"
    TARGET_RED = "target_red"

    ALL = (MINE, OBSTACLE, BRIDGE, TARGET_BLUE, TARGET_RED)


@dataclass
class IntelAccumulator:
    min_votes: int = 3           # 同一格号至少被看到几帧
    ttl_s: float = 20.0          # 观测有效期（无人机绕圈后旧观测应失效）
    max_mines: int = 8
    max_obstacles: int = 8
    max_bridges: int = 2

    _obs: Dict[str, Dict[str, Deque[float]]] = field(
        default_factory=lambda: defaultdict(lambda: defaultdict(deque)), repr=False
    )

    # ---------------- 输入 ----------------
    def observe(self, kind: str, cell: str, now: Optional[float] = None) -> bool:
        if kind not in KIND.ALL:
            raise ValueError(f"未知类别: {kind}")
        if not cell:
            return False
        self._obs[kind][cell].append(time.time() if now is None else now)
        return True

    def observe_many(self, kind: str, cells, now: Optional[float] = None) -> int:
        n = 0
        for c in cells:
            n += bool(self.observe(kind, c, now))
        return n

    def reset(self) -> None:
        self._obs.clear()

    # ---------------- 输出 ----------------
    def _votes(self, kind: str, now: Optional[float]) -> Dict[str, int]:
        """裁剪过期观测后返回 {格号: 票数}，并按票数降序、格号升序排序。"""
        cells = self._obs.get(kind)
        if not cells:
            return {}
        out: Dict[str, int] = {}
        for cell, stamps in cells.items():
            if now is not None:
                while stamps and (now - stamps[0]) > self.ttl_s:
                    stamps.popleft()
            if stamps:
                out[cell] = len(stamps)
        return dict(sorted(out.items(), key=lambda kv: (-kv[1], kv[0])))

    def counts(self, now: Optional[float] = None) -> Dict[str, Dict[str, int]]:
        return {k: self._votes(k, now) for k in KIND.ALL if self._votes(k, now)}

    def snapshot(self, now: Optional[float] = None) -> FieldIntel:
        now = time.time() if now is None else now
        mines_v = self._votes(KIND.MINE, now)
        obst_v = self._votes(KIND.OBSTACLE, now)
        bridge_v = self._votes(KIND.BRIDGE, now)
        blue_v = self._votes(KIND.TARGET_BLUE, now)
        red_v = self._votes(KIND.TARGET_RED, now)

        mines = [c for c, n in mines_v.items() if n >= self.min_votes][: self.max_mines]
        obstacles = [c for c, n in obst_v.items() if n >= self.min_votes][: self.max_obstacles]
        bridges = [c for c, n in bridge_v.items() if n >= self.min_votes][: self.max_bridges]
        # 敌我区是相连两格：取票数更高那一面的前两格
        blue_cells = [c for c, n in blue_v.items() if n >= self.min_votes][:2]
        red_cells = [c for c, n in red_v.items() if n >= self.min_votes][:2]
        blue_score = sum(blue_v.get(c, 0) for c in blue_cells)
        red_score = sum(red_v.get(c, 0) for c in red_cells)

        if blue_score >= red_score and blue_cells:
            face, target_cells = "blue", blue_cells
        elif red_cells:
            face, target_cells = "red", red_cells
        elif blue_cells:
            face, target_cells = "blue", blue_cells
        else:
            face, target_cells = None, []

        return FieldIntel(
            mines=sorted(mines),
            obstacles=sorted(obstacles),
            bridge_a=bridges[0] if bridges else None,
            bridge_b=bridges[1] if len(bridges) > 1 else None,
            target_cells=sorted(target_cells),
            target_face=face,
            ts=now,
            seq=0,  # seq 由发送端维护
        )

    @property
    def is_ready(self) -> bool:
        """是否已有值得下发的情报。

        敌我区情报单独成立：即使一颗雷都没确认，车端也需要知道该不该打。
        """
        st = self.snapshot()
        return bool(st.mines or st.bridge_a or st.bridge_b or st.target_cells)
