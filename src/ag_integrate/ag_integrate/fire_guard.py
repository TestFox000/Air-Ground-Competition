"""禁火保险：只有“该格是蓝色敌方靶”时才允许停靠开火。

官方车端 det_node 不分红蓝、path_executor 见到 '*' 就开火；
误击红色友方每计数 −5（最多 −15）。本模块是开火前的最后一道独立闸门。
"""

from __future__ import annotations

from dataclasses import dataclass

from .world_model import WorldSnapshot


@dataclass(frozen=True)
class FireDecision:
    allowed: bool
    reason: str

    @property
    def denied(self) -> bool:
        return not self.allowed


def decide_fire(snapshot: WorldSnapshot, shoot_cell: str) -> FireDecision:
    target = shoot_cell.strip().upper()
    for ref in snapshot.targets:
        if ref.cell.name == target:
            if ref.is_blue:
                return FireDecision(True, "enemy_blue")
            return FireDecision(False, "friendly_red")
    return FireDecision(False, "no_target_at_cell")
