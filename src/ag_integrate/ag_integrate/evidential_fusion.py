# -*- coding: utf-8 -*-
"""EvidentialFusion：把 evidential_map（log-odds 证据积累）适配成
WorldSnapshot 生产者 —— 让规划层吃到"置信度加权 + 负证据清除 + 粘性禁火"
的稳定态势，替代"整帧替换"的单帧快照。

背景（2026-10-24 接线）：
    现役链路里规划吃 world_model.snapshot —— 整帧替换语义：丢一帧检测，
    雷就从态势里消失（低置信度目标间歇闪断的"老出问题"根源）。
    evidential_map.py 早在 10-17 就写好了完整方案（正证据=D13 置信度加权、
    负证据=看清未检出可清除幽灵、红靶粘性永不击穿），但从未接线 ——
    直接原因是 0x4001 v1 协议不带置信度（IntelObject 无 conf 字段）。
    本模块 = 协议 v2（0x11，目标段尾 1 字节 conf）落地后的接线层。

语义对齐（与 world_model.update 逐条对照，缺一条就会出规划级差异）：
    - PAPER 落格判定：K 列 = 通行箭头（passable），其余 = 雷（mines）
    - 蓝靶去重；红靶粘性由 evidential_map 的 sticky_kinds 承担（永不击穿）
    - 出界目标不进态势
    - 快照只含 confirmed 目标（logit ≥ confirm_logit 或粘性确认）——
      对齐 MatrixFusion 的 confirm 门槛语义（两次高置信确认、单帧误检不进门）

兼容回退：v1 帧（conf 全 1.0）下本融合退化为"两次确认制"——单帧闪断不再
导致目标消失（仍优于整帧替换），但无置信度分辨力；机上 C++ 发 v2 后全能力生效。
"""
from __future__ import annotations

import time
from dataclasses import dataclass

from .evidential_map import EvidenceParams, EvidentialMap, MapSpot, TrackSpot
from .field_frame import COLS, ROWS, LETTERS, CellRef, world_to_cell
from .intel_protocol import IntelFrame, ObjectType
from .world_model import TargetRef, WorldSnapshot

#: paper 落 K 列 ⇒ 通行箭头；其余 ⇒ 雷（与 world_model.update 同一判定）
_BRIDGE_COL = "K"

_ALL_CELLS: tuple[CellRef, ...] = tuple(
    CellRef(c, r) for c in range(COLS) for r in range(ROWS))


@dataclass
class FusionDecision:
    """一次融合决策的可观测记录（写日志/调试用，不影响数据流）。"""
    mines_confirmed: int = 0
    arrows_confirmed: int = 0
    obstacles_confirmed: int = 0
    targets_confirmed: int = 0
    coverage_frac: float = 0.0


class EvidentialFusion:
    """evidential_map → WorldSnapshot 的适配器。

    负证据来源：无人机航拍全视野 ⇒ 每帧对全场格调用 observe_cells
    （画质恒取 params.min_quality —— 满足"看清"的合法下限；画质精细化
    上报留给 dzuav 后续，不在本层造数据）。
    """

    def __init__(self, params: EvidenceParams | None = None,
                 tracked_kinds: frozenset | None = None):
        self.map = EvidentialMap(params, tracked_kinds)
        self.params = self.map.p
        self._last_t: float | None = None
        self._ts_ms: int = 0
        self._snapshot: WorldSnapshot | None = None
        self.decision = FusionDecision()

    # -------- 输入（与 MatrixFusion.add_frame 同签名，额外收 recv_mono） --------
    def add_frame(self, frame: IntelFrame,
                  recv_mono: float | None = None) -> list[MapSpot | TrackSpot]:
        t = time.monotonic() if recv_mono is None else float(recv_mono)
        self._last_t = t
        self._ts_ms = frame.timestamp_ms
        for obj in frame.objects:
            try:
                cell = world_to_cell(obj.x, obj.y)
            except ValueError:
                continue                      # 出界：与 world_model 同语义（忽略）
            if obj.type not in self.map.tracked:
                continue                      # CAR/CELL_LABEL/桶/起止点不进态势
            self.map.add_positive(cell, obj.type,
                                  getattr(obj, "conf", 1.0), t)
        # 负证据：全场覆盖（航拍全视野，quality 取合法下限）
        self.map.observe_cells(_ALL_CELLS, t, self.params.min_quality)
        self._rebuild(t)
        return self.map.confirmed_only(t)

    # -------- 快照（对齐 world_model.snapshot 形状） --------
    @property
    def snapshot(self) -> WorldSnapshot | None:
        return self._snapshot

    def _rebuild(self, t: float) -> None:
        mines: dict[str, CellRef] = {}
        passable: dict[str, CellRef] = {}
        obstacles: dict[str, CellRef] = {}
        targets: list[TargetRef] = []
        dec = FusionDecision(coverage_frac=self.map.coverage_frac())
        for spot in self.map.confirmed_only(t):
            if isinstance(spot, TrackSpot):
                # 移动靶轨道级：报告格取轨道中点（blue_face 判别归 cfg/识别层）
                cells = spot.cells
                mid = cells[len(cells) // 2]
                targets.append(TargetRef(mid, spot.kind is ObjectType.TARGET_BLUE))
                dec.targets_confirmed += 1
                continue
            cell = spot.cell
            kind = spot.kind
            if kind is ObjectType.PAPER:
                if cell.name[:1] == _BRIDGE_COL:
                    passable.setdefault(cell.name, cell)
                    dec.arrows_confirmed += 1
                else:
                    mines.setdefault(cell.name, cell)
                    dec.mines_confirmed += 1
            elif kind is ObjectType.OBSTACLE:
                obstacles.setdefault(cell.name, cell)
                dec.obstacles_confirmed += 1
            elif kind in (ObjectType.TARGET_BLUE, ObjectType.TARGET_RED):
                targets.append(TargetRef(cell, kind is ObjectType.TARGET_BLUE))
                dec.targets_confirmed += 1
        self._snapshot = WorldSnapshot(self._ts_ms,
                                       tuple(mines.values()),
                                       tuple(passable.values()),
                                       tuple(obstacles.values()),
                                       tuple(targets))
        self.decision = dec

    # -------- MatrixFusion 兼容面（矩阵可视化等既有消费者零改动） --------
    def fused_spots(self) -> list:
        """confirmed 目标（可视化/矩阵渲染；形状对齐 field_matrix.ObjectSpot）。"""
        if self._last_t is None:
            return []
        from .field_matrix import ObjectSpot
        out = []
        for m in self.map.confirmed_only(self._last_t):
            if isinstance(m, MapSpot):
                out.append(ObjectSpot(m.kind, m.cell.col, m.cell.row,
                                      m.cell.col, m.cell.row))
        return out

    def outside_spots(self) -> list:
        return []
