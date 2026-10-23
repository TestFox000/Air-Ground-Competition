# -*- coding: utf-8 -*-
r"""感知管线总装：一帧俯视图 → 车侧可吃的 JSON 报文。

```
       ┌────────────── 输入 ──────────────┐
 rgb 俯视图 + 四角点像素（或已标定的 ArenaRegistration）
       └──────────────┬───────────────────┘
                      ▼
  grid.ArenaRegistration（单应 meters↔px，含手性/残差自检）
                      │
   ┌──────────┬───────┴────────┬───────────────┐
   ▼          ▼                ▼               ▼
objects    car_pose      bridge_arrow    （其余可拓展）
雷/障/靶   格名+yaw       half→block
   │          │                │
   └──────────┴────────┬───────┘
                       ▼
              protocol.build_*  →  /dzlink/drone_to_car
                       （std_msgs/String, depth10, BEST_EFFORT）
```

**设计取舍**

* 每一路都可以独立失败：车没检到、箭头没检到 ⇒ 对应报文**不发**，而不是猜一个。
  车侧对缺失有兜底（``drone.py:18-21``：心跳超时只告警、情报过期返回 None、
  "宁可漏打，不可卡死"）。
* 所有中间结果（含被拒样本与理由）都留在 :class:`PerceptionResult` 里，
  便于"为什么没检到"当场复盘 —— 这比多报一个假阳性重要得多。
* **片级覆盖（t5a）**：``process(..., slices=...)`` 时按
  :mod:`coverage` 的几何先验逐片出 ``field_verify{coverage,slice,scan_id}``，
  让"扫过且为空"有语义。coverage 只由航点几何决定，**与检测结果无关**
  （否则"没检到"会被自动解释成"扫过且空"，正是黑图探索最不能接受的循环论证）。
  缺 ``mine/obstacle/target`` 任一类的判据时 **拒绝**出覆盖（:data:`COVERAGE_CLASSES`），
  因为那时"片内未报"可能是"根本检不了"。
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Mapping, Sequence

from . import protocol as proto
from .bridge_arrow import (ArrowDetection, ArrowSpec, BridgeCall, detect_arrows,
                           infer_bridge_passable)
from .car_pose import CarPose, CarSpec, best_car, detect_car
from .colors import FIELD_DEFAULT, PALETTES, Palette, as_rgb_array
from .coverage import SliceCoverage, SlicePlan, as_plan
from .grid import (CORNER_ORDER, ArenaRegistration, GridSpec,
                   register_from_corners, register_from_named_corners)
from .objects import (FIELD_SPECS, MAP_IMAGE_SPECS, FieldObjects, ObjectClassSpec,
                      detect_field_objects)

__all__ = ["PerceptionConfig", "PerceptionResult", "PerceptionPipeline", "config_for",
           "COVERAGE_CLASSES"]

#: 声称 `coverage` 前**必须**能检的三类目标（缺一类就不能说"扫过且为空"）
COVERAGE_CLASSES = ("mine", "obstacle", "target")


@dataclass(frozen=True)
class PerceptionConfig:
    """一路配置。``palette`` 决定"什么颜色算哪类目标"，其余都是几何判据。"""

    spec: GridSpec = field(default_factory=GridSpec)
    palette: Palette = FIELD_DEFAULT
    object_specs: tuple[ObjectClassSpec, ...] = FIELD_SPECS
    car_spec: CarSpec = field(default_factory=CarSpec)
    arrow_spec: ArrowSpec = field(default_factory=ArrowSpec)
    min_object_confidence: float = 0.30
    min_car_confidence: float = 0.35
    emit_empty_field_verify: bool = False
    seq_start: int = 1

    def blocks_types(self) -> dict:
        return {"object_classes": [s.cls for s in self.object_specs],
                "car_class": self.car_spec.cls, "arrow_class": self.arrow_spec.cls,
                "palette": self.palette.name,
                "provisional": self.palette.allow_provisional}


def config_for(kind: str = "field", **overrides) -> PerceptionConfig:
    """两种开箱预设：``"field"``（实物）与 ``"map_image"``（图纸）。"""
    kind = kind.strip().lower()
    if kind in ("field", "实物"):
        base = PerceptionConfig(palette=FIELD_DEFAULT, object_specs=FIELD_SPECS)
    elif kind in ("map_image", "map", "drawing", "图纸"):
        base = PerceptionConfig(
            palette=PALETTES["map_image"], object_specs=MAP_IMAGE_SPECS,
            car_spec=CarSpec(cls="__none__"),
            arrow_spec=ArrowSpec(cls="__none__", bridge_col="K"),
        )
    else:
        raise ValueError(f"未知预设 {kind!r}（可选 field / map_image）")
    if not overrides:
        return base
    return replace(base, **overrides)


@dataclass
class PerceptionResult:
    registration: ArenaRegistration
    messages: list[dict] = field(default_factory=list)
    objects: FieldObjects | None = None
    cars: list[CarPose] = field(default_factory=list)
    arrows: list[ArrowDetection] = field(default_factory=list)
    bridge_calls: list[BridgeCall] = field(default_factory=list)
    slices: list[SliceCoverage] = field(default_factory=list)
    scan_id: str = ""
    diagnostics: dict = field(default_factory=dict)

    @property
    def car(self) -> CarPose | None:
        return self.cars[0] if self.cars else None

    def json_lines(self) -> list[str]:
        return proto.encode_all(self.messages)

    def to_json(self) -> dict:
        return {
            "registration": self.registration.diagnostics(),
            "objects": None if self.objects is None else self.objects.to_json(),
            "cars": [c.to_json() for c in self.cars],
            "arrows": [a.to_json() for a in self.arrows],
            "bridge": [b.to_json() for b in self.bridge_calls],
            "slices": [s.to_json() for s in self.slices],
            "scan_id": self.scan_id,
            "messages": self.messages,
            "diagnostics": self.diagnostics,
        }


class PerceptionPipeline:
    """把各模块串起来。无状态（每次 ``process`` 都从像素重新算）。"""

    def __init__(self, config: PerceptionConfig | None = None) -> None:
        self.config = config or PerceptionConfig()
        self._seq = int(self.config.seq_start)

    # ---------------- 标定 ----------------
    def register(self, corners_px) -> ArenaRegistration:
        """``corners_px`` 可以是 4 点序列（默认顺序 N11,N1,A1,A11）或标签 dict。"""
        if isinstance(corners_px, Mapping):
            return register_from_named_corners(corners_px, self.config.spec,
                                               order=CORNER_ORDER)
        return register_from_corners(corners_px, self.config.spec, order=CORNER_ORDER)

    # ---------------- 主流程 ----------------
    def process(self, rgb, registration: ArenaRegistration | None = None,
                corners_px=None, *, emit_heartbeat: bool = True,
                slices: SlicePlan | str | None = None,
                scan_id: str | None = None,
                omit_empty_classes: bool = False) -> PerceptionResult:
        """一帧 → 报文。

        ``slices``（t5a）：给 :class:`~.coverage.SlicePlan` / 预设名（``"s3"``/``"t1"``/
        ``"full"``）/ ``"A8>E8;N8>K8"`` 时，**每片出一条** ``field_verify``，
        带 ``coverage{complete,cells,mode}`` + ``slice`` + ``scan_id``；
        不给则走旧路径（产物逐字节不变）。
        """
        cfg = self.config
        arr = as_rgb_array(rgb)
        reg = registration if registration is not None else self.register(corners_px)
        res = PerceptionResult(registration=reg)
        plan = as_plan(slices, cfg.spec)

        cls_available = set(cfg.palette.classes())

        # 1) 雷/障/靶
        want_objects = [s for s in cfg.object_specs if s.cls in cls_available]
        if plan is not None:
            missing = [c for c in COVERAGE_CLASSES
                       if c not in {s.cls for s in want_objects}]
            if missing:
                raise ValueError(
                    f"不能声称覆盖：当前调色板/判据检不了 {missing}（片内未报会被当 ABSENT，"
                    "缺类时那是假证据）。现场先用 --palette-json/--provisional 补齐调色板，"
                    "或去掉 --slices 只发差异。")
        if want_objects:
            res.objects = detect_field_objects(
                arr, reg, cfg.palette, want_objects,
                min_confidence=cfg.min_object_confidence)

        # 2) 车
        if cfg.car_spec.cls in cls_available:
            res.cars = detect_car(arr, reg, cfg.palette, cfg.car_spec)

        # 3) 桥顶箭头
        if cfg.arrow_spec.cls in cls_available:
            res.arrows = detect_arrows(arr, reg, cfg.palette, cfg.arrow_spec)
            res.bridge_calls = infer_bridge_passable(res.arrows, cfg.arrow_spec,
                                                     cfg.spec)

        # 4) 组装报文
        msgs: list[dict] = []
        if emit_heartbeat:
            msgs.append(proto.build_heartbeat(seq=self._next_seq()))
        for call in res.bridge_calls:
            msgs.append(proto.build_bridge_passable(
                call.half, call.block, call.confidence, seq=self._next_seq()))

        absent: dict[str, list[str] | None] = {}
        if plan is None:
            if res.objects is not None:
                if res.objects.targets:
                    msgs.append(proto.build_target_zone(
                        res.objects.targets, spec=cfg.spec, seq=self._next_seq()))
                if (res.objects.mines or res.objects.obstacles
                        or cfg.emit_empty_field_verify):
                    msgs.append(proto.build_field_verify(
                        res.objects.mines, res.objects.obstacles, spec=cfg.spec,
                        seq=self._next_seq()))
        else:
            scan = str(scan_id or proto.make_scan_id())
            reports = plan.report()
            mines_all = res.objects.mines if res.objects is not None else []
            obs_all = res.objects.obstacles if res.objects is not None else []
            tgt_all = res.objects.targets if res.objects is not None else []
            for rep in reports:
                cells = set(rep.cells)
                mines = [b for b in mines_all if b in cells]
                obs = [b for b in obs_all if b in cells]
                tgts = [b for b in tgt_all if b in cells]
                msgs.append(proto.build_field_verify(
                    None if (omit_empty_classes and not mines) else mines,
                    None if (omit_empty_classes and not obs) else obs,
                    spec=cfg.spec,
                    coverage={"cells": list(rep.cells),
                              "complete": rep.complete, "mode": plan.mode},
                    slice=rep.slice.to_json(), scan_id=scan,
                    seq=self._next_seq()))
                if tgts:
                    msgs.append(proto.build_target_zone(
                        tgts, spec=cfg.spec, seq=self._next_seq()))
                absent[rep.slice.id] = proto.coverage_absent_cells(
                    {"cells": list(rep.cells), "complete": rep.complete},
                    mines + obs + tgts)
            res.slices = list(reports)
            res.scan_id = scan
        res.messages = msgs

        diag = {
            "config": cfg.blocks_types(),
            "n_cells_registered": reg.self_check()["n_cells"],
            "objects": len(res.objects.detections) if res.objects else 0,
            "objects_rejected": len(res.objects.rejected) if res.objects else 0,
            "conflicts": list(res.objects.conflicts) if res.objects else [],
            "cars": len(res.cars),
            "arrows": len(res.arrows),
            "arrows_below_gate": [a.block for a in res.arrows
                                  if a.confidence < cfg.arrow_spec.min_conf],
            "bridge_calls": [b.to_json() for b in res.bridge_calls],
            "messages": [m["type"] for m in msgs],
            "validate": {m["type"]: proto.validate_payload(m, cfg.spec) for m in msgs},
            "mirror_car_parse": [proto.mirror_car_side_parse(proto.encode(m))
                                 for m in msgs],
        }
        if plan is not None:
            diag["coverage"] = {
                "scan_id": res.scan_id,
                "mode": plan.mode,
                "corridor_m": plan.corridor_m,
                "model": "格心到航段折线距离 ≤ corridor_m（圆头端点；几何先验，与检测无关）",
                "n_slices": len(res.slices),
                "cells_total": cfg.spec.rows * cfg.spec.cols,
                "cells_union": len(plan.union_cells()),
                "incomplete_slices": [c.slice.id for c in res.slices if not c.complete],
                "cells_per_slice": {c.slice.id: len(c.cells) for c in res.slices},
                "absent_per_slice": absent,
                "omit_empty_classes": bool(omit_empty_classes),
                "provisional_palette": bool(cfg.palette.allow_provisional),
            }
            if cfg.palette.allow_provisional:
                diag["coverage"]["warnings"] = [
                    "调色板含**未实测**的占位颜色（allow_provisional=True）⇒ "
                    "覆盖里的 ABSENT 只是「按占位色没看到」，现场标定（D1）前别据此压雷"]
        res.diagnostics = diag
        return res

    # ---------------- 便捷入口 ----------------
    def detect_car_pose(self, rgb, registration, *, min_confidence: float | None = None):
        """只要车位姿（``block + yaw``）。"""
        conf = (self.config.min_car_confidence if min_confidence is None
                else float(min_confidence))
        return best_car(as_rgb_array(rgb), registration, self.config.palette,
                        self.config.car_spec, min_confidence=conf)

    def _next_seq(self) -> int:
        s = self._seq
        self._seq += 1
        return s
