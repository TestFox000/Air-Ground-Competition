# -*- coding: utf-8 -*-
r"""L1 视觉感知 → **场地地图数据**（t59）：把 t2 的感知输出变成"规划器可直接消费"的结构。

## 它解决什么问题

t2 的管线回答"**这一帧里有哪些格**"；规划器要的是"**这张图的地图长什么样**"——
即 ``dzrobot_v2/config/map.yaml`` 的那套 schema：

```yaml
map:
  name: "第五张新地图 …"
  start: "N10"          # 起点格
  end: "N5"             # 终点格
  barrier_col_from: "D" # 隔板起列
  barrier_row_hi: 6     # 隔板行区间（6/7 之间）
  barrier_row_lo: 7
  mines:     ["C1", …]  # 雷（必压）
  obstacles: ["A1", …]  # 障（水桶，不可通行）
  targets:   ["E5","F5"]# 靶区（跨格）
  bridges:   ["K5","K9"]# 桥洞可通行格
```

本模块把 :class:`FieldMap` 输出成**同一 schema**（``map:`` 块键名/类型逐一对齐），
并把"感知特有的东西"（**每格置信度**、**车位姿**、**蓝面朝向**、**标定方法/来源图**、
被拒样本）放进并列的 ``perception:`` 块 —— 这样：
**车侧可以原样吃 ``map:`` 块，而人/下游还能看到每一条是怎么来的、有多可信。**

## 三件必须说清的事（避免误用）

1. **start / end / 隔板** 不是"看出来的"：图纸上是**印字/印刷**，本模块按参考地图
   带入（``start=``/``end=``/``barrier_*=`` 参数），但**能独立验证它们**：
   黄色标注框（起终点/无人机点）会被检出并作为 ``anchors`` 记录 —— 官方图上实测
   正好落在 ``N10 / N5 / N8 / M2``（与 ``map.yaml`` 自记一致）。
2. **蓝面朝向**：判定的**正确传感器是车端云台相机**（它就在车侧用 ``cls==1`` 门控开火）。
   无人机是**俯视**视角，看到的主要是靶的**顶面**，竖直的蓝/红面只能斜着看到 ⇒ 本模块
   给出"能测就测、测不到就明确 unknown"的估计器，并把**几何限制与量化条件**写进报告。
3. **置信度**是"颜色覆盖率"折算的**相对**可信度，不是概率标定值；阈值建议见
   ``CAPABILITY_L1.md``。
"""

from __future__ import annotations

import json
import math
import pathlib
import re
import time
from dataclasses import asdict, dataclass, field
from typing import Iterable, Mapping, Sequence

import numpy as np

from .colors import Palette, as_rgb_array, blob_masks, connected_components, rgb_to_hsv
from .grid import ArenaRegistration, GridSpec
from .objects import FIELD_SPECS, FieldObjects, ObjectClassSpec

__all__ = [
    "MAP_SCHEMA_KEYS",
    "FieldMap",
    "BlueFaceEstimate",
    "Reconciliation",
    "build_field_map",
    "build_field_map_from_result",
    "estimate_blue_face",
    "parse_map_yaml",
    "reconcile_with_map_yaml",
    "write_field_map",
    "confidence_thresholds",
]

#: ``config/map.yaml`` 的 ``map:`` 块键序（本模块输出的键集合与之对齐）
MAP_SCHEMA_KEYS: tuple[str, ...] = (
    "name", "start", "end", "barrier_col_from", "barrier_row_hi", "barrier_row_lo",
    "mines", "obstacles", "targets", "bridges",
)

#: 各类的颜色→置信度折算与建议阈值（依据 t2/t59 实测覆盖率分布，见 CAPABILITY_L1.md）
_DEFAULT_ACCEPT = {"mine": 0.55, "obstacle": 0.60, "target": 0.55, "bridge": 0.55,
                   "car": 0.45, "arrow": 0.45, "label_box": 0.50}


def confidence_thresholds(cls: str | None = None) -> float | dict[str, float]:
    """建议的"收下"阈值。给类名返回单值，不给返回整表。

    依据：满格色块在真图上的实测覆盖率 0.80~0.91（本卡实测），被拒样本集中在 0.30 以下；
    取"实测下沿再留 25% 余量"⇒ 0.55~0.60。雷/靶的判定还需配合**深色像素占比**（t2 证据 C）。
    """
    return _DEFAULT_ACCEPT[cls] if cls else dict(_DEFAULT_ACCEPT)


# --------------------------------------------------------------------------- #
#  蓝面朝向（靶的蓝/红面）
# --------------------------------------------------------------------------- #
@dataclass
class BlueFaceEstimate:
    """靶的"蓝面是否朝向我们"估计。``facing``∈{toward_us, away, unknown}。"""

    facing: str = "unknown"
    confidence: float = 0.0
    blue_frac: float = 0.0
    red_frac: float = 0.0
    evidence: dict = field(default_factory=dict)

    def to_json(self) -> dict:
        return {"facing": self.facing, "confidence": round(self.confidence, 3),
                "blue_frac": round(self.blue_frac, 4), "red_frac": round(self.red_frac, 4),
                "evidence": dict(self.evidence)}


#: 蓝面 / 红面的 HSV 默认阈值（**未标定**，需现场用 perception.cli calibrate 覆盖）
BLUE_FACE_HSV = (195.0, 0.35, 0.25, 265.0, 1.0, 1.0)
RED_FACE_HSV = (345.0, 0.35, 0.25, 15.0, 1.0, 1.0)


def _face_mask(rgb: np.ndarray, spec: tuple[float, ...]) -> np.ndarray:
    hsv = rgb_to_hsv(rgb)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    h0, s0, v0, h1, s1, v1 = spec
    hm = (h >= h0) & (h <= h1) if h0 <= h1 else ((h >= h0) | (h <= h1))
    return hm & (s >= s0) & (s <= s1) & (v >= v0) & (v <= v1)


def estimate_blue_face(rgb, reg: ArenaRegistration, cells: Sequence[str], *,
                       blue_spec: tuple[float, ...] = BLUE_FACE_HSV,
                       red_spec: tuple[float, ...] = RED_FACE_HSV,
                       min_face_frac: float = 0.02, ratio: float = 2.0,
                       margin_frac: float = 0.0,
                       center_band: float = 0.8,
                       allow_frame_colors: bool = False) -> BlueFaceEstimate:
    """在给定格（靶区）范围内数蓝/红面色像素，判定"蓝面朝向"。

    ★ 判据：``蓝面占比 ≥ min_face_frac`` 且 ``蓝 ≥ ratio×红`` ⇒ ``toward_us``；
    反过来 ⇒ ``away``；两者都低于 ``min_face_frac`` ⇒ ``unknown``（**看不出来就说看不出来**）。

    ⚠️ 两个必须防的假阳性（t59 实测踩到过）：
    1. **场地外框是深蓝的**（官方图 ``(48,80,144)`` ⇒ H≈220°、S≈0.67、V≈0.57，正好落在
       "蓝面"色带里）：所以 ROI **默认不外扩**（``margin_frac=0``），并且蓝/红块的**质心必须
       落在 ROI 的中央 ``center_band`` 内** —— 贴着 ROI 边缘的"蓝条"就是外框，一律不算。
    2. **网格线/标注框**：它们是灰/黄，本来就不在蓝/红色带内。

    ⚠️ 视角限制（务必先读）：无人机**俯视**看到的是靶的顶面；蓝/红面是**竖直面**，
    只有在斜视/低空侧望时才可见。所以本函数对"无人机正俯视帧"通常返回 ``unknown``——
    这不是 bug，而是几何事实。蓝面朝向的**权威来源是车端云台相机**（车侧已用 ``cls==1`` 门控）。
    """
    arr = as_rgb_array(rgb)
    spec = reg.spec
    xs0 = ys0 = 10 ** 9
    xs1 = ys1 = -1
    for b in cells:
        poly = reg.cell_polygon_px(b)
        x0, y0 = poly[:, 0].min(), poly[:, 1].min()
        x1, y1 = poly[:, 0].max(), poly[:, 1].max()
        xs0, ys0, xs1, ys1 = min(xs0, int(x0)), min(ys0, int(y0)), max(xs1, int(x1)), max(ys1, int(y1))
    if xs1 <= xs0 or ys1 <= ys0:
        return BlueFaceEstimate(evidence={"reason": "靶格不可解析"})
    mx = int((xs1 - xs0) * margin_frac)
    my = int((ys1 - ys0) * margin_frac)
    X0, Y0 = max(0, xs0 - mx), max(0, ys0 - my)
    X1, Y1 = min(arr.shape[1], xs1 + mx), min(arr.shape[0], ys1 + my)
    sub = arr[Y0:Y1, X0:X1]
    blue = _face_mask(sub, blue_spec)
    red = _face_mask(sub, red_spec)
    n = float(sub.shape[0] * sub.shape[1]) or 1.0

    def _central_frac(mask: np.ndarray) -> float:
        """只有**质心落在 ROI 中央带**内的最大色块才算"面"，否则回 0（防外框假阳性）。"""
        if not mask.any():
            return 0.0
        comps = connected_components(mask, min_pixels=max(8, int(0.005 * n)))
        if not comps:
            return 0.0
        best = max(comps, key=lambda c: c.area)
        h, w = mask.shape
        cy, cx = best.centroid_yx if hasattr(best, "centroid_yx") else (best.cy, best.cx)
        lo_x, hi_x = w * (1 - center_band) / 2.0, w * (1 + center_band) / 2.0
        lo_y, hi_y = h * (1 - center_band) / 2.0, h * (1 + center_band) / 2.0
        if not (lo_x <= cx <= hi_x and lo_y <= cy <= hi_y):
            return 0.0
        return float(best.area) / n

    bf = _central_frac(blue)
    rf = _central_frac(red)
    ev = {"roi": [X0, Y0, X1, Y1], "n_px": int(n),
          "blue_spec": list(blue_spec), "red_spec": list(red_spec),
          "center_band": center_band, "margin_frac": margin_frac,
          "frame_color_guard": "外框深蓝 H≈220° 也在蓝带内 ⇒ 质心必须居中才算面",
          "note": "阈值未标定（BLUE_FACE_HSV/RED_FACE_HSV 默认值）；俯视帧通常看不竖直面"}
    if bf < min_face_frac and rf < min_face_frac:
        return BlueFaceEstimate("unknown", 0.0, bf, rf,
                                dict(ev, reason="蓝/红面像素都太少（俯视看不到竖直面？）"))
    if bf >= min_face_frac and bf >= ratio * max(rf, 1e-9):
        conf = min(1.0, (bf - min_face_frac) / (3 * min_face_frac))
        return BlueFaceEstimate("toward_us", conf, bf, rf, ev)
    if rf >= min_face_frac and rf >= ratio * max(bf, 1e-9):
        conf = min(1.0, (rf - min_face_frac) / (3 * min_face_frac))
        return BlueFaceEstimate("away", conf, bf, rf, ev)
    return BlueFaceEstimate("unknown", 0.0, bf, rf,
                            dict(ev, reason="蓝/红占比未拉开 ≥2× ⇒ 判不了"))


def motion_budget(speed_mps: float = 0.12, fps: float = 30.0,
                  gsd_m_per_px: float = 0.002) -> dict:
    """移动靶的**拖影/帧间位移预算**（给"蓝面朝向 + 跟踪"能力评估用）。

    0.12 m/s、30 fps ⇒ 4 mm/帧；在 2 mm/px 的地面分辨率下 = **2 px/帧**。
    只要靶面在画面里 ≥20 px 宽，2 px 拖影不改变颜色可分性 ⇒ **瓶颈不是拖影，是视角**。
    """
    per_frame_m = speed_mps / max(1e-9, fps)
    return {"speed_mps": speed_mps, "fps": fps, "gsd_m_per_px": gsd_m_per_px,
            "m_per_frame": round(per_frame_m, 5),
            "px_per_frame": round(per_frame_m / max(1e-9, gsd_m_per_px), 3),
            "note": "拖影 ≤2~3 px 时可忽略；真正决定蓝面可见性的是俯仰几何（能否看到竖直面）"}


# --------------------------------------------------------------------------- #
#  地图结构
# --------------------------------------------------------------------------- #
@dataclass
class FieldMap:
    """一张"场地地图"：``map:`` 块与 ``config/map.yaml`` schema 对齐 + 感知证据块。"""

    spec: GridSpec = field(default_factory=GridSpec)
    name: str = ""
    start: str | None = None
    end: str | None = None
    barrier_col_from: str | None = None
    barrier_row_hi: int = 6
    barrier_row_lo: int = 7
    mines: list[str] = field(default_factory=list)
    obstacles: list[str] = field(default_factory=list)
    targets: list[str] = field(default_factory=list)
    bridges: list[str] = field(default_factory=list)
    # ---- 感知证据块 ----
    confidence: dict[str, dict[str, float]] = field(default_factory=dict)
    cell_confidence: dict[str, float] = field(default_factory=dict)
    car: dict | None = None
    blue_face: BlueFaceEstimate | None = None
    anchors: list[str] = field(default_factory=list)
    rejected: list[dict] = field(default_factory=list)
    provenance: dict = field(default_factory=dict)

    # ---------------- 出图 ----------------
    def to_map_dict(self, *, include_cell_m: bool = False) -> dict:
        """``map:`` 块（键名/类型与 ``config/map.yaml`` 对齐）。"""
        out = {
            "name": self.name,
            "start": self.start,
            "end": self.end,
            "barrier_col_from": self.barrier_col_from,
            "barrier_row_hi": int(self.barrier_row_hi),
            "barrier_row_lo": int(self.barrier_row_lo),
            "mines": sorted(self.mines),
            "obstacles": sorted(self.obstacles),
            "targets": sorted(self.targets),
            "bridges": sorted(self.bridges),
        }
        if include_cell_m:
            out["cell_m"] = float(self.spec.cell_m)
        return out

    def to_json_dict(self) -> dict:
        return {"map": self.to_map_dict(),
                "perception": {
                    "confidence": self.confidence,
                    "cell_confidence": self.cell_confidence,
                    "car": self.car,
                    "blue_face": None if self.blue_face is None else self.blue_face.to_json(),
                    "anchors": sorted(self.anchors),
                    "rejected": self.rejected,
                    "provenance": self.provenance,
                }}

    def to_json_text(self, indent: int = 2) -> str:
        return json.dumps(self.to_json_dict(), ensure_ascii=False, indent=indent)

    def to_yaml_text(self) -> str:
        """**手写** YAML（只用标量/字符串列表，避免引入 PyYAML 依赖）。

        结构固定为 ``map:`` + ``perception:`` 两块；字符串一律双引号，列表内联。
        """
        def q(s) -> str:
            return json.dumps(str(s), ensure_ascii=False)

        def inline(items: Iterable[str]) -> str:
            items = sorted(items)
            return "[" + ", ".join(q(i) for i in items) + "]"

        lines = [
            "# 由 perception.field_map 生成 —— 无人机侧感知出图",
            f"# 来源图: {self.provenance.get('image', 'n/a')}",
            f"# 标定:   {self.provenance.get('registration', 'n/a')}",
            f"# 生成时间: {self.provenance.get('ts', 'n/a')}",
            "# ★ map: 块与 dzrobot_v2/config/map.yaml 同 schema；perception: 块是证据，车侧可忽略",
            "map:",
            f"  name: {q(self.name)}",
            f"  start: {q(self.start)}",
            f"  end: {q(self.end)}",
            f"  barrier_col_from: {q(self.barrier_col_from)}",
            f"  barrier_row_hi: {int(self.barrier_row_hi)}",
            f"  barrier_row_lo: {int(self.barrier_row_lo)}",
            f"  mines: {inline(self.mines)}",
            f"  obstacles: {inline(self.obstacles)}",
            f"  targets: {inline(self.targets)}",
            f"  bridges: {inline(self.bridges)}",
            "perception:",
            f"  cell_m: {float(self.spec.cell_m)}",
        ]
        for cls, per in sorted(self.confidence.items()):
            inner = ", ".join(f"{q(b)}: {v:.3f}" for b, v in sorted(per.items()))
            lines.append(f"  confidence_{cls}: {{{inner}}}")
        if self.car:
            lines.append("  car: {" + ", ".join(f"{q(k)}: {json.dumps(v, ensure_ascii=False)}"
                                                for k, v in self.car.items()) + "}")
        else:
            lines.append("  car: null          # 本帧未检出车顶标记（没有就是没有）")
        if self.blue_face:
            lines.append("  blue_face: {" + ", ".join(
                f"{q(k)}: {json.dumps(v, ensure_ascii=False)}"
                for k, v in self.blue_face.to_json().items()) + "}")
        else:
            lines.append("  blue_face: null")
        lines.append(f"  anchors: {inline(self.anchors)}")
        lines.append(f"  rejected_n: {len(self.rejected)}")
        lines.append(f"  ts: {q(self.provenance.get('ts', 'n/a'))}")
        return "\n".join(lines) + "\n"

    def save_json(self, path) -> None:
        pathlib.Path(path).write_text(self.to_json_text(), encoding="utf-8", newline="\n")

    def save_yaml(self, path) -> None:
        pathlib.Path(path).write_text(self.to_yaml_text(), encoding="utf-8", newline="\n")

    # ---------------- 对账 ----------------
    def reconcile(self, other: Mapping[str, object]) -> "Reconciliation":
        return reconcile_with_map_yaml(self, other)


@dataclass
class Reconciliation:
    """与参考地图逐类对账结果。"""

    per_class: dict[str, dict] = field(default_factory=dict)
    scalars: dict[str, dict] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return all(v["equal"] for v in self.per_class.values())

    @property
    def n_diffs(self) -> int:
        return sum(len(v["missing"]) + len(v["extra"]) for v in self.per_class.values()) \
            + sum(0 if v["equal"] else 1 for v in self.scalars.values())

    def to_json(self) -> dict:
        return {"ok": self.ok, "n_diffs": self.n_diffs,
                "per_class": self.per_class, "scalars": self.scalars}

    def report(self) -> str:
        out = [f"对账：{'逐格一致 ✅' if self.ok else f'存在 {self.n_diffs} 处差异 ❌'}"]
        for cls, v in sorted(self.per_class.items()):
            flag = "✅" if v["equal"] else "❌"
            out.append(f"  {flag} {cls:9s} 感知 {len(v['perceived']):2d} / 真值 {len(v['truth']):2d}"
                       f"  交集 {len(v['both'])}  仅感知 {v['extra']}  仅真值 {v['missing']}")
        for k, v in self.scalars.items():
            flag = "✅" if v["equal"] else "❌"
            out.append(f"  {flag} {k}: 感知 {v['perceived']!r} / 真值 {v['truth']!r}")
        return "\n".join(out)


# --------------------------------------------------------------------------- #
#  构造
# --------------------------------------------------------------------------- #
def build_field_map(objects: FieldObjects | None,
                    reg: ArenaRegistration, *,
                    name: str = "感知出图",
                    start: str | None = None, end: str | None = None,
                    barrier_col_from: str | None = None,
                    barrier_row_hi: int = 6, barrier_row_lo: int = 7,
                    car=None, blue_face: BlueFaceEstimate | None = None,
                    anchors: Sequence[str] = (),
                    bridges: Sequence[str] | None = None,
                    image: str | None = None,
                    accept: Mapping[str, float] | None = None) -> FieldMap:
    """从感知输出装配 :class:`FieldMap`。``objects`` 可为 None（只出车位姿/锚点）。

    ★ ``bridges`` 语义（两种口径都支持）：

    * 传了 ``bridges=`` ⇒ **用它**（实拍口径：桥洞通行格由**桥顶箭头**推得
      —— ``pipeline`` 的 ``bridge_calls``，见 :func:`build_field_map_from_result`）；
    * 不传 ⇒ 用**颜色类 ``bridge``** 检出的格（图纸口径：图纸把"通格"画成绿色方块）。
    """
    spec = reg.spec
    accept = dict(accept or {})
    conf: dict[str, dict[str, float]] = {}
    cell_conf: dict[str, float] = {}
    keep: dict[str, list[str]] = {}
    if objects is not None:
        for det in objects.detections:
            thr = float(accept.get(det.cls, _DEFAULT_ACCEPT.get(det.cls, 0.5)))
            if det.confidence < thr:
                continue
            keep.setdefault(det.cls, []).append(det.block)
            conf.setdefault(det.cls, {})[det.block] = round(float(det.confidence), 3)
            cell_conf[det.block] = round(max(float(det.confidence),
                                             cell_conf.get(det.block, 0.0)), 3)
    car_json = None
    if car is not None:
        car_json = car.to_json() if hasattr(car, "to_json") else dict(car)
        blk = car_json.get("block")
        if blk:
            cell_conf[blk] = round(max(float(car_json.get("confidence", 0.0)),
                                       cell_conf.get(blk, 0.0)), 3)

    def _note_for(cls: str) -> str:
        if objects is None:
            return "无感知输出"
        blocks = keep.get(cls, [])
        if not blocks:
            return "本帧未检出（或全部低于阈值）"
        vals = [conf[cls][b] for b in blocks]
        return f"{len(blocks)} 个格，置信度 {min(vals):.2f}~{max(vals):.2f}"

    bridge_list = list(bridges) if bridges is not None else keep.get("bridge", [])
    if bridges is not None:                     # 箭头推得的格：补一条置信度（来源不同）
        conf.setdefault("bridge", {})
        for b in bridge_list:
            conf["bridge"].setdefault(b, round(float(max(
                (c.confidence for c in getattr(objects, "detections", [])
                 if c.block == b), default=0.5)), 3))
    return FieldMap(
        spec=spec, name=name, start=start, end=end,
        barrier_col_from=barrier_col_from,
        barrier_row_hi=barrier_row_hi, barrier_row_lo=barrier_row_lo,
        mines=keep.get("mine", []), obstacles=keep.get("obstacle", []),
        targets=keep.get("target", []), bridges=bridge_list,
        confidence=conf, cell_confidence=cell_conf, car=car_json,
        blue_face=blue_face, anchors=sorted(set(anchors)),
        rejected=list(objects.rejected) if objects is not None else [],
        provenance={
            "image": image or "n/a",
            "registration": (reg.notes[-1] if reg.notes else "四角点手点"),
            "residual_px": round(float(reg.residual_px), 4),
            "cell_px": [round(v, 2) for v in reg.cell_size_px("G6")],
            "cell_m": float(spec.cell_m),
            "bridges_source": ("箭头推得(bridge_calls)" if bridges is not None
                               else "颜色类 bridge(图纸口径)"),
            "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
            "accept_thresholds": {k: float(v) for k, v in
                                 (accept or _DEFAULT_ACCEPT).items()},
            "counts": {c: len(v) for c, v in sorted(keep.items())},
            "notes": {c: _note_for(c) for c in ("mine", "obstacle", "target", "bridge")},
        })


def build_field_map_from_result(result, *, name: str = "感知出图",
                                start: str | None = None, end: str | None = None,
                                barrier_col_from: str | None = None,
                                barrier_row_hi: int = 6, barrier_row_lo: int = 7,
                                blue_face: BlueFaceEstimate | None = None,
                                image: str | None = None,
                                anchors: Sequence[str] = ()) -> FieldMap:
    """从 :class:`perception.pipeline.PerceptionResult` 装配（实拍最常用入口）。

    桥洞通行格取 **pipeline 的 ``bridge_calls``**（桥顶箭头推得），与图纸口径区分开。
    """
    calls = list(getattr(result, "bridge_calls", []) or ())
    return build_field_map(
        getattr(result, "objects", None), result.registration, name=name,
        start=start, end=end, barrier_col_from=barrier_col_from,
        barrier_row_hi=barrier_row_hi, barrier_row_lo=barrier_row_lo,
        car=(result.cars[0] if getattr(result, "cars", None) else None),
        blue_face=blue_face, image=image, anchors=anchors,
        bridges=[c.block for c in calls] if calls else None)


def write_field_map(fm: FieldMap, out_dir, stem: str = "field_map") -> tuple[pathlib.Path, pathlib.Path]:
    """写 ``<stem>.json`` 与 ``<stem>.yaml``，返回两个路径。"""
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    jp, yp = out / f"{stem}.json", out / f"{stem}.yaml"
    fm.save_json(jp)
    fm.save_yaml(yp)
    return jp, yp


# --------------------------------------------------------------------------- #
#  读参考地图 + 逐类对账
# --------------------------------------------------------------------------- #
def parse_map_yaml(text: str) -> dict:
    """从 ``config/map.yaml`` 的**文本**里抽出 ``map:`` 块的键（不依赖 PyYAML）。

    支持 ``key: [ "A", "B" ]``（列表）与 ``key: "值"`` / ``key: 6``（标量）；
    注释行以 ``#`` 开头（缩进后也一样）不会误匹配。
    """
    out: dict[str, object] = {}
    for key in MAP_SCHEMA_KEYS:
        m = re.search(rf"^\s*{key}:\s*(\[[^\]]*\]|[^\s#][^#\n]*)", text, re.M)
        if not m:
            continue
        raw = m.group(1).strip()
        if raw.startswith("["):
            items = [s.strip().strip('"').strip("'") for s in raw[1:-1].split(",")]
            out[key] = [s for s in items if s]
        else:
            val = raw.strip().strip('"').strip("'")
            out[key] = int(val) if re.fullmatch(r"-?\d+", val) else val
    return out


def reconcile_with_map_yaml(fm: FieldMap, truth: Mapping[str, object]) -> Reconciliation:
    """逐类对账：``truth`` 来自 :func:`parse_map_yaml` 或手写 dict。"""
    rec = Reconciliation()
    for cls, key in (("mine", "mines"), ("obstacle", "obstacles"),
                     ("target", "targets"), ("bridge", "bridges")):
        got = set(getattr(fm, key))
        want = set(truth.get(key) or [])
        rec.per_class[cls] = {
            "perceived": sorted(got), "truth": sorted(want),
            "both": sorted(got & want), "extra": sorted(got - want),
            "missing": sorted(want - got), "equal": got == want,
        }
    for key in ("start", "end", "barrier_col_from", "barrier_row_hi", "barrier_row_lo"):
        if key not in truth:
            continue
        got = getattr(fm, key)
        want = truth[key]
        rec.scalars[key] = {"perceived": got, "truth": want,
                            "equal": str(got) == str(want)}
    return rec
