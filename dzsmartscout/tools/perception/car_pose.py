# -*- coding: utf-8 -*-
r"""车顶标记检测与位姿估计（格名 + yaw）。

## 假设与输入

* **颜色板假设**：车顶贴一块纯色板（颜色现场标定，见 ``colors.FIELD_DEFAULT`` 的
  ``car`` 规则，默认 provisional）。比赛规则**没有**规定车顶标记（说明书全文无此项），
  所以"看得见车"这件事本身就需要己方加装标记并实测。
* **可选双标记**：再贴一个小色块在车头方向（``nose_cls``）。单块只能给出
  **轴**（yaw mod 180°），双标记才能定出车头。
* 像素点先经单应映到**场地米制系**再算二阶矩，所以 yaw 是**透视校正后**的，
  不是图像角度（无人机斜视时这两者差很多）。

## yaw 语义（与车侧对齐）

``yaw_deg`` = 车头方向在**场地米制系**里的 ``atan2(dy, dx)``（度）：
``0`` = 车头朝 +x（行号减小，朝上半场）；``+90`` = 朝 +y（列号减小，朝图左）；
``±180`` = 朝 −x（行号增大）。这与 ``dzbrain`` 的 yaw 取值方式一致
（``core/geometry.py:153-157 heading_of = atan2(dy, dx)``、``:145-150`` 的 ``,x``/`,y``
后缀语义）。**不含**车体原始 yaw 的 ``+π/2`` 偏移（``RAW_YAW_TO_MAP_YAW_OFFSET`` 是
串口里程计的换算，不作用于地图航向）。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .colors import Blob, Palette, as_rgb_array, blob_masks
from .grid import ArenaRegistration

__all__ = ["CarSpec", "CarPose", "detect_car", "best_car"]


@dataclass(frozen=True)
class CarSpec:
    """车顶标记的参数。"""

    cls: str = "car"
    nose_cls: str | None = None
    min_cell_frac: float = 0.02     # 面积 / 该格像素面积 的下限
    max_cell_frac: float = 1.20     # 上限（超过多半是把桥面/地垫当车了）
    min_pixels: int = 12
    k_open: int = 0
    k_close: int = 1
    yaw_offset_deg: float = 0.0     # 标记安装角补偿：yaw = 轴角 − offset
    expect_yaw_deg: float | None = None   # 有先验朝向时用于解 180° 歧义
    max_nose_dist_m: float = 0.40   # 主标记↔车头标记的最大合理距离


@dataclass
class CarPose:
    """一次车体检测结果。"""

    block: str
    row_idx: int
    col_idx: int
    x_m: float
    y_m: float
    fx: float
    fy: float
    in_arena: bool
    half: str
    yaw_deg: float
    yaw_ambiguous: bool
    yaw_source: str
    confidence: float
    area_px: int
    area_frac: float
    fill: float
    elongation: float
    offset_from_center_m: tuple[float, float]
    evidence: dict = field(default_factory=dict)

    def to_json(self) -> dict:
        return {
            "block": self.block, "yaw_deg": round(self.yaw_deg, 1),
            "confidence": round(self.confidence, 3),
            "x_m": round(self.x_m, 3), "y_m": round(self.y_m, 3),
            "offset_from_center_m": [round(v, 3) for v in self.offset_from_center_m],
            "half": self.half, "yaw_source": self.yaw_source,
            "yaw_ambiguous": self.yaw_ambiguous,
            "area_px": self.area_px, "area_frac": round(self.area_frac, 3),
            "fill": round(self.fill, 3), "elongation": round(self.elongation, 3),
        }


def _axis_angle_meters(pts_m: np.ndarray) -> tuple[float, float, tuple[float, float],
                                                  tuple[float, float]]:
    """场地米制点的 ``(轴角_deg∈[0,180), 各向异性, 质心, 主轴单位向量)``。"""
    c = pts_m.mean(axis=0)
    d = pts_m - c
    mxx = float((d[:, 0] ** 2).mean())
    myy = float((d[:, 1] ** 2).mean())
    mxy = float((d[:, 0] * d[:, 1]).mean())
    theta = 0.5 * np.arctan2(2.0 * mxy, (mxx - myy))
    tr = mxx + myy
    det = mxx * myy - mxy * mxy
    disc = max(0.0, tr * tr / 4.0 - det)
    lam_max = tr / 2.0 + np.sqrt(disc)
    lam_min = tr / 2.0 - np.sqrt(disc)
    aniso = 0.0 if lam_max <= 1e-15 else float(max(0.0, 1.0 - max(0.0, lam_min) / lam_max))
    u = (float(np.cos(theta)), float(np.sin(theta)))
    return float(np.degrees(theta) % 180.0), aniso, (float(c[0]), float(c[1])), u


def _wrap180(deg: float) -> float:
    return (float(deg) + 180.0) % 360.0 - 180.0


def _blob_points_meters(blob: Blob, reg: ArenaRegistration) -> np.ndarray:
    """``Blob.pixels``（(y,x)）→ 场地米制 ``(N,2)``。"""
    if blob.pixels is None:
        raise ValueError("Blob 未保留像素（want_pixels=False）")
    uv = blob.pixels[:, ::-1].astype(np.float64)      # (y,x) -> (u,v)
    return reg.px_to_meters(uv)


def detect_car(rgb, reg: ArenaRegistration, palette: Palette,
               spec: CarSpec | None = None) -> list[CarPose]:
    """返回**按置信度降序**的车体候选（通常只有 1 个）。"""
    spec = spec or CarSpec()
    arr = as_rgb_array(rgb)
    masks = palette.masks(arr)
    if spec.cls not in masks:
        return []
    blobs = blob_masks(masks[spec.cls], min_pixels=spec.min_pixels,
                       k_open=spec.k_open, k_close=spec.k_close, rgb=arr)
    nose_blobs: list[Blob] = []
    if spec.nose_cls and spec.nose_cls in masks:
        nose_blobs = blob_masks(masks[spec.nose_cls],
                                min_pixels=max(4, spec.min_pixels // 2),
                                k_open=0, k_close=1, rgb=arr)

    out: list[CarPose] = []
    for blob in blobs:
        pts = _blob_points_meters(blob, reg)
        centroid = (float(pts[:, 0].mean()), float(pts[:, 1].mean()))
        hit = reg.meters_to_hit(*centroid)
        cell_area = reg.cell_area_px(hit.block)
        area_frac = blob.area / cell_area if cell_area > 0 else 0.0
        if not (spec.min_cell_frac <= area_frac <= spec.max_cell_frac):
            continue
        axis_deg, aniso, _, axis_u = _axis_angle_meters(pts)

        yaw_deg, ambiguous, source = axis_deg, True, "pca"
        # 双标记：用**主板主轴**定向（长基线 ⇒ 精度高），车头标记只负责决定正反。
        # 只用"主板质心→车头质心"连线的话，小标记的量化误差在短板上会放大到几度。
        if nose_blobs:
            best = None
            for nb in nose_blobs:
                npts = _blob_points_meters(nb, reg)
                nc = (float(npts[:, 0].mean()), float(npts[:, 1].mean()))
                dist = float(np.hypot(nc[0] - centroid[0], nc[1] - centroid[1]))
                if dist < 1e-6 or dist > spec.max_nose_dist_m:
                    continue
                if best is None or dist < best[0]:
                    best = (dist, nc)
            if best is not None:
                _, nc = best
                vx, vy = nc[0] - centroid[0], nc[1] - centroid[1]
                if aniso >= 0.25:
                    if axis_u[0] * vx + axis_u[1] * vy < 0.0:
                        axis_u = (-axis_u[0], -axis_u[1])
                    yaw_deg = float(np.degrees(np.arctan2(axis_u[1], axis_u[0])))
                    source = "board_axis+nose"
                else:
                    yaw_deg = float(np.degrees(np.arctan2(vy, vx)))
                    source = "nose"
                ambiguous = False
        if ambiguous and spec.expect_yaw_deg is not None:
            want = np.radians(spec.expect_yaw_deg)
            c0 = np.radians(yaw_deg)
            if abs(_wrap180(np.degrees(c0 - want))) > 90.0:
                yaw_deg = _wrap180(yaw_deg + 180.0)
        yaw_deg = _wrap180(yaw_deg - spec.yaw_offset_deg)

        # ---- 置信度：面积落在量程中部 + 板明显各向异性 + 不歧义 ----
        span = max(1e-9, spec.max_cell_frac - spec.min_cell_frac)
        pos = (area_frac - spec.min_cell_frac) / span
        conf_area = float(min(1.0, max(0.0, 1.0 - abs(pos - 0.5) * 2.0 * 0.75)))
        conf_aniso = float(min(1.0, aniso / 0.45))
        conf = 0.55 * conf_area + 0.45 * conf_aniso
        if ambiguous:
            conf *= 0.5
        out.append(CarPose(
            block=hit.block, row_idx=hit.row_idx, col_idx=hit.col_idx,
            x_m=hit.x_m, y_m=hit.y_m, fx=hit.fx, fy=hit.fy,
            in_arena=hit.in_arena, half=hit.half, yaw_deg=yaw_deg,
            yaw_ambiguous=ambiguous, yaw_source=source,
            confidence=float(conf), area_px=int(blob.area), area_frac=float(area_frac),
            fill=float(blob.fill), elongation=float(aniso),
            offset_from_center_m=hit.offset_from_center_m,
            evidence={"blob_bbox": [blob.x0, blob.y0, blob.x1, blob.y1],
                      "axis_deg_map": round(axis_deg, 2),
                      "n_nose": len(nose_blobs)},
        ))
    out.sort(key=lambda c: -c.confidence)
    return out


def best_car(rgb, reg: ArenaRegistration, palette: Palette,
             spec: CarSpec | None = None, min_confidence: float = 0.35) -> CarPose | None:
    """取置信度最高的车体；低于 ``min_confidence`` 返回 None（宁可不报）。"""
    cars = detect_car(rgb, reg, palette, spec)
    if not cars or cars[0].confidence < min_confidence:
        return None
    return cars[0]
