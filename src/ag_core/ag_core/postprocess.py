"""YOLO 输出后处理：anchor-free（YOLOv8/11 导出格式）与 anchor-based（YOLOv5 导出格式）。

背景（来自本项目模型体检报告 / model_report.txt）：
  * best_new.rknn  -> YOLOv8n-pose 导出，输出 [1, 9, 8400]
        C = 4(box) + 2(cls: blue/red glowing target) + 3*1(kpt x,y,conf) = 9
  * rest_clean.rknn-> YOLO11n detect 导出，输出 [1, 5, 8400]
        C = 4(box) + 1(cls: explosive) = 5
  * light_det.rknn -> 官方模型，YOLOv5 风格三尺度输出 1x18x{80,40,20}^2
        C = 3 anchors * (5 + nc) = 18  -> nc = 1

关键不确定性（务必用 tools/verify_decode.py 在真机上校准一次）：
  Ultralytics 导出时，box 通常已解码为「输入分辨率下的绝对像素」、cls 已过 sigmoid；
  但 pose 分支在 export 路径下 kpt 可能仍是 **网格单位** 而非像素单位。
  因此这里把 kpt_unit / cls_activation 做成显式参数，不做隐式猜测。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "Detection",
    "letterbox_params",
    "scale_coords",
    "decode_anchor_free",
    "decode_anchored",
    "nms",
    "bbox_iou",
]


@dataclass
class Detection:
    """单个检测结果。坐标默认处于 **模型输入分辨率**（如 640）下的像素系。"""

    cls: int
    score: float
    x1: float
    y1: float
    x2: float
    y2: float
    kpts: Optional[np.ndarray] = field(default=None, repr=False)  # (K,3) x, y, conf

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    def center(self) -> Tuple[float, float]:
        return (self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0

    def kpt(self, i: int = 0) -> Optional[Tuple[float, float, float]]:
        if self.kpts is None or len(self.kpts) <= i:
            return None
        x, y, c = self.kpts[i]
        return float(x), float(y), float(c)

    def to_dict(self) -> dict:
        d = {
            "cls": int(self.cls),
            "score": float(self.score),
            "bbox": [float(self.x1), float(self.y1), float(self.x2), float(self.y2)],
        }
        if self.kpts is not None:
            d["kpts"] = np.asarray(self.kpts).round(3).tolist()
        return d


# --------------------------------------------------------------------------
# 预处理 / 坐标反算
# --------------------------------------------------------------------------
def letterbox_params(h: int, w: int, size: int = 640, stride: int = 32) -> Tuple[float, float, float, int, int]:
    """返回 (scale, pad_x, pad_y, new_w, new_h)，与 Ultralytics letterbox 对齐。"""
    r = min(size / float(h), size / float(w))
    new_w, new_h = int(round(w * r)), int(round(h * r))
    # 与官方一致：按 stride 对齐后再补边
    new_w = int(np.ceil(new_w / stride) * stride)
    new_h = int(np.ceil(new_h / stride) * stride)
    new_w = min(new_w, size)
    new_h = min(new_h, size)
    pad_x = (size - new_w) / 2.0
    pad_y = (size - new_h) / 2.0
    return r, pad_x, pad_y, new_w, new_h


def scale_coords(
    coords: np.ndarray,
    scale: float,
    pad_x: float,
    pad_y: float,
    clip_to: Optional[Tuple[int, int]] = None,
) -> np.ndarray:
    """把模型输入分辨率下的坐标还原到原图分辨率。coords: (..., 2) 或 (..., 4)。"""
    out = np.asarray(coords, dtype=np.float64).copy()
    if out.shape[-1] >= 4:
        out[..., 0] = (out[..., 0] - pad_x) / scale
        out[..., 1] = (out[..., 1] - pad_y) / scale
        out[..., 2] = (out[..., 2] - pad_x) / scale
        out[..., 3] = (out[..., 3] - pad_y) / scale
    else:
        out[..., 0] = (out[..., 0] - pad_x) / scale
        out[..., 1] = (out[..., 1] - pad_y) / scale
    if clip_to is not None:
        w, h = clip_to
        if out.shape[-1] >= 4:
            out[..., 0::2] = np.clip(out[..., 0::2], 0, w)
            out[..., 1::2] = np.clip(out[..., 1::2], 0, h)
        else:
            out[..., 0] = np.clip(out[..., 0], 0, w)
            out[..., 1] = np.clip(out[..., 1], 0, h)
    return out


# --------------------------------------------------------------------------
# NMS
# --------------------------------------------------------------------------
def bbox_iou(box: np.ndarray, boxes: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    """box: (4,)  boxes: (N,4)，均为 x1y1x2y2。"""
    ix1 = np.maximum(box[0], boxes[:, 0])
    iy1 = np.maximum(box[1], boxes[:, 1])
    ix2 = np.minimum(box[2], boxes[:, 2])
    iy2 = np.minimum(box[3], boxes[:, 3])
    inter = np.clip(ix2 - ix1, 0, None) * np.clip(iy2 - iy1, 0, None)
    area_b = (box[2] - box[0]) * (box[3] - box[1])
    area_bs = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    union = area_b + area_bs - inter + eps
    return inter / union


def nms(boxes: np.ndarray, scores: np.ndarray, iou_thres: float) -> List[int]:
    """返回保留下来的索引（按分数从高到低）。"""
    if len(boxes) == 0:
        return []
    order = np.argsort(-scores)
    keep: List[int] = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        ious = bbox_iou(boxes[i], boxes[rest])
        order = rest[ious <= iou_thres]
    return keep


def _as_nc(out: np.ndarray) -> np.ndarray:
    """接受 (1,C,N) / (C,N) / (N,C)，统一成 (N,C)。"""
    a = np.asarray(out)
    if a.ndim == 3:
        a = a[0] if a.shape[0] in (1, 3) or a.shape[0] < a.shape[1] else a
        a = a.reshape(a.shape[0], -1) if a.shape[0] != 1 else a[0]
    if a.ndim == 2 and a.shape[0] < a.shape[1]:
        a = a.T
    return a.astype(np.float32)


def _stride_per_anchor(num: int, img_size: int = 640, strides: Sequence[int] = (8, 16, 32)) -> np.ndarray:
    """按 80x80 / 40x40 / 20x20 三段，给出每个 anchor 对应的 stride。"""
    counts = [(img_size // s) ** 2 for s in strides]
    arr = np.concatenate([np.full(c, s, dtype=np.float32) for c, s in zip(counts, strides)])
    if arr.size != num:
        # 非标准尺寸时退化为全 32（仅影响 kpt_unit='grid' 分支）
        return np.full(num, strides[-1], dtype=np.float32)
    return arr


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(x, -60, 60)))


# --------------------------------------------------------------------------
# anchor-free 解码（YOLOv8-pose / YOLO11-detect 导出格式）
# --------------------------------------------------------------------------
def decode_anchor_free(
    out: np.ndarray,
    nc: int,
    nkpt: int = 0,
    conf_thres: float = 0.25,
    iou_thres: float = 0.45,
    img_size: int = 640,
    max_det: int = 50,
    cls_activation: str = "sigmoid",
    kpt_unit: str = "pixel",
    per_class_nms: bool = True,
) -> List[Detection]:
    """解码 [1, C, N] 形式输出，C = 4 + nc + 3*nkpt。

    参数
    ----
    cls_activation : 'sigmoid'（Ultralytics 导出默认，分数已在 [0,1]）
                     或 'raw'（需自行 sigmoid，少数导出/量化后处理会退化）
    kpt_unit       : 'pixel' 关键点已是输入分辨率像素
                     'grid'  需乘 stride 还原（部分 pose 导出的行为）
                     'norm'  关键点归一化到 0~1，需乘 img_size 还原
                              —— 训练标签就是归一化格式，导出后很可能是这一种
    """
    a = _as_nc(out)
    n, c = a.shape
    expect = 4 + nc + 3 * nkpt
    if c != expect:
        raise ValueError(f"通道数不匹配：输出 {c}，按 nc={nc}, nkpt={nkpt} 应为 {expect}")

    boxes = a[:, :4].astype(np.float32)              # cx, cy, w, h
    cls_raw = a[:, 4 : 4 + nc].astype(np.float32)
    cls_scores = cls_raw if cls_activation == "sigmoid" else sigmoid(cls_raw)

    cls_id = np.argmax(cls_scores, axis=1)
    scores = cls_scores[np.arange(n), cls_id]

    kpts = None
    if nkpt > 0:
        k = a[:, 4 + nc :].reshape(n, nkpt, 3).astype(np.float32)
        if kpt_unit == "grid":
            st = _stride_per_anchor(n, img_size).reshape(n, 1, 1)
            k[..., 0:2] = k[..., 0:2] * st
        elif kpt_unit == "norm":
            k[..., 0:2] = k[..., 0:2] * float(img_size)
        elif kpt_unit != "pixel":
            raise ValueError(f"未知 kpt_unit: {kpt_unit}（可选 pixel / grid / norm）")
        kpts = k

    keep = np.nonzero(scores >= conf_thres)[0]
    if keep.size == 0:
        return []

    boxes = boxes[keep]
    scores = scores[keep]
    cls_id = cls_id[keep]
    if kpts is not None:
        kpts = kpts[keep]

    xyxy = np.empty_like(boxes)
    xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.0
    xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.0
    xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2.0
    xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2.0

    if per_class_nms and nc > 1:
        idxs: List[int] = []
        for c_ in np.unique(cls_id):
            m = np.nonzero(cls_id == c_)[0]
            idxs.extend(m[np.array(nms(xyxy[m], scores[m], iou_thres))])
    else:
        idxs = nms(xyxy, scores, iou_thres)

    idxs = idxs[:max_det]
    dets: List[Detection] = []
    for i in idxs:
        dets.append(
            Detection(
                cls=int(cls_id[i]),
                score=float(scores[i]),
                x1=float(xyxy[i, 0]),
                y1=float(xyxy[i, 1]),
                x2=float(xyxy[i, 2]),
                y2=float(xyxy[i, 3]),
                kpts=(kpts[i].copy() if kpts is not None else None),
            )
        )
    return dets


# --------------------------------------------------------------------------
# anchor-based 解码（YOLOv5 风格，用于官方 light_det.rknn）
# --------------------------------------------------------------------------
DEFAULT_ANCHORS = (
    ((10.0, 13.0), (16.0, 30.0), (33.0, 23.0)),
    ((30.0, 61.0), (62.0, 45.0), (59.0, 119.0)),
    ((116.0, 90.0), (156.0, 198.0), (373.0, 326.0)),
)


def decode_anchored(
    outs: Sequence[np.ndarray],
    anchors: Sequence[Sequence[Sequence[float]]] = DEFAULT_ANCHORS,
    strides: Sequence[int] = (8, 16, 32),
    nc: Optional[int] = None,
    conf_thres: float = 0.25,
    iou_thres: float = 0.45,
    img_size: int = 640,
    max_det: int = 50,
    cls_activation: str = "sigmoid",
) -> List[Detection]:
    """解码 YOLOv5 风格三尺度输出 [(1,3*(5+nc),H,W), ...]。

    nc 为 None 时由通道数反推：nc = C/3 - 5。
    """
    if len(outs) != len(strides):
        raise ValueError(f"输出层数 {len(outs)} 与 strides {len(strides)} 不一致")

    all_boxes, all_scores, all_cls = [], [], []
    for out, anc, stride in zip(outs, anchors, strides):
        a = np.asarray(out, dtype=np.float32)
        if a.ndim == 3:
            a = a[None, ...]
        b, c, h, w = a.shape
        na = len(anc)
        if c % na != 0:
            raise ValueError(f"通道 {c} 不能被 anchor 数 {na} 整除")
        per = c // na
        _nc = nc if nc is not None else per - 5
        if per != 5 + _nc:
            raise ValueError(f"每层通道 {per} 与 nc={_nc} 不匹配（应 5+nc）")

        a = a.reshape(b, na, per, h, w).transpose(0, 1, 3, 4, 2)  # (b,na,h,w,per)
        ancv = np.asarray(anc, dtype=np.float32).reshape(1, na, 1, 1, 2)

        gy, gx = np.meshgrid(np.arange(h), np.arange(w), indexing="ij")
        grid = np.stack((gx, gy), axis=-1).astype(np.float32)  # (h,w,2)
        grid = grid[None, None, :, :, :]

        tx, ty, tw, th, tobj = (
            a[..., 0:1],
            a[..., 1:2],
            a[..., 2:3],
            a[..., 3:4],
            a[..., 4:5],
        )
        cls_part = a[..., 5:]

        sx = sigmoid(tx)
        sy = sigmoid(ty)
        cx = (sx * 2.0 - 0.5 + grid[..., 0:1]) * stride
        cy = (sy * 2.0 - 0.5 + grid[..., 1:2]) * stride
        pw = np.exp(np.clip(tw, -20, 20)) * ancv[..., 0:1]
        ph = np.exp(np.clip(th, -20, 20)) * ancv[..., 1:2]

        obj = sigmoid(tobj)
        cp = cls_part if cls_activation == "sigmoid" else sigmoid(cls_part)
        cls_conf = obj * cp
        if _nc == 1:
            cid = np.zeros(cls_conf.shape[:-1], dtype=np.int64)
            sc = cls_conf[..., 0]
        else:
            cid = np.argmax(cls_conf, axis=-1)
            sc = np.max(cls_conf, axis=-1)

        bw, bh = pw[..., 0], ph[..., 0]
        x1 = cx[..., 0] - bw / 2.0
        y1 = cy[..., 0] - bh / 2.0
        x2 = cx[..., 0] + bw / 2.0
        y2 = cy[..., 0] + bh / 2.0

        all_boxes.append(np.stack([x1, y1, x2, y2], axis=-1).reshape(-1, 4))
        all_scores.append(sc.reshape(-1))
        all_cls.append(cid.reshape(-1))

    boxes = np.concatenate(all_boxes)
    scores = np.concatenate(all_scores)
    cls_ids = np.concatenate(all_cls)

    keep = np.nonzero(scores >= conf_thres)[0]
    if keep.size == 0:
        return []
    boxes, scores, cls_ids = boxes[keep], scores[keep], cls_ids[keep]

    idxs = nms(boxes, scores, iou_thres)[:max_det]
    return [
        Detection(
            cls=int(cls_ids[i]),
            score=float(scores[i]),
            x1=float(boxes[i, 0]),
            y1=float(boxes[i, 1]),
            x2=float(boxes[i, 2]),
            y2=float(boxes[i, 3]),
        )
        for i in idxs
    ]
