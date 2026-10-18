"""HbbDetector 后处理的 Python 等价实现（与 onboard/dzuav/src/HbbDetector.cpp 逐行对应）。

用途：
  - 在 PC 端复现无人机推理后的后处理逻辑，确保评估口径与实机一致；
  - 单测守护 C++ 与 Python 两侧行为对齐；
  - 调参（按类别阈值、边缘过滤）可在 PC 端快速迭代，再同步到 C++。

契约（与 C++ 一致）：
  输入  raw [1, 11, 33600]  float32   （11 = 4 bbox + 7 类）
  类别顺序：mine=0 car=1 arrow=2 target_red=3 cell_label=4 target_blue=5 obstacle=6
  letterbox：top-left 对齐（仅右下 padding，值 114），scale = min(1280/W, 1280/H)
  后处理：按类别 conf 过滤 → 按类别 NMS → mine 边缘 1% 过滤
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 7 类名称（与 types.hpp / main.cpp className 严格一致）
CLASS_NAMES = ["mine", "car", "arrow", "target_red",
               "cell_label", "target_blue", "obstacle"]
NUM_CLASSES = 7

# 默认阈值（与 main.cpp HbbDetector(hbb, .25, .45, edge=0.01) 一致）
DEFAULT_CONF = 0.25
DEFAULT_NMS = 0.45
DEFAULT_EDGE_MINE = 0.01   # 1%，val-B 验证综合精度 +0.76pp


@dataclass
class Detection:
    cls: int          # 0..6
    score: float
    x: int            # 原图坐标（像素）
    y: int
    w: int
    h: int

    @property
    def cx(self) -> float:
        return self.x + self.w / 2.0

    @property
    def cy(self) -> float:
        return self.y + self.h / 2.0


def _nms_per_class(boxes, scores, class_ids, conf_per_class, nms_per_class):
    """按类别分别做 NMS（与 C++ 完全一致：同类内 NMS，不同类重叠保留）。

    boxes  [M,4] x,y,w,h（原图坐标，int）
    返回保留的 Detection 列表。
    """
    import cv2

    keep: list[Detection] = []
    for c in range(NUM_CLASSES):
        mask = class_ids == c
        if not np.any(mask):
            continue
        b = boxes[mask]
        q = scores[mask]
        rects = [[int(x), int(y), int(w), int(h)] for x, y, w, h in b]
        selected = cv2.dnn.NMSBoxes(rects, q.tolist(),
                                    float(conf_per_class[c]),
                                    float(nms_per_class[c]))
        if len(selected) > 0:
            idx = np.array(selected).flatten()
            for k in idx:
                x, y, w, h = b[k]
                keep.append(Detection(cls=c, score=float(q[k]),
                                      x=int(x), y=int(y), w=int(w), h=int(h)))
    return keep


def postprocess(raw: np.ndarray,
                orig_w: int,
                orig_h: int,
                conf_per_class=None,
                nms_per_class=None,
                edge_mine_thr: float = DEFAULT_EDGE_MINE) -> list[Detection]:
    """复现 HbbDetector::infer 的完整后处理。

    Args:
        raw: [1, 11, 33600] 模型原始输出（CHW，C=4+7）。
        orig_w / orig_h: 原图宽高（像素），用于边缘过滤判定。
        conf_per_class: 长度 7 的置信度阈值数组；None → 全部 DEFAULT_CONF。
        nms_per_class:  长度 7 的 NMS IoU 阈值数组；None → 全部 DEFAULT_NMS。
        edge_mine_thr:  mine 边缘过滤阈值（归一化）；0 表示关闭。

    Returns:
        Detection 列表（原图坐标）。
    """
    if conf_per_class is None:
        conf_per_class = np.full(NUM_CLASSES, DEFAULT_CONF, dtype=np.float32)
    else:
        conf_per_class = np.asarray(conf_per_class, dtype=np.float32)
    if nms_per_class is None:
        nms_per_class = np.full(NUM_CLASSES, DEFAULT_NMS, dtype=np.float32)
    else:
        nms_per_class = np.asarray(nms_per_class, dtype=np.float32)

    arr = np.squeeze(raw, axis=0)          # [11, 33600]
    if arr.shape[0] != 4 + NUM_CLASSES:
        raise ValueError(f"通道数 {arr.shape[0]} != 4+{NUM_CLASSES}")

    # 类别分数：argmax + max
    cls_block = arr[4:4 + NUM_CLASSES]            # [7, N]
    class_ids = cls_block.argmax(axis=0)          # [N]
    scores = cls_block.max(axis=0)                # [N]

    # 按类别置信度过滤
    conf_vec = conf_per_class[class_ids]          # [N]
    keep_mask = scores > conf_vec
    if not np.any(keep_mask):
        return []

    arr_keep = arr[:, keep_mask]                  # [11, M]
    class_ids = class_ids[keep_mask]
    scores = scores[keep_mask]

    # bbox → 原图坐标（C++: x=(cx-w/2)/s, y=(cy-h/2)/s, w=raw_w/s, h=raw_h/s）
    s = min(1280.0 / orig_w, 1280.0 / orig_h)
    cx = arr_keep[0]
    cy = arr_keep[1]
    bw = arr_keep[2]
    bh = arr_keep[3]
    x = np.clip((cx - bw / 2) / s, 0, None)
    y = np.clip((cy - bh / 2) / s, 0, None)
    w = np.maximum(bw / s, 1.0)
    h = np.maximum(bh / s, 1.0)
    boxes = np.stack([x, y, w, h], axis=1).astype(np.int32)

    # 按类别 NMS
    detections = _nms_per_class(boxes, scores, class_ids,
                                conf_per_class, nms_per_class)

    # mine 边缘过滤（仅 cls=0）
    if edge_mine_thr > 0.0 and detections:
        ex = edge_mine_thr * orig_w
        ey = edge_mine_thr * orig_h
        filtered = []
        for d in detections:
            if d.cls == 0:
                cx_, cy_ = d.cx, d.cy
                if cx_ < ex or cx_ > orig_w - ex or cy_ < ey or cy_ > orig_h - ey:
                    continue
            filtered.append(d)
        detections = filtered

    return detections
