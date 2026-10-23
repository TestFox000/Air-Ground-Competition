"""HbbDetector 后处理的 Python 等价实现（与 onboard/dzuav/src/HbbDetector.cpp 逐行对应）。

用途：
  - 在 PC 端复现无人机推理后的后处理逻辑，确保评估口径与实机一致；
  - 单测守护 C++ 与 Python 两侧行为对齐；
  - 调参（按类别阈值、边缘过滤）可在 PC 端快速迭代，再同步到 C++。

⚠️ 无人机离线期间这是唯一能验后处理的手段，所以它必须真的是"等价实现"。
   2026-10-20 复查发现它此前是 7 类 + top-left letterbox + edge 0.01，三处都
   与实机不符——一个声称逐行对应却处处对应的镜像比没有镜像更坏，因为有人会信它。
   现已随 C++ 同步为 13 类 / 居中 letterbox / edge 0.005，并由
   tests/test_hbb_postprocess.py 反向锁住类别表、阈值与 video_recon 的一致性。

契约（与 C++ 一致）：
  输入  raw [1, 17, 33600]  float32   （17 = 4 bbox + 13 类）
  类别顺序取自 onboard/dzuav/include/dzuav/types.hpp 的 ObjectClass，
  与 src/ag_integrate/.../video_recon.py 的 NAMES 逐字相同（有测试守）。
  letterbox：**居中**对齐（Ultralytics 口径，四周补 114），
             scale = min(1280/W, 1280/H)，nw/nh 用 half-to-even 取整
             （= C++ cvRound = video_recon 的 round()），解码时减回 pad；
             框坐标用 int 截断（非四舍五入）。三处舍入口径都与 C++ 逐字对应，
             且各有测试守着。
  后处理：按类别 conf 过滤 → 按类别 NMS → paper/格号重叠过滤 → paper 边缘过滤
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 13 类名称（与 types.hpp ObjectClass / main.cpp className / video_recon.NAMES 严格一致）
CLASS_NAMES = ["paper", "cell_label", "car", "target_red", "target_blue", "obstacle",
               "bucket_red", "bucket_green", "bucket_cyan",
               "car_start", "car_end", "uav_start", "uav_end"]
NUM_CLASSES = len(CLASS_NAMES)
PAPER, CELL_LABEL = 0, 1

IN_SIZE = 1280          # 引擎输入边长（= C++ kInputSide）
ANCHORS = 33600         # 160² + 80² + 40²

# 部署阈值（与 main.cpp kDeployConf 逐项一致；由测试反向锁住）
DEFAULT_CONF_PER_CLASS = [0.25, 0.35, 0.25, 0.25, 0.25, 0.40, 0.40,
                          0.40, 0.40, 0.25, 0.25, 0.25, 0.25]
DEFAULT_NMS = 0.45              # = main.cpp kDeployNms
DEFAULT_EDGE_PAPER = 0.005      # = main.cpp kEdgePaperThr（0.5%）
LABEL_OVERLAP_IOU = 0.30        # = HbbDetector.cpp kLabelOverlapIou / video_recon 同值


@dataclass
class Detection:
    cls: int          # 0..12
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


def letterbox_params(orig_w: int, orig_h: int) -> tuple[float, int, int, int, int]:
    """返回 (scale, nw, nh, pad_x, pad_y)，与 HbbDetector::infer 的前处理逐行对应。

    取整用 Python `round()`（round-half-to-even），**不要**换成
    `math.floor(v+0.5)` / `np.rint` 之类：C++ 侧是 `cvRound`，video_recon.py 是
    `round()`，Ultralytics letterbox 也是 half-to-even，三方一致。
    `round(718.5)==718` 而 `lround(718.5)==719`，只在乘积恰好落在 .5 时分叉，
    但那种分叉方向恒定、且只在特定分辨率出现，是最难在现场定位的一类偏差。
    """
    s = min(IN_SIZE / orig_w, IN_SIZE / orig_h)
    nw = max(1, min(IN_SIZE, round(orig_w * s)))
    nh = max(1, min(IN_SIZE, round(orig_h * s)))
    return s, nw, nh, (IN_SIZE - nw) // 2, (IN_SIZE - nh) // 2


def _iou(a, b) -> float:
    """cv::Rect 交并比（a/b 为 (x, y, w, h)）。对应 HbbDetector.cpp rectIoU。"""
    ax2, ay2 = a[0] + a[2], a[1] + a[3]
    bx2, by2 = b[0] + b[2], b[1] + b[3]
    ix = max(0, min(ax2, bx2) - max(a[0], b[0]))
    iy = max(0, min(ay2, by2) - max(a[1], b[1]))
    inter = ix * iy
    if inter <= 0:
        return 0.0
    uni = a[2] * a[3] + b[2] * b[3] - inter
    return inter / uni if uni > 0 else 0.0


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


def _filter_label_overlap(detections: list[Detection],
                          iou_th: float = LABEL_OVERLAP_IOU) -> list[Detection]:
    """paper 框与任一 cell_label 框重叠 IoU > iou_th → 判为标签文字，剔除。

    对应 HbbDetector.cpp 同名逻辑与 video_recon.filter_label_overlap：
    高分辨率近拍下垫面印刷格号（"E9"/"D4"）会被 paper 类误检，实测分数
    0.58/0.88，**调 paper 阈值拦不住**，只能靠与格号框的重叠关系剔。
    """
    labels = [d for d in detections if d.cls == CELL_LABEL]
    if not labels:
        return detections
    kept = []
    for d in detections:
        if d.cls == PAPER:
            box = (d.x, d.y, d.w, d.h)
            if max(_iou(box, (lb.x, lb.y, lb.w, lb.h)) for lb in labels) > iou_th:
                continue
        kept.append(d)
    return kept


def postprocess(raw: np.ndarray,
                orig_w: int,
                orig_h: int,
                conf_per_class=None,
                nms_per_class=None,
                edge_paper_thr: float = DEFAULT_EDGE_PAPER,
                label_overlap_iou: float = LABEL_OVERLAP_IOU) -> list[Detection]:
    """复现 HbbDetector::infer 的完整后处理。

    Args:
        raw: [1, 17, 33600] 模型原始输出（CHW，C=4+13）。
        orig_w / orig_h: 原图宽高（像素），用于 letterbox 反算与边缘过滤判定。
        conf_per_class: 长度 13 的置信度阈值数组；None → DEFAULT_CONF_PER_CLASS。
        nms_per_class:  长度 13 的 NMS IoU 阈值数组；None → 全部 DEFAULT_NMS。
        edge_paper_thr: paper 边缘过滤阈值（归一化）；0 表示关闭。
        label_overlap_iou: paper/格号重叠判据；<=0 表示关闭该层。

    Returns:
        Detection 列表（原图坐标）。
    """
    if conf_per_class is None:
        conf_per_class = np.asarray(DEFAULT_CONF_PER_CLASS, dtype=np.float32)
    else:
        conf_per_class = np.asarray(conf_per_class, dtype=np.float32)
    if nms_per_class is None:
        nms_per_class = np.full(NUM_CLASSES, DEFAULT_NMS, dtype=np.float32)
    else:
        nms_per_class = np.asarray(nms_per_class, dtype=np.float32)

    arr = np.squeeze(raw, axis=0)          # [17, 33600]
    if arr.shape[0] != 4 + NUM_CLASSES:
        raise ValueError(f"通道数 {arr.shape[0]} != 4+{NUM_CLASSES}（11 = 7 类旧模型）")

    # 类别分数：argmax + max（C++ 用严格 > 累积，首个最大值胜出，与此处 argmax 一致）
    cls_block = arr[4:4 + NUM_CLASSES]            # [13, N]
    class_ids = cls_block.argmax(axis=0)          # [N]
    scores = cls_block.max(axis=0)                # [N]

    # 按类别置信度过滤（C++: if (score < conf) continue，等于阈值时这一步放行；
    # 但下一步 NMSBoxes 的分数判据是严格 >，所以净效果 = 严格大于。见同名测试。）
    keep_mask = scores >= conf_per_class[class_ids]
    if not np.any(keep_mask):
        return []

    arr_keep = arr[:, keep_mask]                  # [17, M]
    class_ids = class_ids[keep_mask]
    scores = scores[keep_mask]

    # bbox → 原图坐标。居中 letterbox：先减 pad 再除 scale（C++ 同式）。
    # 漏减 pad 会让所有框整体偏移 pad/s 个原图像素（1920x1080 下 y 偏 420px）。
    s, _nw, _nh, pad_x, pad_y = letterbox_params(orig_w, orig_h)
    cx = arr_keep[0]
    cy = arr_keep[1]
    bw = arr_keep[2]
    bh = arr_keep[3]
    x = np.maximum((cx - bw / 2 - pad_x) / s, 0)
    y = np.maximum((cy - bh / 2 - pad_y) / s, 0)
    w = np.maximum(bw / s, 1.0)
    h = np.maximum(bh / s, 1.0)
    # C++ 用 int() 截断（向零取整），不是四舍五入——这里必须同口径
    boxes = np.stack([x, y, w, h], axis=1).astype(np.int32)

    # 按类别 NMS
    detections = _nms_per_class(boxes, scores, class_ids,
                                conf_per_class, nms_per_class)

    # 标签文字误检过滤（在 NMS 之后、边缘过滤之前，与 C++ 顺序一致）
    if label_overlap_iou > 0.0 and len(detections) > 1:
        detections = _filter_label_overlap(detections, label_overlap_iou)

    # paper 边缘过滤（仅 cls=0）
    if edge_paper_thr > 0.0 and detections:
        ex = edge_paper_thr * orig_w
        ey = edge_paper_thr * orig_h
        detections = [d for d in detections
                      if d.cls != PAPER
                      or not (d.cx < ex or d.cx > orig_w - ex
                              or d.cy < ey or d.cy > orig_h - ey)]

    return detections
