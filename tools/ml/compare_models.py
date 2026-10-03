"""官方 vs 自研 RKNN 模型同集对比（在车端 rk3588 上运行，rknn-toolkit-lite2）。

对比项（同一份标注 val 集，唯一变量=模型）：
    mAP50 / mAP50-95      分类别检测精度（蓝/红分别算 AP 再平均）
    kpt 误差              靶心关键点归一化平均距离（除以 GT 框对角线，尺度无关）
    NPU 延迟              每帧 inference 毫秒数（多次平均）
对比结论才有意义：不能用不同数据/不同标注各报各的数字。

用法（车端，需与 prelabel_rknn.py 同目录以复用解码）：
    python3 compare_models.py --val datasets/field/images/val \
        --labels datasets/field/labels/val \
        --official ~/models/best_new.rknn --ours models/move_target.rknn
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from prelabel_rknn import decode_output, letterbox  # noqa: E402

try:
    import cv2
    import numpy as np
except ImportError:
    print("需要 opencv-python / numpy", file=sys.stderr)
    raise SystemExit(2)


# ---------------------------------------------------------------- 纯函数（离线可测）

def iou(box_a, box_b) -> float:
    """两个 xyxy 框的 IoU。"""
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, box_a[2] - box_a[0]) * max(0.0, box_a[3] - box_a[1])
    area_b = max(0.0, box_b[2] - box_b[0]) * max(0.0, box_b[3] - box_b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def average_precision(predictions, ground_truths, iou_threshold: float) -> float:
    """单类 all-point AP。

    predictions: [(image_id, score, xyxy), ...]
    ground_truths: {image_id: [xyxy, ...]}
    """
    ordered = sorted(predictions, key=lambda item: -item[1])
    matched: dict[str, set] = {image_id: set() for image_id in ground_truths}
    n_gt = sum(len(v) for v in ground_truths.values())
    if n_gt == 0:
        return 0.0
    if not ordered:
        # 有 GT 但模型一个框都没给 → recall 恒为 0
        return 0.0

    tp = np.zeros(len(ordered))
    fp = np.zeros(len(ordered))
    for i, (image_id, _score, box) in enumerate(ordered):
        gts = ground_truths.get(image_id, [])
        best_iou, best_j = 0.0, -1
        for j, gt in enumerate(gts):
            value = iou(box, gt)
            if value > best_iou:
                best_iou, best_j = value, j
        if best_iou >= iou_threshold and best_j not in matched[image_id]:
            tp[i] = 1
            matched[image_id].add(best_j)
        else:
            fp[i] = 1

    tp_cum = np.cumsum(tp)
    fp_cum = np.cumsum(fp)
    precision = tp_cum / np.maximum(tp_cum + fp_cum, 1e-9)
    recall = tp_cum / n_gt
    # all-point：precision 包络后对 recall 积分
    for i in range(len(precision) - 1, 0, -1):
        precision[i - 1] = max(precision[i - 1], precision[i])
    # 补上 recall=0 起点（否则首检之前的区间丢失，AP 系统性偏低）
    recall = np.concatenate(([0.0], recall))
    precision = np.concatenate(([precision[0]], precision))
    return float(np.trapezoid(precision, recall)) if len(recall) > 1 else 0.0


def mean_average_precision(predictions, ground_truths, iou_threshold: float,
                           num_classes: int) -> float:
    """分类别 AP 再平均（mAP）。匹配时类别必须一致，蓝框不能匹配红 GT。

    predictions: [(image_id, score, cls, xyxy), ...]
    ground_truths: {image_id: [(cls, xyxy), ...]}
    无任何 GT 的类别不计入平均。
    """
    aps = []
    for cls in range(num_classes):
        cls_preds = [(image_id, score, box)
                     for image_id, score, pc, box in predictions if pc == cls]
        cls_gts = {
            image_id: [box for gc, box in objects if gc == cls]
            for image_id, objects in ground_truths.items()
        }
        if sum(len(v) for v in cls_gts.values()) == 0:
            continue
        aps.append(average_precision(cls_preds, cls_gts, iou_threshold))
    return float(np.mean(aps)) if aps else 0.0


# ---------------------------------------------------------------- YOLO 标签读取

def read_yolo_label(path, width: int, height: int):
    """返回 (带类框列表, 关键点列表)，两列表按实例对齐：
        boxes: [(cls, xyxy), ...]
        kpts: [[(x, y), ...], ...]
    """
    boxes, kpts = [], []
    if not Path(path).exists():
        return boxes, kpts
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        values = list(map(float, line.split()))
        if len(values) < 5:
            continue
        cls = int(values[0])
        cx, cy, w, h = values[1:5]
        boxes.append((cls, (
            (cx - w / 2) * width, (cy - h / 2) * height,
            (cx + w / 2) * width, (cy + h / 2) * height,
        )))
        tail = values[5:]
        points = []
        for k in range(len(tail) // 3):
            points.append((tail[k * 3] * width, tail[k * 3 + 1] * height))
        kpts.append(points)
    return boxes, kpts


# ---------------------------------------------------------------- 模型评估

def predict_model(rknn, image, size: int, conf: float, iou_thr: float,
                  num_classes: int, num_kpts: int):
    h, w = image.shape[:2]
    canvas, scale, pad = letterbox(image, size)
    outputs = rknn.inference(inputs=[canvas[None]])
    boxes, scores, points, class_ids = decode_output(
        outputs[0], conf, iou_thr, num_classes, num_kpts
    )
    pad_x, pad_y = pad
    xyxy = []
    for cx, cy, bw, bh in boxes:
        x1 = (cx - bw / 2 - pad_x) / scale
        y1 = (cy - bh / 2 - pad_y) / scale
        x2 = (cx + bw / 2 - pad_x) / scale
        y2 = (cy + bh / 2 - pad_y) / scale
        xyxy.append((x1, y1, x2, y2))
    pred_kpts = []
    for instance in points:
        coords = []
        for px, py, _v in instance:
            coords.append(((px - pad_x) / scale, (py - pad_y) / scale))
        pred_kpts.append(coords)
    return xyxy, scores, pred_kpts, class_ids


def evaluate(model_path: str, image_paths, labels_dir: str, size: int,
             num_classes: int, num_kpts: int, warmup: int = 5) -> dict:
    from rknnlite.api import RKNNLite

    rknn = RKNNLite()
    rknn.load_rknn(model_path)
    rknn.init_runtime()

    predictions = []
    ground_truths: dict[str, list] = {}
    kpt_errors: list[float] = []
    latencies: list[float] = []

    for path in image_paths:
        image = cv2.imread(str(path))
        h, w = image.shape[:2]
        gt_boxes, gt_kpts = read_yolo_label(
            Path(labels_dir) / (path.stem + ".txt"), w, h
        )
        ground_truths[path.stem] = gt_boxes

        # 计时
        canvas, _, _ = letterbox(image, size)
        for _ in range(warmup):
            rknn.inference(inputs=[canvas[None]])
        t0 = time.perf_counter()
        xyxy, scores, pred_kpts, pred_classes = predict_model(
            rknn, image, size, 0.001, 0.7, num_classes, num_kpts
        )
        latencies.append((time.perf_counter() - t0) * 1000)

        for box, score, cls in zip(xyxy, scores, pred_classes):
            predictions.append((path.stem, float(score), int(cls), box))

        # 关键点误差：优先在同类 GT 中按 IoU 匹配，距离除以 GT 框对角线
        for p_box, points, p_cls in zip(xyxy, pred_kpts, pred_classes):
            candidates = [
                (gt_box, gt_points)
                for (gt_cls, gt_box), gt_points in zip(gt_boxes, gt_kpts)
                if gt_cls == p_cls
            ]
            if not candidates:
                candidates = list(zip(
                    [box for _cls, box in gt_boxes], gt_kpts
                ))
            if not candidates:
                continue
            j = int(np.argmin([iou(p_box, gt) * -1 for gt, _ in candidates]))
            gt_box, gt_points = candidates[j]
            diag = ((gt_box[2] - gt_box[0]) ** 2 +
                    (gt_box[3] - gt_box[1]) ** 2) ** 0.5 or 1.0
            for k, (px, py) in enumerate(points):
                gx, gy = gt_points[k]
                kpt_errors.append(((px - gx) ** 2 + (py - gy) ** 2) ** 0.5 / diag)

    rknn.release()

    map50 = mean_average_precision(predictions, ground_truths, 0.5, num_classes)
    map_vector = [mean_average_precision(predictions, ground_truths, t, num_classes)
                  for t in np.arange(0.5, 1.0, 0.05)]
    return {
        "mAP50": map50,
        "mAP50-95": float(np.mean(map_vector)),
        "kpt_error": float(np.mean(kpt_errors)) if kpt_errors else float("nan"),
        "latency_ms": float(np.mean(latencies)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--val", required=True, help="val 图片目录")
    parser.add_argument("--labels", required=True, help="val 标签目录")
    parser.add_argument("--official", required=True)
    parser.add_argument("--ours", required=True)
    parser.add_argument("--size", type=int, default=640)
    parser.add_argument("--nc", type=int, default=2, help="类别数（官方=2）")
    parser.add_argument("--nkpt", type=int, default=1, help="关键点数（官方=1）")
    args = parser.parse_args()

    image_paths = sorted(Path(args.val).glob("*.jpg"))
    if not image_paths:
        print("val 目录无图片", file=sys.stderr)
        return 2

    print(f"评估 {len(image_paths)} 张 val 图 ...")
    official = evaluate(args.official, image_paths, args.labels, args.size,
                        args.nc, args.nkpt)
    ours = evaluate(args.ours, image_paths, args.labels, args.size,
                    args.nc, args.nkpt)

    print(f"\n{'指标':<12}{'官方模型':>14}{'自研模型':>14}")
    for key in ("mAP50", "mAP50-95", "kpt_error", "latency_ms"):
        a, b = official[key], ours[key]
        print(f"{key:<12}{a:>14.4f}{b:>14.4f}")

    print("\n判定：AP 更高、kpt_error 更低、latency 更小（或持平）即胜出。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
