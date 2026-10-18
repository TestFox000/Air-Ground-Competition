"""按类别置信度 + paper 边缘过滤阈值扫描，搜索综合精度最优组合。

综合精度口径（与省赛评估一致，类别+IoU≥0.5 全对才算正确）：
    TP = 预测与某 GT 同类且 IoU≥0.5
    FP = 预测无匹配 GT
    FN = GT 无匹配预测
    comprehensive = TP / (TP + FP + FN)

输入：
    --preds   预测标签目录，YOLO 格式 + 置信度列：class cx cy w h score
    --gts     GT 标签目录，YOLO 格式：class cx cy w h
    --images  原图目录（仅取文件名列表，不读图）
输出：
    打印每个类别的 TP/FP/FN + 综合精度，按综合精度降序，取前 N 个组合。

用法（在训练机上，模型推理出带分数的预测标签后）：
    python tools/ml/sweep_thresholds.py \
        --preds runs/pred_labels --gts E:/ml_datasets/aerial_field_d13/labels/val \
        --images E:/ml_datasets/aerial_field_d13/images/val --top 10
    # 旧 7cls 留档数据：--num-classes 7 --class-names mine,car,arrow,...
"""

from __future__ import annotations

import argparse
import sys
from itertools import product
from pathlib import Path

import numpy as np

# D13 十三类（2026-10-16 起现役；与 types.hpp ObjectClass 顺序一致）
D13_CLASS_NAMES = ["paper", "cell_label", "car", "target_red", "target_blue",
                   "obstacle", "bucket_red", "bucket_green", "bucket_cyan",
                   "car_start", "car_end", "uav_start", "uav_end"]


# ---------------------------------------------------------------- IO

def read_pred_file(path: Path) -> list[tuple[int, float, float, float, float, float]]:
    """class, cx, cy, w, h, score（归一化坐标）。"""
    items = []
    if not path.exists():
        return items
    for line in path.read_text(encoding="utf-8").strip().splitlines():
        parts = line.split()
        if len(parts) < 6:
            continue
        cls = int(parts[0])
        cx, cy, w, h, score = map(float, parts[1:6])
        items.append((cls, cx, cy, w, h, score))
    return items


def read_gt_file(path: Path) -> list[tuple[int, float, float, float, float]]:
    items = []
    if not path.exists():
        return items
    for line in path.read_text(encoding="utf-8").strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cls = int(parts[0])
        cx, cy, w, h = map(float, parts[1:5])
        items.append((cls, cx, cy, w, h))
    return items


# ---------------------------------------------------------------- 匹配

def _iou_xyxy(a, b) -> float:
    x1 = max(a[0], b[0]); y1 = max(a[1], b[1])
    x2 = min(a[2], b[2]); y2 = min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    aa = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    bb = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return inter / (aa + bb - inter) if (aa + bb - inter) > 0 else 0.0


def _to_xyxy(cx, cy, w, h):
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def evaluate(preds_by_image, gts_by_image,
             conf_per_class, edge_paper_thr, iou_thr=0.5, num_classes=13):
    """给定阈值组合，计算各类 TP/FP/FN 与综合精度。

    num_classes: 类别数（决定 TP/FP/FN 向量长度与 conf_per_class 长度）；
    标签里 cls >= num_classes 的框（如旧线数据混入新类别表）直接跳过并计数，
    不再 IndexError。边缘过滤仅作用 cls=0（paper，与 HbbDetector.cpp 同语义）。
    """
    tp = np.zeros(num_classes, dtype=int)
    fp = np.zeros(num_classes, dtype=int)
    fn = np.zeros(num_classes, dtype=int)
    skipped = 0

    for img_id in gts_by_image:
        gts = [g for g in gts_by_image[img_id] if g[0] < num_classes]
        skipped += len(gts_by_image[img_id]) - len(gts)
        preds = preds_by_image.get(img_id, [])

        # 按类别置信度过滤 + paper 边缘过滤
        filtered = []
        for cls, cx, cy, w, h, score in preds:
            if cls >= num_classes:
                skipped += 1
                continue
            if score < conf_per_class[cls]:
                continue
            if cls == 0 and edge_paper_thr > 0.0:
                if (cx < edge_paper_thr or cx > 1 - edge_paper_thr
                        or cy < edge_paper_thr or cy > 1 - edge_paper_thr):
                    continue
            filtered.append((cls, cx, cy, w, h, score))

        # 同类内按 score 降序，贪心匹配 GT
        for c in range(num_classes):
            cls_preds = sorted(
                [(cx, cy, w, h, s) for (cc, cx, cy, w, h, s) in filtered if cc == c],
                key=lambda x: -x[4])
            cls_gts = [(cx, cy, w, h) for (cc, cx, cy, w, h) in gts if cc == c]
            matched_gt = set()
            for (cx, cy, w, h, _s) in cls_preds:
                box = _to_xyxy(cx, cy, w, h)
                best_iou, best_j = 0.0, -1
                for j, (gx, gy, gw, gh) in enumerate(cls_gts):
                    if j in matched_gt:
                        continue
                    giou = _iou_xyxy(box, _to_xyxy(gx, gy, gw, gh))
                    if giou > best_iou:
                        best_iou, best_j = giou, j
                if best_iou >= iou_thr and best_j >= 0:
                    tp[c] += 1
                    matched_gt.add(best_j)
                else:
                    fp[c] += 1
            fn[c] += len(cls_gts) - len(matched_gt)

    total_tp = int(tp.sum())
    total_fp = int(fp.sum())
    total_fn = int(fn.sum())
    denom = total_tp + total_fp + total_fn
    comp = total_tp / denom if denom > 0 else 0.0
    return comp, tp, fp, fn, skipped


# ---------------------------------------------------------------- 扫描

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--preds", required=True, help="预测标签目录（含 score 列）")
    ap.add_argument("--gts", required=True, help="GT 标签目录")
    ap.add_argument("--images", help="原图目录（用于枚举图片名）")
    ap.add_argument("--conf-min", type=float, default=0.05)
    ap.add_argument("--conf-max", type=float, default=0.50)
    ap.add_argument("--conf-step", type=float, default=0.05)
    ap.add_argument("--edge-values", type=float, nargs="+",
                    default=[0.0, 0.005, 0.01, 0.015, 0.02],
                    help="paper 边缘过滤阈值候选（归一化；仅作用 cls=0）")
    ap.add_argument("--per-class", action="store_true",
                    help="按类别独立扫描 conf（默认全局统一 conf）")
    ap.add_argument("--top", type=int, default=10)
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--num-classes", type=int, default=13,
                    help="类别数（默认 13=D13 现役；旧 7cls 留档数据用 7）")
    ap.add_argument("--class-names", default=None,
                    help="逗号分隔类别名表（默认取 D13 前 --num-classes 个）")
    args = ap.parse_args()

    num_classes = args.num_classes
    if args.class_names:
        class_names = [n.strip() for n in args.class_names.split(",") if n.strip()]
        if len(class_names) != num_classes:
            print(f"--class-names 给了 {len(class_names)} 个名，"
                  f"与 --num-classes {num_classes} 不一致", file=sys.stderr)
            return 2
    elif num_classes <= len(D13_CLASS_NAMES):
        class_names = D13_CLASS_NAMES[:num_classes]
    else:
        class_names = D13_CLASS_NAMES + [f"cls{i}" for i in range(
            len(D13_CLASS_NAMES), num_classes)]

    preds_dir = Path(args.preds)
    gts_dir = Path(args.gts)
    if not gts_dir.exists():
        print(f"GT 目录不存在: {gts_dir}", file=sys.stderr)
        return 2

    # 枚举图片：优先用 images 目录，否则用 gts 目录的文件名
    if args.images:
        stems = {p.stem for p in Path(args.images).glob("*")
                 if p.suffix.lower() in (".jpg", ".jpeg", ".png")}
    else:
        stems = {p.stem for p in gts_dir.glob("*.txt")}

    preds_by_image = {s: read_pred_file(preds_dir / f"{s}.txt") for s in stems}
    gts_by_image = {s: read_gt_file(gts_dir / f"{s}.txt") for s in stems}

    n_gt_total = sum(len(v) for v in gts_by_image.values())
    print(f"图片数: {len(stems)}  GT 总数: {n_gt_total}")

    conf_values = list(np.arange(args.conf_min, args.conf_max + 1e-9, args.conf_step))

    results = []
    skipped_total = 0
    if args.per_class:
        # 按类别独立扫描：组合爆炸，限制每类候选数
        if len(conf_values) > 6:
            conf_values = list(np.linspace(args.conf_min, args.conf_max, 6))
        print(f"按类别扫描：每类 {len(conf_values)} 个候选 × "
              f"{len(args.edge_values)} edge = "
              f"{len(conf_values)**num_classes * len(args.edge_values):,} 组合（可能较慢）")
        for conf_combo in product(conf_values, repeat=num_classes):
            for edge in args.edge_values:
                comp, tp, fp, fn, skipped = evaluate(
                    preds_by_image, gts_by_image,
                    list(conf_combo), edge, args.iou, num_classes)
                skipped_total = max(skipped_total, skipped)
                results.append((comp, list(conf_combo), edge, tp, fp, fn))
    else:
        # 全局统一 conf（快速）
        for conf in conf_values:
            for edge in args.edge_values:
                comp, tp, fp, fn, skipped = evaluate(
                    preds_by_image, gts_by_image,
                    [conf] * num_classes, edge, args.iou, num_classes)
                skipped_total = max(skipped_total, skipped)
                results.append((comp, [conf] * num_classes, edge, tp, fp, fn))
    if skipped_total:
        print(f"注意：{skipped_total} 个框的类别号 >= {num_classes}，已跳过"
              f"（检查 --num-classes 是否与标签类别表一致）")

    results.sort(key=lambda r: -r[0])
    print(f"\n共 {len(results)} 组合，Top {args.top}：")
    print("-" * 100)
    header = f"{'comp':>7} {'conf':>6} {'edge':>6}  "
    header += "  ".join(f"{n[:4]:>10}" for n in class_names)
    print(header)
    for comp, conf, edge, tp, fp, fn in results[:args.top]:
        per_cls = "  ".join(f"{tp[c]}/{fp[c]}/{fn[c]:>10}"
                            for c in range(num_classes))
        print(f"{comp*100:6.2f}% {conf[0]:6.3f} {edge:6.3f}  {per_cls}")

    if results:
        best = results[0]
        print(f"\n最优：综合精度 {best[0]*100:.2f}%  "
              f"conf={best[1][0]:.3f}  edge_paper={best[2]:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
