"""PC 端用官方 hbb_detector.onnx 批量预标注航拍帧（onnxruntime，GPU 优先）。

用途：hbb_detector 是特化「无人机俯视看场地」的官方 7 类模型，与航拍视频
同视角。用它先跑一遍抽帧图，输出 YOLO 检测标签草稿，人工只做「校正/补标」。
与车端 prelabel_rknn.py 的区别：本脚本跑 ONNX、无关键点、输入 1280×1280，
在普通 PC（带显卡）上即可运行，不占用飞机/车端硬件。

官方 7 类（顺序对应 dzuav main.cpp / intel_protocol，不可改）：
    0 mine 爆炸物卡片  1 car 车  2 arrow 桥洞箭头  3 target_red 友方靶
    4 cell_label 格号文字  5 target_blue 敌方靶  6 obstacle 障碍(桶等)
注意官方模型不含「黄色桥洞本体」「起点/终点橙格」，这些必须人工补标。

输出（每张图一个 txt，归一化坐标）：
    class cx cy w h
可选可视化（--draw-dir）：把检测框画到图上，供抽查草稿质量。

用法：
    python tools/ml/prelabel_onnx.py --model models/release/hbb_detector.onnx \\
        --images capture/aerial_field/images --out capture/aerial_field/labels_draft
退出码：0 成功；2 参数/依赖；3 模型或图片问题。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("需要 opencv-python / numpy", file=sys.stderr)
    raise SystemExit(2)


CLASS_NAMES = ["mine", "car", "arrow", "target_red",
               "cell_label", "target_blue", "obstacle"]


# ---------------------------------------------------------------- 纯函数（可测）

def letterbox(image, size: int):
    """等比缩放 + 灰边填充到 size×size；返回 (canvas, scale, (padx,pady))。"""
    h, w = image.shape[:2]
    scale = min(size / w, size / h)
    new_w, new_h = round(w * scale), round(h * scale)
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
    canvas[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = resized
    return canvas, scale, (pad_x, pad_y)


def decode_detect(output, conf_thr: float, iou_thr: float,
                  num_classes: int = 7):
    """把 [1,C,N]（C=4+nc）解码为 (boxes, scores, class_ids)。

    boxes  [M,4] cx,cy,w,h（size 坐标系）
    scores [M]
    class_ids [M]
    NMS 按类别分别做，不同类重叠框都保留。
    """
    arr = np.squeeze(output, axis=0)
    if arr.ndim != 2:
        raise ValueError(f"输出维度异常: {arr.shape}")
    channels, _anchors = arr.shape
    if channels != 4 + num_classes:
        raise ValueError(
            f"通道数 {channels} != 4+{num_classes}类={4 + num_classes}"
        )

    arr = arr.T  # [N, C]
    cls_block = arr[:, 4:4 + num_classes]
    class_ids = np.argmax(cls_block, axis=1)
    scores = cls_block.max(axis=1)

    keep_mask = scores > conf_thr
    arr, class_ids, scores = arr[keep_mask], class_ids[keep_mask], scores[keep_mask]
    if arr.size == 0:
        return np.zeros((0, 4)), np.zeros(0), np.zeros(0, dtype=np.int64)

    boxes = arr[:, :4].copy()

    keep_indices: list[int] = []
    for c in range(num_classes):
        local = np.where(class_ids == c)[0]
        if len(local) == 0:
            continue
        rects = []
        for i in local:
            cx, cy, bw, bh = boxes[i]
            rects.append([cx - bw / 2, cy - bh / 2, bw, bh])
        selected = cv2.dnn.NMSBoxes(
            rects, scores[local].tolist(), conf_thr, iou_thr
        )
        if len(selected) > 0:
            keep_indices.extend(local[np.array(selected).flatten()].tolist())
    keep_indices = sorted(keep_indices)
    if not keep_indices:
        return np.zeros((0, 4)), np.zeros(0), np.zeros(0, dtype=np.int64)
    return boxes[keep_indices], scores[keep_indices], class_ids[keep_indices]


def to_yolo_lines(boxes, scores, class_ids, scale: float, pad,
                  orig_w: int, orig_h: int) -> list[str]:
    """size 坐标系 → 原图 → 归一化，生成 YOLO 检测行（无关键点）。"""
    pad_x, pad_y = pad
    lines = []
    for (cx, cy, bw, bh), _score, cls in zip(boxes, scores, class_ids):
        cx = (cx - pad_x) / scale
        cy = (cy - pad_y) / scale
        bw, bh = bw / scale, bh / scale
        cx_n = min(max(cx / orig_w, 0), 1)
        cy_n = min(max(cy / orig_h, 0), 1)
        w_n = min(bw / orig_w, 1)
        h_n = min(bh / orig_h, 1)
        lines.append(f"{int(cls)} {cx_n:.6f} {cy_n:.6f} {w_n:.6f} {h_n:.6f}")
    return lines


# ---------------------------------------------------------------- 可视化

_DRAW_COLORS = {
    0: (0, 255, 255),    # mine 黄
    1: (255, 128, 0),    # car 橙
    2: (0, 255, 0),      # arrow 绿
    3: (0, 0, 255),      # target_red 红
    4: (255, 0, 255),    # cell_label 紫
    5: (255, 0, 0),      # target_blue 蓝(按 BGR 显示)
    6: (200, 200, 200),  # obstacle 灰白
}


def draw_predictions(image, boxes, scores, class_ids, scale: float, pad):
    """把草稿框画到原图（返回 BGR 图副本）。"""
    pad_x, pad_y = pad
    canvas = image.copy()
    for (cx, cy, bw, bh), score, cls in zip(boxes, scores, class_ids):
        x1 = int((cx - bw / 2 - pad_x) / scale)
        y1 = int((cy - bh / 2 - pad_y) / scale)
        x2 = int((cx + bw / 2 - pad_x) / scale)
        y2 = int((cy + bh / 2 - pad_y) / scale)
        color = _DRAW_COLORS.get(int(cls), (255, 255, 255))
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 4)
        label = f"{CLASS_NAMES[int(cls)]} {score:.2f}"
        cv2.putText(canvas, label, (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    return canvas


# ---------------------------------------------------------------- 主流程

def build_session(model_path: str, use_gpu: bool):
    import onnxruntime as ort

    providers = ["CUDAExecutionProvider", "CPUExecutionProvider"] if use_gpu \
        else ["CPUExecutionProvider"]
    session = ort.InferenceSession(model_path, providers=providers)
    return session


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", required=True)
    parser.add_argument("--images", required=True)
    parser.add_argument("--out", required=True, help="草稿标签输出目录")
    parser.add_argument("--size", type=int, default=1280)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--cpu", action="store_true", help="强制 CPU 推理")
    parser.add_argument("--draw-dir", help="同时输出画框可视化图到该目录")
    parser.add_argument("--limit", type=int, help="只处理前 N 张（试跑用）")
    args = parser.parse_args()

    image_paths = sorted(Path(args.images).glob("*.jpg"))
    if args.limit:
        image_paths = image_paths[: args.limit]
    if not image_paths:
        print("图片目录无 jpg", file=sys.stderr)
        return 2

    session = build_session(args.model, use_gpu=not args.cpu)
    input_name = session.get_inputs()[0].name

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    draw_dir = Path(args.draw_dir) if args.draw_dir else None
    if draw_dir:
        draw_dir.mkdir(parents=True, exist_ok=True)

    counts = {name: 0 for name in CLASS_NAMES}
    empty = 0
    for path in image_paths:
        image = cv2.imread(str(path))
        if image is None:
            continue
        h, w = image.shape[:2]
        canvas_bgr, scale, pad = letterbox(image, args.size)
        # ultralytics ONNX 期望 RGB、0~1、NCHW
        rgb = cv2.cvtColor(canvas_bgr, cv2.COLOR_BGR2RGB)
        blob = rgb.transpose(2, 0, 1)  # HWC RGB → CHW
        blob = np.ascontiguousarray(blob, dtype=np.float32) / 255.0
        output = session.run(None, {input_name: blob[None]})
        boxes, scores, class_ids = decode_detect(
            output[0], args.conf, args.iou, len(CLASS_NAMES)
        )
        lines = to_yolo_lines(boxes, scores, class_ids, scale, pad, w, h)
        (out_dir / (path.stem + ".txt")).write_text(
            "\n".join(lines), encoding="utf-8"
        )
        if not lines:
            empty += 1
        for cls in class_ids:
            counts[CLASS_NAMES[int(cls)]] += 1

        if draw_dir:
            vis = draw_predictions(image, boxes, scores, class_ids, scale, pad)
            cv2.imwrite(str(draw_dir / path.name), vis)

    print(f"处理 {len(image_paths)} 张，各类草稿数："
          + " ".join(f"{k}={v}" for k, v in counts.items() if v))
    print(f"空标签 {empty} 张；草稿标签 → {out_dir.resolve()}")
    if draw_dir:
        print(f"可视化 → {draw_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
