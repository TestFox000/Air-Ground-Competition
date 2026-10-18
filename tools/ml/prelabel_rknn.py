"""在香橙派（rk3588）上用官方 RKNN 模型批量预标注分帧图片。

用途：人工从零画框/点很贵。官方 best_new.rknn 虽然是车端视角训练的，
但先用它对抽出的帧跑一遍，输出 YOLO-pose 标签，人工只需「校正」而非「从零标」。
本脚本在车端运行（依赖 rknn-toolkit-lite2，车端一般已装）。

输出（每张图一个 txt，归一化坐标）：
    class cx cy w h x1 y1 v1 x2 y2 v2 ...

通道布局（官方 best_new.rknn 输出 [1,9,8400]，与车端 light_det.rknn 同文件）：
    [0:4]           cx cy w h
    [4:4+nc]        类别分数（官方 nc=2：0=蓝面敌方 1=红面友方）
    [4+nc:...]      关键点，每点 x y v（官方 nk=1：靶心）
    即标准 ultralytics-pose 公式：通道数 = 4 + nc + nk*3
    兼容 8 通道模型（4+nc+nk*2，无 visibility 分）：解码时 v 自动补 1.0

用法（车端）：
    python3 prelabel_rknn.py --model ~/models/best_new.rknn \
        --images ~/prelabel/images --out ~/prelabel/labels --size 640
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("需要 opencv-python / numpy（车端一般自带）", file=sys.stderr)
    raise SystemExit(2)


# ---------------------------------------------------------------- 预处理

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


# ---------------------------------------------------------------- 输出解析

def decode_output(output, conf_thr: float, iou_thr: float,
                  num_classes: int = 2, num_kpts: int = 1):
    """把 [1,C,8400]（或 [C,8400]）解码为 (boxes, scores, kpts, class_ids)。

    支持两种通道布局（自动判别）：
      * 含 visibility：C = 4 + num_classes + num_kpts*3（标准 ultralytics-pose，9 通道）
      * 无 visibility：C = 4 + num_classes + num_kpts*2（8 通道模型，v 统一补 1.0）
    返回：
        boxes     [N,4]  cx,cy,w,h（640 坐标系）
        scores    [N]    各类别中的最大分类分
        kpts      [N,nk,3] x,y,v（640 坐标系，无 visibility 时 v=1.0）
        class_ids [N]    argmax 类别号
    """
    arr = np.squeeze(output, axis=0)
    if arr.ndim != 2:
        raise ValueError(f"输出维度异常: {arr.shape}")
    channels, anchors = arr.shape
    expected_viz = 4 + num_classes + num_kpts * 3
    expected_noviz = 4 + num_classes + num_kpts * 2
    if channels == expected_viz:
        has_visibility = True
    elif channels == expected_noviz:
        has_visibility = False
    else:
        raise ValueError(
            f"通道数 {channels} 与 4+{num_classes}类+{num_kpts}点 不符"
            f"（含visibility={expected_viz} / 无visibility={expected_noviz}）；"
            "用 --nc / --nkpt 指定模型真实布局"
        )

    arr = arr.T  # [8400, C]

    cls_block = arr[:, 4:4 + num_classes]
    class_ids = np.argmax(cls_block, axis=1)
    scores = cls_block.max(axis=1)
    keep_mask = scores > conf_thr
    arr = arr[keep_mask]
    class_ids = class_ids[keep_mask]
    scores = scores[keep_mask]
    if arr.size == 0:
        return (np.zeros((0, 4)), np.zeros(0),
                np.zeros((0, num_kpts, 3)), np.zeros(0, dtype=np.int64))

    boxes = arr[:, :4].copy()
    raw_kpts = arr[:, 4 + num_classes:]
    if has_visibility:
        kpts = raw_kpts.reshape(-1, num_kpts, 3)
    else:
        # 8 通道模型无 visibility 分数，统一补 1.0（靶心恒可见）
        xy = raw_kpts.reshape(-1, num_kpts, 2)
        v = np.ones((xy.shape[0], num_kpts, 1), dtype=xy.dtype)
        kpts = np.concatenate([xy, v], axis=2)

    # 按类别分别 NMS（不同类的重叠框都要保留）
    keep_indices: list[int] = []
    for c in range(num_classes):
        local = np.where(class_ids == c)[0]
        if len(local) == 0:
            continue
        top_left = []
        for i in local:
            cx, cy, bw, bh = boxes[i]
            top_left.append([cx - bw / 2, cy - bh / 2, bw, bh])
        selected = cv2.dnn.NMSBoxes(
            top_left, scores[local].tolist(), conf_thr, iou_thr
        )
        if len(selected) > 0:
            keep_indices.extend(local[np.array(selected).flatten()].tolist())
    keep_indices = sorted(keep_indices)
    if not keep_indices:
        return (np.zeros((0, 4)), np.zeros(0),
                np.zeros((0, num_kpts, 3)), np.zeros(0, dtype=np.int64))
    return (boxes[keep_indices], scores[keep_indices],
            kpts[keep_indices], class_ids[keep_indices])


def to_yolo_lines(boxes, scores, kpts, class_ids, scale: float, pad,
                  orig_w: int, orig_h: int) -> list[str]:
    """640 坐标系 → 原图坐标 → 归一化，生成 YOLO-pose 行。"""
    pad_x, pad_y = pad
    lines = []
    for box, score, points, cls in zip(boxes, scores, kpts, class_ids):
        cx, cy, w, h = box
        # 去 padding → 去 scale
        cx = (cx - pad_x) / scale
        cy = (cy - pad_y) / scale
        w, h = w / scale, h / scale
        cx_n = min(max(cx / orig_w, 0), 1)
        cy_n = min(max(cy / orig_h, 0), 1)
        w_n = min(w / orig_w, 1)
        h_n = min(h / orig_h, 1)
        parts = [f"{int(cls)} {cx_n:.6f} {cy_n:.6f} {w_n:.6f} {h_n:.6f}"]
        for px, py, pv in points:
            ox = min(max((px - pad_x) / scale, 0), orig_w) / orig_w
            oy = min(max((py - pad_y) / scale, 0), orig_h) / orig_h
            parts.append(f"{ox:.6f} {oy:.6f} {float(pv):.4f}")
        lines.append(" ".join(parts))
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", required=True)
    parser.add_argument("--images", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--size", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--nc", type=int, default=2,
                        help="类别数（官方 best_new=2：蓝/红）")
    parser.add_argument("--nkpt", type=int, default=1,
                        help="每目标关键点数（官方=1：靶心）")
    args = parser.parse_args()

    try:
        from rknnlite.api import RKNNLite
    except ImportError:
        print("车端缺 rknn-toolkit-lite2", file=sys.stderr)
        return 2

    rknn = RKNNLite()
    if rknn.load_rknn(args.model) != 0 or rknn.init_runtime() != 0:
        print("模型加载/运行时初始化失败", file=sys.stderr)
        return 3

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    image_paths = sorted(
        p for p in Path(args.images).glob("*.jpg")
    )
    total_objects = 0
    for path in image_paths:
        image = cv2.imread(str(path))
        if image is None:
            continue
        h, w = image.shape[:2]
        canvas, scale, pad = letterbox(image, args.size)
        outputs = rknn.inference(inputs=[canvas[None]])
        boxes, scores, kpts, class_ids = decode_output(
            outputs[0], args.conf, args.iou, args.nc, args.nkpt
        )
        lines = to_yolo_lines(boxes, scores, kpts, class_ids,
                              scale, pad, w, h)
        (out_dir / (path.stem + ".txt")).write_text(
            "\n".join(lines), encoding="utf-8"
        )
        total_objects += len(lines)

    rknn.release()
    print(f"处理 {len(image_paths)} 张图，预标注目标 {total_objects} 个 → {out_dir}")
    print("下一步：人工校正 labels（X-AnyLabeling 支持 YOLO-pose 格式）。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
