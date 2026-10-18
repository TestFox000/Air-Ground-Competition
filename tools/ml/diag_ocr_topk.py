"""OCR NCC 低分根因诊断：对大字 crop 输出 top-5 匹配，验证 Arial 模板假设。

用法：
    python tools/ml/diag_ocr_topk.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from ultralytics import YOLO

sys.path.insert(0, str(Path(__file__).parent))
from ocr_cell_prototype import (  # noqa: E402
    CELL_LABEL_CLS, CONF_DET, DEVICE, IOU_DET, OUT_DIR, VAL_DIR,
    YOLO_WEIGHTS, square_crop, render_cell_templates, UPSCALE,
    MIN_TEXT_RATIO, MAX_TEXT_RATIO, ncc_match, TMPL_HEIGHT,
)

# 待诊断的 (帧, 框索引, v4c 结果)——选 text_h 大、文字清晰的案例
CASES = [
    ("frame_00170.jpg", 1, "H2"),   # text_h=72px
    ("frame_00170.jpg", 2, "H3"),   # text_h=60px
    ("frame_00017.jpg", 1, "M5"),   # text_h=112px luma 路径
    ("frame_00022.jpg", None, "N3"),  # 取该帧 conf 最高的框
    ("frame_00175.jpg", None, "H5"),  # 全场最高 NCC 0.49 的案例
]


def extract_text_mask_tight(crop: Image.Image) -> tuple[np.ndarray, float, str] | None:
    """收紧版集聚合：垂直重叠 + 高度相似约束，抗噪点膨胀。"""
    arr = cv2.cvtColor(np.array(crop), cv2.COLOR_RGB2BGR)
    big = cv2.resize(arr, None, fx=UPSCALE, fy=UPSCALE, interpolation=cv2.INTER_CUBIC)
    total = big.shape[0] * big.shape[1]
    hsv = cv2.cvtColor(big, cv2.COLOR_BGR2HSV)
    mask_hsv = cv2.inRange(hsv, (95, 30, 20), (150, 255, 180))
    n_blue = int((mask_hsv > 0).sum())
    if total * 0.002 <= n_blue <= total * 0.20:
        mask, path = mask_hsv, "hsv"
    else:
        lab = cv2.cvtColor(big, cv2.COLOR_BGR2LAB)
        l = cv2.createCLAHE(2.0, (8, 8)).apply(lab[:, :, 0])
        mask, path = cv2.adaptiveThreshold(l, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                           cv2.THRESH_BINARY_INV, 31, 8), "luma"
    k_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)
    from ocr_cell_prototype import _largest_block_h
    h_est = _largest_block_h(mask)
    if h_est < 8:
        return None
    k_close = cv2.getStructuringElement(
        cv2.MORPH_RECT, (max(3, int(h_est * 0.35)), max(3, int(h_est * 0.25))))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)

    n_lab, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n_lab <= 1:
        return None
    areas = stats[1:, cv2.CC_STAT_AREA]
    seed = 1 + int(np.argmax(areas))
    sx, sy, sw, sh = (stats[seed, i] for i in
                      (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                       cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
    clus = [(sx, sy, sw, sh)]
    for j in range(1, n_lab):
        if j == seed:
            continue
        x, y, w_, h_ = (stats[j, i] for i in
                        (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                         cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
        # 高度相似（同行字符）: 0.35~1.6 倍字符高
        if not (0.35 * sh <= h_ <= 1.6 * sh):
            continue
        # 垂直重叠或紧邻: 块间垂直间隙 ≤ 0.3 倍字符高
        gap_v = max(y - (sy + sh), sy - (y + h_))
        if gap_v > 0.3 * sh:
            continue
        # 水平在 seed 附近 ±2.5 倍字符高内
        if x > sx + sw + 2.5 * sh or x + w_ < sx - 2.5 * sh:
            continue
        if stats[j, cv2.CC_STAT_AREA] < max(30.0, 0.05 * sh * sh):
            continue
        clus.append((x, y, w_, h_))
    X1 = min(b[0] for b in clus)
    Y1 = min(b[1] for b in clus)
    X2 = max(b[0] + b[2] for b in clus)
    Y2 = max(b[1] + b[3] for b in clus)
    if Y2 - Y1 < 6 or X2 - X1 < 4:
        return None
    return mask[Y1:Y2 + 1, X1:X2 + 1], (Y2 - Y1 + 1) / UPSCALE, path


def topk_match(text_img: np.ndarray, templates: dict, k: int = 5):
    h, w = text_img.shape
    scaled = cv2.resize(text_img, (max(2, int(round(w * TMPL_HEIGHT / h))), TMPL_HEIGHT),
                       interpolation=cv2.INTER_AREA)
    scores = []
    for cell, variants in templates.items():
        best = max(ncc_match(scaled, t) for t in variants)
        scores.append((best, cell))
    scores.sort(reverse=True)
    return scores[:k]


def main() -> None:
    print("[1/3] 渲染模板...")
    templates = render_cell_templates()
    print("[2/3] 加载 YOLO...")
    yolo = YOLO(str(YOLO_WEIGHTS))
    print("[3/3] 诊断案例...")
    dbg = OUT_DIR / "diag_topk"
    dbg.mkdir(parents=True, exist_ok=True)
    for frame_name, idx, expect in CASES:
        img_path = VAL_DIR / frame_name
        results = yolo.predict(source=str(img_path), conf=CONF_DET, iou=IOU_DET,
                               classes=[CELL_LABEL_CLS], verbose=False, device=DEVICE)
        boxes = results[0].boxes
        img = Image.open(img_path).convert("RGB")
        n = len(boxes)
        i = idx if idx is not None else 0
        x1, y1, x2, y2 = boxes.xyxy[i].tolist()
        crop = square_crop(img, x1, y1, x2, y2)
        got = extract_text_mask_tight(crop)
        if got is None:
            print(f"{frame_name} #{i} (期望 {expect}): 提取失败")
            continue
        text_img, text_h, path = got
        ratio = text_h / crop.height
        # 保存文字块可视化
        cv2.imencode(".png", text_img)[1].tofile(str(dbg / f"{Path(frame_name).stem}_{i}.png"))
        if not (MIN_TEXT_RATIO <= ratio <= MAX_TEXT_RATIO):
            print(f"{frame_name} #{i} (期望 {expect}): ratio={ratio:.2f} 出界 "
                  f"text_h={text_h:.1f} path={path}")
            continue
        top5 = topk_match(text_img, templates)
        rank = next((r for r, (s, c) in enumerate(top5, 1) if c == expect), None)
        print(f"{frame_name} #{i} (期望 {expect}): text_h={text_h:.1f} path={path} ratio={ratio:.2f}")
        print(f"    top5: {[(c, round(s, 3)) for s, c in top5]}  期望排名: {rank}")


if __name__ == "__main__":
    main()
