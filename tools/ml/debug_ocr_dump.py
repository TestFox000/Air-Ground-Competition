"""OCR 原型调试：dump mask 提取/分割的中间产物，定位 I1/I7 系统性误判的根因。

用法：
    python tools/ml/debug_ocr_dump.py --frames frame_00017 --n-dump 10
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

sys.path.insert(0, str(Path(__file__).parent))
from ocr_cell_prototype import (  # noqa: E402
    CELL_LABEL_CLS, CONF_DET, DEVICE, IOU_DET, OUT_DIR, ROOT, V4C_JSON,
    VAL_DIR, YOLO_WEIGHTS, square_crop, UPSCALE, _largest_block_h,
)
import json


def dump_frame(frame_name: str, n_dump: int) -> None:
    import json as _json
    v4c_by_frame = {r["name"]: r for r in _json.loads(V4C_JSON.read_text(encoding="utf-8"))}
    v4c = v4c_by_frame[frame_name]
    img_path = VAL_DIR / frame_name
    yolo = YOLO(str(YOLO_WEIGHTS))
    results = yolo.predict(source=str(img_path), conf=CONF_DET, iou=IOU_DET,
                           classes=[CELL_LABEL_CLS], verbose=False, device=DEVICE)
    img = Image.open(img_path).convert("RGB")
    dbg_dir = OUT_DIR / "debug" / Path(frame_name).stem
    dbg_dir.mkdir(parents=True, exist_ok=True)

    # 挑要 dump 的框：OCR 有分歧的 + OCR 失败的各一半，按 det_conf 对齐 v4c
    boxes = results[0].boxes
    n = len(boxes)
    dump_idx = list(range(min(n_dump, n)))

    for i in dump_idx:
        x1, y1, x2, y2 = boxes.xyxy[i].tolist()
        det_conf = float(boxes.conf[i])
        crop = square_crop(img, x1, y1, x2, y2)
        v4c_id = v4c["dets"][i]["cell_id"] if i < len(v4c["dets"]) else None

        # --- 中间产物（v2 双路径） ---
        arr = cv2.cvtColor(np.array(crop), cv2.COLOR_RGB2BGR)
        big = cv2.resize(arr, None, fx=UPSCALE, fy=UPSCALE, interpolation=cv2.INTER_CUBIC)
        hsv = cv2.cvtColor(big, cv2.COLOR_BGR2HSV)
        mask_hsv = cv2.inRange(hsv, (95, 30, 20), (150, 255, 180))
        n_blue = int((mask_hsv > 0).sum())
        total = big.shape[0] * big.shape[1]
        lab = cv2.cvtColor(big, cv2.COLOR_BGR2LAB)
        l = cv2.createCLAHE(2.0, (8, 8)).apply(lab[:, :, 0])
        mask_luma = cv2.adaptiveThreshold(l, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                          cv2.THRESH_BINARY_INV, 31, 8)
        if total * 0.002 <= n_blue <= total * 0.20:
            mask, path = mask_hsv, "hsv"
        else:
            mask, path = mask_luma, "luma"
        # v2 清理：开 → 估字符高（最大块） → 闭 → 开
        k_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        m_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)
        h_est = _largest_block_h(m_clean)
        if h_est >= 8:
            k_close = cv2.getStructuringElement(
                cv2.MORPH_RECT, (max(3, int(h_est * 0.35)), max(3, int(h_est * 0.25))))
            m_clean = cv2.morphologyEx(m_clean, cv2.MORPH_CLOSE, k_close)
            m_clean = cv2.morphologyEx(m_clean, cv2.MORPH_OPEN, k_open)

        # 文字块定位：seed 聚集合同款
        n_lab, labels, stats, _ = cv2.connectedComponentsWithStats(m_clean, connectivity=8)
        if n_lab > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            seed = 1 + int(np.argmax(areas))
            sx, sy, sw, sh = (stats[seed, i] for i in
                              (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                               cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
            min_area = max(20.0, 0.02 * sh * sh)
            clus = []
            for j in range(1, n_lab):
                x, y, w_, h_ = (stats[j, i] for i in
                                (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                                 cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
                v_ok = (y < sy + sh + 0.8 * sh) and (y + h_ > sy - 0.8 * sh)
                h_ok = (x < sx + sw + 2.5 * sh) and (x + w_ > sx - 2.5 * sh)
                if v_ok and h_ok and stats[j, cv2.CC_STAT_AREA] >= min_area:
                    clus.append((x, y, w_, h_))
            if clus:
                X1 = min(b[0] for b in clus)
                Y1 = min(b[1] for b in clus)
                X2 = max(b[0] + b[2] for b in clus)
                Y2 = max(b[1] + b[3] for b in clus)
                text_img = m_clean[Y1:Y2 + 1, X1:X2 + 1]
                vis_clus = np.zeros_like(m_clean)
                vis_clus[Y1:Y2 + 1, X1:X2 + 1] = text_img
            else:
                vis_clus = m_clean
                text_img = np.zeros((1, 1), np.uint8)
        else:
            vis_clus = m_clean
            text_img = np.zeros((1, 1), np.uint8)
        text_h = text_img.shape[0] / UPSCALE

        # --- 拼图：原图 | 放大 | HSV mask | 用的 mask | 聚集文字块 ---
        pad = 6
        crop_rgb = cv2.cvtColor(np.array(crop), cv2.COLOR_RGB2BGR)
        h0 = big.shape[0]
        panels = [cv2.resize(crop_rgb, (h0, h0)), big,
                  cv2.cvtColor(mask_hsv, cv2.COLOR_GRAY2BGR),
                  cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
                  cv2.cvtColor(vis_clus, cv2.COLOR_GRAY2BGR)]
        canvas = np.full((h0, sum(p.shape[1] for p in panels) + pad * (len(panels) + 1), 3), 40, np.uint8)
        x = pad
        for p in panels:
            canvas[:, x:x + p.shape[1]] = p
            x += p.shape[1] + pad
        # 标注
        cv2.putText(canvas, f"#{i} v4c={v4c_id} conf={det_conf:.2f} path={path} "
                    f"text_h={text_h:.1f}px h_est={h_est}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        labels_txt = ["crop", "4x", "hsv", "mask", "roi"]
        x = pad
        for lt, p in zip(labels_txt, panels):
            cv2.putText(canvas, lt, (x + 4, h0 - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            x += p.shape[1] + pad
        out = dbg_dir / f"case_{i:02d}.png"
        cv2.imencode(".png", canvas)[1].tofile(str(out))
        print(f"  #{i}: v4c={v4c_id} det={det_conf:.2f} path={path} text_h={text_h:.1f}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", nargs="+", default=["frame_00017.jpg"])
    ap.add_argument("--n-dump", type=int, default=10)
    args = ap.parse_args()
    for f in args.frames:
        print(f"dump {f} ...")
        dump_frame(f, args.n_dump)
    print(f"输出目录：{OUT_DIR / 'debug'}")


if __name__ == "__main__":
    main()
