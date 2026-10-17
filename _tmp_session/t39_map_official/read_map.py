# -*- coding: utf-8 -*-
r"""t39 官方地图逐格识读（重写版：格网锚点来自**蓝色外框内边** + 灰网格线最小二乘）。

- 只读 map_official.png；产物只落本目录。
- 格名约定由图中四条边的刻度确认：列 A→N 自左向右、行 1→11 自上向下，A1 在左上。
"""

from __future__ import annotations

import json
import pathlib
import sys

import numpy as np
from PIL import Image

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, r"D:\User\Desktop\比赛存档\v1\dzsmartscout\tools")
from perception.colors import as_rgb_array, blob_masks      # noqa: E402
from perception.grid import GridSpec                        # noqa: E402

IMG = HERE / "map_official.png"
SPEC = GridSpec()


def masks_of(img):
    r, g, b = (img[..., i].astype(np.int16) for i in range(3))
    mx = np.max(img, axis=2).astype(np.int16)
    mn = np.min(img, axis=2).astype(np.int16)
    sat = mx - mn
    luma = img.astype(np.float64) @ np.array([0.299, 0.587, 0.114])
    return {
        "red": (r > 200) & (g < 140) & (b < 140) & (sat > 80),
        "yellow": (r > 210) & (g > 170) & (b < 150) & (sat > 80),
        "magenta": (r > 190) & (b > 150) & (g < 160) & (sat > 60),
        "green": (g > 150) & (r < 130) & (b < 130) & (sat > 60),
        "blue": (b > r + 30) & (b > g + 10) & (b > 90),
        "grayline": ((np.abs(r - 208) < 16) & (np.abs(g - 208) < 16)
                     & (np.abs(b - 208) < 16)),
        "luma": luma,
    }


def lattice_from_frame(m: dict, img: np.ndarray) -> dict:
    """蓝框内边定 arena 外接矩形；灰网格线做最小二乘精修格距/相位。"""
    blue = m["blue"]
    H, W = blue.shape
    # 左右两条竖直蓝带的内边（对每行取最左/最右蓝段）
    left_in, right_in, top_in, bot_in = [], [], [], []
    for y in range(H):
        xs = np.flatnonzero(blue[y])
        if xs.size == 0:
            continue
        # 左侧连续段
        k = 0
        while k + 1 < xs.size and xs[k + 1] - xs[k] <= 2:
            k += 1
        left_in.append(int(xs[k]))
        k2 = xs.size - 1
        while k2 - 1 >= 0 and xs[k2] - xs[k2 - 1] <= 2:
            k2 -= 1
        right_in.append(int(xs[k2]))
    for x in range(W):
        ys = np.flatnonzero(blue[:, x])
        if ys.size == 0:
            continue
        k = 0
        while k + 1 < ys.size and ys[k + 1] - ys[k] <= 2:
            k += 1
        top_in.append(int(ys[k]))
        k2 = ys.size - 1
        while k2 - 1 >= 0 and ys[k2] - ys[k2 - 1] <= 2:
            k2 -= 1
        bot_in.append(int(ys[k2]))
    x_left = int(np.median(left_in)) + 1
    x_right = int(np.median(right_in)) - 1
    y_top = int(np.median(top_in)) + 1
    y_bot = int(np.median(bot_in)) - 1
    s_x0 = (x_right - x_left + 1) / SPEC.cols
    s_y0 = (y_bot - y_top + 1) / SPEC.rows

    def refine(mask2d, a0, s0, n, axis):
        proj = mask2d.sum(axis=axis)
        best = None
        for s in np.arange(s0 - 0.6, s0 + 0.6, 0.002):
            for a in np.arange(a0 - 2.0, a0 + 2.0, 0.05):
                idx = np.arange(0, n + 1)
                pos = a + idx * s
                ok = (pos >= 0) & (pos < (len(proj) - 1))
                if ok.sum() < n:
                    continue
                vals = [max(proj[int(round(p)) - 1], proj[int(round(p))],
                            proj[min(len(proj) - 1, int(round(p)) + 1)])
                        for p in pos[ok]]
                score = float(np.mean(vals))
                if best is None or score > best[0]:
                    best = (score, float(a), float(s))
        return best[1], best[2], best[0]

    x0, sx, sc_x = refine(m["grayline"], float(x_left), s_x0, SPEC.cols, axis=0)
    y0, sy, sc_y = refine(m["grayline"].T, float(y_top), s_y0, SPEC.rows, axis=0)
    return {"x0": round(x0, 3), "y0": round(y0, 3), "sx": round(sx, 4),
            "sy": round(sy, 4), "frame_inner": [x_left, y_top, x_right, y_bot],
            "line_score_x": round(sc_x, 1), "line_score_y": round(sc_y, 1),
            "cell_px": round((sx + sy) / 2, 3)}


def cell_rect(lat, ci, ri):
    x0 = int(round(lat["x0"] + ci * lat["sx"]))
    y0 = int(round(lat["y0"] + ri * lat["sy"]))
    x1 = int(round(lat["x0"] + (ci + 1) * lat["sx"]))
    y1 = int(round(lat["y0"] + (ri + 1) * lat["sy"]))
    return x0, y0, x1, y1


def main() -> int:
    img = as_rgb_array(Image.open(IMG))
    m = masks_of(img)
    lat = lattice_from_frame(m, img)
    print("格网:", json.dumps(lat, ensure_ascii=False))

    classes = {"obstacle": "red", "mine_or_box": "yellow",
               "target": "magenta", "bridge": "green"}
    # ---- 逐格覆盖率矩阵 ----
    cov = {c: np.zeros((SPEC.rows, SPEC.cols)) for c in classes}
    for ri in range(SPEC.rows):
        for ci in range(SPEC.cols):
            x0, y0, x1, y1 = cell_rect(lat, ci, ri)
            for c, mk in classes.items():
                sub = m[mk][y0:y1, x0:x1]
                cov[c][ri, ci] = float(sub.mean()) if sub.size else 0.0

    # ---- 黄块分流：雷（纯色）vs 标注框（含深色文字）----
    luma = m["luma"]
    yellow_blobs = blob_masks(m["yellow"], min_pixels=150, k_close=1, rgb=img)

    def to_cell(cx, cy):
        return (int((cx - lat["x0"]) / lat["sx"]), int((cy - lat["y0"]) / lat["sy"]))

    mines, boxes = [], []
    for b in yellow_blobs:
        ci, ri = to_cell(b.cx, b.cy)
        block = SPEC.index_to_block(ri, ci)
        x0, y0, x1, y1 = cell_rect(lat, ci, ri)
        cell_area = max(1, (x1 - x0) * (y1 - y0))
        sub = luma[b.y0:b.y1 + 1, b.x0:b.x1 + 1]
        dark = float((sub < 150).mean())
        item = {"block": block, "row_idx": ri, "col_idx": ci,
                "bbox": [b.x0, b.y0, b.x1, b.y1], "area_px": int(b.area),
                "cell_frac": round(b.area / cell_area, 3),
                "fill": round(b.fill, 3), "dark_frac": round(dark, 4),
                "mean_rgb": [round(v, 1) for v in b.mean_rgb],
                "cell_coverage": round(float(cov["mine_or_box"][ri, ci]), 3),
                "confidence": round(min(1.0, float(cov["mine_or_box"][ri, ci]) / 0.6), 3)}
        (boxes if dark > 0.02 else mines).append(item)

    def cells_from_cov(key, thr=0.30):
        out = []
        for ri in range(SPEC.rows):
            for ci in range(SPEC.cols):
                v = float(cov[key][ri, ci])
                if v >= thr:
                    out.append({"block": SPEC.index_to_block(ri, ci),
                                "row_idx": ri, "col_idx": ci,
                                "coverage": round(v, 3),
                                "confidence": round(min(1.0, v / 0.6), 3)})
        return out

    result = {
        "image": str(IMG), "image_size": [img.shape[1], img.shape[0]],
        "lattice": lat, "coverage_threshold": 0.30,
        "mines": sorted(mines, key=lambda d: (d["row_idx"], d["col_idx"])),
        "label_boxes": sorted(boxes, key=lambda d: (d["row_idx"], d["col_idx"])),
        "obstacles": cells_from_cov("obstacle"),
        "targets": cells_from_cov("target"),
        "bridges": cells_from_cov("bridge"),
        "coverage": {k: [[round(float(v), 3) for v in row] for row in cov[k]]
                     for k in cov},
    }
    (HERE / "read_map_result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8", newline="\n")

    print("\n雷:", [d["block"] for d in result["mines"]])
    print("标注框:", [(d["block"], d["dark_frac"], d["area_px"]) for d in result["label_boxes"]])
    print("障:", [d["block"] for d in result["obstacles"]])
    print("靶:", [d["block"] for d in result["targets"]])
    print("通:", [d["block"] for d in result["bridges"]])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
