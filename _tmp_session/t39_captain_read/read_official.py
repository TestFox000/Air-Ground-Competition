#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""队长独立复核：官方贴图 map_official.png（654x522）确定性识读。
方法：官方图按 912/654 放大到与 t28src 同宽 → 复用 t28 识图三证据（色块几何/标签可见/块内文字）。
输出：本目录 official_identify.txt。只读官方图，不写任何其它目录。
"""
from __future__ import annotations
import os
from collections import deque
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t39_map_official\map_official.png"
OUT = os.path.join(HERE, "official_identify.txt")
COL = "ABCDEFGHIJKLMN"
NCOL, ROWS = 14, 11

im = Image.open(SRC).convert("RGB")
W, H = im.size
SCALE = 912.0 / W
im2 = im.resize((int(round(W * SCALE)), int(round(H * SCALE))), Image.LANCZOS)
a = np.asarray(im2).astype(np.int16)
W2, H2, _ = a.shape
r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
lum = a.mean(axis=2)

masks = {
    "red": (r > 170) & (g < 110) & (b < 110),
    "yel": (r > 190) & (g > 160) & (b < 130),
    "mag": (r > 170) & (g < 130) & (b > 150),
    "grn": (r < 140) & (g > 120) & (b < 150),
}

def comps(mask, minpix=400):
    HH, WW = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    out = []
    for y0 in range(HH):
        for x0 in np.where(mask[y0] & ~seen[y0])[0]:
            if seen[y0, x0]:
                continue
            q, xs, ys = deque([(y0, x0)]), [], []
            seen[y0, x0] = True
            while q:
                y, x = q.popleft()
                xs.append(x); ys.append(y)
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < HH and 0 <= nx < WW and mask[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        q.append((ny, nx))
            if len(xs) >= minpix:
                out.append(dict(px=len(xs), x0=min(xs), x1=max(xs), y0=min(ys), y1=max(ys),
                                cx=float(np.mean(xs)), cy=float(np.mean(ys))))
    return sorted(out, key=lambda d: (d["y0"], d["x0"]))

L = ["=== 队长独立识读 official（放大 %.4fx → %dx%d）===" % (SCALE, W2, H2)]
raw = {k: comps(m) for k, m in masks.items()}
blk = {}
for k, cs in raw.items():
    keep = [d for d in cs if 55 <= (d["x1"] - d["x0"] + 1) <= 66 and 55 <= (d["y1"] - d["y0"] + 1) <= 66]
    blk[k] = keep
    L.append("色块 %s：原始 %d → 方形 %d" % (k, len(cs), len(keep)))
    for d in cs:
        w, h = d["x1"] - d["x0"] + 1, d["y1"] - d["y0"] + 1
        sq = 55 <= w <= 66 and 55 <= h <= 66
        L.append("   %s px=%6d bbox x[%3d,%3d] y[%3d,%3d] %3dx%-3d c=(%.1f,%.1f)"
                 % ("方" if sq else "剔", d["px"], d["x0"], d["x1"], d["y0"], d["y1"], w, h, d["cx"], d["cy"]))

reds = blk["red"]
L.append("")
if len(reds) >= 4:
    cw = 64.0
    x0g = min(d["cx"] for d in reds)
    idxs = [round((d["cx"] - x0g) / cw) for d in reds]
    A = np.vstack([np.ones(len(reds)), np.array(idxs, float)]).T
    solx, *_ = np.linalg.lstsq(A, np.array([d["cx"] for d in reds]), rcond=None)
    y0g = min(d["cy"] for d in reds)
    idys = [round((d["cy"] - y0g) / cw) for d in reds]
    B = np.vstack([np.ones(len(reds)), np.array(idys, float)]).T
    soly, *_ = np.linalg.lstsq(B, np.array([d["cy"] for d in reds]), rcond=None)
    ax_, cw = float(solx[0]), float(solx[1])
    ay_, chh = float(soly[0]), float(soly[1])
    L.append("格网反推（红块 LSQ）：格宽 %.3f 格高 %.3f 相位 x=%.2f y=%.2f" % (cw, chh, ax_, ay_))
    L.append("格线 x0=%.1f y0=%.1f；场地 %.0fx%.0f（14x11）" % (ax_ - cw / 2, ay_ - chh / 2, 14 * cw, 11 * chh))
    res = [abs(d["cx"] - (ax_ + round((d["cx"] - ax_) / cw) * cw)) for d in reds]
    res += [abs(d["cy"] - (ay_ + round((d["cy"] - ay_) / chh) * chh)) for d in reds]
    L.append("红块拟合残差 max %.2f mean %.2f" % (max(res), float(np.mean(res))))
    def rc(cx, cy):
        return (int(round((cx - ax_) / cw)), int(round((cy - ay_) / chh)))
    L.append("")
    L.append("落格清单：")
    cells = {}
    for k, cs in blk.items():
        for d in cs:
            ci, ri = rc(d["cx"], d["cy"])
            if not (0 <= ci < NCOL and 0 <= ri < ROWS):
                L.append("  [%s] 落格出界 (%s) c=(%.1f,%.1f)" % (k, "?", d["cx"], d["cy"]))
                continue
            sub = a[d["y0"]:d["y1"] + 1, d["x0"]:d["x1"] + 1]
            dark = float((lum[d["y0"]:d["y1"] + 1, d["x0"]:d["x1"] + 1] < 90).mean())
            mean_rgb = tuple(round(float(x)) for x in sub.reshape(-1, 3).mean(axis=0))
            cells.setdefault((k, ci, ri), []).append(d)
            L.append("  [%s] %s%s idx(%d,%d) px=%d 深色=%.3f rgb=%s" % (k, COL[ci], ri + 1, ci, ri, d["px"], dark, mean_rgb))
    L.append("")
    L.append("=== 与现役 config/map.yaml 对照 ===")
    cfg = {
        "mines": ["B1", "G2", "H3", "A4", "D8", "F9", "I10", "E11"],
        "obstacles": ["D1", "F4", "A5", "H5", "B7", "C9", "E10", "G11"],
        "targets": ["B11", "C11"],
        "bridges": ["K5", "K10"],
    }
    for cls in ("red", "mag", "grn"):
        got = sorted(set(COL[c] + str(ri + 1) for (k, c, ri) in cells if k == cls))
        key = {"red": "obstacles", "mag": "targets", "grn": "bridges"}[cls]
        L.append("%s 官方=%s / config=%s" % (key, got, sorted(cfg[key])))
    yel_all = sorted(set(COL[c] + str(ri + 1) for (k, c, ri) in cells if k == "yel"))
    L.append("黄色块全部=%s" % yel_all)
else:
    L.append("红色块 <4 个，无法定标。原始红块：")
    for d in raw["red"]:
        L.append("   px=%d bbox x[%d,%d] y[%d,%d]" % (d["px"], d["x0"], d["x1"], d["y0"], d["y1"]))

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(L) + "\n")
print("written", OUT, len(L), "lines")
