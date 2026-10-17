# -*- coding: utf-8 -*-
"""几何探针：搞清楚蓝色外框、刻度标签带、灰网格线的确切位置。"""
import pathlib
import sys

import numpy as np
from PIL import Image

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, r"D:\User\Desktop\比赛存档\v1\dzsmartscout\tools")
from perception.colors import as_rgb_array

img = as_rgb_array(Image.open(HERE / "map_official.png"))
r, g, b = (img[..., i].astype(np.int16) for i in range(3))
mx = np.max(img, axis=2).astype(np.int16)
mn = np.min(img, axis=2).astype(np.int16)
sat = mx - mn
blue = (b > r + 30) & (b > g + 10) & (b > 90)
gray_line = (np.abs(r - 208) < 16) & (np.abs(g - 208) < 16) & (np.abs(b - 208) < 16)
darktxt = (mx < 150) & (sat < 60)
yellow = (r > 210) & (g > 170) & (b < 150) & (sat > 80)
red = (r > 200) & (g < 140) & (b < 140) & (sat > 80)
mag = (r > 190) & (b > 150) & (g < 160) & (sat > 60)
grn = (g > 140) & (r < 150) & (b < 150) & (sat > 60)


def runs(mask1d):
    out, cur = [], None
    for i, v in enumerate(mask1d):
        if v and cur is None:
            cur = i
        elif not v and cur is not None:
            out.append((cur, i - 1))
            cur = None
    if cur is not None:
        out.append((cur, len(mask1d) - 1))
    return out


print("blue bbox:", np.nonzero(blue.any(axis=1))[0][[0, -1]].tolist(),
      np.nonzero(blue.any(axis=0))[0][[0, -1]].tolist())
for y in (3, 8, 12, 20, 30, 260, 500, 510, 516, 520):
    print(f"row {y:3d}: blue runs {runs(blue[y])[:8]}")
for x in (3, 8, 12, 20, 30, 320, 640, 646, 650):
    print(f"col {x:3d}: blue runs {runs(blue[:, x])[:8]}")
print("gray_line colsum top-20 cols:", np.argsort(-gray_line.sum(axis=0))[:20].tolist())
print("gray_line rowsum top-16 rows:", np.argsort(-gray_line.sum(axis=1))[:16].tolist())
print("darktxt bbox:",
      np.nonzero(darktxt.any(axis=1))[0][[0, -1]].tolist(),
      np.nonzero(darktxt.any(axis=0))[0][[0, -1]].tolist())
for name, m in (("yellow", yellow), ("red", red), ("mag", mag), ("grn", grn)):
    ys, xs = np.nonzero(m)
    print(f"{name}: n={m.sum()} bbox x[{xs.min()},{xs.max()}] y[{ys.min()},{ys.max()}]")
# 逐列/逐行统计各色，用来定位色块
print("\nper-column colored counts (x: red/yellow/mag/grn):")
for x in range(0, img.shape[1], 8):
    print(f"  x={x:3d} r={red[:, x].sum():3d} y={yellow[:, x].sum():3d} "
          f"m={mag[:, x].sum():3d} g={grn[:, x].sum():3d}")
