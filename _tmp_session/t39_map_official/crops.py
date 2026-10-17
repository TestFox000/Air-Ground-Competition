# -*- coding: utf-8 -*-
"""① 生成两块"贴标签"放大图（供人工核对格名）② 打印行 6/7 分界处的真实像素色。"""
import pathlib
import sys

import numpy as np
from PIL import Image

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, r"D:\User\Desktop\比赛存档\v1\dzsmartscout\tools")
from perception.colors import as_rgb_array

img = Image.open(HERE / "map_official.png").convert("RGB")
a = as_rgb_array(img)
for name, (x0, y0, x1, y1), scale in (("zoom_A_G_r1_6", (7, 8, 330, 286), 2),
                                      ("zoom_A_J_r6_11", (7, 282, 470, 514), 2)):
    c = img.crop((x0, y0, x1, y1))
    c = c.resize((c.width * scale, c.height * scale), Image.LANCZOS)
    c.save(HERE / f"{name}.png")
    print(name, c.size)

print("\n=== 行分界处的像素色（列 D~N 各取格中心 x）===")
xs = [int(7.9 + (ci + 0.5) * 45.586) for ci in range(3, 14)]
for k in (3, 6, 8):
    y = int(round(10.95 + k * 45.507))
    for dy in (-3, -2, -1, 0, 1, 2, 3):
        row = a[y + dy, xs[0]:xs[-1] + 1]
        vals = row[::20]
        print(f"  k={k} y={y+dy}: " + " ".join(f"{tuple(int(v) for v in p)}" for p in vals[:7]))
    print()

print("=== 行 6/7 分界整行的近灰/近白占比（x 8..646） ===")
for k in (3, 5, 6, 8):
    y = int(round(10.95 + k * 45.507))
    seg = a[y - 1:y + 2, 8:647].reshape(-1, 3).astype(int)
    gray = ((np.abs(seg - 208).max(axis=1) < 16)).mean()
    white = ((seg.min(axis=1) > 235)).mean()
    mat = ((np.abs(seg - 240).max(axis=1) < 12)).mean()
    print(f"  row{k}/{k+1} y≈{y}: 灰线 {gray:.3f} 近白 {white:.3f} 地垫色 {mat:.3f}")
