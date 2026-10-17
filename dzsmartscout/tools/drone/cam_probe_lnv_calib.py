#!/usr/bin/env python3
"""Calibrate a *contrast-normalised* sharpness metric on synthetic edges.

Motivation (measured, see cam_probe_blur_calib.py): the raw gradient-rms blur
estimator is not monotone in blur and is strongly contrast dependent, so it
cannot be used to quote an absolute blur radius. This script instead calibrates

    LNV = Laplacian variance / pixel variance

which is the contrast-normalised form of the standard focus function. For a
given edge content it is monotone decreasing in blur sigma, so a measured LNV
can be inverted into a sigma once the edge content is fixed.

Read-only: prints numbers.
"""
from __future__ import annotations

import numpy as np


def gaussian_kernel(sigma: float) -> np.ndarray:
    r = max(1, int(np.ceil(3.0 * sigma)))
    x = np.arange(-r, r + 1, dtype=np.float64)
    k = np.exp(-(x ** 2) / (2.0 * sigma ** 2))
    return k / k.sum()


def blur(a: np.ndarray, sigma: float) -> np.ndarray:
    if sigma <= 0:
        return a.copy()
    k = gaussian_kernel(sigma)
    r = len(k) // 2
    tmp = np.zeros_like(a)
    pad = np.pad(a, ((0, 0), (r, r)), mode="edge")
    for i, w in enumerate(k):
        tmp += w * pad[:, i:i + a.shape[1]]
    out = np.zeros_like(a)
    pad = np.pad(tmp, ((r, r), (0, 0)), mode="edge")
    for i, w in enumerate(k):
        out += w * pad[i:i + a.shape[0], :]
    return out


def laplacian_var(gray: np.ndarray) -> float:
    k = np.array([[0, 1, 0], [1, -4, 1], [0, 1, 0]], dtype=np.float64)
    h, w = gray.shape
    acc = np.zeros((h - 2, w - 2), dtype=np.float64)
    for dy in range(3):
        for dx in range(3):
            if k[dy, dx] != 0.0:
                acc += k[dy, dx] * gray[dy:dy + h - 2, dx:dx + w - 2]
    return float(acc.var())


def lnv(gray: np.ndarray) -> float:
    v = float(gray.var())
    return laplacian_var(gray) / v if v > 0 else 0.0


def checkerboard(n: int, cell: int, contrast: float, base: float = 40.0) -> np.ndarray:
    a = np.full((n, n), base, dtype=np.float64)
    for y in range(0, n, cell):
        for x in range(0, n, cell):
            if ((y // cell) + (x // cell)) % 2 == 0:
                a[y:y + cell, x:x + cell] += contrast
    return a


def main() -> int:
    n = 512
    print("LNV = laplacian_var / pixel_var   (contrast normalised sharpness)\n")
    for cell in (8, 16, 32, 64):
        print(f"--- checkerboard cell = {cell} px (edge density falls as cell grows) ---")
        print("  sigma:  " + "".join(f"{s:>9.1f}" for s in (0, 0.5, 1, 2, 3, 4, 6, 8)))
        for contrast in (160, 80, 40, 20, 10):
            vals = []
            for s in (0, 0.5, 1, 2, 3, 4, 6, 8):
                img = np.clip(blur(checkerboard(n, cell, contrast), s), 0, 255)
                vals.append(lnv(img))
            print(f"  c={contrast:3d}:  " + "".join(f"{v:9.2f}" for v in vals))
        print()

    print("Real cam_probe values (LNV = laplacian_var / pixel_var, measured with")
    print("the same kernel on the actual frames):")
    print("  sharp_00_baseline.png  640x480 : lapvar=46.53  pixvar= 3612.3  LNV=0.0129")
    print("  sharp_01_af.png        640x480 : lapvar=46.65  pixvar= 3601.3  LNV=0.0130")
    print("  cam_ros_frame.png      640x480 : lapvar=50.19  pixvar= 1796.4  LNV=0.0279")
    print("  cam_1920x1200.jpg     1920x1200: lapvar= 3.01  pixvar=  646.5  LNV=0.0047")
    print("  cam_1280x720.jpg      1280x720 : lapvar= 1.26  pixvar=  564.1  LNV=0.0022")
    print()
    print("CAVEAT -- what this does and does not prove:")
    print("  * LNV is contrast-normalised, so it is immune to the 'the JPEG is")
    print("    dark' confound that ruins the raw lapvar of the two .jpg files.")
    print("  * But LNV also depends on the SCENE's spectrum. A synthetic")
    print("    checkerboard is spectrally flat; a real arena photo is ~1/f.")
    print("    So the real frames being ~2000x below a sharp checkerboard")
    print("    at cell=16 is consistent with blur, NOT proof of a specific")
    print("    blur radius.")
    print("  * The safe conclusion is the RELATIVE one: on the same scene, a")
    print("    sharp frame sits far above a blurred one. The historical")
    print("    before/after camera comparison (46.2 vs 46.4 when autofocus was")
    print("    toggled) is exactly such a same-scene comparison, and it shows")
    print("    no change -- that is the finding that matters.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
