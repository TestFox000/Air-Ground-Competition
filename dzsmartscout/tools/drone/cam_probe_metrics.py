#!/usr/bin/env python3
"""Offline sharpness / edge metrics for cam_probe images.

Read-only: reads images, prints numbers. No writes to the drone, no writes
outside stdout.

Metrics per image:
  - size / mode / file size
  - Laplacian variance (global, and on a central crop)
  - Tenengrad (mean of squared Sobel gradient magnitude)
  - edge density (Canny pixels / total)
  - Bayer/color statistics: mean RGB, saturation fraction, highlight clipping
  - 1D power spectrum slope proxy: ratio of high-band vs low-band energy

Usage:
  python cam_probe_metrics.py <dir_or_files...>
"""
from __future__ import annotations

import os
import sys

import numpy as np
from PIL import Image


def load(path: str) -> Image.Image:
    return Image.open(path)


def to_gray(img: Image.Image) -> np.ndarray:
    return np.asarray(img.convert("L"), dtype=np.float64)


def laplacian_var(gray: np.ndarray) -> float:
    # 5-point Laplacian, same kernel cv2.Laplacian default (ksize=1)
    k = np.array([[0, 1, 0], [1, -4, 1], [0, 1, 0]], dtype=np.float64)
    h, w = gray.shape
    acc = np.zeros((h - 2, w - 2), dtype=np.float64)
    for dy in range(3):
        for dx in range(3):
            if k[dy, dx] != 0.0:
                acc += k[dy, dx] * gray[dy:dy + h - 2, dx:dx + w - 2]
    return float(acc.var())


def tenengrad(gray: np.ndarray) -> float:
    gx = np.zeros_like(gray)
    gy = np.zeros_like(gray)
    gx[:, 1:-1] = gray[:, 2:] - gray[:, :-2]
    gy[1:-1, :] = gray[2:, :] - gray[:-2, :]
    return float(np.mean(gx ** 2 + gy ** 2))


def edge_density(gray: np.ndarray, lo: float = 60.0, hi: float = 160.0) -> float:
    """Approximate Canny: gradient magnitude thresholded, thin by non-max on axis."""
    gx = np.zeros_like(gray)
    gy = np.zeros_like(gray)
    gx[:, 1:-1] = gray[:, 2:] - gray[:, :-2]
    gy[1:-1, :] = gray[2:, :] - gray[:-2, :]
    mag = np.hypot(gx, gy)
    strong = mag >= hi
    weak = mag >= lo
    # 8-connected hysteresis grow (few passes is enough for a share metric)
    m = strong.copy()
    for _ in range(12):
        grown = m.copy()
        grown[1:, :] |= m[:-1, :]
        grown[:-1, :] |= m[1:, :]
        grown[:, 1:] |= m[:, :-1]
        grown[:, :-1] |= m[:, 1:]
        grown &= weak
        if grown.sum() == m.sum():
            break
        m = grown
    return float(m.sum()) / float(gray.size)


def spectrum_band_ratio(gray: np.ndarray) -> tuple[float, float]:
    """Radially averaged power spectrum: high-band (>0.25 cyc/px) share."""
    g = gray - gray.mean()
    win = np.outer(np.hanning(g.shape[0]), np.hanning(g.shape[1]))
    f = np.fft.fftshift(np.abs(np.fft.fft2(g * win)) ** 2)
    h, w = f.shape
    cy, cx = h // 2, w // 2
    yy, xx = np.mgrid[0:h, 0:w]
    r = np.hypot((yy - cy) / (h / 2.0), (xx - cx) / (w / 2.0))
    total = f.sum()
    high = f[r > 0.25].sum()
    nyq = f[r > 0.5].sum()
    return float(high / total), float(nyq / total)


def color_stats(img: Image.Image) -> dict:
    a = np.asarray(img.convert("RGB"), dtype=np.float64)
    return {
        "mean_rgb": [round(float(v), 1) for v in a.reshape(-1, 3).mean(axis=0)],
        "std_gray": round(float(np.asarray(img.convert("L"), dtype=np.float64).std()), 1),
        "frac_clip_hi": round(float((a.max(axis=2) >= 250).mean()), 4),
        "frac_clip_lo": round(float((a.max(axis=2) <= 5).mean()), 4),
        # saturation: max-min channel spread, share of clearly coloured pixels
        "frac_colored": round(float(((a.max(axis=2) - a.min(axis=2)) > 40).mean()), 4),
    }


def report(path: str) -> None:
    size = os.path.getsize(path)
    img = load(path)
    gray = to_gray(img)
    h, w = gray.shape
    print(f"\n=== {os.path.basename(path)} ===")
    print(f"file_bytes      : {size}")
    print(f"size(w x h)     : {w} x {h}   mode={img.mode}  pixels={w * h}")
    print(f"laplacian_var   : {laplacian_var(gray):.2f}")
    pix_var = float(gray.var())
    print(f"pixel_var       : {pix_var:.1f}")
    print(f"LNV (lapvar/var): {laplacian_var(gray) / pix_var:.4f}" if pix_var else
          "LNV (lapvar/var): n/a")
    # central crop = 50% linear
    ch, cw = int(h * 0.5), int(w * 0.5)
    y0, x0 = (h - ch) // 2, (w - cw) // 2
    print(f"lapvar_center50 : {laplacian_var(gray[y0:y0 + ch, x0:x0 + cw]):.2f}")
    # 128x128 centre patch (focus metric on fine detail)
    p = 64 if min(h, w) >= 128 else min(h, w) // 2
    print(f"lapvar_patch{2 * p} : {laplacian_var(gray[h // 2 - p:h // 2 + p, w // 2 - p:w // 2 + p]):.2f}")
    print(f"tenengrad       : {tenengrad(gray):.2f}")
    print(f"edge_density    : {edge_density(gray):.5f}")
    hi, nyq = spectrum_band_ratio(gray)
    print(f"psd_high_share  : {hi:.4f}   psd_nyquist_share: {nyq:.4f}")
    cs = color_stats(img)
    print(f"mean_rgb        : {cs['mean_rgb']}   gray_std={cs['std_gray']}")
    print(f"frac_clip_hi    : {cs['frac_clip_hi']}  frac_clip_lo={cs['frac_clip_lo']}  frac_colored={cs['frac_colored']}")


def main(argv: list[str]) -> int:
    targets: list[str] = []
    for a in argv[1:]:
        if os.path.isdir(a):
            for name in sorted(os.listdir(a)):
                if name.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".webp")):
                    targets.append(os.path.join(a, name))
        else:
            targets.append(a)
    if not targets:
        print(__doc__)
        return 2
    for t in targets:
        report(t)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
