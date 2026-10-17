#!/usr/bin/env python3
"""Optical-transfer / detail-limit analysis for cam_probe images.

Adds to cam_probe_metrics.py:
  - radially averaged power spectrum, normalised, in 8 radial bands
  - estimated half-power spatial frequency and "resolvable pixels per feature"
  - 3x3 patch grid of Laplacian variance (field curvature / corner softness)
  - 2D autocorrelation half-width of the (high-pass) image = blur correlation
"""
from __future__ import annotations

import os
import sys

import numpy as np
from PIL import Image


def radial_bands(gray: np.ndarray, nband: int = 8) -> list[float]:
    g = gray - gray.mean()
    win = np.outer(np.hanning(g.shape[0]), np.hanning(g.shape[1]))
    f = np.fft.fftshift(np.abs(np.fft.fft2(g * win)) ** 2)
    h, w = f.shape
    cy, cx = h // 2, w // 2
    yy, xx = np.mgrid[0:h, 0:w]
    r = np.hypot((yy - cy) / (h / 2.0), (xx - cx) / (w / 2.0))
    edges = np.linspace(0.0, 1.0, nband + 1)
    total = f.sum()
    out = []
    for i in range(nband):
        m = (r >= edges[i]) & (r < edges[i + 1])
        out.append(float(f[m].sum() / total) if total else 0.0)
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


def patch_grid(gray: np.ndarray) -> np.ndarray:
    h, w = gray.shape
    ph, pw = h // 3, w // 3
    m = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            m[i, j] = laplacian_var(gray[i * ph:(i + 1) * ph, j * pw:(j + 1) * pw])
    return m


def autocorr_width(gray: np.ndarray) -> float:
    """Half-width of the normalised autocorrelation of the high-passed image,
    in pixels: a proxy for the point-spread blur correlation length."""
    hp = gray - np.asarray(Image.fromarray(gray.astype(np.uint8)).filter(
        __import__("PIL.ImageFilter", fromlist=["ImageFilter"]).GaussianBlur(4)), dtype=np.float64)
    hp = hp - hp.mean()
    f = np.fft.fft2(hp)
    ac = np.fft.fftshift(np.real(np.fft.ifft2(f * np.conj(f))))
    ac /= ac.max() if ac.max() else 1.0
    cy, cx = np.array(ac.shape) // 2
    line = ac[cy, cx:]
    below = np.nonzero(line < 0.5)[0]
    return float(below[0]) if len(below) else float(len(line))


def report(path: str) -> None:
    img = Image.open(path)
    gray = np.asarray(img.convert("L"), dtype=np.float64)
    print(f"\n=== {os.path.basename(path)}  {img.size[0]}x{img.size[1]} ===")
    bands = radial_bands(gray)
    print("  radial PSD share (0->nyq): " + " ".join(f"{v:.4f}" for v in bands))
    # cumulative
    cum = np.cumsum(bands)
    half = next((i for i, v in enumerate(cum) if v >= 0.5), len(cum) - 1)
    print(f"  half-power band index   : {half}/{len(bands)}  (~{half / len(bands):.2f} of nyquist)")
    m = patch_grid(gray)
    print("  3x3 patch laplacian var :")
    for row in m:
        print("      " + " ".join(f"{v:9.2f}" for v in row))
    print(f"  patch spread (max/min)  : {m.max() / m.min():.2f}" if m.min() > 0 else
          f"  patch spread (max/min)  : inf (min={m.min():.2f})")
    print(f"  autocorr half-width     : {autocorr_width(gray):.2f} px")


def main(argv: list[str]) -> int:
    targets: list[str] = []
    for a in argv[1:]:
        if os.path.isdir(a):
            for name in sorted(os.listdir(a)):
                if name.lower().endswith((".png", ".jpg", ".jpeg")):
                    targets.append(os.path.join(a, name))
        else:
            targets.append(a)
    for t in targets:
        report(t)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
