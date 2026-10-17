#!/usr/bin/env python3
"""Sanity-calibrate the edge-blur estimator (read-only, prints numbers).

Builds synthetic step edges blurred by a known Gaussian sigma with controlled
contrast / noise / edge density, runs the same estimator used on cam_probe
frames, and prints estimated vs true sigma.

Purpose: decide whether the large sigma estimates from the dark, low-contrast
cam_1920x1200.jpg / cam_1280x720.jpg are real blur or an artefact of low
contrast (a histogram-percentile estimator is contrast dependent).
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
        return a
    k = gaussian_kernel(sigma)
    r = len(k) // 2
    pad = np.pad(a, ((0, 0), (r, r)), mode="edge")
    out = np.zeros_like(a)
    for i, w in enumerate(k):
        out += w * pad[:, i:i + a.shape[1]]
    pad = np.pad(out, ((r, r), (0, 0)), mode="edge")
    out = np.zeros_like(a)
    for i, w in enumerate(k):
        out += w * pad[i:i + a.shape[0], :]
    return out


def gradient_profile(gray: np.ndarray) -> np.ndarray:
    gx = np.zeros_like(gray)
    gy = np.zeros_like(gray)
    gx[:, 1:-1] = gray[:, 2:] - gray[:, :-2]
    gy[1:-1, :] = gray[2:, :] - gray[:-2, :]
    return np.hypot(gx, gy)


def estimate(gray: np.ndarray, percentile: float = 99.0) -> float:
    g = gradient_profile(gray)
    thr = np.percentile(g, percentile)
    sel = g > thr
    if not np.any(sel):
        return 0.0
    meas_rms = float(np.sqrt(np.mean(g[sel] ** 2)))
    extra2 = max(meas_rms ** 2 - 1.0, 0.0)
    return float(np.sqrt(extra2)) / (2.0 * np.sqrt(np.pi))


def checkerboard(n: int, cell: int, contrast: float, offset: float = 0.0) -> np.ndarray:
    """A grid of squares: many step edges, like a printed arena grid."""
    a = np.full((n, n), offset, dtype=np.float64)
    for y in range(0, n, cell):
        for x in range(0, n, cell):
            if ((y // cell) + (x // cell)) % 2 == 0:
                a[y:y + cell, x:x + cell] += contrast
    return a


def main() -> int:
    n = 512
    print("true_sigma  contrast  noise  cellsize  estimated_sigma  error")
    for contrast, noise, cell in ((120, 0.0, 16), (120, 0.0, 32), (40, 0.0, 16),
                                  (12, 0.0, 16), (120, 3.0, 16), (12, 3.0, 16),
                                  (120, 12.0, 16), (12, 12.0, 16)):
        for true_sigma in (0.0, 1.0, 2.0, 4.0, 8.0):
            img = checkerboard(n, cell, contrast, 40.0)
            img = blur(img, true_sigma)
            rng = np.random.default_rng(0)
            if noise > 0:
                img = img + rng.normal(0.0, noise, img.shape)
            img = np.clip(img, 0, 255)
            est = estimate(img)
            err = est - true_sigma
            print(f"{true_sigma:10.1f}  {contrast:8d}  {noise:5.1f}  {cell:8d}  "
                  f"{est:15.2f}  {err:+6.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
