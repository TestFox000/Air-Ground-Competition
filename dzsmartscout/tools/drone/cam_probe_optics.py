#!/usr/bin/env python3
"""Estimate edge-spread width (px) from cam_probe images + optics table.

Edge-spread width is estimated by the classic "gradient-ratio" method:
build an ideal step edge, run it through a Gaussian blur of sigma s, and match
the measured 2nd-moment ratio of the gradient profile. The reported number is
the full 10%-90% transition width (2.563 * sigma).

The optics table then converts that absolute blur into the ground sample
distance / altitude budget for reading a printed mark of a given physical size.

Read-only: prints to stdout only.
"""
from __future__ import annotations

import os
import sys

import numpy as np
from PIL import Image

# --- camera intrinsics taken from dzuav MapLocalizer.cpp (calibrated at 1920x1080) ---
FX_1920 = 995.153295
FY_1080 = 997.758431
# --- optics of the field (dzbrain geometry.py) ---
FIELD_CELL_M = 0.35
FIELD_X_M = 10.35  # row 11 centre -> row 1 centre = 10 * 0.35
FIELD_Y_M = 4.55   # col N centre -> col A centre = 13 * 0.35


def gradient_profile(gray: np.ndarray) -> np.ndarray:
    gx = np.zeros_like(gray)
    gy = np.zeros_like(gray)
    gx[:, 1:-1] = gray[:, 2:] - gray[:, :-2]
    gy[1:-1, :] = gray[2:, :] - gray[:-2, :]
    return np.hypot(gx, gy)


def measure(gray: np.ndarray, percentile: float = 99.0) -> tuple[float, float, float]:
    """Edge-spread estimate.

    A perfect 1-pixel step edge has a gradient profile that is a single sample
    wide: rms width = 1 px.  Blurring with a Gaussian of sigma s (a) lowers the
    peak and (b) is convolved with the 1-px pixel aperture, giving an rms
    width of sqrt(1^2 + (2*sqrt(pi)*s)^2) px.  We invert that relation on the
    rms of the gradient magnitude over the strongest (99th percentile) edges.
    Reported second value is the 10%-90% transition width = 2.563 * sigma.
    """
    g = gradient_profile(gray)
    thr = np.percentile(g, percentile)
    sel = g > thr
    if not np.any(sel):
        return 0.0, 0.0, 0.0
    meas_rms = float(np.sqrt(np.mean(g[sel] ** 2)))
    extra2 = max(meas_rms ** 2 - 1.0, 0.0)
    sigma = float(np.sqrt(extra2)) / (2.0 * np.sqrt(np.pi))
    return sigma, sigma * 2.563, meas_rms


def optics_table() -> None:
    print("\n=== optics: ground sample distance vs altitude (nadir, f=995.15 px) ===")
    print("  alt[m]   footprint_x[m]  footprint_y[m]   GSD@1920[mm/px]  GSD@640[mm/px]")
    for alt in (1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0):
        fx = FX_1920
        fx_640 = FX_1920 * 640.0 / 1920.0
        fxy = FY_1080
        fxy_640 = FY_1080 * 480.0 / 1080.0
        fp_x = alt * 1920.0 / fx
        fp_y = alt * 1080.0 / fxy
        print(f"  {alt:5.1f}    {fp_x:11.2f}    {fp_y:11.2f}    "
              f"{alt / fx * 1000:13.2f}    {alt / fx_640 * 1000:12.2f}")


def read_budget(sigma_px_full: float) -> None:
    """How big (in meters on the ground) must a printed mark be, per altitude."""
    print("\n=== readability budget: mark size needed for N px across ===")
    print("  (blur sigma measured on the native 640x480 frames; GSD computed at")
    print("   each native resolution; requirement = 2.5 px/stroke, stroke = 12% of mark)")
    for res, w in (("1920x1200", 1920.0), ("1280x960", 1280.0), ("640x480", 640.0)):
        fx = FX_1920 * w / 1920.0
        print(f"\n  --- native {res} ---")
        print("   alt[m]  GSD[mm/px]  mark needed @10px [mm]  @15px  @20px")
        for alt in (1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0):
            gsd = alt / fx * 1000.0
            print(f"   {alt:5.1f}   {gsd:9.2f}    {gsd * 10:19.1f}  {gsd * 15:6.1f}  "
                  f"{gsd * 20:6.1f}")


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
        img = Image.open(t)
        gray = np.asarray(img.convert("L"), dtype=np.float64)
        sigma, w10_90, meas = measure(gray)
        print(f"{os.path.basename(t):26s} {img.size[0]:4d}x{img.size[1]:<4d} "
              f"blur_sigma={sigma:6.2f}px  10-90width={w10_90:6.2f}px  "
              f"rms_grad_at_edges={meas:7.2f}")
    optics_table()
    read_budget(0.0)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
