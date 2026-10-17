#!/usr/bin/env python3
"""Read EXIF/JPEG tables for cam_probe images (data-quality triage).

Prints: JPEG quantization-table means per table, EXIF tag dump, and a
histogram summary that distinguishes "scene is dark" from "image is blurry".
"""
from __future__ import annotations

import os
import sys

from PIL import Image, ExifTags


def qtable_summary(img: Image.Image) -> None:
    q = getattr(img, "quantization", None)
    if not q:
        print("  quant tables    : none (not a JPEG or not exposed)")
        return
    for k in sorted(q):
        vals = list(q[k])
        print(f"  quant table {k}   : n={len(vals)} mean={sum(vals) / len(vals):.1f} "
              f"min={min(vals)} max={max(vals)}")
    # crude quality proxy: sum of the luma table
    if 0 in q:
        s = sum(q[0])
        print(f"  luma table sum  : {s}  (lower sum = higher quality; "
              f"~2000=very high, ~8000=mediocre, >15000=poor)")


def exif_summary(img: Image.Image) -> None:
    exif = img.getexif()
    if not exif:
        print("  exif            : none")
        return
    for tag_id, value in exif.items():
        name = ExifTags.TAGS.get(tag_id, str(tag_id))
        if name in ("MakerNote", "UserComment"):
            value = f"<{len(bytes(value)) if isinstance(value, bytes) else 'obj'} bytes>"
        print(f"  exif {name:<20}: {value}")
    ifd = exif.get_ifd(0x8769)
    for tag_id, value in (ifd or {}).items():
        name = ExifTags.TAGS.get(tag_id, str(tag_id))
        print(f"  exifX {name:<19}: {value}")


def histogram(img: Image.Image) -> None:
    g = img.convert("L")
    h = g.histogram()
    total = sum(h)
    dark = sum(h[:40]) / total
    mid = sum(h[40:216]) / total
    bright = sum(h[216:]) / total
    print(f"  lum histogram   : dark(<40)={dark:.3f}  mid={mid:.3f}  bright(>215)={bright:.3f}")
    # occupied bins = how much tonal range the scene actually uses
    occupied = sum(1 for v in h if v > total * 1e-5)
    print(f"  occupied bins   : {occupied}/256 (>0.001% of pixels)")
    # dynamic range proxy
    nz = [i for i, v in enumerate(h) if v > 0]
    if nz:
        print(f"  used range      : {nz[0]}..{nz[-1]}  span={nz[-1] - nz[0]}")


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
        print(f"\n=== {os.path.basename(t)}  format={img.format} mode={img.mode} size={img.size} ===")
        qtable_summary(img)
        exif_summary(img)
        histogram(img)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
