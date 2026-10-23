#!/usr/bin/env python3
"""xwd -> PNG 转换器（只依赖标准库 + numpy + PIL）

为什么需要它：无人机上没有 ImageMagick / netpbm / scrot / ffmpeg，
而 gnome-screenshot 在 GNOME 会话里会通过 D-Bus 委托 gnome-shell 截图，
无视 DISPLAY 环境变量（会拍到 :0 而不是 VNC 的 :1）。
xwd 是唯一能精确指定 display 的现成工具，但它输出的是 XWD 位图。

用法: python3 xwd2png.py <in.xwd> <out.png> [max_width]
"""
import struct
import sys

import numpy as np
from PIL import Image


def parse_xwd(path):
    with open(path, 'rb') as fh:
        raw = fh.read()

    if len(raw) < 100:
        raise SystemExit('file too short to be XWD')

    le = struct.unpack('<25I', raw[:100])
    be = struct.unpack('>25I', raw[:100])
    if 100 <= le[0] <= 1_000_000:
        hdr, endian = le, 'little'
    elif 100 <= be[0] <= 1_000_000:
        hdr, endian = be, 'big'
    else:
        raise SystemExit('unrecognised XWD header (header_size=%d/%d)' % (le[0], be[0]))

    (header_size, file_version, pixmap_format, pixmap_depth, width, height,
     xoffset, byte_order, bitmap_unit, bitmap_bit_order, bitmap_pad,
     bits_per_pixel, bytes_per_line, visual_class, red_mask, green_mask,
     blue_mask, bits_per_rgb, colormap_entries, ncolors,
     win_w, win_h, win_x, win_y, bdrwidth) = hdr

    info = dict(header_size=header_size, version=file_version,
                pixmap_format=pixmap_format, depth=pixmap_depth,
                width=width, height=height, bpp=bits_per_pixel,
                bytes_per_line=bytes_per_line, byte_order=byte_order,
                ncolors=ncolors, masks=(hex(red_mask), hex(green_mask), hex(blue_mask)))

    off = header_size + ncolors * 12  # XWDColor 每条 12 字节
    body = raw[off:]

    if pixmap_format != 2:
        raise SystemExit('only ZPixmap (2) supported, got %d' % pixmap_format)
    if bits_per_pixel not in (24, 32):
        raise SystemExit('only 24/32bpp supported, got %d' % bits_per_pixel)

    need = bytes_per_line * height
    if len(body) < need:
        raise SystemExit('truncated pixel data: need %d have %d' % (need, len(body)))

    rows = np.frombuffer(body[:need], dtype=np.uint8).reshape(height, bytes_per_line)

    # byte_order: 0 = LSBFirst (B,G,R[,X])，1 = MSBFirst ([X,]R,G,B)
    if bits_per_pixel == 32:
        arr = rows[:, :width * 4].reshape(height, width, 4)
        rgb = arr[:, :, 2::-1].copy() if byte_order == 0 else arr[:, :, 1:4].copy()
    else:  # 24bpp：每像素 3 字节，行按 4 字节对齐填充
        arr = rows[:, :width * 3].reshape(height, width, 3)
        rgb = arr[:, :, ::-1].copy() if byte_order == 0 else arr.copy()

    return rgb, info


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    max_width = int(sys.argv[3]) if len(sys.argv) > 3 else 1400

    rgb, info = parse_xwd(src)
    img = Image.fromarray(rgb, 'RGB')
    if max_width and img.width > max_width:
        ratio = max_width / img.width
        img = img.resize((max_width, max(1, int(round(img.height * ratio)))), Image.LANCZOS)

    unique = np.unique(rgb.reshape(-1, 3), axis=0)
    img.save(dst)
    print('XWD:', info)
    print('unique_colors=%d  mean_rgb=%s' % (len(unique), np.round(rgb.reshape(-1, 3).mean(axis=0), 1).tolist()))
    print('saved %s (%dx%d)' % (dst, img.width, img.height))


if __name__ == '__main__':
    main()
