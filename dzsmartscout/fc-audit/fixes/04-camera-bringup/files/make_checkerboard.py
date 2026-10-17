#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 A4 打印用的棋盘格标定板（给 ROS 2 camera_calibration 用）。

默认规格：内角点 8x6（即 9x7 个方格）、方格边长 25.0 mm、A4 横向、300 DPI。

为什么是 25 mm：A4 横向可用 297x210 mm，9x7 格要塞得下还得留边距，
min(297/9, 210/7) = 30 mm 是理论上限，25 mm 留出舒适边距。

用法：
    python make_checkerboard.py [输出目录]

产出：
    checkerboard_8x6_25mm_A4.pdf   打印用（按 100% / 实际尺寸打印）
    checkerboard_8x6_25mm_A4.png   备用（也可全屏显示在屏幕上，效果次之）

注意：打印机能缩放，所以**不要相信标称的 25.0 mm** —— 打印后用尺子量
5 个方格的实际长度，再按实测值给 cameracalibrator 传 --square。
本脚本会在页面下边距画一根 125.0 mm 的校验标尺方便测量。
"""
import os
import sys

from PIL import Image, ImageDraw, ImageFont

DPI = 300
MM = DPI / 25.4                       # 每毫米多少像素
PAGE_W_MM, PAGE_H_MM = 297.0, 210.0   # A4 横向
COLS, ROWS = 9, 7                     # 方格数（内角点 = 8x6）
SQUARE_MM = 25.0


def mm2px(mm: float) -> float:
    return mm * MM


def load_font(size: int):
    """优先中文字体；失败则回退（调用方据此决定用不用中文）。"""
    for path in (r"C:\Windows\Fonts\msyh.ttc",
                 r"C:\Windows\Fonts\arial.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                 "arial.ttf"):
        try:
            return ImageFont.truetype(path, size), True
        except Exception:
            continue
    try:
        return ImageFont.load_default(size=size), False
    except Exception:
        return ImageFont.load_default(), False


def build_png() -> Image.Image:
    W = int(round(mm2px(PAGE_W_MM)))
    H = int(round(mm2px(PAGE_H_MM)))
    img = Image.new("RGB", (W, H), (255, 255, 255))
    dr = ImageDraw.Draw(img)

    board_w_mm = COLS * SQUARE_MM
    board_h_mm = ROWS * SQUARE_MM
    x0_mm = (PAGE_W_MM - board_w_mm) / 2.0
    y0_mm = (PAGE_H_MM - board_h_mm) / 2.0

    # 逐格用"边界的整数像素"来画，避免累积舍入误差
    for c in range(COLS):
        for r in range(ROWS):
            if (c + r) % 2 == 0:
                continue  # 白格不画（背景已是白）
            left = int(round(mm2px(x0_mm + c * SQUARE_MM)))
            right = int(round(mm2px(x0_mm + (c + 1) * SQUARE_MM)))
            top = int(round(mm2px(y0_mm + r * SQUARE_MM)))
            bottom = int(round(mm2px(y0_mm + (r + 1) * SQUARE_MM)))
            dr.rectangle([left, top, right - 1, bottom - 1], fill=(0, 0, 0))

    # 外框（便于对边界）
    dr.rectangle([int(round(mm2px(x0_mm))), int(round(mm2px(y0_mm))),
                  int(round(mm2px(x0_mm + board_w_mm))) - 1,
                  int(round(mm2px(y0_mm + board_h_mm))) - 1],
                 outline=(0, 0, 0), width=2)

    f_big, cjk = load_font(int(mm2px(4.2)))
    f_sm, _ = load_font(int(mm2px(3.0)))

    top_txt = ("相机标定板 · Camera Calibration Target" if cjk
               else "Camera Calibration Target")
    sub_txt = (f"内角点 8x6 · 方格 {SQUARE_MM:.1f} mm · A4 横向 · 必须按 100% 原尺寸打印"
               if cjk else
               f"inner corners 8x6 · square {SQUARE_MM:.1f} mm · A4 landscape · print at 100%")
    dr.text((int(mm2px(x0_mm)), int(mm2px(6.0))), top_txt, font=f_big, fill=(0, 0, 0))
    dr.text((int(mm2px(x0_mm)), int(mm2px(12.5))), sub_txt, font=f_sm, fill=(0, 0, 0))

    # 校验标尺：5 格 = 125.0 mm
    bar_mm = 5 * SQUARE_MM
    bx0 = int(round(mm2px(x0_mm)))
    bx1 = int(round(mm2px(x0_mm + bar_mm)))
    by = int(round(mm2px(y0_mm + board_h_mm + 9.0)))
    dr.line([bx0, by, bx1, by], fill=(0, 0, 0), width=3)
    tick = int(mm2px(2.5))
    dr.line([bx0, by - tick, bx0, by + tick], fill=(0, 0, 0), width=3)
    dr.line([bx1, by - tick, bx1, by + tick], fill=(0, 0, 0), width=3)
    bar_txt = (f"↑ 这段 = 5 格 = {bar_mm:.1f} mm；打印后用尺子量它，"
               f"若不等于 {bar_mm:.1f} mm，把实测值/5 作为 --square 传入"
               if cjk else
               f"this bar = 5 squares = {bar_mm:.1f} mm; measure it and pass measured/5 as --square")
    dr.text((bx0, by + tick + int(mm2px(1.5))), bar_txt, font=f_sm, fill=(0, 0, 0))

    return img


def main() -> int:
    outdir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(outdir, exist_ok=True)
    img = build_png()

    png = os.path.join(outdir, "checkerboard_8x6_25mm_A4.png")
    pdf = os.path.join(outdir, "checkerboard_8x6_25mm_A4.pdf")
    img.save(png, dpi=(DPI, DPI))
    img.save(pdf, "PDF", resolution=DPI)   # resolution 决定 PDF 里的物理尺寸

    for p in (png, pdf):
        print(f"{p}  ({os.path.getsize(p)} 字节)")
    print(f"页面 {PAGE_W_MM:.0f}x{PAGE_H_MM:.0f} mm @ {DPI} DPI, "
          f"板面 {COLS*SQUARE_MM:.0f}x{ROWS*SQUARE_MM:.0f} mm, 方格 {SQUARE_MM:.1f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
