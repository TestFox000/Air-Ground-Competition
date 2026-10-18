#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""laser_offset 标定工具：量「激光落点」相对「画面中心」的偏差 → 算 laser_offset_x/y。

为什么必须标：开火判据是
    aim_err_x = |dx_filt + lead - laser_offset_x_| <= fire_dz
而 `laser_offset_x_/y_` 代码默认 0 —— 等于假设"激光与相机同轴"。
真实不同轴时，即使把目标**稳稳放画面正中**，弹着也会固定偏一段（开火框才 6px！），
表现为"瞄得很正却总打偏"。标定后这段系统偏差被扣掉，单发命中率 p 可从 ~0.3 抬到 ~0.9
（见 shot_budget_sim.py：10 发预算下 p 是唯一决定性因素）。

用法（两种）：
  ① 离线标注（推荐，零依赖 ROS）：截一张【能看到激光点】的相机画面
       python laser_offset_calib.py --image shot.png
     鼠标点一下激光点 → 打印 offset 与 yaml 片段；按 s 存图、q 退出。
  ② 纯计算（无图也想核算）：
       python laser_offset_calib.py --compute --dot-x 340 --dot-y 250 --img-w 640 --img-h 480

约定（与 C++ 一致）：画面坐标 x 右为正、y 下为正；画面中心默认 (w/2, h/2)。
    laser_offset_x = dot_x - cx      laser_offset_y = dot_y - cy
"""
from __future__ import annotations

import argparse


def compute_offset(dot_x: float, dot_y: float, cx: float, cy: float):
    return dot_x - cx, dot_y - cy


def yaml_snippet(ox: float, oy: float) -> str:
    return ("dzrknn_dataprocess:\n"
            "  ros__parameters:\n"
            f"    laser_offset_x: {ox:.1f}   # 激光落点相对画面中心(px, 右为正)\n"
            f"    laser_offset_y: {oy:.1f}   # (下为正)")


def report(ox: float, oy: float, dot, center, img):
    print("=" * 60)
    print(f"画面      : {img[0]}×{img[1]}")
    print(f"画面中心  : ({center[0]:.1f}, {center[1]:.1f})")
    print(f"激光落点  : ({dot[0]:.1f}, {dot[1]:.1f})")
    print(f"laser_offset_x / y = ({ox:+.1f}, {oy:+.1f}) px")
    dup = max(abs(ox), abs(oy))
    print(f"偏差量级  : {dup:.1f}px  →  开火框 6px 下"
          f"{'【必须标定】否则系统性打偏' if dup > 3 else '（影响较小，可选）'}")
    print("-" * 60)
    print(yaml_snippet(ox, oy))
    print("回退：laser_offset_x/y 改回 0.0")


def interactive(image_path: str, cx=None, cy=None, save_path=None):
    import cv2  # noqa: PLC0415

    img = cv2.imread(image_path)
    if img is None:
        print(f"[ERR] 读不到图: {image_path}")
        return
    h, w = img.shape[:2]
    if cx is None:
        cx = w / 2.0
    if cy is None:
        cy = h / 2.0
    state = {"dot": None}

    def on_click(event, x, y, flags, _p):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["dot"] = (float(x), float(y))
            canvas = img.copy()
            _draw(canvas, cx, cy, state["dot"])
            cv2.imshow(win, canvas)
            ox, oy = compute_offset(x, y, cx, cy)
            report(ox, oy, (x, y), (cx, cy), (w, h))

    def _draw(canvas, cx, cy, dot):
        cv2.drawMarker(canvas, (int(cx), int(cy)), (0, 255, 0),
                       cv2.MARKER_CROSS, 24, 2)          # 画面中心
        if dot:
            cv2.circle(canvas, (int(dot[0]), int(dot[1])), 8, (0, 0, 255), 2)
            cv2.arrowedLine(canvas, (int(cx), int(cy)), (int(dot[0]), int(dot[1])),
                            (255, 0, 0), 2)
            ox, oy = compute_offset(dot[0], dot[1], cx, cy)
            cv2.putText(canvas, f"off=({ox:+.1f},{oy:+.1f})px", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    win = "laser_offset calib - 点击激光点, s=存图 q=退出"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_click)
    _draw(img.copy(), cx, cy, None)
    cv2.imshow(win, img.copy())
    print(f"提示：把激光点常亮对准画面，点一下【激光落点】。中心=({cx:.0f},{cy:.0f})")
    while True:
        k = cv2.waitKey(20) & 0xFF
        if k == ord("q"):
            break
        if k == ord("s") and state["dot"]:
            out = save_path or (image_path.rsplit(".", 1)[0] + "_offset.png")
            canvas = img.copy()
            _draw(canvas, cx, cy, state["dot"])
            cv2.imwrite(out, canvas)
            print(f"已存标注图: {out}")
    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", help="离线标注：截图路径")
    ap.add_argument("--compute", action="store_true", help="纯计算")
    ap.add_argument("--dot-x", type=float)
    ap.add_argument("--dot-y", type=float)
    ap.add_argument("--img-w", type=float, default=640)
    ap.add_argument("--img-h", type=float, default=480)
    ap.add_argument("--cx", type=float, help="画面中心 x（默认 img_w/2）")
    ap.add_argument("--cy", type=float, help="画面中心 y（默认 img_h/2）")
    ap.add_argument("--cx-frac", type=float, help="中心相对宽的比例（默认 0.5）")
    a = ap.parse_args()

    if a.image:
        interactive(a.image, a.cx, a.cy)
    elif a.compute:
        if a.dot_x is None or a.dot_y is None:
            print("请给 --dot-x/--dot-y")
            return
        cx = a.cx if a.cx is not None else a.img_w / 2.0
        cy = a.cy if a.cy is not None else a.img_h / 2.0
        ox, oy = compute_offset(a.dot_x, a.dot_y, cx, cy)
        report(ox, oy, (a.dot_x, a.dot_y), (cx, cy), (a.img_w, a.img_h))
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
