# -*- coding: utf-8 -*-
"""本机采集训练集：调用无人机 AR0234 相机（USB），录像并按 2fps 抽帧保存。

用法（在本机 Windows 上执行，手持电脑移动采集）:
  # 自动选 AR0234 相机，采集 120 秒
  python tools/capture_dataset_local.py --duration 120 --fps 2

  # 指定相机 index（先用 --list 查看）
  python tools/capture_dataset_local.py --camera 1 --duration 120

  # 只抽帧不录像
  python tools/capture_dataset_local.py --no-video --duration 120

  # 列出所有相机
  python tools/capture_dataset_local.py --list
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2


def list_cameras(max_index: int = 5):
    """列出所有可用相机。"""
    print("可用相机:")
    for i in range(max_index):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"  index={i}: {w}x{h}")
            cap.release()
        else:
            print(f"  index={i}: 无")


def find_ar0234(max_index: int = 5) -> int:
    """自动找 AR0234 相机：实测哪个 index 能出 1920x1200 帧。

    注意不能只看默认分辨率（很多 UVC 相机默认都报 640x480），
    必须实际 set 1920x1200 并取一帧验证。
    """
    for i in range(max_index):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
            ok, frame = cap.read()
            cap.release()
            if ok and frame.shape[1] >= 1920 and frame.shape[0] >= 1200:
                return i
    return -1


def main() -> int:
    ap = argparse.ArgumentParser(description="本机采集训练集：AR0234 录像 + 2fps 抽帧")
    ap.add_argument("--list", action="store_true", help="列出所有相机后退出")
    ap.add_argument("--camera", type=int, default=-1,
                    help="相机 index（-1=自动找 AR0234）")
    ap.add_argument("--width", type=int, default=1920, help="采集宽度")
    ap.add_argument("--height", type=int, default=1200, help="采集高度")
    ap.add_argument("--cam-fps", type=int, default=30, help="相机帧率")
    ap.add_argument("--fps", type=float, default=2.0, help="抽帧帧率")
    ap.add_argument("--duration", type=float, default=120.0,
                    help="采集时长（秒），0=手动停止")
    ap.add_argument("--out", default="runs/capture_dataset", help="输出目录")
    ap.add_argument("--no-video", action="store_true", help="不保存原始录像")
    args = ap.parse_args()

    if args.list:
        list_cameras()
        return 0

    # 选相机
    cam_idx = args.camera
    if cam_idx < 0:
        cam_idx = find_ar0234()
        if cam_idx < 0:
            print("未找到 AR0234 相机，请用 --list 查看后用 --camera 指定 index",
                  file=sys.stderr)
            return 1
        print(f"自动选择 AR0234 相机: index={cam_idx}")

    cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.cam_fps)
    if not cap.isOpened():
        print(f"无法打开相机 index={cam_idx}", file=sys.stderr)
        return 1

    # 预热：DSHOW 流刚启动时取帧可能失败、属性可能返回 -1，
    # 必须重试等第一帧到位，并以实际帧尺寸为准
    warm, t0 = None, time.time()
    while time.time() - t0 < 5.0:
        ok, warm = cap.read()
        if ok:
            break
        time.sleep(0.05)
    if warm is None:
        print("相机取帧失败（5 秒无帧），请检查 USB 连接", file=sys.stderr)
        cap.release()
        return 1

    real_h, real_w = warm.shape[:2]
    # 实测帧率：该相机 CAP_PROP_FPS 常返回 -1，用 1 秒计数代替
    n, t1 = 0, time.time()
    while time.time() - t1 < 1.0:
        ok, _ = cap.read()
        if ok:
            n += 1
    real_fps = n / (time.time() - t1) if n else float(args.cam_fps)
    print(f"相机已打开: {real_w}x{real_h} @ 实测 {real_fps:.1f}fps")

    out_dir = Path(args.out)
    img_dir = out_dir / "images"
    img_dir.mkdir(parents=True, exist_ok=True)

    writer = None
    if not args.no_video:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(out_dir / "video.mp4"), fourcc,
                                 real_fps, (real_w, real_h))
        if not writer.isOpened():
            print("警告: 录像写入失败，只抽帧", file=sys.stderr)
            writer = None

    frame_interval = 1.0 / args.fps
    last_save = 0.0
    saved = 0
    start = time.time()

    print(f"采集: 抽帧={args.fps}fps, 时长={args.duration}s, 输出={img_dir}")
    print("按 'q' 停止")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("取帧失败")
                break

            now = time.time()
            elapsed = now - start

            if writer is not None:
                writer.write(frame)

            if now - last_save >= frame_interval:
                fname = img_dir / f"frame_{saved:06d}.jpg"
                cv2.imwrite(str(fname), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved += 1
                last_save = now
                if saved % 10 == 0:
                    print(f"  已保存 {saved} 帧, {elapsed:.0f}s")

            small = cv2.resize(frame, (640, 400))
            cv2.imshow("capture (q=quit)", small)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            if args.duration > 0 and elapsed >= args.duration:
                print(f"达到时长 {args.duration}s")
                break
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()

    print(f"\n完成: 保存 {saved} 张到 {img_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
