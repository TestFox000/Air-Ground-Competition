#!/usr/bin/env python3
"""Orin 端采集训练集：调用无人机下视相机，录像并按 2fps 抽帧保存。

用法（在 Orin 上执行）:
  # USB UVC 相机（/dev/video0）
  python3 orin_capture_dataset.py --camera uvc --device /dev/video0 --duration 120 --fps 2

  # CSI 相机（nvargus）
  python3 orin_capture_dataset.py --camera argus --duration 120 --fps 2

  # 已有视频文件抽帧
  python3 orin_capture_dataset.py --video recording.mp4 --fps 2

输出:
  runs/capture_dataset/
    images/        抽帧图片 (frame_000001.jpg ...)
    video.mp4      原始录像（可选 --no-video 跳过）
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def make_gst_pipeline(device: str = "0", width: int = 1920, height: int = 1080, fps: int = 30) -> str:
    """CSI 相机 GStreamer pipeline（nvargus）。"""
    return (
        f"nvarguscamerasrc sensor-id={device} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"format=NV12, framerate={fps}/1 ! "
        f"nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
    )


def open_camera(args) -> cv2.VideoCapture:
    """打开相机，返回 VideoCapture。"""
    if args.camera == "uvc":
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.cam_fps)
    elif args.camera == "argus":
        gst = make_gst_pipeline(args.device, args.width, args.height, args.cam_fps)
        cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    elif args.camera == "video":
        cap = cv2.VideoCapture(args.video)
    else:
        raise ValueError(f"未知相机类型: {args.camera}")

    if not cap.isOpened():
        raise SystemExit(f"无法打开相机: {args.camera} (device={args.device})")
    return cap


def main() -> int:
    ap = argparse.ArgumentParser(description="Orin 采集训练集：录像 + 2fps 抽帧")
    ap.add_argument("--camera", choices=["uvc", "argus", "video"], default="uvc",
                    help="相机类型: uvc(USB) / argus(CSI) / video(已有视频)")
    ap.add_argument("--device", default="/dev/video0", help="UVC 设备路径或 argus sensor-id")
    ap.add_argument("--video", default="", help="已有视频文件路径（--camera video 时用）")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--cam-fps", type=int, default=30, help="相机采集帧率")
    ap.add_argument("--fps", type=float, default=2.0, help="抽帧帧率（保存图片的帧率）")
    ap.add_argument("--duration", type=float, default=120.0, help="采集时长（秒），0=手动停止")
    ap.add_argument("--out", default="runs/capture_dataset", help="输出目录")
    ap.add_argument("--no-video", action="store_true", help="不保存原始录像，只抽帧")
    args = ap.parse_args()

    out_dir = Path(args.out)
    img_dir = out_dir / "images"
    img_dir.mkdir(parents=True, exist_ok=True)

    cap = open_camera(args)
    real_fps = cap.get(cv2.CAP_PROP_FPS) or args.cam_fps
    print(f"相机已打开: {args.camera}, 实际帧率={real_fps:.1f}fps, 分辨率={args.width}x{args.height}")

    # 录像写入器
    writer = None
    if not args.no_video and args.camera != "video":
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(out_dir / "video.mp4"), fourcc, real_fps, (args.width, args.height))
        if not writer.isOpened():
            print("警告: 录像写入器打开失败，只抽帧不录像", file=sys.stderr)
            writer = None

    frame_interval = 1.0 / args.fps  # 抽帧间隔（秒）
    last_save = 0.0
    frame_count = 0
    saved_count = 0
    start_time = time.time()

    print(f"开始采集: 抽帧={args.fps}fps, 时长={args.duration}s, 输出={img_dir}")
    print("按 'q' 停止采集")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("取帧失败，退出")
                break

            now = time.time()
            elapsed = now - start_time

            # 录像
            if writer is not None:
                writer.write(frame)

            # 抽帧保存
            if now - last_save >= frame_interval:
                fname = img_dir / f"frame_{saved_count:06d}.jpg"
                cv2.imwrite(str(fname), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved_count += 1
                last_save = now

                if saved_count % 10 == 0:
                    print(f"  已保存 {saved_count} 帧, 已采集 {elapsed:.0f}s")

            frame_count += 1

            # 显示（小窗口，不阻塞）
            small = cv2.resize(frame, (640, 360))
            cv2.imshow("capture (q=quit)", small)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("用户停止采集")
                break

            if args.duration > 0 and elapsed >= args.duration:
                print(f"达到采集时长 {args.duration}s，停止")
                break

    except KeyboardInterrupt:
        print("\n被中断，停止采集")
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()

    print(f"\n采集完成: 保存 {saved_count} 张图片到 {img_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
