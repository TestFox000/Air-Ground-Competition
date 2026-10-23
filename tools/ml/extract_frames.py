"""视频分帧 + 去重：把空中拍摄的场地视频转成不重复的候选训练图片。

两个必须解决的问题（踩过的坑）：
1. 视频 30fps 直接全抽 → 相邻帧几乎一模一样，既浪费标注量，又会在后续
   train/val 切分时造成「数据泄漏」（同一场景两边都在 → 指标虚高）。
2. 相机悬停时连续几百帧内容不变 → 等间隔抽帧也会抽到一堆重复图。

做法：
    先按「每秒 N 帧」等间隔粗抽 → 再用感知哈希（pHash, DCT）去掉
    汉明距离过近的近似帧 → 只保留内容真正变化的帧。
输出 images/*.jpg + manifest.json（记录每帧在原视频中的时间戳，供后续
按「拍摄段落」分组切分 train/val，杜绝泄漏）。

用法：
    python tools/ml/extract_frames.py --video <视频> --out capture/field
    python tools/ml/extract_frames.py --video <视频> --out capture/field \
        --fps 2 --hamming-threshold 5
退出码：0=成功；2=参数/依赖；3=视频打不开。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("缺少依赖，请先执行: python -m pip install opencv-python-headless numpy",
          file=sys.stderr)
    raise SystemExit(2)


def phash(frame, hash_size: int = 8, highfreq: int = 32) -> int:
    """感知哈希：灰度缩放到 32x32 → DCT → 取左上 8x8 低频（跳过 DC 项），
    与中值比较生成 64bit。画面轻微移动/光照变化时哈希基本稳定。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    resized = cv2.resize(gray, (highfreq, highfreq), interpolation=cv2.INTER_AREA)
    dct = cv2.dct(np.float32(resized))
    low = dct[:hash_size, 1 : hash_size + 1]   # 跳过 DC（第0列），只取交流低频
    median = np.median(low)
    bits = (low > median).flatten()
    value = 0
    for bit in bits:
        value = (value << 1) | int(bit)
    return value


def hamming(a: int, b: int) -> int:
    return (a ^ b).bit_count()


def extract(video: str, out: str, sample_fps: float, threshold: int,
            jpeg_quality: int = 95) -> dict:
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise RuntimeError(f"视频打不开: {video}")

    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    step = max(1, round(src_fps / sample_fps))

    out_dir = Path(out)
    images_dir = out_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    kept_hashes: list[int] = []
    manifest_frames: list[dict] = []
    index = 0
    scanned = 0
    frame_index = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_index % step == 0:
            scanned += 1
            digest = phash(frame)
            # 与所有已保留帧比，任一过近即判重（悬停场景下重复帧不会相邻地出现在列表末尾）
            if all(hamming(digest, previous) > threshold for previous in kept_hashes):
                name = f"frame_{index:05d}.jpg"
                cv2.imwrite(
                    str(images_dir / name), frame,
                    [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
                )
                kept_hashes.append(digest)
                manifest_frames.append({
                    "file": name,
                    "src_frame": frame_index,
                    "src_time_s": round(frame_index / src_fps, 3),
                })
                index += 1
        frame_index += 1
    cap.release()

    manifest = {
        "source_video": str(video),
        "source_fps": round(src_fps, 3),
        "source_frames": total,
        "sample_fps": sample_fps,
        "hamming_threshold": threshold,
        "scanned": scanned,
        "kept": index,
        "frames": manifest_frames,
    }
    (out_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--video", required=True, help="输入视频")
    parser.add_argument("--out", required=True, help="输出目录（images/ + manifest.json）")
    parser.add_argument("--fps", type=float, default=2.0,
                        help="粗抽帧率：每秒取几帧（默认 2）")
    parser.add_argument("--hamming-threshold", type=int, default=5,
                        help="pHash 汉明距离 <= 此值视为重复（默认 5，越大去重越狠）")
    args = parser.parse_args()

    try:
        manifest = extract(args.video, args.out, args.fps, args.hamming_threshold)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 3

    print(f"粗扫 {manifest['scanned']} 帧 → 去重保留 {manifest['kept']} 帧")
    print(f"输出: {Path(args.out).resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
