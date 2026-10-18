"""爆炸物卡片画廊：裁剪全部 mine 卡片 → pHash 聚类去重 → 编号画廊图。

用途：爆炸物卡片内容子类（手雷/C4/雷管/…）由人工定义。逐张看 9000+ 个框
不现实，且同一张卡会在多帧重复。本脚本按画面内容 pHash 把同款卡片聚成一簇，
每簇选最清晰（Laplacian 方差最大）的一张，拼成带编号画廊，人工按编号给
子类即可。聚类结果存 clusters.json，供后续把人工子类写回标签。

用法：
    python tools/ml/card_gallery.py --images capture/aerial_field/images \\
        --labels capture/aerial_field/labels --out capture/aerial_field/gallery
退出码：0 成功；2 参数/依赖。
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
    print("需要 opencv-python / numpy", file=sys.stderr)
    raise SystemExit(2)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from extract_frames import phash, hamming  # noqa: E402


# ---------------------------------------------------------------- 纯函数（可测）

def assign_cluster(digest: int, reps: list[int], threshold: int) -> int:
    """与各簇代表哈希比较，距离最近且 <= 阈值则入该簇，否则新建簇。"""
    best, best_dist = -1, threshold + 1
    for i, rep in enumerate(reps):
        dist = hamming(digest, rep)
        if dist <= threshold and dist < best_dist:
            best, best_dist = i, dist
    return best


def sharpness(crop) -> float:
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


# ---------------------------------------------------------------- 构建画廊

def build(images_dir: str, labels_dir: str, out_dir: str, frame_step: int,
          threshold: int, tile: int, per_sheet: int) -> dict:
    images_dir, labels_dir = sorted(Path(images_dir).glob("*.jpg")), \
        Path(labels_dir)
    sampled = images_dir[::frame_step]

    reps: list[int] = []
    best_crops: dict[int, np.ndarray] = {}
    best_sharp: dict[int, float] = {}
    sources: dict[int, dict] = {}
    scanned = 0

    for image_path in sampled:
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        h, w = image.shape[:2]
        label_path = labels_dir / (image_path.stem + ".txt")
        if not label_path.exists():
            continue
        for raw in label_path.read_text(encoding="utf-8").splitlines():
            v = list(map(float, raw.split()))
            if int(v[0]) != 0:
                continue
            cx, cy, bw, bh_ = v[1] * w, v[2] * h, v[3] * w, v[4] * h
            # 向外扩 15%，保证卡片白边完整
            x1 = max(0, int(cx - bw * .575))
            y1 = max(0, int(cy - bh_ * .575))
            x2 = min(w, int(cx + bw * .575))
            y2 = min(h, int(cy + bh_ * .575))
            crop = image[y1:y2, x1:x2]
            if crop.size == 0:
                continue
            scanned += 1
            digest = phash(crop)
            cid = assign_cluster(digest, reps, threshold)
            if cid < 0:
                cid = len(reps)
                reps.append(digest)
                best_crops[cid], best_sharp[cid] = crop, sharpness(crop)
                sources[cid] = {"frame": image_path.name, "x1": x1, "y1": y1,
                                "x2": x2, "y2": y2}
            else:
                value = sharpness(crop)
                if value > best_sharp[cid]:
                    best_crops[cid], best_sharp[cid] = crop, value
                    sources[cid] = {"frame": image_path.name, "x1": x1, "y1": y1,
                                    "x2": x2, "y2": y2}

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "clusters.json").write_text(
        json.dumps(sources, ensure_ascii=False, indent=2), encoding="utf-8")
    # 待填表：人工在 subtype 列写子类（手雷/C4/雷管/非爆炸物…）
    (out / "subtype_mapping.csv").write_text(
        "cluster,subtype\n" + "\n".join(str(i) for i in range(len(reps))) + "\n",
        encoding="utf-8",
    )

    n = len(reps)
    ncol = 10
    for start in range(0, n, per_sheet):
        ids = list(range(start, min(start + per_sheet, n)))
        nrow = (len(ids) + ncol - 1) // ncol
        sheet = np.full((nrow * (tile + 26), ncol * (tile + 6), 3),
                        255, dtype=np.uint8)
        for i, cid in enumerate(ids):
            r, c = divmod(i, ncol)
            im = cv2.resize(best_crops[cid], (tile, tile))
            y, x = r * (tile + 26) + 24, c * (tile + 6) + 3
            sheet[y:y + tile, x:x + tile] = im
            cv2.putText(sheet, f"{cid:03d}", (x, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        cv2.imwrite(str(out / f"gallery_{start // per_sheet}.jpg"), sheet)

    return {"scanned": scanned, "clusters": n, "frames": len(sampled)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True)
    parser.add_argument("--labels", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--frame-step", type=int, default=4)
    parser.add_argument("--threshold", type=int, default=20,
                        help="pHash 同卡判定距离（越大越容易合并；20 实测接近格子数）")
    parser.add_argument("--tile", type=int, default=160)
    parser.add_argument("--per-sheet", type=int, default=50)
    args = parser.parse_args()

    result = build(args.images, args.labels, args.out, args.frame_step,
                   args.threshold, args.tile, args.per_sheet)
    print(f"采样 {result['frames']} 帧、{result['scanned']} 个卡片框 "
          f"→ {result['clusters']} 种样式；画廊 → {Path(args.out).resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
