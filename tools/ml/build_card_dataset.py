"""从「画廊子类表」构建爆炸物卡片内容分类数据集（第二阶段，裁剪图分类）。

流程：
    1. 用户在 gallery/subtype_mapping.csv 填好每个簇的子类；
    2. 对每个 mine（类 0）裁剪图算 pHash，归到画廊最近代表簇（距离 ≤ 阈值），
       取出该簇子类；未填子类或距离过远的裁剪丢弃并报告；
    3. 按帧时间块整块切 train/val（复用 split_groups，防相邻帧泄漏），
       输出「文件夹即类别」结构（Ultralytics 分类任务直接可用）。
输出：
    <out>/{train,val}/<子类>/*.jpg

用法：
    python tools/ml/build_card_dataset.py --images capture/aerial_field/images \\
        --labels capture/aerial_field/labels --gallery capture/aerial_field/gallery \\
        --out datasets/card_cls
退出码：0 成功；2 参数；3 子类表未填。
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("需要 opencv / numpy", file=sys.stderr)
    raise SystemExit(2)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from extract_frames import phash, hamming            # noqa: E402
from build_dataset import split_groups              # noqa: E402


# ---------------------------------------------------------------- 纯函数（可测）

def read_subtype_mapping(path: Path) -> dict[int, str]:
    """读填好的 csv → {cluster: subtype}；空 subclass 跳过。"""
    result = {}
    with path.open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            subtype = (row.get("subtype") or "").strip()
            if subtype:
                result[int(row["cluster"])] = subtype
    return result


def nearest_cluster(digest: int, rep_hashes: list[int],
                    threshold: int) -> int | None:
    """pHash → 最近代表簇；距离 > 阈值返回 None。"""
    if not rep_hashes:
        return None
    distances = [hamming(digest, rep) for rep in rep_hashes]
    best = int(np.argmin(distances))
    return best if distances[best] <= threshold else None


# ---------------------------------------------------------------- 构建

def build(images_dir: str, labels_dir: str, gallery: str, out: str,
          block: int, threshold: int, ratios, seed: int) -> dict:
    images_dir, labels_dir, gallery = (Path(images_dir), Path(labels_dir),
                                      Path(gallery))
    mapping = read_subtype_mapping(gallery / "subtype_mapping.csv")
    if not mapping:
        raise SystemExit("subtype_mapping.csv 未填子类，先完成画廊标注")
    sources = json.loads((gallery / "clusters.json").read_text(encoding="utf-8"))
    # 代表哈希从簇源图直接算
    rep_hashes = []
    for cid in range(len(sources)):
        src = sources[str(cid)]
        im = cv2.imread(str(images_dir / src["frame"]))
        crop = im[src["y1"]:src["y2"], src["x1"]:src["x2"]]
        rep_hashes.append(phash(crop))

    image_paths = sorted(images_dir.glob("*.jpg"))
    # 收集 (frame_index, subtype, crop)
    records = []
    dropped = 0
    for fidx, path in enumerate(image_paths):
        image = cv2.imread(str(path))
        h, w = image.shape[:2]
        label_path = labels_dir / (path.stem + ".txt")
        if not label_path.exists():
            continue
        for line in label_path.read_text(encoding="utf-8").splitlines():
            values = list(map(float, line.split()))
            if int(values[0]) != 0:
                continue
            cx, cy, bw, bh_ = (values[1] * w, values[2] * h,
                               values[3] * w, values[4] * h)
            x1, y1 = max(0, int(cx - bw * .575)), max(0, int(cy - bh_ * .575))
            x2, y2 = int(cx + bw * .575), int(cy + bh_ * .575)
            crop = image[y1:y2, x1:x2]
            if crop.size == 0:
                continue
            cid = nearest_cluster(phash(crop), rep_hashes, threshold)
            if cid is None or cid not in mapping:
                dropped += 1
                continue
            records.append((fidx, mapping[cid], crop))

    # 帧 → 时间块 → train/val
    frame_blocks = [i // block for i in range(len(image_paths))]
    assignment = split_groups(frame_blocks, ratios, seed)

    out_dir = Path(out)
    counts = {"train": 0, "val": 0, "test": 0}
    for fidx, subtype, crop in records:
        subset = assignment[frame_blocks[fidx]]
        cls_dir = out_dir / subset / subtype
        cls_dir.mkdir(parents=True, exist_ok=True)
        name = f"f{fidx:03d}_{counts[subset]:05d}.jpg"
        cv2.imwrite(str(cls_dir / name), crop)
        counts[subset] += 1
    return counts | {"dropped": dropped, "subtypes": len(set(mapping.values()))}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True)
    parser.add_argument("--labels", required=True)
    parser.add_argument("--gallery", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--block", type=int, default=8)
    parser.add_argument("--threshold", type=int, default=20)
    parser.add_argument("--ratios", type=float, nargs=3, default=[0.8, 0.1, 0.1],
                        metavar=("TRAIN", "VAL", "TEST"))
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    ratios = tuple(args.ratios)
    if abs(sum(ratios) - 1.0) > 1e-6:
        print("ratios 之和必须为 1", file=sys.stderr)
        return 2

    result = build(args.images, args.labels, args.gallery, args.out,
                   args.block, args.threshold, ratios, args.seed)
    print(f"子类 {result['subtypes']} 个；train/val/test = "
          f"{result['train']}/{result['val']}/{result['test']}；"
          f"丢弃 {result['dropped']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
