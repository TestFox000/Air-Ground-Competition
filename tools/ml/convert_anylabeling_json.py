"""把 X-AnyLabeling 导出的 JSON 标注转换为 YOLO 格式标签。

输入：X-AnyLabeling 标注目录（每图一个 .json，shapes 含 rectangle）。
输出：YOLO 标签目录（每图一个 .txt，class cx cy w h，归一化）。

标注类别 → YOLO 类号映射（默认针对 7cls 检测模型）：
    red bucket / green bucket / blue bucket → obstacle (1)
    car                                      → car (4)
    plane                                    → 跳过（无人机本体，非检测目标）
    target                                   → target_blue (5)  [⚠️ 颜色未定，见下]

⚠️ "target" 颜色歧义：
    X-AnyLabeling 标注中所有标靶均标为 "target"，未区分红蓝。
    若用于车端标靶 pose 模型（仅 blue/red 两类），需人工确认每张 target 的颜色后
    修正 --target-class 参数，或在 JSON 中把 label 改为 "target_blue"/"target_red"。
    当前默认归入 target_blue(5)，仅用于 7cls 粗检训练。

用法：
    # 7cls 检测模型标签（buckets→obstacle, plane跳过）
    python tools/ml/convert_anylabeling_json.py \\
        --json-dir E:/ml_datasets/manual_200/images \\
        --out-dir E:/ml_datasets/manual_200/labels_yolo \\
        --preset 7cls

    # seed_157 标靶种子训练标签（6类：blue bucket/car/green bucket/plane/red bucket/target）
    python tools/ml/convert_anylabeling_json.py \\
        --json-dir E:/ml_datasets/manual_200/images \\
        --out-dir E:/ml_datasets/seed_157/labels_all \\
        --preset seed
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# 7cls 检测模型类号映射（标签名 → 类号；None 表示跳过）
LABEL_MAP_7CLS: dict[str, int | None] = {
    "red bucket": 1,
    "green bucket": 1,
    "blue bucket": 1,
    "car": 4,
    "plane": None,        # 无人机本体，非 7cls 检测目标
    "target": 5,          # ⚠️ 默认 target_blue，颜色待确认
}

# seed_157 标靶种子训练类号映射（6类，对应 seed_157/data.yaml）
LABEL_MAP_SEED: dict[str, int | None] = {
    "blue bucket": 0,
    "car": 1,
    "green bucket": 2,
    "plane": 3,
    "red bucket": 4,
    "target": 5,
}

# 类名（仅用于统计打印）
CLASS_NAMES = {
    0: "mine", 1: "obstacle", 2: "arrow", 3: "cell_label",
    4: "car", 5: "target_blue", 6: "target_red",
}

CLASS_NAMES_SEED = {
    0: "blue bucket", 1: "car", 2: "green bucket",
    3: "plane", 4: "red bucket", 5: "target",
}

PRESETS = {
    "7cls": (LABEL_MAP_7CLS, CLASS_NAMES),
    "seed": (LABEL_MAP_SEED, CLASS_NAMES_SEED),
}


def convert_json(json_path: Path, label_map: dict[str, int | None]) -> list[str]:
    """单个 JSON → YOLO 行列表。"""
    data = json.loads(json_path.read_text(encoding="utf-8"))
    img_w = float(data["imageWidth"])
    img_h = float(data["imageHeight"])
    lines: list[str] = []
    for shape in data.get("shapes", []):
        label = shape["label"]
        cls = label_map.get(label)
        if cls is None:
            continue
        if shape.get("shape_type") != "rectangle":
            continue
        pts = shape["points"]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        x1, x2 = min(xs), max(xs)
        y1, y2 = min(ys), max(ys)
        cx = (x1 + x2) / 2.0 / img_w
        cy = (y1 + y2) / 2.0 / img_h
        w = (x2 - x1) / img_w
        h = (y2 - y1) / img_h
        # 裁剪到 [0,1]
        cx, cy = min(max(cx, 0.0), 1.0), min(max(cy, 0.0), 1.0)
        w, h = min(max(w, 0.0), 1.0), min(max(h, 0.0), 1.0)
        if w <= 0 or h <= 0:
            continue
        lines.append(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    return lines


def convert(json_dir: str, out_dir: str, label_map: dict[str, int | None]) -> dict:
    json_dir = Path(json_dir)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stats = {"files": 0, "empty": 0, "boxes": 0, "by_class": {}, "skipped": {}}
    for jp in sorted(json_dir.glob("*.json")):
        lines = convert_json(jp, label_map)
        stats["files"] += 1
        if not lines:
            stats["empty"] += 1
        (out_dir / (jp.stem + ".txt")).write_text(
            "\n".join(lines), encoding="utf-8"
        )
        for ln in lines:
            cls = int(ln.split()[0])
            stats["boxes"] += 1
            stats["by_class"][cls] = stats["by_class"].get(cls, 0) + 1
    # 统计被跳过的标签
    for jp in sorted(json_dir.glob("*.json")):
        data = json.loads(jp.read_text(encoding="utf-8"))
        for shape in data.get("shapes", []):
            label = shape["label"]
            if label not in label_map or label_map[label] is None:
                stats["skipped"][label] = stats["skipped"].get(label, 0) + 1
    return stats


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--json-dir", required=True, help="X-AnyLabeling JSON 目录")
    parser.add_argument("--out-dir", required=True, help="YOLO txt 输出目录")
    parser.add_argument(
        "--preset",
        choices=list(PRESETS.keys()),
        default="7cls",
        help="类号映射预设：7cls(无人机7类检测) / seed(seed_157标靶种子6类)",
    )
    parser.add_argument(
        "--target-class",
        type=int,
        default=None,
        help="覆盖 target 标签映射到的类号（7cls默认5=target_blue；seed预设不生效）",
    )
    args = parser.parse_args()

    label_map, class_names = PRESETS[args.preset]
    label_map = dict(label_map)
    if args.target_class is not None:
        label_map["target"] = args.target_class

    stats = convert(args.json_dir, args.out_dir, label_map)
    print(f"处理文件 {stats['files']} 个（空标签 {stats['empty']} 个）")
    print(f"输出框总数 {stats['boxes']}")
    by = stats["by_class"]
    print("各类框数：" + " ".join(
        f"{class_names.get(c, c)}={by[c]}" for c in sorted(by)
    ))
    if stats["skipped"]:
        print("跳过标签：" + " ".join(
            f"{k}={v}" for k, v in sorted(stats["skipped"].items())
        ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
