"""把 hbb 官方 7 类草稿标签转换为我们确认的类别体系，供人工标注与最终训练。

输入：prelabel_onnx.py 生成的 labels_draft/（官方类号，坐标已对原图归一化）。
输出：
    labels/               新类号标签（每图一个 txt）
    labeling/data.yaml    人工标注工具（X-AnyLabeling）用类表，含临时类

最终类别（11 类，顺序即训练类号）：
    0 mine          爆炸物卡片
    1 cell_label    格号文字
    2 arrow         桥洞箭头
    3 bridge        黄色桥洞（人工/HSV 补标）
    4 car_start     智能车起点（橙格）
    5 car_end       智能车终点（橙格）
    6 uav_start     无人机起点（橙格）
    7 uav_end       无人机终点（橙格，H 标志）
    8 bucket_red    红桶
    9 bucket_green  荧光绿桶
    10 bucket_cyan  青桶
仅标注阶段使用的临时类：
    11 bucket_unknown 桶颜色无法自动判定（标注工具里改成 8/9/10）
    12 orange_raw     HSV 补出的橙格（标注工具里改成 4~7）

官方草稿 → 新类号：
    0 mine→0   4 cell_label→1   2 arrow→2   6 obstacle→桶颜色判定
    1 car / 3 target_red / 5 target_blue 在航拍中不存在（误检）→ 丢弃

用法：
    python tools/ml/convert_draft_labels.py \\
        --images capture/aerial_field/images --draft capture/aerial_field/labels_draft \\
        --out capture/aerial_field/labels --yaml capture/aerial_field/labeling/data.yaml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("需要 opencv-python / numpy", file=sys.stderr)
    raise SystemExit(2)


FINAL_NAMES = [
    "mine", "cell_label", "arrow", "bridge",
    "car_start", "car_end", "uav_start", "uav_end",
    "bucket_red", "bucket_green", "bucket_cyan",
]
TEMP_NAMES = ["bucket_unknown", "orange_raw"]

# 官方 7 类 → 新类号（obstacle 6 走颜色判定；其余丢弃）
_OFFICIAL_MAP = {0: 0, 4: 1, 2: 2}
_BUCKET_ID = {"red": 8, "green": 9, "cyan": 10}


# ---------------------------------------------------------------- 纯函数（可测）

def classify_bucket(b: float, g: float, r: float) -> str | None:
    """按 obstacle 框核心区域平均 BGR 判定桶颜色。

    返回 'red' / 'green' / 'cyan' / None（灯光反光、浅色纸张等非桶目标）。
    阈值依据：754 个实测 obstacle 裁剪统计（见 docs/会话记录）。
    """
    # 整体高亮且通道差异小 → 地砖灯光反光 / 浅色纸，不是桶
    if min(b, g, r) > 105 and max(b, g, r) - min(b, g, r) < 60:
        return None
    # 红桶：R 显著领先
    if r - g > 25 and r - b > 25:
        return "red"
    # 荧光绿桶：G 领先、B 很低
    if g - r > 30 and b < 85:
        return "green"
    # 青桶（含暗部）：B 相对 R 领先；亮组 G/B 都高
    if b - r > 10:
        return "cyan"
    # 过渡帧：按 B 与 R 的大小归入青/绿；R 最大归红
    if r > g and r > b:
        return "red"
    return "cyan" if b > r else "green"


def convert_line(values: list[float], color: str | None) -> str | None:
    """单行官方标签 → 新类号行；坐标原样保留（均为原图归一化）。"""
    cls = int(values[0])
    if cls in _OFFICIAL_MAP:
        new_cls = _OFFICIAL_MAP[cls]
    elif cls == 6:
        if color is None:
            return None  # 非桶误检，丢弃
        new_cls = _BUCKET_ID[color]
    else:
        # car / target_red / target_blue：航拍无此目标
        return None
    coords = " ".join(f"{v:.6f}" for v in values[1:5])
    return f"{new_cls} {coords}"


def labeling_yaml_text(images_abs: Path) -> str:
    """生成 X-AnyLabeling 导入 YOLO 标签用的 yaml（含临时类）。"""
    lines = [f"path: {images_abs.parent.as_posix()}", f"train: {images_abs.name}",
             "val: ", "names:"]
    for i, name in enumerate(FINAL_NAMES + TEMP_NAMES):
        lines.append(f"  {i}: {name}")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------- 转换

def convert(images_dir: str, draft_dir: str, out_dir: str) -> dict:
    images_dir, draft_dir, out_dir = Path(images_dir), Path(draft_dir), Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stats = {"kept": 0, "dropped": 0, "bucket_unknown": 0, "by_class": {}}

    for image_path in sorted(images_dir.glob("*.jpg")):
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        h, w = image.shape[:2]
        draft = draft_dir / (image_path.stem + ".txt")
        out_lines = []
        if draft.exists():
            for raw in draft.read_text(encoding="utf-8").splitlines():
                values = list(map(float, raw.split()))
                if len(values) < 5:
                    continue
                color = None
                if int(values[0]) == 6:
                    cx, cy, bw, bh = (values[1] * w, values[2] * h,
                                      values[3] * w, values[4] * h)
                    x1, y1 = max(0, int(cx - bw / 2)), max(0, int(cy - bh / 2))
                    x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
                    crop = image[y1:y2, x1:x2]
                    if crop.size:
                        ch, cw = crop.shape[:2]
                        core = crop[int(ch * .2):int(ch * .8),
                                    int(cw * .2):int(cw * .8)]
                        b, g, r = core.reshape(-1, 3).mean(0)
                        color = classify_bucket(float(b), float(g), float(r))
                line = convert_line(values, color)
                if line is None:
                    stats["dropped"] += 1
                    continue
                out_lines.append(line)
                stats["kept"] += 1
                cid = int(line.split()[0])
                stats["by_class"][cid] = stats["by_class"].get(cid, 0) + 1
        (out_dir / (image_path.stem + ".txt")).write_text(
            "\n".join(out_lines), encoding="utf-8"
        )
    return stats


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True)
    parser.add_argument("--draft", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--yaml", help="标注工具类表输出路径")
    args = parser.parse_args()

    stats = convert(args.images, args.draft, args.out)
    print(f"保留 {stats['kept']} 行，丢弃误检 {stats['dropped']} 行")
    by = stats["by_class"]
    print("各新类号行数：" + " ".join(
        f"{FINAL_NAMES[i] if i < len(FINAL_NAMES) else i}={by[i]}"
        for i in sorted(by)
    ))
    if args.yaml:
        yaml_path = Path(args.yaml)
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        yaml_path.write_text(
            labeling_yaml_text(Path(args.images).resolve()), encoding="utf-8"
        )
        print(f"标注类表 → {yaml_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
