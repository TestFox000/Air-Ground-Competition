"""OCR 版橙格自动细分（用隔离 venv 的解释器运行，勿用训练环境）。

比 classify_orange.py 的「分类器投票」更可靠：直接识别橙格上印刷的中文：
    智能车起点 → 类 4 car_start
    智能车终点 → 类 5 car_end
    无人机起点 → 类 6 uav_start
    无人机终点 → 类 7 uav_end
无人机终点格上实际是 H 圆标（无文字）→ OCR 无结果时用形状检测
（亮像素最大轮廓面积占比大、圆度高）判为 7。识别失败保留类 12，报告
人工兜底。

运行（注意解释器）：
    capture\\.ocr_venv\\Scripts\\python.exe tools/ml/classify_orange_ocr.py \\
        --images capture/aerial_field/images --labels capture/aerial_field/labels
    加 --limit 1 试跑不写文件。
退出码：0 成功；2 依赖/参数。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("需要 opencv / numpy", file=sys.stderr)
    raise SystemExit(2)


ORANGE_CLS = 12

# 关键词 → 类号（按 4 字前缀匹配，OCR 残缺时仍可用：
#     「无人机终」→7、「智能车起」→4，4 字前缀已足够区分）
_KEYWORD_CLS = [
    ("智能车终", 5),
    ("智能车起", 4),
    ("无人机终", 7),
    ("无人机起", 6),
]


# ---------------------------------------------------------------- 纯函数（可测）

def classify_text(text: str):
    """OCR 文本 → 类号；无匹配返回 None。

    裁剪框会带入格子角上的行号数字（如「智能车10起点」），先剔除数字与空格。
    """
    text = "".join(ch for ch in text if not ch.isspace() and not ch.isdigit())
    for keyword, cls in _KEYWORD_CLS:
        if text.startswith(keyword):
            return cls
    return None


def upscale_if_small(crop, min_dim: int = 320):
    """小裁剪放大（高空帧文字太小 OCR 直接失败；放大显著提高命中率）。"""
    h, w = crop.shape[:2]
    if max(h, w) >= min_dim:
        return crop
    scale = min_dim / max(h, w)
    return cv2.resize(crop, (max(1, int(w * scale)), max(1, int(h * scale))),
                      interpolation=cv2.INTER_CUBIC)


def circularity_of_bright(crop, min_area_fraction: float = 0.25):
    """H 圆标检测：亮像素最大轮廓的面积占比与圆度。

    返回 (是否圆形 H 标, 面积占比, 圆度)。
    """
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    contours = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[0]
    if not contours:
        return False, 0.0, 0.0
    biggest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(biggest)
    fraction = area / max(crop.shape[0] * crop.shape[1], 1)
    perimeter = cv2.arcLength(biggest, True) or 1e-9
    circ = 4 * np.pi * area / (perimeter ** 2)
    return (fraction >= min_area_fraction and circ >= 0.55,
            float(fraction), float(circ))


def crop_box(image, values: list[float], expand: float = 0.12):
    """按归一化框裁剪（小幅外扩保证文字完整）。"""
    h, w = image.shape[:2]
    cx, cy, bw, bh_ = (values[0] * w, values[1] * h,
                       values[2] * w, values[3] * h)
    x1 = max(0, int(cx - bw * (0.5 + expand)))
    y1 = max(0, int(cy - bh_ * (0.5 + expand)))
    x2 = min(w, int(cx + bw * (0.5 + expand)))
    y2 = min(h, int(cy + bh_ * (0.5 + expand)))
    return image[y1:y2, x1:x2]


# ---------------------------------------------------------------- 主流程

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True)
    parser.add_argument("--labels", required=True)
    parser.add_argument("--limit", type=int, help="只处理前 N 帧且不写文件")
    args = parser.parse_args()

    try:
        from rapidocr_onnxruntime import RapidOCR
    except ImportError:
        print("需要在 OCR venv 中运行（rapidocr-onnxruntime）", file=sys.stderr)
        return 2

    engine = RapidOCR()
    image_paths = sorted(Path(args.images).glob("*.jpg"))
    if args.limit:
        image_paths = image_paths[: args.limit]

    counts = {4: 0, 5: 0, 6: 0, 7: 0}
    unresolved = []
    for path in image_paths:
        image = cv2.imread(str(path))
        label_path = Path(args.labels) / (path.stem + ".txt")
        lines = label_path.read_text(
            encoding="utf-8").splitlines() if label_path.exists() else []
        output_lines = []
        for line in lines:
            values = line.split()
            if int(values[0]) != ORANGE_CLS:
                output_lines.append(line)
                continue
            coords = list(map(float, values[1:5]))
            crop = crop_box(image, coords)
            if min(crop.shape[:2]) < 8:
                output_lines.append(line)
                unresolved.append(f"{path.name}:too_small")
                continue
            work = upscale_if_small(crop)
            result, _elapsed = engine(work)
            text = "".join(item[1] for item in result) if result else ""
            cls_id = classify_text(text)
            if cls_id is None and not text:
                is_circle, _fraction, _circ = circularity_of_bright(work)
                if is_circle:
                    cls_id = 7
            if cls_id is None:
                output_lines.append(line)
                unresolved.append(f"{path.name}:ocr={text!r}")
            else:
                coord_text = " ".join(f"{v:.6f}" for v in coords)
                output_lines.append(f"{cls_id} {coord_text}")
                counts[cls_id] += 1
        if not args.limit:
            label_path.write_text("\n".join(output_lines) + "\n", encoding="utf-8")

    print(f"car_start={counts[4]} car_end={counts[5]} "
          f"uav_start={counts[6]} uav_end={counts[7]} "
          f"未决={len(unresolved)}")
    for item in unresolved[:15]:
        print("  " + item)
    if args.limit:
        print("[试跑] 未写文件")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
