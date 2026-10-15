# -*- coding: utf-8 -*-
"""把 runs/capture_20261014_1918 的 YOLO 预标注转成 X-AnyLabeling JSON。

改编自仓库 convert_yolo_to_xanylabeling_json.py：
- 路径指向本次采集目录
- 类别 id 映射回真实类名（原版硬编码为 mine）
- JSON 写到 images/ 同目录（X-AnyLabeling 默认在图片旁找同名 json）
"""
import json
from pathlib import Path

from PIL import Image

BASE = Path(__file__).resolve().parent.parent / "runs" / "capture_20261014_1918"
IMG_DIR = BASE / "images"
LBL_DIR = BASE / "labels"

# 与 capture/aerial_field/labels/classes.txt 一致
NAMES = [
    "mine", "cell_label", "arrow", "bridge", "car_start", "car_end",
    "uav_start", "uav_end", "bucket_red", "bucket_green", "bucket_cyan",
    "bucket_unknown", "orange_raw",
]

exts = {".jpg", ".jpeg", ".png", ".bmp"}
imgs = sorted(p for p in IMG_DIR.iterdir() if p.suffix.lower() in exts)
print(f"图片数: {len(imgs)}")

count = 0
for img_path in imgs:
    lbl_path = LBL_DIR / f"{img_path.stem}.txt"
    json_path = IMG_DIR / f"{img_path.stem}.json"

    shapes = []
    if lbl_path.exists() and lbl_path.stat().st_size > 0:
        with Image.open(img_path) as im:
            W, H = im.size
        for line in lbl_path.read_text(encoding="utf-8").strip().splitlines():
            parts = line.split()
            if len(parts) != 5:
                continue
            cid, cx, cy, w, h = int(parts[0]), *map(float, parts[1:])
            label = NAMES[cid] if 0 <= cid < len(NAMES) else f"class_{cid}"
            x1 = (cx - w / 2) * W
            y1 = (cy - h / 2) * H
            x2 = (cx + w / 2) * W
            y2 = (cy + h / 2) * H
            shapes.append({
                "label": label,
                "points": [[round(x1, 1), round(y1, 1)], [round(x2, 1), round(y2, 1)]],
                "group_id": None,
                "shape_type": "rectangle",
                "flags": {},
            })

    with Image.open(img_path) as im:
        W, H = im.size
    data = {
        "version": "5.5.0",
        "flags": {},
        "shapes": shapes,
        "imagePath": img_path.name,
        "imageData": None,
        "imageHeight": H,
        "imageWidth": W,
    }
    json_path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    if shapes:
        count += 1

print(f"生成 JSON: {len(imgs)} 个，其中 {count} 个有标注框")
print(f"JSON 位置: {IMG_DIR}")
