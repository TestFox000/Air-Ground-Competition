"""把 YOLO 预标注标签转成 X-AnyLabeling 默认的 JSON 格式，打开图片即可显示。"""
import json
from pathlib import Path
from PIL import Image

BASE = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\mine_official")
IMG_DIR = BASE / "images"
LBL_DIR = BASE / "labels"
JSON_DIR = BASE / "images"  # X-AnyLabeling 默认在图片同目录找同名 json

exts = {".jpg", ".jpeg", ".png", ".bmp"}
imgs = [p for p in IMG_DIR.iterdir() if p.suffix.lower() in exts]
print(f"图片数: {len(imgs)}")

count = 0
for img_path in imgs:
    lbl_path = LBL_DIR / f"{img_path.stem}.txt"
    json_path = JSON_DIR / f"{img_path.stem}.json"

    if not lbl_path.exists() or lbl_path.stat().st_size == 0:
        # 空标签也要生成空 json，避免 X-AnyLabeling 报"找不到标签"
        shapes = []
    else:
        with Image.open(img_path) as im:
            W, H = im.size
        shapes = []
        for line in lbl_path.read_text(encoding="utf-8").strip().splitlines():
            parts = line.split()
            if len(parts) != 5:
                continue
            _, cx, cy, w, h = map(float, parts)
            x1 = (cx - w / 2) * W
            y1 = (cy - h / 2) * H
            x2 = (cx + w / 2) * W
            y2 = (cy + h / 2) * H
            shapes.append({
                "label": "mine",
                "points": [[round(x1, 1), round(y1, 1)], [round(x2, 1), round(y2, 1)]],
                "group_id": None,
                "shape_type": "rectangle",
                "flags": {},
            })

    data = {
        "version": "5.5.0",
        "flags": {},
        "shapes": shapes,
        "imagePath": img_path.name,
        "imageData": None,
        "imageHeight": Image.open(img_path).size[1],
        "imageWidth": Image.open(img_path).size[0],
    }
    json_path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    if shapes:
        count += 1

print(f"生成 JSON: {len(imgs)} 个，其中 {count} 个有标注框")
print(f"JSON 位置: {JSON_DIR}")
