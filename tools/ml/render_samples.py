"""Render 4 sample images with bounding boxes for category mapping review."""
import json, os, base64, io
from PIL import Image, ImageDraw, ImageFont

SAMPLES = [
    (r"E:\ml_datasets\manual_hq_5\batch1\VID_20261011_124506_f006040.jpg",
     r"E:\ml_datasets\manual_hq_5\batch1\VID_20261011_124506_f006040.json",
     "tile_0: green target + red bucket + target"),
    (r"E:\ml_datasets\manual_hq_5\batch1\VID_20261011_124506_f000240.jpg",
     r"E:\ml_datasets\manual_hq_5\batch1\VID_20261011_124506_f000240.json",
     "tile_1: green target + blue bucket"),
    (r"E:\ml_datasets\manual_hq_5\batch1\VID_20261011_124745_f008120.jpg",
     r"E:\ml_datasets\manual_hq_5\batch1\VID_20261011_124745_f008120.json",
     "tile_2: r'r typo + green target + target"),
    (r"E:\ml_datasets\manual_hq_5\batch2\VID_20261011_124745_f004880.jpg",
     r"E:\ml_datasets\manual_hq_5\batch2\VID_20261011_124745_f004880.json",
     "tile_3: RR typo + red bucket + blue bucket + target"),
]

COLOR_MAP = {
    "green target": (0, 255, 0),
    "target": (255, 0, 0),
    "red bucket": (255, 165, 0),
    "blue bucket": (0, 100, 255),
    "green bucket": (0, 200, 200),
    "r'r": (255, 255, 0),
    "RR": (255, 255, 0),
}

def bbox_from_points(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), min(ys), max(xs), max(ys)

def render_one(img_path, json_path, title, out_w=640):
    img = Image.open(img_path).convert("RGB")
    w, h = img.size
    scale = out_w / w
    out_h = int(h * scale)
    img = img.resize((out_w, out_h), Image.LANCZOS)
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("arial.ttf", 22)
    except Exception:
        font = ImageFont.load_default()

    with open(json_path, encoding="utf-8") as f:
        data = json.load(f)

    for shape in data["shapes"]:
        label = shape["label"]
        x1, y1, x2, y2 = bbox_from_points(shape["points"])
        x1 *= scale; y1 *= scale; x2 *= scale; y2 *= scale
        color = COLOR_MAP.get(label, (255, 255, 255))
        draw.rectangle([x1, y1, x2, y2], outline=color, width=4)
        draw.rectangle([x1, y1 - 28, x1 + len(label) * 14 + 12, y1], fill=color)
        draw.text((x1 + 6, y1 - 26), label, fill=(0, 0, 0), font=font)

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=55)
    return base64.b64encode(buf.getvalue()).decode()

if __name__ == "__main__":
    results = []
    for img_p, json_p, title in SAMPLES:
        b64 = render_one(img_p, json_p, title)
        results.append((title, b64))
    out_dir = r"E:\OneDrive - mails.qust.edu.cn\桌面\air_ground\tools\ml"
    with open(os.path.join(out_dir, "_sample_b64.json"), "w", encoding="utf-8") as f:
        json.dump(results, f)
    print(f"Rendered {len(results)} samples -> _sample_b64.json")
