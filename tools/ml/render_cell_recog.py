"""渲染格号识别效果：YOLO 检测 cell_label → 裁剪 → cell_classifier_v3 识别 → 标注 → 网页。

用法：
    python tools/ml/render_cell_recog.py
"""
from __future__ import annotations

import base64
import io
import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

# --- 路径 ---
ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
YOLO_WEIGHTS = ROOT / r"runs\detect\runs\detect\aerial_field_finetuned_v2\weights\best.pt"
CELL_WEIGHTS = ROOT / r"runs\cell_classifier_v4c\best.pt"
VAL_DIR = ROOT / r"capture\datasets\aerial_field_mixed\images\val"
LABEL_DIR = ROOT / r"capture\datasets\aerial_field_mixed\labels\val"
OUT_DIR = ROOT / r"runs\cell_recog_render"
OUT_DIR.mkdir(parents=True, exist_ok=True)
DEVICE = "cpu"   # GPU 被并行训练占用，渲染走 CPU

# cell_classifier 模块路径
sys.path.insert(0, str(ROOT / "reference" / "onboard" / "dzuav" / "src"))
from cell_classifier.predictor import CellIdRecognizer  # noqa: E402

# 类别号：cell_label = 1
CELL_LABEL_CLS = 1
CONF_DET = 0.25  # YOLO 检测置信度阈值
IOU_DET = 0.45


def load_font(size: int = 18) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    """尝试加载中文字体，失败则返回默认。"""
    candidates = [
        r"C:\Windows\Fonts\msyhbd.ttc",
        r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\simhei.ttf",
        r"C:\Windows\Fonts\arial.ttf",
    ]
    for path in candidates:
        if Path(path).is_file():
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                continue
    return ImageFont.load_default()


def square_crop(img: Image.Image, x1: float, y1: float, x2: float, y2: float) -> Image.Image:
    """把检测框扩成正方形后裁剪，中心对齐。"""
    w = x2 - x1
    h = y2 - y1
    side = max(w, h) * 1.15  # 留 15% 边距
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    sx1 = max(0, int(cx - side / 2))
    sy1 = max(0, int(cy - side / 2))
    sx2 = min(img.width, int(cx + side / 2))
    sy2 = min(img.height, int(cy + side / 2))
    crop = img.crop((sx1, sy1, sx2, sy2))
    # 强制正方形（Resize 会处理，但这里保证输入是方的）
    side_px = max(crop.width, crop.height)
    canvas = Image.new("RGB", (side_px, side_px), (232, 232, 232))
    canvas.paste(crop, ((side_px - crop.width) // 2, (side_px - crop.height) // 2))
    return canvas


def render_image(
    img_path: Path,
    yolo: YOLO,
    recognizer: CellIdRecognizer,
    font,
) -> tuple[Image.Image, list[dict]]:
    """对单张图做检测+识别+标注，返回标注图和结果列表。"""
    img = Image.open(img_path).convert("RGB")

    # YOLO 检测，只取 cell_label 类
    results = yolo.predict(
        source=str(img_path),
        conf=CONF_DET,
        iou=IOU_DET,
        classes=[CELL_LABEL_CLS],
        verbose=False,
        device=DEVICE,
    )
    boxes = results[0].boxes
    detections: list[dict] = []
    crops: list[Image.Image] = []
    coords: list[tuple[float, float, float, float]] = []

    if boxes is not None and len(boxes) > 0:
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            crop = square_crop(img, x1, y1, x2, y2)
            crops.append(crop)
            coords.append((x1, y1, x2, y2))
            detections.append({"det_conf": conf})

    # cell_classifier 识别（批量 + TTA）
    preds = recognizer.predict(crops) if crops else []

    draw = ImageDraw.Draw(img)
    font_large = load_font(20)

    for i, (pred, det) in enumerate(zip(preds, detections)):
        x1, y1, x2, y2 = coords[i]
        det_conf = det["det_conf"]

        if pred is None:
            # 置信度不足 → 红色框
            color = (220, 50, 50)
            label = f"?? det={det_conf:.2f}"
            det["cell_id"] = None
            det["joint_conf"] = None
        else:
            # 识别成功 → 绿色框
            color = (30, 180, 60)
            label = f"{pred.cell_id} {pred.joint_confidence:.2f}"
            det["cell_id"] = pred.cell_id
            det["joint_conf"] = pred.joint_confidence
            det["col_conf"] = pred.column_confidence
            det["row_conf"] = pred.row_confidence

        draw.rectangle([x1, y1, x2, y2], outline=color, width=2)
        # 标签背景
        bbox = draw.textbbox((x1, y1), label, font=font_large)
        draw.rectangle([bbox[0] - 2, bbox[1] - 2, bbox[2] + 2, bbox[3] + 2], fill=color)
        draw.text((x1, y1), label, fill=(255, 255, 255), font=font_large)

    return img, detections


def image_to_base64(img: Image.Image, quality: int = 70) -> str:
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def main() -> None:
    print("[1/4] 加载 YOLO 检测模型...")
    yolo = YOLO(str(YOLO_WEIGHTS))

    print("[2/4] 加载 cell_classifier_v4c 识别模型...")
    recognizer = CellIdRecognizer(
        str(CELL_WEIGHTS),
        device=DEVICE,
        half=False,
        rotation_tta=True,
        confidence_threshold=0.45,
    )
    font = load_font()

    frame_files = sorted(VAL_DIR.glob("frame_*.jpg"))
    print(f"[3/4] 处理 {len(frame_files)} 张航拍帧...")

    all_results: list[dict] = []
    for idx, img_path in enumerate(frame_files, 1):
        ann_path = LABEL_DIR / (img_path.stem + ".txt")
        gt_count = 0
        if ann_path.is_file():
            with ann_path.open("r", encoding="utf-8") as f:
                for line in f:
                    parts = line.strip().split()
                    if parts and int(parts[0]) == CELL_LABEL_CLS:
                        gt_count += 1

        annotated, dets = render_image(img_path, yolo, recognizer, font)
        b64 = image_to_base64(annotated)
        n_ok = sum(1 for d in dets if d["cell_id"] is not None)
        all_results.append({
            "name": img_path.name,
            "gt_cell_labels": gt_count,
            "detected": len(dets),
            "recognized": n_ok,
            "b64": b64,
            "dets": dets,
        })
        print(f"  [{idx}/{len(frame_files)}] {img_path.name}: "
              f"GT={gt_count} 检测={len(dets)} 识别={n_ok}")

    print("[4/4] 生成网页...")
    html_parts: list[str] = []
    html_parts.append("""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>格号识别效果 - 航拍抽帧集</title>
<style>
  body { font-family: system-ui, "Microsoft YaHei", sans-serif; margin: 0; padding: 20px; background: #1a1a1a; color: #e0e0e0; }
  h1 { font-size: 20px; margin: 0 0 16px; }
  .stats { display: flex; gap: 24px; margin-bottom: 20px; font-size: 14px; }
  .stats span { color: #0fdc78; font-weight: 600; }
  .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(420px, 1fr)); gap: 16px; }
  .card { background: #262626; border-radius: 8px; padding: 10px; }
  .card img { width: 100%; height: auto; border-radius: 4px; display: block; }
  .meta { margin-top: 8px; font-size: 12px; color: #aaa; line-height: 1.5; }
  .meta b { color: #e0e0e0; }
  .legend { font-size: 12px; margin-bottom: 12px; }
  .legend .dot { display: inline-block; width: 12px; height: 12px; border-radius: 2px; vertical-align: middle; margin-right: 4px; }
</style>
</head>
<body>
<h1>格号识别效果（航拍抽帧 val 集）</h1>
<div class="legend">
  <span class="dot" style="background:#1eb43c"></span>识别成功（格号 + 联合置信度）
  <span class="dot" style="background:#dc3232;margin-left:16px"></span>识别失败（置信度 &lt; 0.45，宁缺勿错）
</div>
""")

    total_gt = sum(r["gt_cell_labels"] for r in all_results)
    total_det = sum(r["detected"] for r in all_results)
    total_rec = sum(r["recognized"] for r in all_results)
    # total_det 可能为 0（一帧都没检出），防零除
    rec_rate = f"{total_rec/total_det*100:.1f}%" if total_det else "N/A(无检出)"
    html_parts.append(
        f'<div class="stats">'
        f'<div>帧数：<span>{len(all_results)}</span></div>'
        f'<div>真值格号总数：<span>{total_gt}</span></div>'
        f'<div>YOLO 检出：<span>{total_det}</span></div>'
        f'<div>识别成功：<span>{total_rec}</span></div>'
        f'<div>识别率（检出中）：<span>{rec_rate}</span></div>'
        f'</div><div class="grid">'
    )

    for r in all_results:
        html_parts.append(
            f'<div class="card">'
            f'<img src="data:image/jpeg;base64,{r["b64"]}" alt="{r["name"]}">'
            f'<div class="meta">'
            f'<b>{r["name"]}</b><br>'
            f'真值格号：{r["gt_cell_labels"]} | YOLO检出：{r["detected"]} | 识别成功：{r["recognized"]}'
            f'</div></div>'
        )

    html_parts.append("</div></body></html>")

    out_html = OUT_DIR / "cell_recog_val_v4c.html"
    out_html.write_text("\n".join(html_parts), encoding="utf-8")
    out_json = OUT_DIR / "cell_recog_val_v4c.json"
    out_json.write_text(
        json.dumps([{k: v for k, v in r.items() if k != "b64"} for r in all_results],
                   ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"完成：{out_html}")
    print(f"JSON：{out_json}")


if __name__ == "__main__":
    main()
