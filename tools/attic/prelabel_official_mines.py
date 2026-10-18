"""用 best.pt 对官方爆炸物图集生成 YOLO 预标注 + 可视化 review。

输出:
  capture/mine_official/images/  - 官方图副本
  capture/mine_official/labels/  - YOLO 标签 (cls 0 = mine)
  capture/mine_official/review/  - 画框可视化
"""
from pathlib import Path
import shutil
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

SRC = Path(r"E:\人工智能算法精英大赛-空地协同侦排挑战赛\手雷图片+通行通道图片\1-爆炸物\B-爆炸物")
DST = Path(r"capture\mine_official")
IMG_DIR, LBL_DIR, REV_DIR = DST / "images", DST / "labels", DST / "review"
for d in (IMG_DIR, LBL_DIR, REV_DIR):
    d.mkdir(parents=True, exist_ok=True)

WEIGHTS = Path(r"runs\detect\runs\detect\aerial_field\weights\best.pt")
CONF = 0.10  # 低阈值预标注，宁可多框让人删
CLASS_ID = 0  # mine
CLASS_NAME = "mine"

model = YOLO(str(WEIGHTS))
exts = {".jpg", ".jpeg", ".png", ".bmp"}
src_imgs = sorted([p for p in SRC.iterdir() if p.suffix.lower() in exts])
print(f"官方图片: {len(src_imgs)} 张")

try:
    font = ImageFont.truetype("msyh.ttc", 28)
except OSError:
    font = ImageFont.load_default()

stats = {"labeled": 0, "empty": 0, "boxes": 0}
empty_files = []

for p in src_imgs:
    # 1) 复制原图
    dst_img = IMG_DIR / p.name
    if not dst_img.exists():
        shutil.copy2(p, dst_img)

    # 2) 预标注 (CPU, 不影响训练)
    res = model.predict(source=str(dst_img), imgsz=640, conf=CONF,
                         device="cpu", verbose=False)[0]
    W, H = res.orig_shape[1], res.orig_shape[0]

    lines = []
    boxes_vis = []
    for b in res.boxes:
        if int(b.cls) != CLASS_ID:
            continue
        x1, y1, x2, y2 = b.xyxy[0].tolist()
        cx, cy = (x1 + x2) / 2 / W, (y1 + y2) / 2 / H
        w, h = (x2 - x1) / W, (y2 - y1) / H
        lines.append(f"{CLASS_ID} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
        boxes_vis.append((x1, y1, x2, y2, float(b.conf)))
        stats["boxes"] += 1

    (LBL_DIR / f"{p.stem}.txt").write_text("\n".join(lines), encoding="utf-8")
    if lines:
        stats["labeled"] += 1
    else:
        stats["empty"] += 1
        empty_files.append(p.name)

    # 3) 可视化（PIL, 兼容中文路径; 漏检的标红题注）
    im = Image.open(dst_img).convert("RGB")
    dr = ImageDraw.Draw(im)
    for x1, y1, x2, y2, b_conf in boxes_vis:
        dr.rectangle([x1, y1, x2, y2], outline=(0, 255, 0), width=4)
        dr.text((x1, max(0, y1 - 34), ), f"{CLASS_NAME} {b_conf:.2f}",
                fill=(0, 255, 0), font=font)
    if not boxes_vis:
        dr.text((10, 10), "NO DETECTION - NEEDS MANUAL LABEL",
                fill=(255, 0, 0), font=font)
    # 缩到 800px 宽省空间
    scale = 800 / im.width
    im = im.resize((800, int(im.height * scale)))
    im.save(REV_DIR / f"{p.stem}.jpg", quality=85)

print("\n" + "=" * 50)
print(f"有预标注框: {stats['labeled']} 张")
print(f"完全漏检(需手动画框): {stats['empty']} 张")
print(f"预标注总框数: {stats['boxes']}")
print(f"\n需手动标注的图片:")
for n in empty_files:
    print(f"  {n}")
