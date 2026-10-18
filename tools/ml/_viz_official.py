"""可视化官方图: GT框(绿) vs 预测框(红+conf), 保存对比图。"""
from __future__ import annotations

import random
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
IMG = ROOT / "capture/mine_official/images"
LBL = ROOT / "capture/mine_official/labels"
OUT = ROOT / "runs/detect/official_check"
OUT.mkdir(parents=True, exist_ok=True)

random.seed(0)
model = YOLO(str(ROOT / "runs/detect/runs/detect/aerial_field_finetuned_b2/weights/best.pt"))
try:
    font = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 40)
except OSError:
    font = ImageFont.load_default()

# 选3张: 最高conf的117, 中等conf的一张, 以及无GT高conf的043(GT漏标嫌疑)
targets = ["explosive_117", "explosive_096", "explosive_043"]
for stem in targets:
    img = Image.open(IMG / f"{stem}.jpg").convert("RGB")
    W, H = img.size
    dr = ImageDraw.Draw(img)
    lp = LBL / f"{stem}.txt"
    if lp.is_file():
        for line in lp.open(encoding="utf-8"):
            p = line.split()
            if p and p[0] == "0":
                cx, cy, w, h = map(float, p[1:5])
                # 原式 x2 写成 (cx+w/2)*H*0+W*(cx+w/2)，*H*0 是废表达式，直接按 W 算
                x1, y1, x2, y2 = (cx-w/2)*W, (cy-h/2)*H, (cx+w/2)*W, (cy+h/2)*H
                dr.rectangle([x1, y1, x2, y2], outline=(0, 255, 0), width=6)
                dr.text((x1+4, y1+4), "GT", fill=(0, 255, 0), font=font)
    r = model.predict(str(IMG / f"{stem}.jpg"), conf=0.05, imgsz=1280, verbose=False, device=0)[0]
    for box in r.boxes:
        if int(box.cls) != 0:
            continue
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        c = float(box.conf)
        dr.rectangle([x1, y1, x2, y2], outline=(255, 0, 0), width=4)
        dr.text((x1+4, y2-48), f"{c:.2f}", fill=(255, 0, 0), font=font)
    img.thumbnail((900, 900))
    img.save(OUT / f"diag_{stem}.jpg")
    print(f"saved {OUT / f'diag_{stem}.jpg'} (GT绿框={sum(1 for l in lp.open() if l.split() and l.split()[0]=='0') if lp.is_file() else 0})")
