"""用 best.pt 测试官方爆炸物图集的泛化能力。"""
import sys
from pathlib import Path
from ultralytics import YOLO

IMG_DIR = Path(r"E:\人工智能算法精英大赛-空地协同侦排挑战赛\手雷图片+通行通道图片\1-爆炸物\B-爆炸物")
OUT_DIR = Path(r"capture\aerial_field\review_mine_generalization")
WEIGHTS = Path(r"runs\detect\runs\detect\aerial_field\weights\best.pt")

OUT_DIR.mkdir(parents=True, exist_ok=True)

model = YOLO(str(WEIGHTS))

exts = {".jpg", ".jpeg", ".png", ".bmp"}
imgs = sorted([p for p in IMG_DIR.iterdir() if p.suffix.lower() in exts])
print(f"测试图片: {len(imgs)} 张")

detected = 0
total_mine = 0
confs = []
missed = []

for i, p in enumerate(imgs):
    results = model.predict(
        source=str(p),
        imgsz=640,
        conf=0.25,
        device="cpu",
        verbose=False,
        save=False,
    )[0]
    mine_boxes = [b for b in results.boxes if int(b.cls) == 0]
    if mine_boxes:
        detected += 1
        for b in mine_boxes:
            total_mine += 1
            confs.append(float(b.conf))
    else:
        missed.append(p.name)
    # 保存前 30 张的可视化
    if i < 30:
        r = model.predict(
            source=str(p), imgsz=640, conf=0.25, device="cpu",
            verbose=False, save=True, project=str(OUT_DIR), name="vis",
            exist_ok=True,
        )
    if (i + 1) % 20 == 0:
        print(f"  {i+1}/{len(imgs)} 已检测 {detected} 张有爆炸物")

print("\n" + "=" * 50)
print(f"总图片数: {len(imgs)}")
print(f"检测到爆炸物的图片: {detected} ({detected/len(imgs)*100:.1f}%)")
print(f"漏检图片: {len(missed)} ({len(missed)/len(imgs)*100:.1f}%)")
print(f"总检测框数: {total_mine}")
if confs:
    print(f"平均置信度: {sum(confs)/len(confs):.3f}")
    print(f"最低置信度: {min(confs):.3f}")
    print(f"最高置信度: {max(confs):.3f}")
print(f"\n漏检图片名（前 20）:")
for n in missed[:20]:
    print(f"  {n}")
print(f"\n可视化结果保存在: {OUT_DIR / 'vis'}")
