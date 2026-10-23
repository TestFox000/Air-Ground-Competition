# -*- coding: utf-8 -*-
"""构建雷分类器数据集：正样本=航拍雷，负样本=航拍非雷目标。"""
from pathlib import Path
from PIL import Image
import random, shutil

AERIAL = Path(r"E:\ml_datasets\aerial_field_7cls")
OUT = Path(r"E:\ml_datasets\mine_classifier")
NAMES = {0:"mine",1:"car",2:"arrow",3:"target_red",4:"cell_label",5:"target_blue",6:"obstacle"}

for split in ["train","val"]:
    for cls in ["mine","non_mine"]:
        (OUT/"images"/split/cls).mkdir(parents=True, exist_ok=True)

random.seed(42)
mine_count = {"train":0,"val":0}
non_count = {"train":0,"val":0}

for split in ["train","val"]:
    img_dir = AERIAL/"images"/split
    lbl_dir = AERIAL/"labels"/split
    for img_file in sorted(img_dir.glob("*.jpg")):
        img = Image.open(img_file).convert("RGB")
        W,H = img.size
        lbl_file = lbl_dir/(img_file.stem+".txt")
        if not lbl_file.exists(): continue
        for line in lbl_file.read_text().strip().splitlines():
            if not line: continue
            parts = line.split()
            cls = int(parts[0])
            cx,cy,w,h = map(float, parts[1:5])
            x1=int((cx-w/2)*W); y1=int((cy-h/2)*H)
            x2=int((cx+w/2)*W); y2=int((cy+h/2)*H)
            x1,y1=max(0,x1),max(0,y1); x2,y2=min(W,x2),min(H,y2)
            if x2-x1<5 or y2-y1<5: continue
            crop = img.crop((x1,y1,x2,y2)).resize((96,96))
            if cls==0:
                crop.save(OUT/"images"/split/"mine"/f"{img_file.stem}_{mine_count[split]}.jpg",quality=90)
                mine_count[split]+=1
            else:
                # 负样本：每个非雷类别采样一部分，避免类别不平衡
                if random.random()<0.3:
                    crop.save(OUT/"images"/split/"non_mine"/f"{img_file.stem}_{non_count[split]}.jpg",quality=90)
                    non_count[split]+=1

print(f"mine: train={mine_count['train']}, val={mine_count['val']}")
print(f"non_mine: train={non_count['train']}, val={non_count['val']}")

# 写data.yaml
(OUT/"data.yaml").write_text(f"""path: {OUT.as_posix()}
train: images/train
val: images/val
names:
  0: mine
  1: non_mine
""", encoding="utf-8")
print("data.yaml已生成")
