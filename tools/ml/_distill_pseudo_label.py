# -*- coding: utf-8 -*-
"""蒸馏数据集构建：航拍 11 类 → 官方 7 类本体 + 厂家模型伪标注（保桶/车/靶类）+ 官方 74 张精确雷。
输出 E:/ml_datasets/aerial_field_7cls（OneDrive 之外，图片硬链接零成本）。

类别契约（与 onboard C++ className 一致）：
  0 mine / 1 car / 2 arrow / 3 target_red / 4 cell_label / 5 target_blue / 6 obstacle

规则：
- 人工真值直转：mine(0)→mine(0)、cell_label(1)→cell_label(4)、arrow(2)→arrow(2)、
  bucket_red/green/cyan(8/9/10)→obstacle(6)；bridge/car_*/uav_* 丢弃（厂家不检桥，车端有先验）
- 厂家伪标注（仅场地帧/官方图；室内调试帧全是桌椅误检必须排除）：
  obstacle conf>=0.5、car conf>=0.35、target_red conf>=0.35；与人工框 IoU>0.4 时人工优先
- 场地帧判定：人工标签 cell_label 行数 >= 20；官方图 = 文件名 explosive_*
"""
import json
import shutil
from pathlib import Path

from ultralytics import YOLO

ROOT = Path(r"E:/OneDrive - mails.qust.edu.cn/桌面/air_ground")
MIXED = ROOT / "capture/datasets/aerial_field_mixed"
OUT = Path(r"E:/ml_datasets/aerial_field_7cls")
BOXES_JSON = ROOT / "runs/distill_7cls/_factory_boxes.json"

CLS_MAP = {0: 0, 1: 4, 2: 2, 8: 6, 9: 6, 10: 6}          # 11类→7类
PSEUDO = {1: 0.35, 3: 0.35, 5: 0.30, 6: 0.50}           # 厂家7类cls -> conf阈值
NAMES7 = {0: "mine", 1: "car", 2: "arrow", 3: "target_red",
          4: "cell_label", 5: "target_blue", 6: "obstacle"}

# ---------- 1. 补 val 帧的厂家推理（train 已在 _factory_boxes.json） ----------
records = json.load(open(BOXES_JSON, encoding="utf-8"))
model = YOLO(str(ROOT / "models/release/hbb_detector.onnx"))
for split in ["train", "val", "test"]:
    for img in sorted((MIXED / "images" / split).glob("*.jpg")):
        if img.name in records:
            continue
        r = model.predict(str(img), imgsz=1280, conf=0.2, verbose=False, device=0)[0]
        records[img.name] = [(int(c), float(f), [float(v) for v in b])
                              for c, f, b in zip(r.boxes.cls, r.boxes.conf, r.boxes.xyxy)]
        print("inferred:", split, img.name)
json.dump(records, open(BOXES_JSON, "w", encoding="utf-8"))
print(f"厂家推理全集: {len(records)} 帧")

# ---------- 2. 帧分类：场地帧 / 官方图 / 室内帧 ----------
def frame_kind(split, stem):
    if stem.startswith("explosive"):
        return "official"
    lbl = MIXED / "labels" / split / (stem + ".txt")
    if not lbl.exists():
        return "indoor"
    n_cell = sum(1 for line in open(lbl, encoding="utf-8")
                 if line.strip() and int(line.split()[0]) == 1)
    return "field" if n_cell >= 20 else "indoor"

# ---------- 3. 工具 ----------
def xywhn_to_xyxy(b, W, H):
    cx, cy, w, h = b
    return [cx * W - w * W / 2, cy * H - h * H / 2, cx * W + w * W / 2, cy * H + h * H / 2]

def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0

def xyxy_to_xywhn(a, W, H):
    return [(a[0] + a[2]) / 2 / W, (a[1] + a[3]) / 2 / H,
            (a[2] - a[0]) / W, (a[3] - a[1]) / H]

# ---------- 4. 逐帧转换 ----------
stats = {}
def bump(kind, cls, n=1):
    for k in ([kind, "all"] if kind else ["all"]):
        stats.setdefault(k, {})
        stats[k][cls] = stats[k].get(cls, 0) + n

if OUT.exists():
    shutil.rmtree(OUT)
(OUT / "images").mkdir(parents=True)
(OUT / "labels").mkdir(parents=True)

n_skip = {"indoor": 0, "no_objects": 0}
for split in ["train", "val", "test"]:
    (OUT / "images" / split).mkdir()
    (OUT / "labels" / split).mkdir()
    for img in sorted((MIXED / "images" / split).glob("*.jpg")):
        kind = frame_kind(split, img.stem)
        if kind == "indoor":   # 室内帧对 7 类无意义（无人工真值且伪标注全是垃圾）
            n_skip["indoor"] += 1
            continue
        from PIL import Image
        with Image.open(img) as im:   # 只读 header 拿尺寸（官方图分辨率不一，不能用固定 4K）
            W, H = im.size
        gt_xyxy = []
        lines = []
        lbl_path = MIXED / "labels" / split / (img.stem + ".txt")
        if lbl_path.exists():
            for line in open(lbl_path, encoding="utf-8"):
                parts = line.split()
                if len(parts) < 5:
                    continue
                src = int(parts[0])
                if src not in CLS_MAP:
                    continue
                dst = CLS_MAP[src]
                xyxy = xywhn_to_xyxy([float(v) for v in parts[1:5]], W, H)
                gt_xyxy.append(xyxy)
                lines.append((dst, xyxy_to_xywhn(xyxy, W, H)))
                bump(kind, dst)
        # 厂家伪标注（仅 obstacle/car/target_red，人工优先）
        for cls, conf, xyxy in records.get(img.name, []):
            if cls not in PSEUDO or conf < PSEUDO[cls]:
                continue
            if any(iou(xyxy, g) > 0.4 for g in gt_xyxy):
                continue
            lines.append((cls, xyxy_to_xywhn(xyxy, W, H)))
            bump(kind, cls)
            bump(kind + "_pseudo", cls)
        if not lines:   # 无任何 7 类目标的帧跳过（避免海量负样本稀释正样本梯度）
            n_skip["no_objects"] += 1
            continue
        with open(OUT / "labels" / split / (img.stem + ".txt"), "w") as fp:
            for cls, b in lines:
                fp.write(f"{cls} {b[0]:.6f} {b[1]:.6f} {b[2]:.6f} {b[3]:.6f}\n")
        link = OUT / "images" / split / img.name
        if not link.exists():
            import os
            os.link(img, link)

# ---------- 5. data.yaml + 统计 ----------
(OUT / "data.yaml").write_text(
    f"path: {OUT.as_posix()}\ntrain: images/train\nval: images/val\ntest: images/test\n\n"
    f"nc: 7\nnames:\n" + "".join(f"  {k}: {v}\n" for k, v in NAMES7.items()),
    encoding="utf-8")

print("\n===== 各类框统计（人工直转 + 厂家伪标注）=====")
for kind in ["field", "official", "field_pseudo", "official_pseudo", "all"]:
    tag = {"field": "场地帧", "official": "官方图", "field_pseudo": "场地帧·伪标注",
           "official_pseudo": "官方图·伪标注", "all": "合计"}.get(kind, kind)
    row = stats.get(kind, {})
    print(f"[{tag}] " + "  ".join(f"{NAMES7[c]}={row.get(c, 0)}" for c in range(7)))
print("跳过: 室内帧", n_skip["indoor"], " 无目标帧", n_skip["no_objects"])
print("data.yaml ->", OUT / "data.yaml")
