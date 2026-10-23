# -*- coding: utf-8 -*-
"""分析：场地帧（有人工格号标注的帧）上厂家 car/靶/obstacle 的检出分布。
室内调试帧（无格号标注）必须排除——厂家会在室内把桌椅鞋检成 obstacle/car。"""
import collections
import json
from pathlib import Path

ROOT = Path(r"E:/OneDrive - mails.qust.edu.cn/桌面/air_ground")
DATASET = ROOT / "capture/datasets/aerial_field_mixed"
NAMES = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]

records = json.load(open(ROOT / "runs/distill_7cls/_factory_boxes.json", encoding="utf-8"))

# 场地帧判定：人工标签里 cell_label(1) 行数 >= 20
field_frames = {}
for split in ["train", "val"]:
    for lbl in (DATASET / "labels" / split).glob("*.txt"):
        n_cell = sum(1 for line in open(lbl, encoding="utf-8")
                     if line.strip() and int(line.split()[0]) == 1)
        if n_cell >= 20:
            field_frames[lbl.stem] = split

print(f"场地帧: {len(field_frames)} / {len(records)} （其余为室内帧/官方图）")

# 分组统计
groups = collections.defaultdict(list)  # (split, is_field) -> [(cls, conf)]
car_boxes = collections.defaultdict(list)   # 帧名 -> [xyxy]
tgt_boxes = collections.defaultdict(list)
for name, boxes in records.items():
    key = field_frames.get(Path(name).stem)   # records 的 key 带 .jpg 后缀
    is_field = key is not None
    is_official = name.startswith("explosive")
    tag = "官方图" if is_official else ("场地帧" if is_field else "室内帧")
    for cls, conf, xyxy in boxes:
        groups[(tag, cls)].append((conf, xyxy))
        if cls in (1, 3, 5):
            (car_boxes if cls == 1 else tgt_boxes)[name].append((cls, conf, xyxy))

for tag in ["场地帧", "室内帧", "官方图"]:
    print(f"\n===== {tag} =====")
    for cls in [1, 3, 5, 6]:
        items = groups.get((tag, cls), [])
        if not items:
            print(f"  {NAMES[cls]:<12} total=0")
            continue
        confs = sorted(c for c, _ in items)
        n = len(confs)
        hi = sum(c >= 0.5 for c in confs)
        print(f"  {NAMES[cls]:<12} total={n:<4} >=0.5:{hi:<4} median={confs[n//2]:.3f} max={confs[-1]:.3f}")

# 场地帧上 car/靶 框的中心像素分布（看是否同一物体的重复检出）
print("\n===== 场地帧 car 检出明细 =====")
for name, boxes in sorted(car_boxes.items()):
    stem = Path(name).stem  # field_frames 以 stem 为键，records 的 key 带 .jpg 后缀
    if stem not in field_frames:
        continue
    for cls, conf, xyxy in boxes:
        cx, cy = (xyxy[0]+xyxy[2])/2, (xyxy[1]+xyxy[3])/2
        print(f"  {name}: car conf={conf:.2f} center=({cx:.0f},{cy:.0f}) size={xyxy[2]-xyxy[0]:.0f}x{xyxy[3]-xyxy[1]:.0f}")

print("\n===== 场地帧 靶 检出明细 =====")
for name, boxes in sorted(tgt_boxes.items()):
    stem = Path(name).stem  # 同上：必须用 stem 查 field_frames
    if stem not in field_frames:
        continue
    for cls, conf, xyxy in boxes:
        cx, cy = (xyxy[0]+xyxy[2])/2, (xyxy[1]+xyxy[3])/2
        print(f"  {name}: {NAMES[cls]} conf={conf:.2f} center=({cx:.0f},{cy:.0f}) size={xyxy[2]-xyxy[0]:.0f}x{xyxy[3]-xyxy[1]:.0f}")
