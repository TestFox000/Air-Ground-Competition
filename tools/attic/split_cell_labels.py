"""均匀划分 train/val，确保每个字母和数字在 val 中均匀分布。"""
import csv
import random
from collections import defaultdict
from pathlib import Path

CSV = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\datasets\cell_labels\annotations.csv")

records = []
with CSV.open("r", encoding="utf-8-sig", newline="") as f:
    for row in csv.DictReader(f):
        records.append(row)

random.seed(42)

# 按数字分组，每个数字（1-11）选 1-2 个做 val，均匀分布
by_row = defaultdict(list)
for r in records:
    by_row[r["row"]].append(r)

val_set = set()
# 每个数字选 2 个做 val（11 数字 × 2 = 22）
for row, rs in by_row.items():
    random.shuffle(rs)
    for r in rs[:2]:
        val_set.add(r["cell_id"])

# 确保每个字母在 train 中至少有 8 个样本（即 val 中不超过 3 个同字母）
by_col_val = defaultdict(int)
for cid in val_set:
    col = cid[0]
    by_col_val[col] += 1

# 确保 train 覆盖所有字母和数字
train_letters = set()
train_rows = set()
for r in records:
    if r["cell_id"] not in val_set:
        train_letters.add(r["column"])
        train_rows.add(r["row"])

# 如果有字母/数字不在 train，把它的一个 val 改回 train
for col in "ABCDEFGHIJKLMN":
    if col not in train_letters:
        for r in records:
            if r["column"] == col and r["cell_id"] in val_set:
                val_set.discard(r["cell_id"])
                break
for row in [str(i) for i in range(1, 12)]:
    if row not in train_rows:
        for r in records:
            if r["row"] == row and r["cell_id"] in val_set:
                val_set.discard(r["cell_id"])
                break

# 写回
for r in records:
    r["split"] = "val" if r["cell_id"] in val_set else "train"

with CSV.open("w", encoding="utf-8-sig", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(records[0].keys()))
    w.writeheader()
    w.writerows(records)

# 统计
from collections import Counter
train_rows_dist = Counter(r["row"] for r in records if r["split"] == "train")
val_rows_dist = Counter(r["row"] for r in records if r["split"] == "val")
train_cols_dist = Counter(r["column"] for r in records if r["split"] == "train")
val_cols_dist = Counter(r["column"] for r in records if r["split"] == "val")

train_n = sum(1 for r in records if r["split"] == "train")
val_n = sum(1 for r in records if r["split"] == "val")
print(f"train: {train_n}, val: {val_n}")
print(f"val 数字分布: {dict(sorted(val_rows_dist.items(), key=lambda x: int(x[0])))}")
print(f"val 字母分布: {dict(sorted(val_cols_dist.items()))}")
print(f"train 字母覆盖: {len(train_cols_dist)}/14, 数字覆盖: {len(train_rows_dist)}/11")
