"""Merge Wave1 (seed_157) + Wave2 (manual_hq_5) into one target dataset.

Category mapping (6 classes, same as seed_157):
  0: blue bucket
  1: car
  2: green bucket   <- green target merged here
  3: plane
  4: red bucket     <- r'r, RR typos merged here
  5: target

Split: sort by (video_id, frame_number), time-block split 85/15.
"""
import json, os, shutil, re
from pathlib import Path

CLS_MAP = {
    "blue bucket": 0,
    "car": 1,
    "green bucket": 2,
    "green target": 2,   # merge into green bucket
    "plane": 3,
    "red bucket": 4,
    "r'r": 4,            # typo -> red bucket
    "RR": 4,             # typo -> red bucket
    "target": 5,
}

SEED157 = Path(r"E:\ml_datasets\seed_157")
HQ5 = Path(r"E:\ml_datasets\manual_hq_5")
OUT = Path(r"E:\ml_datasets\target_merged")

def parse_frame_key(name):
    """Extract (video_id, frame_number) from filename.
    m001_VID_20261011_124506_f001440.jpg or VID_20261011_124506_f001440.jpg
    """
    m = re.search(r"VID_(\d+_\d+)_f(\d+)", name)
    if m:
        return (m.group(1), int(m.group(2)))
    return (name, 0)

def bbox_from_points(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), min(ys), max(xs), max(ys)

def yolo_line(label, points, img_w, img_h):
    x1, y1, x2, y2 = bbox_from_points(points)
    cx = (x1 + x2) / 2 / img_w
    cy = (y1 + y2) / 2 / img_h
    w = (x2 - x1) / img_w
    h = (y2 - y1) / img_h
    return f"{label} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"

def main():
    if OUT.exists():
        shutil.rmtree(OUT)
    (OUT / "images" / "train").mkdir(parents=True)
    (OUT / "images" / "val").mkdir(parents=True)
    (OUT / "labels" / "train").mkdir(parents=True)
    (OUT / "labels" / "val").mkdir(parents=True)

    samples = {}  # key: (video_id, frame) -> (img_path, label_lines, source)

    # --- Wave1: seed_157 (already YOLO labels) ---
    w1_count = 0
    for split in ("train", "val"):
        img_dir = SEED157 / "images" / split
        lbl_dir = SEED157 / "labels" / split
        if not img_dir.exists():
            continue
        for img in img_dir.glob("*.jpg"):
            key = parse_frame_key(img.name)
            lbl = lbl_dir / (img.stem + ".txt")
            if not lbl.exists():
                continue
            lines = lbl.read_text(encoding="utf-8").strip().splitlines()
            # normalize: keep only valid class lines
            clean = []
            for ln in lines:
                parts = ln.split()
                if len(parts) == 5:
                    try:
                        c = int(parts[0])
                        if 0 <= c <= 5:
                            clean.append(" ".join(parts))
                    except ValueError:
                        pass
            samples[key] = (str(img), clean, "wave1")
            w1_count += 1

    # --- Wave2: manual_hq_5 (JSON -> YOLO) ---
    w2_count = 0
    skip_label = 0
    for json_path in HQ5.rglob("*.json"):
        with open(json_path, encoding="utf-8") as f:
            data = json.load(f)
        img_w = data.get("imageWidth", 3840)
        img_h = data.get("imageHeight", 2160)
        img_name = data.get("imagePath", json_path.stem + ".jpg")
        img_path = json_path.parent / img_name
        if not img_path.exists():
            # try same stem
            img_path = json_path.parent / (json_path.stem + ".jpg")
        if not img_path.exists():
            continue
        key = parse_frame_key(img_path.name)
        lines = []
        for shape in data.get("shapes", []):
            lbl = shape["label"]
            if lbl not in CLS_MAP:
                skip_label += 1
                continue
            cls = CLS_MAP[lbl]
            try:
                lines.append(yolo_line(cls, shape["points"], img_w, img_h))
            except Exception:
                pass
        if key in samples:
            # prefer wave1 (already YOLO), but if wave2 has more boxes, use wave2
            if len(lines) > len(samples[key][1]):
                samples[key] = (str(img_path), lines, "wave2")
        else:
            samples[key] = (str(img_path), lines, "wave2")
        w2_count += 1

    # --- Sort & split by time block ---
    sorted_keys = sorted(samples.keys())
    n = len(sorted_keys)
    val_count = max(1, int(n * 0.15))
    # take last 15% as val (time-block: later frames = val)
    val_keys = set(sorted_keys[-val_count:])

    train_n, val_n = 0, 0
    cls_counter = {0:0,1:0,2:0,3:0,4:0,5:0}
    for key in sorted_keys:
        img_path, lines, src = samples[key]
        split = "val" if key in val_keys else "train"
        # unique filename: prefix source to avoid collision
        stem = Path(img_path).stem
        if src == "wave1":
            out_name = stem
        else:
            out_name = "hq_" + stem
        shutil.copy2(img_path, OUT / "images" / split / (out_name + ".jpg"))
        with open(OUT / "labels" / split / (out_name + ".txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        for ln in lines:
            c = int(ln.split()[0])
            cls_counter[c] += 1
        if split == "train":
            train_n += 1
        else:
            val_n += 1

    # data.yaml
    yaml = f"""path: E:\\ml_datasets\\target_merged
train: images/train
val: images/val
names:
  0: blue bucket
  1: car
  2: green bucket
  3: plane
  4: red bucket
  5: target
"""
    (OUT / "data.yaml").write_text(yaml, encoding="utf-8")

    print(f"Wave1 images: {w1_count}")
    print(f"Wave2 images: {w2_count} (skipped labels: {skip_label})")
    print(f"Total unique frames: {n}")
    print(f"Train: {train_n}, Val: {val_n}")
    print(f"Class distribution: {cls_counter}")
    print(f"Output: {OUT}")

if __name__ == "__main__":
    main()
