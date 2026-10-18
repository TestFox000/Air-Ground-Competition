# -*- coding: utf-8 -*-
"""障碍copy-paste增强：把val集中漏检的障碍裁出，回贴到航拍训练背景。

解决域差异：让检测器直接学习"航拍视角下漏检障碍的纹理/外观"。
针对 obstacle FN=30 瓶颈（四模型均漏检同一批障碍，纯数据域差异问题）。

⚠️ 历史泄漏警示：本脚本旧版本直接从 val 切分取材（MISSED_FRAMES 全部位于
   images/val），贴进 train 后造成 train/val 泄漏，基于该数据集得到的评估
   指标（val mAP / FN 数）均不可信，需用干净 val 重新切分数据集后重测。
   现已加断言：取材帧若落在当前 val/test 切分内将直接拒绝运行，
   须先把这些帧移出 val（或改用 train 帧取材）再使用本脚本。

输出：新的训练图片+标签，追加到7cls数据集train。
用法: python tools/ml/build_copypaste_obstacle.py [--n 500]
"""
from pathlib import Path
import argparse
import random
import numpy as np
from PIL import Image, ImageFilter, ImageEnhance

# 漏检障碍所在val帧（来自eval_comprehensive诊断：00016~00023, 00096~00103, 00255）
MISSED_FRAMES = [
    "frame_00016", "frame_00017", "frame_00018", "frame_00019",
    "frame_00020", "frame_00021", "frame_00022", "frame_00023",
    "frame_00096", "frame_00097", "frame_00098", "frame_00099",
    "frame_00100", "frame_00101", "frame_00102", "frame_00103",
    "frame_00255",
]

VAL_IMG = Path(r"E:\ml_datasets\aerial_field_7cls\images\val")
VAL_LBL = Path(r"E:\ml_datasets\aerial_field_7cls\labels\val")
TRAIN_IMG = Path(r"E:\ml_datasets\aerial_field_7cls\images\train")
TRAIN_LBL = Path(r"E:\ml_datasets\aerial_field_7cls\labels\train")
OUT_IMG = Path(r"E:\ml_datasets\aerial_field_7cls_obstacle_cp\images\train")
OUT_LBL = Path(r"E:\ml_datasets\aerial_field_7cls_obstacle_cp\labels\train")

OBSTACLE_CLS = 6  # 7cls中obstacle的类号

random.seed(42)


def assert_sources_not_in_eval_split(stems):
    """禁止取材帧落在 val/test 切分（参照 build_copypaste_mine_v2.py 排除 val 帧的写法）。

    历史版本直接从 val 帧取材贴回 train，造成评估泄漏；这里按 val/test 目录的
    实际帧集合做硬断言，一旦重叠立即拒绝运行。
    """
    eval_stems = set()
    for split_dir in (VAL_IMG, VAL_IMG.parent / "test"):  # images/val 与 images/test
        if split_dir.is_dir():
            eval_stems |= {p.stem for p in split_dir.iterdir() if p.is_file()}
    leaked = sorted(set(stems) & eval_stems)
    assert not leaked, (
        f"取材帧落在 val/test 切分内，会造成 train/val 泄漏: {leaked[:5]} ..."
        " 请先把这些帧移出 val/test（重新切分）或改用 train 帧取材")


def crop_obstacles_from_val():
    """从漏检帧裁出障碍区域（带padding）。"""
    assert_sources_not_in_eval_split(MISSED_FRAMES)
    obstacles = []
    for stem in MISSED_FRAMES:
        img_file = VAL_IMG / f"{stem}.jpg"
        lbl_file = VAL_LBL / f"{stem}.txt"
        if not img_file.exists() or not lbl_file.exists():
            continue
        img = Image.open(img_file).convert("RGB")
        W, H = img.size
        for line in lbl_file.read_text().strip().splitlines():
            parts = line.split()
            if not parts or int(parts[0]) != OBSTACLE_CLS:
                continue
            cx, cy, w, h = map(float, parts[1:5])
            pad = 0.15  # 多裁15%边缘，保留障碍周围上下文
            x1 = int((cx - w / 2 - pad * w) * W)
            y1 = int((cy - h / 2 - pad * h) * H)
            x2 = int((cx + w / 2 + pad * w) * W)
            y2 = int((cy + h / 2 + pad * h) * H)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(W, x2), min(H, y2)
            if x2 - x1 < 20 or y2 - y1 < 20:
                continue
            crop = img.crop((x1, y1, x2, y2))
            obstacles.append(crop)
    print(f"从漏检val帧裁出 {len(obstacles)} 个障碍")
    return obstacles


def get_train_backgrounds():
    """获取训练背景图（优先选obstacle少的图作为背景）。"""
    bgs = []
    for img_file in sorted(TRAIN_IMG.glob("*.jpg")):
        lbl_file = TRAIN_LBL / (img_file.stem + ".txt")
        obs_count = 0
        if lbl_file.exists():
            for line in lbl_file.read_text().strip().splitlines():
                if line and int(line.split()[0]) == OBSTACLE_CLS:
                    obs_count += 1
        bgs.append((img_file, obs_count))
    bgs.sort(key=lambda x: x[1])  # obstacle少的优先
    return bgs


def augment_crop(crop):
    """对障碍裁剪做几何+颜色增强，增加多样性。"""
    # 随机旋转 ±15°
    if random.random() < 0.7:
        angle = random.uniform(-15, 15)
        crop = crop.rotate(angle, expand=True, fillcolor=(0, 0, 0))
    # 随机缩放 0.7~1.3
    if random.random() < 0.6:
        scale = random.uniform(0.7, 1.3)
        nw = max(20, int(crop.width * scale))
        nh = max(20, int(crop.height * scale))
        crop = crop.resize((nw, nh), Image.LANCZOS)
    # 随机透视（用仿射近似）
    if random.random() < 0.4:
        shear = random.uniform(-0.15, 0.15)
        crop = crop.transform(
            crop.size, Image.AFFINE,
            (1, shear, 0, 0, 1, 0), resample=Image.BILINEAR
        )
    # 颜色抖动
    if random.random() < 0.5:
        crop = ImageEnhance.Color(crop).enhance(random.uniform(0.7, 1.3))
    if random.random() < 0.5:
        crop = ImageEnhance.Brightness(crop).enhance(random.uniform(0.8, 1.2))
    if random.random() < 0.5:
        crop = ImageEnhance.Contrast(crop).enhance(random.uniform(0.7, 1.3))
    # 轻微模糊模拟航拍距离感
    if random.random() < 0.4:
        crop = crop.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.3, 1.2)))
    return crop


def paste_obstacle_on_bg(bg_img, obs_crop, target_area_ratio=0.02):
    """把障碍缩放到航拍尺寸后随机贴到背景上。
    target_area_ratio: 障碍占画面面积比例（航拍障碍约0.5%~5%）。
    """
    W, H = bg_img.size
    bg_area = W * H
    target_area = bg_area * target_area_ratio
    ow, oh = obs_crop.size
    aspect = ow / oh
    new_h = int(np.sqrt(target_area / aspect))
    new_w = int(new_h * aspect)
    new_w = max(20, min(new_w, W // 2))
    new_h = max(20, min(new_h, H // 2))
    obs_resized = obs_crop.resize((new_w, new_h), Image.LANCZOS)

    # 随机位置（避开边缘）
    margin = 15
    x = random.randint(margin, max(margin, W - new_w - margin))
    y = random.randint(margin, max(margin, H - new_h - margin))

    bg_pasted = bg_img.copy()
    bg_pasted.paste(obs_resized, (x, y))

    cx = (x + new_w / 2) / W
    cy = (y + new_h / 2) / H
    w = new_w / W
    h = new_h / H
    return bg_pasted, (cx, cy, w, h)


def build_copypaste_dataset(n_samples=500):
    """构建障碍copy-paste增强数据集。"""
    obstacles = crop_obstacles_from_val()
    if not obstacles:
        print("未裁到任何障碍，退出")
        return
    bgs = get_train_backgrounds()
    print(f"训练背景图: {len(bgs)} 张")

    OUT_IMG.mkdir(parents=True, exist_ok=True)
    OUT_LBL.mkdir(parents=True, exist_ok=True)

    count = 0
    for i in range(n_samples):
        bg_file, _ = random.choice(bgs)
        bg_img = Image.open(bg_file).convert("RGB")

        # 读取背景原有标签
        lbl_file = TRAIN_LBL / (bg_file.stem + ".txt")
        existing_labels = []
        if lbl_file.exists():
            for line in lbl_file.read_text().strip().splitlines():
                if line:
                    existing_labels.append(line)

        # 贴2~5个障碍
        n_obs = random.randint(2, 5)
        new_obs_labels = []
        for _ in range(n_obs):
            obs_crop = random.choice(obstacles)
            obs_crop = augment_crop(obs_crop)
            # 障碍面积比例范围（航拍障碍较大）
            area_ratio = random.uniform(0.005, 0.04)
            bg_img, (cx, cy, w, h) = paste_obstacle_on_bg(bg_img, obs_crop, area_ratio)
            new_obs_labels.append(f"{OBSTACLE_CLS} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")

        out_name = f"obs_cp_{i:04d}.jpg"
        bg_img.save(OUT_IMG / out_name, quality=90)
        all_labels = existing_labels + new_obs_labels
        (OUT_LBL / (Path(out_name).stem + ".txt")).write_text("\n".join(all_labels) + "\n")
        count += 1
        if (i + 1) % 100 == 0:
            print(f"  已生成 {i+1}/{n_samples}")

    print(f"\n完成！生成 {count} 张障碍copy-paste增强图 → {OUT_IMG}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=500, help="生成样本数")
    args = ap.parse_args()
    build_copypaste_dataset(n_samples=args.n)
