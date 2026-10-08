"""OpenCV OCR 格号识别原型 v2 — 三重冗余管线第二识别器。

验证教员建议：OCR 无训练参数，可避免 CNN 的批次过拟合问题。

v1 教训（ocr_vs_v4c 第一版：OCR 成功率 1%，一致率 0）：
  - 实际场地格号存在"黄绿底黑字"批次，深蓝 HSV 颜色先验失效
  - 自适应二值化兜底噪点极多，笔画碎裂 → 字符分割产出碎片 → 全匹配成 I1/I7

v2 策略：
  - mask 双路径：HSV 深蓝有效（0.2%~20% 像素）用 HSV，否则走 CLAHE+亮度自适应二值化
  - 加强闭运算连接笔画碎片；bbox 用 5%~95% 分位像素范围抗远处噪点
  - 整字匹配替代字符分割：文字块直接与 154 个合法格号模板 × 7 角度做 NCC
    （结果天然是合法格号，规避分割碎片问题）

评估：32 个 val 帧，与 v4c 交叉对比；按文字高度分层（教员边界：≥18px 可行，<12px 不可行）

用法：
    python tools/ml/ocr_cell_prototype.py
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
YOLO_WEIGHTS = ROOT / r"runs\detect\runs\detect\aerial_field_finetuned_v2\weights\best.pt"
VAL_DIR = ROOT / r"capture\datasets\aerial_field_mixed\images\val"
V4C_JSON = ROOT / r"runs\cell_recog_render\cell_recog_val_v4c.json"
OUT_DIR = ROOT / "runs\ocr_prototype"
OUT_DIR.mkdir(parents=True, exist_ok=True)

CELL_LABEL_CLS = 1
CONF_DET = 0.25
IOU_DET = 0.45
UPSCALE = 4                # 放大倍数（文字高度 15-25px → 60-100px）
DEVICE = "cpu"             # GPU 被训练占用

LETTERS = "ABCDEFGHIJKLMN"
N_ROWS = 11
ALL_CELLS = [f"{c}{r}" for c in LETTERS for r in range(1, N_ROWS + 1)]
FONT_PATH = "C:/Windows/Fonts/arial.ttf"

TMPL_HEIGHT = 48                                    # 模板统一高度
ANGLES = [-15, -12, -9, -6, -3, 0, 3, 6, 9, 12, 15]  # 多角度模板（航拍偏航）
MIN_CELL_CONF = 0.50                                 # 整字最低 NCC 分
MIN_TEXT_RATIO = 0.12                                # 文字高度/crop 高度 合理范围
MAX_TEXT_RATIO = 0.60


# ---------- 检测与裁剪（与 render_cell_recog.py 完全一致） ----------

def square_crop(img: Image.Image, x1: float, y1: float, x2: float, y2: float) -> Image.Image:
    """把检测框扩成正方形后裁剪，中心对齐。"""
    w = x2 - x1
    h = y2 - y1
    side = max(w, h) * 1.15
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    sx1 = max(0, int(cx - side / 2))
    sy1 = max(0, int(cy - side / 2))
    sx2 = min(img.width, int(cx + side / 2))
    sy2 = min(img.height, int(cy + side / 2))
    crop = img.crop((sx1, sy1, sx2, sy2))
    side_px = max(crop.width, crop.height)
    canvas = Image.new("RGB", (side_px, side_px), (232, 232, 232))
    canvas.paste(crop, ((side_px - crop.width) // 2, (side_px - crop.height) // 2))
    return canvas


# ---------- 整字模板渲染 ----------

def _binary_tight(img_l: Image.Image) -> np.ndarray | None:
    """灰度图 → 二值紧致 bbox 裁剪。"""
    arr = np.array(img_l)
    ys, xs = np.nonzero(arr > 60)
    if len(ys) < 4:
        return None
    crop = arr[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    h = crop.shape[0]
    if h <= 4:
        return None
    w = max(2, int(round(crop.shape[1] * TMPL_HEIGHT / h)))
    crop = cv2.resize(crop, (w, TMPL_HEIGHT), interpolation=cv2.INTER_AREA)
    return (crop >= 128).astype(np.uint8) * 255


def render_cell_templates() -> dict[str, list[np.ndarray]]:
    """渲染 154 个合法格号 × 多角度 的二值整字模板。"""
    templates: dict[str, list[np.ndarray]] = {}
    font = ImageFont.truetype(FONT_PATH, TMPL_HEIGHT * 2)  # 2x 渲染后缩小，抗锯齿
    size = TMPL_HEIGHT * 6
    for cell in ALL_CELLS:
        variants = []
        for angle in ANGLES:
            img = Image.new("L", (size, size), 0)
            draw = ImageDraw.Draw(img)
            bbox = draw.textbbox((0, 0), cell, font=font)
            w = bbox[2] - bbox[0]
            h = bbox[3] - bbox[1]
            x = (size - w) // 2 - bbox[0]
            y = (size - h) // 2 - bbox[1]
            draw.text((x, y), cell, font=font, fill=255)
            img = img.rotate(angle, resample=Image.BICUBIC, expand=False)
            t = _binary_tight(img)
            if t is not None:
                variants.append(t)
        templates[cell] = variants
    return templates


def ncc_match(a: np.ndarray, b: np.ndarray) -> float:
    """两个二值图的对齐 NCC（pad 到同尺寸，中心对齐）。"""
    h, w = a.shape
    th, tw = b.shape
    H = max(h, th) + 2
    W = max(w, tw) + 2
    pa = np.zeros((H, W), np.uint8)
    pa[(H - h) // 2:(H - h) // 2 + h, (W - w) // 2:(W - w) // 2 + w] = a
    pb = np.zeros((H, W), np.uint8)
    pb[(H - th) // 2:(H - th) // 2 + th, (W - tw) // 2:(W - tw) // 2 + tw] = b
    res = cv2.matchTemplate(pa, pb, cv2.TM_CCOEFF_NORMED)
    return float(res.max())


def match_cell(text_img: np.ndarray, templates: dict[str, list[np.ndarray]]) -> tuple[str, float]:
    """文字块在 154 格号模板集内做 NCC，返回 (最佳格号, 置信度)。"""
    h, w = text_img.shape
    if h < 4 or w < 4:
        return "", 0.0
    scaled = cv2.resize(text_img, (max(2, int(round(w * TMPL_HEIGHT / h))), TMPL_HEIGHT),
                       interpolation=cv2.INTER_AREA)
    best_cell, best_score = "", -1.0
    for cell, variants in templates.items():
        for tmpl in variants:
            score = ncc_match(scaled, tmpl)
            if score > best_score:
                best_score = score
                best_cell = cell
    return best_cell, max(0.0, best_score)


# ---------- 文字 mask 提取 ----------

def _largest_block_h(mask: np.ndarray) -> int:
    """最大连通域高度（估字符高度，用于自适应闭运算核）。"""
    n_lab, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n_lab <= 1:
        return 0
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return int(stats[idx, cv2.CC_STAT_HEIGHT])


def extract_text_mask(crop: Image.Image) -> tuple[np.ndarray, float, str] | None:
    """提取文字二值块。返回 (文字块 mask, 原始 crop 中文字高度 px, mask路径) 或 None。

    文字块定位：最大连通域 seed + 邻近聚集合（不依赖固定 ROI 比例，
    因为大牌子批次文字纵向位置浮动，固定比例会截断文字）。
    """
    arr = cv2.cvtColor(np.array(crop), cv2.COLOR_RGB2BGR)
    big = cv2.resize(arr, None, fx=UPSCALE, fy=UPSCALE, interpolation=cv2.INTER_CUBIC)
    Hb, Wb = big.shape[:2]
    total = Hb * Wb

    # 路径 A：HSV 深蓝（真实深蓝字批次干净有效）
    hsv = cv2.cvtColor(big, cv2.COLOR_BGR2HSV)
    mask_hsv = cv2.inRange(hsv, (95, 30, 20), (150, 255, 180))
    n_blue = int((mask_hsv > 0).sum())

    if total * 0.002 <= n_blue <= total * 0.20:
        mask = mask_hsv
        path = "hsv"
    else:
        # 路径 B：CLAHE + 亮度自适应二值化（暗文字，颜色无关）
        lab = cv2.cvtColor(big, cv2.COLOR_BGR2LAB)
        l = cv2.createCLAHE(2.0, (8, 8)).apply(lab[:, :, 0])
        mask = cv2.adaptiveThreshold(l, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                     cv2.THRESH_BINARY_INV, 31, 8)
        path = "luma"

    # 清理：开运算去小噪点 → 估字符高度 → 闭运算连接断笔 → 再开
    k_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)
    h_est = _largest_block_h(mask)
    if h_est < 8:
        return None
    k_close = cv2.getStructuringElement(
        cv2.MORPH_RECT, (max(3, int(h_est * 0.35)), max(3, int(h_est * 0.25))))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)

    # 文字块定位：最大连通域 seed → 邻近聚集合（同行 ±0.8h 垂直 / ±2.5h 水平）
    n_lab, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n_lab <= 1:
        return None
    areas = stats[1:, cv2.CC_STAT_AREA]
    seed = 1 + int(np.argmax(areas))
    sx, sy, sw, sh = (stats[seed, i] for i in
                      (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                       cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
    text_img = np.zeros((sh, 1), np.uint8)
    min_area = max(20.0, 0.02 * sh * sh)
    boxes = []
    for j in range(1, n_lab):
        x, y, w_, h_ = (stats[j, i] for i in
                        (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                         cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
        # 与 seed 同一文字行：垂直重叠+邻近，水平在 seed 左右 2.5 倍字符高内
        v_ok = (y < sy + sh + 0.8 * sh) and (y + h_ > sy - 0.8 * sh)
        h_ok = (x < sx + sw + 2.5 * sh) and (x + w_ > sx - 2.5 * sh)
        if v_ok and h_ok and stats[j, cv2.CC_STAT_AREA] >= min_area:
            boxes.append((x, y, w_, h_))
    if not boxes:
        return None
    X1 = min(b[0] for b in boxes)
    Y1 = min(b[1] for b in boxes)
    X2 = max(b[0] + b[2] for b in boxes)
    Y2 = max(b[1] + b[3] for b in boxes)
    if Y2 - Y1 < 6 or X2 - X1 < 4:
        return None
    text_img = mask[Y1:Y2 + 1, X1:X2 + 1]
    text_h_orig = (Y2 - Y1 + 1) / UPSCALE
    return text_img, text_h_orig, path


# ---------- OCR 主入口 ----------

def ocr_cell_id(crop: Image.Image, templates: dict) -> dict:
    """对单个 crop 做 OCR。返回 {cell_id, conf, text_h, path, err}。"""
    out = {"cell_id": None, "conf": 0.0, "text_h": None, "path": None, "err": None}
    result = extract_text_mask(crop)
    if result is None:
        out["err"] = "no_text"
        return out
    text_img, text_h, path = result
    out["text_h"] = round(text_h, 1)
    out["path"] = path
    ratio = text_h / crop.height
    if ratio < MIN_TEXT_RATIO or ratio > MAX_TEXT_RATIO:
        out["err"] = f"ratio={ratio:.2f}"
        return out
    cell, conf = match_cell(text_img, templates)
    out["conf"] = round(conf, 3)
    if conf < MIN_CELL_CONF:
        out["err"] = f"conf={conf:.2f}"
        return out
    out["cell_id"] = cell
    return out


# ---------- 主流程 ----------

def main() -> None:
    print("[1/5] 渲染整字模板（154 格号 × 11 角度）...")
    templates = render_cell_templates()

    print("[2/5] 加载 YOLO 检测模型（CPU）...")
    yolo = YOLO(str(YOLO_WEIGHTS))

    v4c_data = json.loads(V4C_JSON.read_text(encoding="utf-8"))
    v4c_by_frame = {r["name"]: r for r in v4c_data}

    frame_files = sorted(VAL_DIR.glob("frame_*.jpg"))
    print(f"[3/5] 处理 {len(frame_files)} 张航拍帧...")

    all_results: list[dict] = []
    for idx, img_path in enumerate(frame_files, 1):
        v4c = v4c_by_frame.get(img_path.name, {"dets": []})
        results = yolo.predict(source=str(img_path), conf=CONF_DET, iou=IOU_DET,
                               classes=[CELL_LABEL_CLS], verbose=False, device=DEVICE)
        boxes = results[0].boxes
        img = Image.open(img_path).convert("RGB")
        dets: list[dict] = []
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                det_conf = float(box.conf[0])
                crop = square_crop(img, x1, y1, x2, y2)
                ocr = ocr_cell_id(crop, templates)
                ocr["det_conf"] = det_conf
                ocr["side_px"] = crop.width
                dets.append(ocr)
        # 与 v4c 按索引对齐（同样确定性 CPU 推理，顺序一致）
        for i, d in enumerate(dets):
            d["v4c_id"] = v4c["dets"][i]["cell_id"] if i < len(v4c.get("dets", [])) else None
            d["v4c_conf"] = v4c["dets"][i].get("joint_conf") if i < len(v4c.get("dets", [])) else None
            d["agree"] = (d["cell_id"] is not None and d["cell_id"] == d["v4c_id"])
        n_ocr = sum(1 for d in dets if d["cell_id"] is not None)
        n_v4c = sum(1 for d in dets if d["v4c_id"] is not None)
        print(f"  [{idx}/{len(frame_files)}] {img_path.name}: 检出={len(dets)} "
              f"OCR成功={n_ocr} v4c成功={n_v4c}")
        all_results.append({"name": img_path.name, "dets": dets})

    print("[4/5] 汇总统计...")
    total_det = sum(len(r["dets"]) for r in all_results)
    ocr_ok = [d for r in all_results for d in r["dets"] if d["cell_id"] is not None]
    v4c_ok = [d for r in all_results for d in r["dets"] if d["v4c_id"] is not None]
    both = [d for d in ocr_ok if d["v4c_id"] is not None]
    agree = [d for d in both if d["cell_id"] == d["v4c_id"]]
    ocr_only = [d for d in ocr_ok if d["v4c_id"] is None]
    v4c_only = [d for d in v4c_ok if d["cell_id"] is None]

    # 按文字高度分层（教员边界：≥18px 可行，<12px 不可行）
    def layer(h: float | None) -> str:
        if h is None:
            return "no_text"
        if h < 12:
            return "h<12px"
        if h < 18:
            return "12-18px"
        return "h>=18px"

    stats = {
        "总检出": total_det,
        "OCR成功": len(ocr_ok),
        "OCR成功率": round(len(ocr_ok) / total_det * 100, 1),
        "v4c成功": len(v4c_ok),
        "双方都成功": len(both),
        "双方一致": len(agree),
        "一致率(双方成功中)": round(len(agree) / len(both) * 100, 1) if both else None,
        "OCR独有(v4c失败)": len(ocr_only),
        "v4c独有(OCR失败)": len(v4c_only),
        "分层": {},
    }
    for lyr in ["h<12px", "12-18px", "h>=18px", "no_text"]:
        sub = [d for r in all_results for d in r["dets"] if layer(d["text_h"]) == lyr]
        if not sub:
            continue
        s_ok = [d for d in sub if d["cell_id"] is not None]
        s_both = [d for d in s_ok if d["v4c_id"] is not None]
        s_agree = [d for d in s_both if d["cell_id"] == d["v4c_id"]]
        stats["分层"][lyr] = {
            "检出": len(sub),
            "OCR成功": len(s_ok),
            "OCR成功率": round(len(s_ok) / len(sub) * 100, 1),
            "一致率(双方成功中)": round(len(s_agree) / len(s_both) * 100, 1) if s_both else None,
        }
    # mask 路径分布
    for p in ["hsv", "luma"]:
        sub = [d for r in all_results for d in r["dets"] if d["path"] == p]
        s_ok = [d for d in sub if d["cell_id"] is not None]
        stats[f"路径_{p}"] = {"检出": len(sub), "OCR成功": len(s_ok)}

    print("[5/5] 写出结果...")
    out_json = OUT_DIR / "ocr_vs_v4c_v2.json"
    out_json.write_text(json.dumps(
        {"stats": stats, "frames": all_results}, ensure_ascii=False, indent=2),
        encoding="utf-8")
    print(json.dumps(stats, ensure_ascii=False, indent=2))
    print(f"完成：{out_json}")

    print("\n--- 分歧清单（OCR vs v4c 都识别但结果不同）---")
    n_disagree = 0
    for r in all_results:
        for d in r["dets"]:
            if d["cell_id"] and d["v4c_id"] and d["cell_id"] != d["v4c_id"]:
                n_disagree += 1
                if n_disagree <= 20:
                    print(f"  {r['name']}: OCR={d['cell_id']}({d['conf']:.2f}) "
                          f"v4c={d['v4c_id']}({d['v4c_conf']:.2f}) text_h={d['text_h']} path={d['path']}")
    print(f"分歧总数：{n_disagree}")


if __name__ == "__main__":
    main()
