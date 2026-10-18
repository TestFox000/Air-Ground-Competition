# -*- coding: utf-8 -*-
"""场地二维矩阵渲染图：把识别结果画成 11×14 彩色格子图（中文标注）。

格号来源：格号识别器真值锚点 + 最近邻归属（Voronoi），
不依赖 PnP/poly 世界坐标投影，避免投影系统偏移导致格号错位。

输出：原图（带检测框，中文类别）+ 矩阵图（中文类别+格号）并排拼接。

用法:
  python tools/render_field_matrix.py --image capture/aerial_field/images/frame_00124.jpg
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tools" / "ml"))
sys.path.insert(0, str(REPO / "reference" / "onboard" / "dzuav" / "src"))
sys.path.insert(0, str(REPO / "src" / "ag_integrate"))

from sim_match import SimChain, NAME2TYPE, ObjectType  # noqa: E402
from eval_cell_frames import to_cid  # noqa: E402
from ag_integrate.field_frame import COLS, ROWS  # noqa: E402

# ---- 显示折叠：D13 目标 -> 渲染类别（kind）----
# 2026-10-16 起模型是 hbb_d13_best.pt（13 类），ObjectType.MINE/ARROW 两个
# 枚举成员已删除——本模块此前以它们为 COLORS/CN_NAMES 的键，import 即
# AttributeError，工具已死过一次。现按 monitor_link.py 已建好的范式重建：
# _KIND_BY_TYPE 映射 + PAPER 按落格判定（桥洞列=通行箭头，其余=雷，
# 与 world_model.py:120-127 同口径；列字母由 --bridge-col 给，默认 K——
# 2026-10-15 现场核实，契约文档里的 L 已过时）。
_KIND_BY_TYPE = {
    ObjectType.OBSTACLE: "obstacle",
    ObjectType.TARGET_BLUE: "blue",
    ObjectType.TARGET_RED: "red",
    ObjectType.CAR: "car",
    ObjectType.BUCKET_RED: "bucket_r",
    ObjectType.BUCKET_GREEN: "bucket_g",
    ObjectType.BUCKET_CYAN: "bucket_c",
    ObjectType.CAR_START: "car_start",
    ObjectType.CAR_END: "car_end",
    ObjectType.UAV_START: "uav_start",
    ObjectType.UAV_END: "uav_end",
    # PAPER 不在此表：落格分派（桥洞列=arrow/其余=mine），见 _kind()
    # CELL_LABEL 不进态势矩阵（格号只用于锚点定位）
}


def _kind(t: ObjectType, cell_name: str, bridge_col: str = "K") -> str:
    """目标类别 -> 渲染 kind。PAPER 按落格分雷/箭头，未知类别落到 'unknown'。"""
    if t is ObjectType.PAPER:
        return "arrow" if cell_name.startswith(bridge_col.upper()) else "mine"
    return _KIND_BY_TYPE.get(t, "unknown")


# 类别配色 (RGB)
COLORS = {
    "mine": (230, 0, 0),           # 红 爆炸物（非桥洞列 PAPER）
    "obstacle": (255, 140, 0),     # 橙 障碍
    "blue": (0, 120, 230),         # 蓝 敌方
    "red": (230, 230, 0),          # 黄 友方
    "arrow": (0, 210, 0),          # 绿 桥洞（桥洞列 PAPER）
    "car": (220, 0, 220),          # 紫 车
    "bucket_r": (200, 60, 60),     # 暗红 红桶
    "bucket_g": (60, 180, 60),     # 绿 绿桶
    "bucket_c": (60, 180, 200),    # 青 青桶
    "car_start": (90, 90, 220),    # 蓝紫 智能车起点
    "car_end": (90, 90, 220),      # 蓝紫 智能车终点
    "uav_start": (150, 90, 200),   # 紫 无人机起点
    "uav_end": (150, 90, 200),     # 紫 无人机终点
    "unknown": (128, 128, 128),    # 灰 兜底（枚举新增类别而本工具没跟上）
}
CN_NAMES = {
    "mine": "爆炸物",
    "obstacle": "障碍",
    "blue": "蓝靶",
    "red": "红靶",
    "arrow": "桥洞",
    "car": "车",
    "bucket_r": "红桶",
    "bucket_g": "绿桶",
    "bucket_c": "青桶",
    "car_start": "车起点",
    "car_end": "车终点",
    "uav_start": "机起点",
    "uav_end": "机终点",
    "unknown": "未知",
}
LETTERS = "ABCDEFGHIJKLMN"

# 中文字体（Windows 微软雅黑）
_FONT_PATHS = [
    r"C:\Windows\Fonts\msyh.ttc",
    r"C:\Windows\Fonts\simhei.ttf",
    r"C:\Windows\Fonts\simsun.ttc",
]


def _load_font(size):
    for fp in _FONT_PATHS:
        if Path(fp).exists():
            try:
                return ImageFont.truetype(fp, size)
            except Exception:
                continue
    return ImageFont.load_default()


def _PRIORITY(kind):
    # 同格多目标取高优先级（键为 _kind 产出的渲染类别）
    return {"mine": 0, "obstacle": 1, "blue": 2, "red": 3,
            "arrow": 4, "car": 5}.get(kind, 99)


def fit_lattice(anchors, min_inliers=6):
    """逆向反推：由已识别格号锚点拟合全场格网（RANSAC 单应）。

    原理：场地平面→图像是透视变换（单应），锚点格号(col,row)与
    像素(cx,cy)一一对应。用足够多的真锚点拟合单应矩阵后：
      - 内点 = 与格网一致的真锚点
      - 外点 = 误读/误检的假锚点（如重复识别的 H5、错位的 B7），自动剔除
      - 任何像素都能反推格号——包括标签未识别成功的格子
    返回 (cell_of_pixel_fn, n_inlier, n_outlier, rejected)，
    拟合失败返回 (None, 0, 0, [])。
    """
    if len(anchors) < 8:
        return None, 0, 0, []
    src = np.array([[a[0], a[1]] for a in anchors], dtype=np.float32)
    dst = np.array([[a[2], a[3]] for a in anchors], dtype=np.float32)
    # 自适应阈值：最近邻锚点间距中值 × 0.35（透视下格距随位置变化）
    d2 = ((dst[:, None, 0] - dst[None, :, 0]) ** 2
          + (dst[:, None, 1] - dst[None, :, 1]) ** 2)
    np.fill_diagonal(d2, np.inf)
    thr = max(15.0, 0.35 * float(np.median(np.sqrt(d2.min(axis=1)))))
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, thr)
    if H is None or mask is None:
        return None, 0, 0, []
    keep = mask.ravel().astype(bool)
    inl = int(keep.sum())
    in_cols = {anchors[i][0] for i in range(len(anchors)) if keep[i]}
    in_rows = {anchors[i][1] for i in range(len(anchors)) if keep[i]}
    # 覆盖检查：内点须横跨 >=3 列、>=2 行，避免退化为共线拟合
    if inl < min_inliers or len(in_cols) < 3 or len(in_rows) < 2:
        return None, 0, 0, []
    Hinv = np.linalg.inv(H)
    rejected = [anchors[i] for i in range(len(anchors)) if not keep[i]]

    def cell_of_pixel(px, py):
        """像素 → 反推格号 (col, row)；画面外/场外返回 None。"""
        v = Hinv @ np.array([px, py, 1.0])
        fc, fr = v[0] / v[2], v[1] / v[2]
        col, row = int(round(fc)), int(round(fr))
        if not (0 <= col < COLS and 0 <= row < ROWS):
            return None
        # 反投影像素残差sanity：格中心映射回来距离过大视为外推失真
        c = H @ np.array([col, row, 1.0])
        cx, cy = c[0] / c[2], c[1] / c[2]
        if (cx - px) ** 2 + (cy - py) ** 2 > (3.0 * thr) ** 2:
            return None
        return col, row

    return cell_of_pixel, inl, len(anchors) - inl, rejected


def detect_and_localize(image_path, yolo_pt, cell_pt,
                        conf=0.45, anchor_conf=0.80, bridge_col: str = "K"):
    """跑识别 + 格号锚点定位，返回目标列表、带框原图、定位元信息。

    bridge_col: 桥洞所在列字母，默认 K（2026-10-15 现场核实，与 world_model/
    mission_router 的桥列口径一致；若实际桥洞在其他列用 --bridge-col 指定）。
    """
    bcol = LETTERS.index(bridge_col.upper())
    frame = cv2.imdecode(np.fromfile(str(image_path), dtype=np.uint8),
                         cv2.IMREAD_COLOR)
    args = SimpleNamespace(conf=conf, anchor_conf=anchor_conf,
                           no_lattice=False, port=50000, device="auto")
    chain = SimChain(yolo_pt, cell_pt, args)

    K = chain._scaled_k((frame.shape[1], frame.shape[0]))
    img = cv2.undistort(frame, K, chain.dist, None, K)
    pred = chain.yolo.predict(img, conf=0.15, iou=0.45, imgsz=1280,
                              verbose=False, device=chain.device)[0]
    boxes, clss, confs = [], [], []
    for b, c, k in zip(pred.boxes.xyxy.cpu().numpy(),
                       pred.boxes.cls.cpu().numpy().astype(int),
                       pred.boxes.conf.cpu().numpy()):
        name = chain.yolo.names[c]
        if name not in NAME2TYPE:
            continue
        boxes.append(b)
        clss.append(NAME2TYPE[name])
        confs.append(float(k))

    # ---- 桥洞：场地约束先验（桥洞列 1~9 行固定为桥洞）----
    # 7cls 模型对 ARROW 类检测失效，按场地约束把桥洞列直接标记为桥洞。
    # 列由 bridge_col 参数决定（默认 L，见契约；行范围 1-9 是历史经验值，
    # 现场拓扑待核实——若桥洞列贯通行数不同需同步调整）。
    arrow_cells = set((bcol, r) for r in range(9))  # 桥洞列 row 0-8（即 X1-X9）

    # ---- 格号识别 ----
    cell_idx = [i for i, t in enumerate(clss) if t == ObjectType.CELL_LABEL]
    crops, centers = [], []
    for i in cell_idx:
        x1, y1, x2, y2 = boxes[i]
        ccx, ccy = (x1 + x2) / 2, (y1 + y2) / 2
        side = max(x2 - x1, y2 - y1) * 1.3
        sx1 = max(0, int(ccx - side / 2))
        sy1 = max(0, int(ccy - side / 2))
        sx2 = min(img.shape[1], int(ccx + side / 2))
        sy2 = min(img.shape[0], int(ccy + side / 2))
        crops.append(Image.fromarray(cv2.cvtColor(
            img[sy1:sy2, sx1:sx2], cv2.COLOR_BGR2RGB)))
        centers.append((ccx, ccy))

    direct = {}
    for j, pr in enumerate(chain.rec.predict(crops) if crops else []):
        if pr is not None and pr.joint_confidence >= conf:
            direct[cell_idx[j]] = (pr.cell_id, pr.joint_confidence)

    # ---- 格号锚点：识别器真值格号 + 像素坐标 ----
    anchors = []
    for i, (cid, cf) in direct.items():
        col = ord(cid[0]) - 65
        row = int(cid[1:]) - 1
        cx, cy = centers[cell_idx.index(i)]
        anchors.append((col, row, cx, cy))

    anchor_arr = np.array([[a[2], a[3], a[0], a[1]] for a in anchors])

    def nn_cell_of_pixel(px, py):
        """像素坐标 -> (col, row)。最近邻格号标签（Voronoi 兜底）。"""
        if not anchors:
            return None
        d = np.sqrt((anchor_arr[:, 0] - px) ** 2 +
                    (anchor_arr[:, 1] - py) ** 2)
        j = int(np.argmin(d))
        return int(anchor_arr[j, 2]), int(anchor_arr[j, 3])

    # ---- 逆向反推：锚点拟合全场格网，覆盖标签未识别的格子 ----
    lattice_fn, n_inlier, n_outlier, rejected = fit_lattice(anchors)
    if lattice_fn is not None:
        def cell_of_pixel(px, py):
            return lattice_fn(px, py)
    else:
        def cell_of_pixel(px, py):
            return nn_cell_of_pixel(px, py)

    targets = []
    direct_cells = {(a[0], a[1]) for a in anchors}
    inferred_cells = set()  # 反推格：目标落入但无直接锚点的格
    kinds: dict[int, str] = {}   # 检测索引 -> 渲染 kind（标注原图复用）
    for i, (t, cf) in enumerate(zip(clss, confs)):
        if t == ObjectType.CELL_LABEL:
            continue
        x1, y1, x2, y2 = boxes[i]
        px, py = (x1 + x2) / 2, (y1 + y2) / 2
        cr = cell_of_pixel(px, py)
        if cr is None:
            continue
        col, row = cr
        # D13 落格判语义（与 world_model.py:120-127 同口径）：桥洞列的
        # paper = 通行箭头，其余格的 paper = 雷。老 7cls 时代"桥洞列 mine
        # 是误检"的过滤随之消失——桥洞列物理不埋雷，paper 落桥洞列本义
        # 就是箭头标志；paper 是 12cm 图卡（单格），按框中心落格即可。
        kind = _kind(t, to_cid(col, row), bridge_col)
        kinds[i] = kind
        if (col, row) not in direct_cells:
            inferred_cells.add((col, row))  # 格号反推所得
        targets.append({"kind": kind, "cell": to_cid(col, row),
                        "col": col, "row": row, "px": px, "py": py,
                        "conf": cf})

    # 加入场地先验桥洞（桥洞列 1-9 行）
    for (col, row) in arrow_cells:
        targets.append({"kind": "arrow", "cell": to_cid(col, row),
                        "col": col, "row": row, "px": 0, "py": 0, "conf": 1.0})

    # ---- 带检测框的原图（中文标注）----
    render = img.copy()
    for i, (t, cf) in enumerate(zip(clss, confs)):
        x1, y1, x2, y2 = map(int, boxes[i])
        # CELL_LABEL 未入 targets：取落格重算；paper 颜色/名称按落格分雷/箭头
        kind = kinds.get(i)
        if kind is None:
            cr = cell_of_pixel((x1 + x2) / 2, (y1 + y2) / 2)
            kind = (_kind(t, to_cid(*cr), bridge_col)
                    if t is ObjectType.PAPER and cr is not None
                    else _KIND_BY_TYPE.get(t, "unknown"))
        color = COLORS.get(kind, COLORS["unknown"])
        bgr = (color[2], color[1], color[0])
        cv2.rectangle(render, (x1, y1), (x2, y2), bgr, 2)
        if t != ObjectType.CELL_LABEL:
            label = f"{CN_NAMES.get(kind, CN_NAMES['unknown'])} {cf:.2f}"
            # 用 PIL 画中文标签
            pil = Image.fromarray(cv2.cvtColor(render, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil)
            font = _load_font(18)
            tw = draw.textlength(label, font=font)
            draw.rectangle([x1, y1 - 24, x1 + tw + 8, y1], fill=bgr)
            draw.text((x1 + 4, y1 - 22), label, font=font, fill=(255, 255, 255))
            render = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)

    meta = {"n_cell": len(direct), "n_anchor": len(anchors),
            "method": "lattice" if lattice_fn is not None
                      else ("nn" if anchors else "none"),
            "n_inlier": n_inlier, "n_outlier": n_outlier,
            "rejected": [to_cid(a[0], a[1]) for a in rejected],
            "inferred": sorted(to_cid(c, r) for c, r in inferred_cells),
            "anchors": anchors}  # [(col, row, cx, cy), ...]
    return targets, render, meta


def render_field_matrix(targets, title="", cell_px=80, special=None,
                        inferred=None):
    """把目标列表渲染成 11×14 彩色矩阵图（中文类别+格号）。

    special: {cell_id: 中文标签}，在格子上叠加特殊标记（如起点/终点）。
    inferred: 反推格 cell_id 集合（目标落入但格号标签未直接识别，
              由格网反推得出），渲染为虚线边框。
    布局：[图例行] [11×14 矩阵+行列标] [title 行]
    """
    legend_h = 36
    title_h = 28
    axis_w = 28
    grid_w = COLS * cell_px
    grid_h = ROWS * cell_px
    w = axis_w + grid_w
    h = legend_h + grid_h + axis_w + title_h
    pil = Image.new("RGB", (w, h), (255, 255, 255))
    draw = ImageDraw.Draw(pil)

    # 场地规则：一格至多一个目标。同格多个检测视为噪声，
    # 取置信度最高的一个作为该格唯一目标。
    cell_targets = {}
    for tg in targets:
        key = (tg["col"], tg["row"])
        if key not in cell_targets or tg["conf"] > cell_targets[key]["conf"]:
            cell_targets[key] = tg

    font_main = _load_font(int(cell_px * 0.40))
    font_cell = _load_font(int(cell_px * 0.22))
    font_axis = _load_font(20)
    font_leg = _load_font(18)

    gx0, gy0 = axis_w, legend_h  # 网格左上角

    # 画格子（一格一目标，cell_targets 值为单目标 dict）
    inferred = inferred or set()
    for (col, row), main in cell_targets.items():
        color = COLORS.get(main["kind"], (200, 200, 200))
        x0 = gx0 + col * cell_px
        y0 = gy0 + row * cell_px
        draw.rectangle([x0, y0, x0 + cell_px, y0 + cell_px], fill=color)
        if main["cell"] in inferred:
            # 反推格：虚线边框（格号由周边锚点反推，标签未直接识别）
            dash, gap = 8, 5
            for edge in ("top", "bottom", "left", "right"):
                if edge == "top":
                    xs, ys, xe, ye = x0, y0, x0 + cell_px, y0
                elif edge == "bottom":
                    xs, ys, xe, ye = x0, y0 + cell_px, x0 + cell_px, y0 + cell_px
                elif edge == "left":
                    xs, ys, xe, ye = x0, y0, x0, y0 + cell_px
                else:
                    xs, ys, xe, ye = x0 + cell_px, y0, x0 + cell_px, y0 + cell_px
                seg = dash + gap
                n = max(1, int(abs(xe - xs) + abs(ye - ys)) // seg)
                for k in range(n):
                    t0 = k * seg / n
                    t1 = (k * seg + dash) / n
                    p1 = (xs + (xe - xs) * t0, ys + (ye - ys) * t0)
                    p2 = (xs + (xe - xs) * t1, ys + (ye - ys) * t1)
                    draw.line([p1, p2], fill=(20, 20, 20), width=3)
        cn = CN_NAMES[main["kind"]]
        bbox = draw.textbbox((0, 0), cn, font=font_main)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        draw.text((x0 + (cell_px - tw) / 2,
                   y0 + (cell_px - th) / 2 - 2),
                  cn, font=font_main, fill=(255, 255, 255))
        draw.text((x0 + 4, y0 + cell_px - int(cell_px * 0.28)),
                  main["cell"], font=font_cell, fill=(255, 255, 255))

    # 特殊点标注（起点/终点），画在格子顶部
    font_spec = _load_font(int(cell_px * 0.24))
    if special:
        for cid, label in special.items():
            col = ord(cid[0]) - 65
            row = int(cid[1:]) - 1
            x0 = gx0 + col * cell_px
            y0 = gy0 + row * cell_px
            # 深蓝色半透明条
            draw.rectangle([x0, y0, x0 + cell_px, y0 + 22],
                           fill=(0, 0, 139))
            bbox = draw.textbbox((0, 0), label, font=font_spec)
            tw = bbox[2] - bbox[0]
            draw.text((x0 + (cell_px - tw) / 2, y0 + 2),
                      label, font=font_spec, fill=(255, 255, 255))

    # 网格线
    for c in range(COLS + 1):
        x = gx0 + c * cell_px
        draw.line([(x, gy0), (x, gy0 + grid_h)], fill=(180, 180, 180))
    for r in range(ROWS + 1):
        y = gy0 + r * cell_px
        draw.line([(gx0, y), (gx0 + grid_w, y)], fill=(180, 180, 180))

    # 列标 A..N（底部）
    for c in range(COLS):
        ch = LETTERS[c]
        bbox = draw.textbbox((0, 0), ch, font=font_axis)
        tw = bbox[2] - bbox[0]
        x = gx0 + c * cell_px + (cell_px - tw) / 2
        draw.text((x, gy0 + grid_h + 2), ch, font=font_axis, fill=(60, 60, 60))
    # 行标 1..11（左侧）
    for r in range(ROWS):
        num = str(r + 1)
        bbox = draw.textbbox((0, 0), num, font=font_axis)
        tw = bbox[2] - bbox[0]
        x = (axis_w - tw) / 2
        y = gy0 + r * cell_px + (cell_px - 20) / 2
        draw.text((x, y), num, font=font_axis, fill=(60, 60, 60))

    # 图例（顶部）
    legend = [("mine", "爆炸物"), ("obstacle", "障碍"),
              ("blue", "蓝靶(敌)"),
              ("red", "红靶(友)"),
              ("arrow", "桥洞"), ("car", "车")]
    for i, (k, txt) in enumerate(legend):
        cx = 10 + i * 135
        draw.rectangle([cx, 6, cx + 22, 28], fill=COLORS[k])
        draw.text((cx + 28, 6), txt, font=font_leg, fill=(40, 40, 40))
    if inferred:
        lx = 10 + len(legend) * 135
        draw.rectangle([lx, 6, lx + 22, 28], outline=(20, 20, 20), width=3)
        draw.text((lx + 28, 6), "虚线=反推格", font=font_leg, fill=(40, 40, 40))

    # title（底部单独一行）
    if title:
        draw.text((10, legend_h + grid_h + axis_w + 2),
                  title, font=font_axis, fill=(30, 30, 30))

    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


def _build_gt_targets(bridge_col: str = "K"):
    """从用户提供的 GT 构建 targets 列表，用于对比渲染。

    桥洞列默认 K（2026-10-15 现场核实，与 world_model/mission_router 同口径；
    本 GT 手工转录自桥洞在 K 列的裁判图）。
    """
    def expand(spec):
        cells = set()
        for part in spec.replace("，", ",").split(","):
            part = part.strip()
            if not part:
                continue
            if "-" in part:
                a, b = part.split("-")
                col = a[0]
                r1, r2 = int(a[1:]), int(b[1:])
                for r in range(r1, r2 + 1):
                    cells.add(f"{col}{r}")
            else:
                cells.add(part)
        return cells

    mines = (expand("A2-A4") | expand("B5-B6") | expand("C1-C7") |
             expand("D1-D8") | expand("E1-E9") | expand("F2-F9") |
             expand("G1-G9") | expand("H3-H8") |
             expand("I1,I2") | expand("I5-I9") |
             expand("J1-J9") | expand("M3-M9") | expand("N1-N4,N6-N7"))
    obstacles = expand("B2,H2,I4,H9,F1")
    arrows = expand(f"{bridge_col}1-{bridge_col}9")  # 桥洞列 1-9 行（行范围为历史经验值）

    targets = []
    for cid in mines:
        col, row = ord(cid[0]) - 65, int(cid[1:]) - 1
        targets.append({"kind": "mine", "cell": cid,
                        "col": col, "row": row, "px": 0, "py": 0, "conf": 1.0})
    for cid in obstacles:
        col, row = ord(cid[0]) - 65, int(cid[1:]) - 1
        targets.append({"kind": "obstacle", "cell": cid,
                        "col": col, "row": row, "px": 0, "py": 0, "conf": 1.0})
    for cid in arrows:
        col, row = ord(cid[0]) - 65, int(cid[1:]) - 1
        targets.append({"kind": "arrow", "cell": cid,
                        "col": col, "row": row, "px": 0, "py": 0, "conf": 1.0})
    return targets


GT_SPECIAL = {"M2": "无人机终点", "N5": "智能车终点",
              "N8": "无人机起点", "N10": "智能车起点"}


def main():
    ap = argparse.ArgumentParser(description="场地二维矩阵渲染图（中文+拼图）")
    ap.add_argument("--image", type=Path, required=True)
    ap.add_argument("--out", type=Path,
                    default=REPO / "runs/matrix_test/field_matrix.png")
    ap.add_argument("--title", type=str, default="")
    ap.add_argument("--cell-px", type=int, default=80, help="矩阵每格像素")
    ap.add_argument("--bridge-col", default="K",
                    help="桥洞所在列字母（默认 K，2026-10-15 现场核实）")
    ap.add_argument("--show-gt", action="store_true",
                    help="同时输出 GT 真值矩阵对比")
    args = ap.parse_args()

    yolo = REPO / "runs/detect/runs/detect/runs/detect/aerial_field_7cls/weights/best.pt"
    cell = REPO / "runs/cell_classifier_v4c/best.pt"
    targets, render_img, meta = detect_and_localize(args.image, yolo, cell,
                                                    bridge_col=args.bridge_col)
    if args.title:
        title = args.title
    elif meta["method"] == "lattice":
        title = (f"{args.image.name}  格网反推: 锚点{meta['n_inlier']}/"
                 f"{meta['n_anchor']}(剔{meta['n_outlier']}假) "
                 f"反推格{len(meta['inferred'])} targets={len(targets)}")
    else:
        title = (f"{args.image.name}  cells={meta['n_cell']} "
                 f"anchors={meta['n_anchor']} targets={len(targets)}")

    det_matrix = render_field_matrix(targets, f"[检测] {title}",
                                     cell_px=args.cell_px, special=GT_SPECIAL,
                                     inferred=set(meta["inferred"]))

    if args.show_gt:
        gt_targets = _build_gt_targets(args.bridge_col)
        gt_matrix = render_field_matrix(gt_targets, "[GT 真值]",
                                        cell_px=args.cell_px,
                                        special=GT_SPECIAL)
        # 检测矩阵 | GT 矩阵 水平拼接
        matrix = np.hstack([det_matrix, gt_matrix])
    else:
        matrix = det_matrix

    # ---- 原图(上) + 矩阵图(下) 垂直拼接，宽度统一为矩阵图宽度 ----
    mh, mw = matrix.shape[:2]
    rh, rw = render_img.shape[:2]
    scale = mw / rw
    render_scaled = cv2.resize(render_img, (mw, int(rh * scale)))
    combo = np.vstack([render_scaled, matrix])

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(cv2.imencode(".png", combo)[1])
    print(f"已输出: {args.out}  ({combo.shape[1]}x{combo.shape[0]})")
    print(f"检测目标数: {len(targets)}, 格号锚点: {meta['n_anchor']}, "
          f"定位方式: {meta['method']}")
    if meta["method"] == "lattice":
        print(f"格网反推: 内点{meta['n_inlier']} 剔除假锚点{meta['n_outlier']}"
              f"{meta['rejected']}")
        print(f"反推格（目标落入、标签未识别）: {meta['inferred']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
