# -*- coding: utf-8 -*-
"""赛前视频场地侦察：D13 + v4d 全机械化识别 → 场地态势 → 路线规划。

比赛日工作流（无人值守，一条命令）：
    python video_recon.py 场地巡拍.mp4
    python video_recon.py 场地巡拍.mp4 --out 输出目录 --stride 15

链路（每一环都是代码规则，不依赖任何人工/agent 介入）：
  1. hbb_d13.onnx（OpenCV DNN，无需 onnxruntime/torch）逐帧检测 13 类目标
  2. cell_label 框 → 正方形裁剪(max(w,h)*1.3，与 v4d 训练分布一致)
     → cell_classifier_v4d.onnx 双头(letter14/row11) + 4 向旋转 TTA 读格号
  3. 穷举一致性仿射（C(n,3) 子集 + 共识投票）：3-7 个标签的小样本下
     迭代剔除无法从 1 个误读自救，穷举是唯一可靠解；拟合失败帧在帧距内
     经读数验收门（validate_affine_carry）复用最近成功仿射，抬网格帧覆盖率
  4. 落格投票四重门控：
     - 锚点门控：物体必须落在帧内标签 1.5 格内（锚点区外外推不可信）
     - 残差门控：连续坐标吸附格心残差 <= 0.35 格
     - 每帧每类每格至多 1 票（防单帧重复计票）
     - 票数 >= 2 且 paper 均值置信度 >= 0.35（人体/阴影误检过滤）
  5. 幻影消除：弱格(<=3票)与强格(>=3x票)从未同帧共现、且票期被强格
     夹芯包裹 → 判定为同一物理物体的误归属（透视偏移/锚点误读拽偏），
     剔除。同帧共现 = 两个独立物体，保留。
  6. K 列 paper = 桥洞箭头(可通行)，其余列 paper = 雷
  7. 视频未覆盖要素（桥洞格/移动靶/起终点）用裁判系统默认值，
     来源在 route_plan.json 的 source 字段明确标注
  8. plan_route → 压缩(保护雷/桥洞/轨道/射击停靠格) → 车端命令 + 渲染

几何模型说明：共线标签使单应性在锚点区外病态外推（实测毒化落格），
纯仿射 + 锚点门控是覆盖与精度的实测最优平衡。

输出（--out 目录，默认 视频侦察_YYYYMMDD）：
    field_state.json   态势：雷/幻影日志/投票明细/覆盖度/分数校准
    frames_meta.json   每帧检测与网格拟合明细（全程可追溯）
    route_plan.json    路线 + 压缩点 + 车端命令 + 校验结论
    route_render.png   路线渲染图
    annotated/         标注帧抽樣（检测框 + 格号读数）
    label_crops/       格号裁剪图抽样（v4d 读数人工抽检用）
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime
from itertools import combinations
from pathlib import Path

import cv2
import numpy as np

try:                                    # 包内导入（python -m ag_integrate.video_recon）
    from .field_frame import CellRef, COLS, LETTERS, ROWS
    from .world_model import TargetRef, WorldSnapshot
    from .mission_router import compress_route, plan_route
    from .car_adapter import adapt
except ImportError:                     # 直接以文件路径运行（比赛现场快捷用法）
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from ag_integrate.field_frame import CellRef, COLS, LETTERS, ROWS
    from ag_integrate.world_model import TargetRef, WorldSnapshot
    from ag_integrate.mission_router import compress_route, plan_route
    from ag_integrate.car_adapter import adapt

# ---------------------------------------------------------------- 常量
NAMES = ["paper", "cell_label", "car", "target_red", "target_blue", "obstacle",
         "bucket_red", "bucket_green", "bucket_cyan",
         "car_start", "car_end", "uav_start", "uav_end"]
CONF = {  # PC 侦察工作点：paper=0.22 / cell_label=0.35 / 靶标·车·起止=0.25 / 障碍桶=0.40
    # （与机载 C++ kDeployConf 非同源：那边 paper=0.25、cell_label=0.50，改一边务必核对另一边）
    "paper": 0.22, "cell_label": 0.35, "car": 0.25,
    "target_red": 0.25, "target_blue": 0.25, "obstacle": 0.40,
    "bucket_red": 0.40, "bucket_green": 0.40, "bucket_cyan": 0.40,
    "car_start": 0.25, "car_end": 0.25, "uav_start": 0.25, "uav_end": 0.25,
}
OBJ_CLASSES = [n for n in NAMES if n != "cell_label"]
NMS_IOU = 0.45
IN_SIZE = 1280
V4D_IN = 96
V4D_MIN_CONF = 0.65          # letter/row 双头 softmax 下限（TTA 最优旋转）
V4D_PAD = 1.3                # 与 autolabel_v4d 训练裁剪一致：正方形 side = max(w,h)*1.3
VOTE_MIN = 2                 # 落格最少票数
MAX_VOTE_RESIDUAL = 0.35     # 物体连续坐标吸附到格心的最大残差（格单位）
ANCHOR_GATE = 1.5            # 物体距帧内标签锚点的最大切比雪夫距离（格单位）
GRID_MAE_MAX = 0.45          # 帧内网格拟合接受门限
FIT_INLIER = 0.45            # 一致性拟合内点半径（格单位）
CARRY_MAX_STEPS = 3          # 拟合失败帧时间外推旧仿射的最大帧距（已处理帧计）
CARRY_INLIER = 0.45          # 外推验收门：读数投影残差半径（格单位，同 FIT_INLIER 口径）
PAPER_MEAN_CONF_MIN = 0.35   # paper 格均值置信度下限（人体/阴影误检过滤）
TARGET_MIN_VOTES = 2         # 靶标最少票数（单票多为人体误检）
TARGET_MEAN_CONF_MIN = 0.40

BRIDGE_COL = "K"
BRIDGE_COL_IDX = LETTERS.index(BRIDGE_COL)
# 裁判系统默认（视频未覆盖要素回退值，来源在输出中标注）
PASSABLE_DFT = (f"{BRIDGE_COL}4", f"{BRIDGE_COL}9")
# 2026-10-17 官方裁判地图：轨道 D7-E7（曾错为 D8-E8）、车 N10->N5（曾错为 N11->N6）。
# 这些回退值会和 match_params.yaml 一起决定彩排路线；两处分裂时，
# 视频没拍到的要素就会按过期格号规划（本仓 _MID_BOUNDARY 事故同款）。
# 一致性由 tests/test_field_config_consistency.py 锁定，改一处必须改两处。
TRACK_DFT = ("D7", "E7")
BLUE_FACE_DFT = "S"
START_DFT, GOAL_DFT = "N10", "N5"

# 模型默认搜索路径（相对运行目录逐级向上找）
D13_SEARCH = ("dk_train_pack_20261016/models/hbb_d13.onnx",)
V4D_SEARCH = ("cloud_train_20261015/deploy_pkg/cell_classifier_v4d_20261015.onnx",)


def cell_name(col: int, row: int) -> str:
    return f"{LETTERS[col]}{row + 1}"


def parse_cell(name: str) -> tuple[int, int]:
    col = LETTERS.index(name[0].upper())
    row = int(name[1:]) - 1
    if not (0 <= col < COLS and 0 <= row < ROWS):
        raise ValueError(f"格号越界: {name!r}")
    return col, row


def imwrite_u(path: Path, img: np.ndarray) -> None:
    """cv2.imwrite 在 Windows 中文路径下静默失败，用 imencode 规避。"""
    ok, buf = cv2.imencode(path.suffix, img)
    if ok:
        path.write_bytes(buf.tobytes())


# ---------------------------------------------------------------- 纯函数（可测）
def fit_consensus_affine(px: np.ndarray, grid: np.ndarray
                         ) -> tuple[np.ndarray | None, np.ndarray, float]:
    """穷举一致性仿射：C(n,3) 基础子集 + 共识投票，抗 v4d 误读毒化。
    px(N,2) 标签像素中心 -> grid(N,2) 整数格坐标，返回 (A(3,2), mask, mae)。"""
    n = len(px)
    if n < 3:
        return None, np.zeros(n, bool), 9.9
    x = np.hstack([px, np.ones((n, 1))])
    best = None                    # (inlier_count, -mae, a, mask)
    for combo in combinations(range(n), 3):
        idx = list(combo)
        if np.linalg.matrix_rank(x[idx]) < 3:
            continue               # 共线三点无解
        try:
            a, *_ = np.linalg.lstsq(x[idx], grid[idx], rcond=None)
        except np.linalg.LinAlgError:
            continue
        r = np.linalg.norm(x @ a - grid, axis=1)
        inl = r < FIT_INLIER
        if inl.sum() < 3:
            continue
        try:                        # 共识内点精解
            a2, *_ = np.linalg.lstsq(x[inl], grid[inl], rcond=None)
        except np.linalg.LinAlgError:
            continue
        r2 = np.linalg.norm(x @ a2 - grid, axis=1)
        inl2 = r2 < FIT_INLIER
        if inl2.sum() < 3:
            continue
        cand = (int(inl2.sum()), -float(r2[inl2].mean()), a2, inl2)
        if best is None or (cand[0], cand[1]) > (best[0], best[1]):
            best = cand
    if best is None or best[0] < 3 or -best[1] > GRID_MAE_MAX:
        return None, np.zeros(n, bool), 9.9
    return best[2], best[3], -best[1]


def validate_affine_carry(aff: "np.ndarray | None", reads: list
                          ) -> tuple[np.ndarray, float] | None:
    """时间外推验收门：拟合失败的帧复用最近成功仿射前的独立安全检查。

    穷举拟合要 ≥3 读数且 C(n,3) 有干净子集；3 读数含 1 误读、或只有 2 个
    可读标签的帧会整帧拟合失败（= 整帧零投票，网格帧覆盖率 50~62% 的主因）。
    上一帧还成立的仿射，相机没大动就仍然成立——本函数用当前帧的可信读数
    独立验证这一点：≥2 个读数的 px→grid 投影残差全部 < CARRY_INLIER 且
    均值 ≤ GRID_MAE_MAX 才放行。两个独立读数同时碰巧吻合旧仿射的概率极低；
    相机一旦平移/旋转，残差立刻爆门拒绝（fail-safe：宁丢帧不投错票）。

    返回 (参与验收的读数下标数组, mae)；不达标返回 None。
    """
    if aff is None or len(reads) < 2:
        return None
    px = np.array([[(b[0] + b[2]) / 2, (b[1] + b[3]) / 2] for *_, b in reads])
    grid = np.array([[li, ri] for li, ri, *_ in reads], float)
    r = np.linalg.norm(np.hstack([px, np.ones((len(px), 1))]) @ aff - grid, axis=1)
    keep = np.where(r < CARRY_INLIER)[0]
    if len(keep) < 2:
        return None
    mae = float(r[keep].mean())
    if mae > GRID_MAE_MAX:
        return None
    return keep, mae


def circular_mode(vals: list[float], bins: int = 20) -> float:
    """1D 环形众数（分数部分聚类），返回众数中心。"""
    if not vals:
        return 0.5
    hist, edges = np.histogram(np.array(vals) % 1.0, bins=bins, range=(0, 1))
    ext = np.concatenate([hist[-1:], hist, hist[:1]])   # 环形平滑
    k = int(np.argmax(ext[1:-1]))
    return (edges[k] + edges[k + 1]) / 2


def anchor_within(qc: float, qr: float, anchors: list[tuple[int, int]],
                  dist: float = ANCHOR_GATE) -> bool:
    """锚点门控：物体连续坐标必须在帧内标签锚点切比雪夫距离 dist 内。"""
    return any(max(abs(qc - a0), abs(qr - a1)) <= dist for a0, a1 in anchors)


def snap_cell(qc: float, qr: float, fc: float, fr: float,
              max_resid: float = MAX_VOTE_RESIDUAL) -> tuple[int, int] | None:
    """连续格坐标吸附到格心；残差过大返回 None。"""
    col, row = int(round(qc - fc)), int(round(qr - fr))
    if max(abs(qc - fc - col), abs(qr - fr - row)) > max_resid:
        return None
    if not (0 <= col < COLS and 0 <= row < ROWS):
        return None
    return col, row


def eliminate_phantoms(cells: list[str], votes: Counter,
                       vframes: dict[str, set]
                       ) -> tuple[list[str], dict[str, str]]:
    """幻影消除：弱格(<=3票)若与某强格(>=6票且>=3x)从未同帧共现、且票期
    被强格夹芯包裹（强格在弱格前后都有票），判定为同一物理物体的误归属
    （透视偏移/v4d 误读锚点拽偏），剔除。同帧共现 = 两个独立物体，保留。"""
    phantoms: dict[str, str] = {}
    strong = [c for c in cells if votes.get(c, 0) >= 6]
    for c in cells:
        n = votes.get(c, 0)
        if n > 3:
            continue
        for s in strong:
            if votes.get(s, 0) < 3 * n:
                continue
            fs, fy = vframes.get(c, set()), vframes.get(s, set())
            if fs & fy:                   # 同帧共现：独立物体
                continue
            if fs and fy and min(fs) > min(fy) and max(fs) < max(fy):
                phantoms[c] = s
                break
    return [c for c in cells if c not in phantoms], phantoms


def confirmed_cells(cls_list: list[str], votes: dict[str, Counter],
                    vconf: dict[str, dict[str, float]],
                    vframes: dict[str, dict[str, set]],
                    min_votes: int = VOTE_MIN,
                    min_mean_conf: float = 0.0,
                    phantom_kill: bool = False) -> list[str]:
    """跨类合并投票 + 票数/均值置信度门控 + 可选幻影消除。"""
    merged: Counter = Counter()
    for cls in cls_list:
        merged.update(votes.get(cls, Counter()))
    cells = []
    for c, v in merged.items():
        if v < min_votes:
            continue
        if min_mean_conf:
            mconf = sum(vconf.get(k, {}).get(c, 0.0) for k in cls_list) / max(v, 1)
            if mconf < min_mean_conf:
                continue
        cells.append(c)
    if phantom_kill and cells:
        merged_frames: dict[str, set] = defaultdict(set)
        for cls in cls_list:
            for c, fs in vframes.get(cls, {}).items():
                merged_frames[c] |= fs
        cells, _ph = eliminate_phantoms(cells, merged, merged_frames)
    return sorted(cells)


def split_mines_passable(paper_cells: list[str],
                         bridge_col_idx: int = BRIDGE_COL_IDX
                         ) -> tuple[list[str], list[str]]:
    """K 列 paper = 桥洞箭头(可通行)，其余列 paper = 雷。"""
    mines = [c for c in paper_cells if LETTERS.index(c[0]) != bridge_col_idx]
    passable = [c for c in paper_cells if LETTERS.index(c[0]) == bridge_col_idx]
    return mines, passable


def shoot_keep_cells(targets: list[TargetRef]) -> set[str]:
    """射击停靠格（蓝面侧轨道端点邻格）——压缩时必须保留。"""
    keep: set[str] = set()
    for t in targets:
        if t.track is None or not t.blue_face:
            continue
        a, b = t.track
        if a.row == b.row:                # 水平轨道：S=row+1 / N=row-1
            d = 1 if t.blue_face == "S" else -1
            keep |= {CellRef(a.col, a.row + d).name, CellRef(b.col, b.row + d).name}
        else:                             # 垂直轨道：E=col+1 / W=col-1
            d = 1 if t.blue_face == "E" else -1
            keep |= {CellRef(a.col + d, a.row).name, CellRef(b.col + d, b.row).name}
    return keep


# ---------------------------------------------------------------- 模型封装
def _iou(a: list[float], b: list[float]) -> float:
    """两 xyxy 框的 IoU。"""
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter <= 0:
        return 0.0
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


LABEL_OVERLAP_IOU = 0.30


def filter_label_overlap(dets: list[dict],
                         iou_th: float = LABEL_OVERLAP_IOU) -> list[dict]:
    """标签文字误检过滤（2026-10-17 v2 实拍实锤）：

    高分辨率（>=720p）近拍下，大字号印刷格号（"E9"/"D4"等）会被 paper 类
    误检为雷贴纸（f00700: "E9"两个字各检出 paper 0.58/0.88；f01300 同）。
    规则：paper 框与任一 cell_label 框重叠 IoU > iou_th → 判为标签文字，剔除。
    真实雷贴纸贴在格心、印刷格号在角部，物理上不重叠；K 列桥洞箭头在桥顶
    挡板上，也不与垫面格号重叠。"""
    label_boxes = [d["box"] for d in dets if d["cls"] == "cell_label"]
    if not label_boxes:
        return dets
    return [d for d in dets
            if d["cls"] != "paper"
            or max((_iou(d["box"], lb) for lb in label_boxes), default=0.0) <= iou_th]


class D13Detector:
    """hbb_d13.onnx（OpenCV DNN）：letterbox 1280 → [1,17,33600] 解码 + 逐类 NMS。"""

    def __init__(self, onnx_path: str | Path):
        self.net = cv2.dnn.readNetFromONNX(str(onnx_path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)

    def detect(self, img_bgr: np.ndarray) -> list[dict]:
        """返回 [{cls, conf, box=[x1,y1,x2,y2](原图坐标)}]，坐标已映射回原图。"""
        h, w = img_bgr.shape[:2]
        scale = IN_SIZE / max(h, w)
        nw, nh = round(w * scale), round(h * scale)
        resized = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((IN_SIZE, IN_SIZE, 3), 114, np.uint8)
        pad_x, pad_y = (IN_SIZE - nw) // 2, (IN_SIZE - nh) // 2
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
        self.net.setInput(cv2.dnn.blobFromImage(
            canvas, 1 / 255.0, (IN_SIZE, IN_SIZE), swapRB=True))
        out = self.net.forward()[0]              # (17, 33600)
        boxes, scores = out[:4, :].T, out[4:, :]  # cx,cy,w,h @1280 | (13,33600)
        dets: list[dict] = []
        for c, name in enumerate(NAMES):
            th = CONF[name]
            s = scores[c]
            m = s > th
            if not m.any():
                continue
            b, sc = boxes[m], s[m]
            xywh = np.stack([(b[:, 0] - b[:, 2] / 2 - pad_x) / scale,
                             (b[:, 1] - b[:, 3] / 2 - pad_y) / scale,
                             b[:, 2] / scale, b[:, 3] / scale], axis=1)
            keep = cv2.dnn.NMSBoxes(xywh.tolist(), sc.tolist(), th, NMS_IOU)
            for i in np.array(keep).flatten():
                x, y, w_, h_ = xywh[i]
                dets.append({"cls": name, "conf": float(sc[i]),
                             "box": [float(x), float(y), float(x + w_), float(y + h_)]})
        return filter_label_overlap(dets)


class V4dReader:
    """cell_classifier_v4d.onnx：正方形裁剪 + 4 向旋转 TTA 读格号。
    训练裁剪分布 = autolabel_v4d：side = max(w,h)*1.3 居中正方形。"""

    def __init__(self, onnx_path: str | Path):
        self.net = cv2.dnn.readNetFromONNX(str(onnx_path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.out_names = self.net.getUnconnectedOutLayersNames()

    @staticmethod
    def _softmax(x: np.ndarray) -> np.ndarray:
        e = np.exp(x - x.max())
        return e / e.sum()

    @staticmethod
    def crop_square(img: np.ndarray, box: list[float],
                    pad: float = V4D_PAD) -> np.ndarray | None:
        """检测框 → 居中正方形裁剪（与训练分布一致）。"""
        x1, y1, x2, y2 = box
        ccx, ccy = (x1 + x2) / 2, (y1 + y2) / 2
        side = max(x2 - x1, y2 - y1) * pad
        sx1 = max(0, int(ccx - side / 2)); sy1 = max(0, int(ccy - side / 2))
        sx2 = min(img.shape[1], int(ccx + side / 2))
        sy2 = min(img.shape[0], int(ccy + side / 2))
        crop = img[sy1:sy2, sx1:sx2]
        return crop if crop.size else None

    def read(self, crop_bgr: np.ndarray) -> tuple[int, int, float, float]:
        """crop -> (letter_idx 0..13, row_idx 0..10, letter_conf, row_conf)。
        旋转 TTA：手持视角标签可能旋转 90/180/270，取双头联合置信最优。"""
        best = None
        for k in range(4):
            c = np.rot90(crop_bgr, k) if k else crop_bgr
            self.net.setInput(cv2.dnn.blobFromImage(
                cv2.resize(c, (V4D_IN, V4D_IN)), 1 / 255.0, (V4D_IN, V4D_IN), swapRB=True))
            lg, rg = self.net.forward(self.out_names)
            lp, rp = self._softmax(lg[0]), self._softmax(rg[0])
            li, ri = int(lp.argmax()), int(rp.argmax())
            cand = (min(float(lp[li]), float(rp[ri])), li, ri, float(lp[li]), float(rp[ri]))
            if best is None or cand[0] > best[0]:
                best = cand
        return best[1], best[2], best[3], best[4]


# ---------------------------------------------------------------- 侦察主流程
def _find_model(search: tuple[str, ...], explicit: str | None) -> Path:
    if explicit:
        p = Path(explicit)
        if p.exists():
            return p
        raise FileNotFoundError(explicit)
    for base in [Path.cwd(), *Path.cwd().parents[:3]]:
        for rel in search:
            p = base / rel
            if p.exists():
                return p
    raise FileNotFoundError(f"未找到模型，尝试过 {search}，用 --d13/--v4d 显式指定")


def recon_video(video: str | Path, out_dir: str | Path,
                d13_onnx: str | Path | None = None,
                v4d_onnx: str | Path | None = None,
                stride: int = 15, max_frames: int = 0,
                keep_annots: int = 8, keep_crops: int = 40) -> dict:
    """视频 → 场地态势。返回 field_state dict（同时落盘 frames_meta.json）。"""
    video, out_dir = Path(video), Path(out_dir)
    (out_dir / "annotated").mkdir(parents=True, exist_ok=True)
    (out_dir / "label_crops").mkdir(parents=True, exist_ok=True)
    d13 = D13Detector(_find_model(D13_SEARCH, d13_onnx and str(d13_onnx)))
    v4d = V4dReader(_find_model(V4D_SEARCH, v4d_onnx and str(v4d_onnx)))

    cap = cv2.VideoCapture(str(video))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    idxs = list(range(0, total, stride))
    if max_frames:
        idxs = idxs[:max_frames]
    print(f"[recon] 视频总帧 {total}，采样 {len(idxs)} 帧 (stride={stride})")

    frames_meta: list[dict] = []
    cont_pos: dict[str, list[tuple[float, float, float]]] = defaultdict(list)
    crop_saved = 0
    t0 = time.time()
    last_aff: np.ndarray | None = None    # 最近成功仿射（时间外推缓存）
    last_step = -10 ** 9                  # 它对应的已处理帧号

    for fi, fidx in enumerate(idxs):
        cap.set(cv2.CAP_PROP_POS_FRAMES, fidx)
        ok, frame = cap.read()
        if not ok:
            continue
        dets = d13.detect(frame)
        labels = [d for d in dets if d["cls"] == "cell_label"]
        objs = [d for d in dets if d["cls"] != "cell_label"]

        # v4d 读格号（正方形裁剪 + TTA + 双头置信门控）
        reads = []
        for d in labels:
            crop = v4d.crop_square(frame, d["box"])
            if crop is None:
                continue
            li, ri, lc, rc = v4d.read(crop)
            if lc >= V4D_MIN_CONF and rc >= V4D_MIN_CONF:
                reads.append((li, ri, lc, rc, d["box"]))
                if crop_saved < keep_crops:
                    imwrite_u(out_dir / "label_crops" /
                              f"f{fidx:05d}_{cell_name(li, ri)}_L{lc:.2f}_R{rc:.2f}.jpg", crop)
                    crop_saved += 1

        # 帧内网格：穷举一致性仿射（抗 1-2 个误读毒化）；拟合失败（含仅
        # 2 读数的帧）在帧距内用最近成功仿射时间外推补位——validate_affine_carry
        # 的读数验收门把关，相机没大动才放行，直接抬网格帧覆盖率
        grid_ok, mae = False, 9.9
        grid_src = None
        px = grid = aff = inl = None
        if len(reads) >= 3:
            px = np.array([[(b[0] + b[2]) / 2, (b[1] + b[3]) / 2] for *_, b in reads])
            grid = np.array([[li, ri] for li, ri, *_ in reads], float)
            aff, inl, mae = fit_consensus_affine(px, grid)
            grid_ok = aff is not None
            if grid_ok:
                reads = [r for r, k in zip(reads, inl) if k]
        if grid_ok:
            grid_src = "fit"
            last_aff, last_step = aff, fi
        elif last_aff is not None and fi - last_step <= CARRY_MAX_STEPS:
            carried = validate_affine_carry(last_aff, reads)
            if carried is not None:
                keep, mae = carried
                aff, grid_ok, grid_src = last_aff, True, "carry"
                reads = [reads[i] for i in keep]

        # 物体连续格坐标（仿射: [x,y,1] @ A = [col,row]）
        frame_objs = []
        if grid_ok:
            for d in objs:
                x1, y1, x2, y2 = d["box"]
                p = np.array([(x1 + x2) / 2, (y1 + y2) / 2, 1.0]) @ aff
                qc, qr = float(p[0]), float(p[1])
                if -0.5 <= qc < COLS + 0.5 and -0.5 <= qr < ROWS + 0.5:
                    cont_pos[d["cls"]].append((qc, qr, d["conf"]))
                    frame_objs.append({"cls": d["cls"], "conf": d["conf"],
                                       "q": [round(qc, 3), round(qr, 3)],
                                       "box": [round(v, 1) for v in d["box"]]})

        frames_meta.append({"frame": fidx, "n_dets": len(dets), "n_labels_read": len(reads),
                            "grid_mae": round(mae, 3), "grid_ok": grid_ok,
                            "grid_src": grid_src,
                            "labels": [cell_name(li, ri) for li, ri, *_ in reads],
                            "objs": frame_objs})

        if fi % keep_annots == 0:      # 标注帧抽样
            vis = frame.copy()
            for li, ri, lc, rc, b in reads:
                cv2.rectangle(vis, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (255, 180, 0), 1)
                cv2.putText(vis, cell_name(li, ri), (int(b[0]), int(b[1]) - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 180, 0), 1)
            for d in objs:
                x1, y1, x2, y2 = map(int, d["box"])
                color = (0, 0, 255) if d["cls"] == "paper" else (0, 200, 255)
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 1)
                cv2.putText(vis, f"{d['cls'][:6]}{d['conf']:.2f}", (x1, y2 + 11),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
            imwrite_u(out_dir / "annotated" / f"f{fidx:05d}.jpg", vis)

        if (fi + 1) % 20 == 0:
            print(f"[recon] {fi + 1}/{len(idxs)} 帧  网格OK "
                  f"{sum(1 for m in frames_meta if m['grid_ok'])}  {time.time() - t0:.0f}s")
    cap.release()

    n_ok = sum(1 for m in frames_meta if m["grid_ok"])
    print(f"[recon] 检测完成：{len(frames_meta)} 帧中 {n_ok} 帧网格拟合成功，"
          f"耗时 {time.time() - t0:.0f}s")

    # 全局分数校准：标签在格内位置一致 → paper 落格心分数众数即偏移
    frac_c = [q[0] for q in cont_pos.get("paper", [])]
    frac_r = [q[1] for q in cont_pos.get("paper", [])]
    if len(frac_c) < 8:
        frac_c = [q[0] for cls in OBJ_CLASSES for q in cont_pos.get(cls, [])]
        frac_r = [q[1] for cls in OBJ_CLASSES for q in cont_pos.get(cls, [])]
    fc, fr = circular_mode(frac_c), circular_mode(frac_r)
    print(f"[recon] 分数校准 offset = ({fc:.2f}, {fr:.2f})"
          f"  paper连续点 {len(cont_pos.get('paper', []))} 个")

    # 落格投票（锚点门控 + 残差门控 + 每帧每类每格 1 票）
    votes: dict[str, Counter] = defaultdict(Counter)
    vframes: dict[str, dict[str, set]] = defaultdict(dict)
    vconf: dict[str, dict[str, float]] = defaultdict(dict)
    seen: dict[int, set[tuple[str, str]]] = defaultdict(set)
    for m in frames_meta:
        if not m["grid_ok"]:
            continue
        anchors = [parse_cell(n) for n in m["labels"]]
        for o in m["objs"]:
            qc, qr = o["q"]
            if not anchor_within(qc, qr, anchors):
                continue
            cr = snap_cell(qc, qr, fc, fr)
            if cr is None:
                continue
            key = (o["cls"], cell_name(*cr))
            if key not in seen[m["frame"]]:
                seen[m["frame"]].add(key)
                votes[o["cls"]][key[1]] += 1
                vframes[o["cls"]].setdefault(key[1], set()).add(m["frame"])
                vconf[o["cls"]][key[1]] = vconf[o["cls"]].get(key[1], 0.0) + o["conf"]

    # 可见覆盖（该格被有效网格帧覆盖的次数）
    coverage: Counter = Counter()
    for m in frames_meta:
        if not m["grid_ok"]:
            continue
        cells = [parse_cell(n) for n in m["labels"]]
        cols = [c for c, _ in cells]
        rows = [r for _, r in cells]
        for c in range(min(cols) - 1, max(cols) + 2):
            for r in range(min(rows) - 1, max(rows) + 2):
                if 0 <= c < COLS and 0 <= r < ROWS:
                    coverage[cell_name(c, r)] += 1

    # 场势聚合
    paper_cells = confirmed_cells(["paper"], votes, vconf, vframes,
                                  VOTE_MIN, PAPER_MEAN_CONF_MIN, phantom_kill=True)
    mines, passable = split_mines_passable(paper_cells)
    obstacles = confirmed_cells(["obstacle", "bucket_red", "bucket_green", "bucket_cyan"],
                                votes, vconf, vframes)
    tgt_red = confirmed_cells(["target_red"], votes, vconf, vframes,
                              TARGET_MIN_VOTES, TARGET_MEAN_CONF_MIN)
    tgt_blue = confirmed_cells(["target_blue"], votes, vconf, vframes,
                               TARGET_MIN_VOTES, TARGET_MEAN_CONF_MIN)
    car_start = confirmed_cells(["car_start"], votes, vconf, vframes, 2)
    car_end = confirmed_cells(["car_end"], votes, vconf, vframes, 2)
    uav_marks = {cls: confirmed_cells([cls], votes, vconf, vframes, 2)
                 for cls in ("uav_start", "uav_end")}

    _, phantom_log = eliminate_phantoms(
        [c for c, v in votes["paper"].items() if v >= VOTE_MIN],
        votes["paper"], vframes.get("paper", {}))

    field_state = {
        "video": str(video), "stride": stride,
        "frames_sampled": len(frames_meta), "frames_grid_ok": n_ok,
        "frac_offset": [round(fc, 3), round(fr, 3)],
        "mines_video": mines, "passable_video": passable,
        "phantoms_eliminated": phantom_log,
        "obstacles_video": obstacles,
        "targets_video": {"red": tgt_red, "blue": tgt_blue},
        "car_start_video": car_start, "car_end_video": car_end,
        "uav_marks_video": uav_marks,
        "votes_detail": {cls: dict(cnt.most_common()) for cls, cnt in votes.items()},
        "coverage": dict(coverage.most_common()),
    }
    (out_dir / "field_state.json").write_text(
        json.dumps(field_state, ensure_ascii=False, indent=2), encoding="utf-8")
    (out_dir / "frames_meta.json").write_text(
        json.dumps(frames_meta, ensure_ascii=False), encoding="utf-8")
    print(f"[recon] 态势: 雷 {mines} | K列箭头 {passable} | 障碍 {obstacles}"
          f" | 靶 红{tgt_red} 蓝{tgt_blue} | 幻影 {phantom_log}")
    return field_state


# ---------------------------------------------------------------- 规划
def plan_from_state(field_state: dict, out_dir: str | Path) -> dict:
    """场地态势 → 路线规划 + 车端命令 + 渲染。视频未覆盖要素用裁判系统默认。"""
    out_dir = Path(out_dir)

    def cell(n: str) -> CellRef:
        c, r = parse_cell(n)
        return CellRef(c, r)

    mines = field_state["mines_video"]
    passable = field_state["passable_video"]
    obstacles = field_state["obstacles_video"]
    tgt_blue = field_state["targets_video"]["blue"]
    tgt_red = field_state["targets_video"]["red"]
    car_start = field_state["car_start_video"]
    car_end = field_state["car_end_video"]

    use_passable = passable or list(PASSABLE_DFT)
    use_start = car_start[0] if len(car_start) == 1 else START_DFT
    use_goal = car_end[0] if len(car_end) == 1 else GOAL_DFT

    targets: list[TargetRef] = []
    for c in tgt_blue:
        targets.append(TargetRef(cell(c), True))
    for c in tgt_red:
        targets.append(TargetRef(cell(c), False))
    if not targets:                     # 移动靶默认（视频未见时）
        targets = [TargetRef(cell(TRACK_DFT[0]), True,
                             track=(cell(TRACK_DFT[0]), cell(TRACK_DFT[1])),
                             blue_face=BLUE_FACE_DFT)]

    snapshot = WorldSnapshot(
        timestamp_ms=0,
        mines=tuple(cell(c) for c in mines),
        passable=tuple(cell(c) for c in use_passable),
        obstacles=tuple(cell(c) for c in obstacles),
        targets=tuple(targets),
    )
    route = plan_route(snapshot, cell(use_start), cell(use_goal),
                       start_heading="y", final_heading="y")

    # 压缩（保护雷/桥洞/轨道/射击停靠格）+ 车端命令
    track_names = {c.name for t in targets if t.track for c in t.track}
    keep = (set(mines) | set(use_passable) | track_names
            | set(tgt_blue) | set(tgt_red) | shoot_keep_cells(targets))
    compact = compress_route(route, keep)
    cmds = adapt(snapshot, compact, use_start, "y", "y")

    cells = [cell(n) for n in route]
    length, turns = 0.0, 0
    prev = None
    for a, b in zip(cells, cells[1:]):
        d = (b.col - a.col, b.row - a.row)
        length += (d[0] ** 2 + d[1] ** 2) ** 0.5
        if prev is not None and (abs(d[0] - prev[0]) + abs(d[1] - prev[1])) in (1, 2):
            turns += 1
        prev = d
    route_set = set(route)
    missing = [c for c in mines if c not in route_set]
    viol = [c for c in route if c in obstacles]

    route_plan = {
        "source": {"mines/obstacles/targets": "视频模型识别",
                   "未覆盖要素": "裁判系统默认",
                   "passable": use_passable, "start": use_start, "goal": use_goal},
        "snapshot": {"mines": mines, "passable": use_passable, "obstacles": obstacles,
                     "targets": [f"{'蓝' if t.is_blue else '红'}@{t.cell.name}" for t in targets]},
        "route": list(route), "route_len_cells": len(route),
        "route_len_diag": round(length, 2), "turns": turns,
        "compact": list(compact),
        "car_cmds": {"reset": cmds.reset, "route": cmds.route,
                     "shoot_cells": list(cmds.shoot_cells)},
        "check": {"排雷覆盖": "全部覆盖" if not missing else f"缺失{missing}",
                  "障碍避让": "通过" if not viol else f"穿障{viol}"},
    }
    (out_dir / "route_plan.json").write_text(
        json.dumps(route_plan, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[plan] 路线 {len(route)} 格: {' → '.join(route)}")
    print(f"[plan] 压缩 {len(compact)} 点 | 排雷覆盖 {'✓' if not missing else missing}"
          f" | 避障 {'✓' if not viol else viol}")
    _render_plan(snapshot, route, compact, cmds, use_start, use_goal,
                 turns, length, out_dir / "route_render.png")
    return route_plan


def _render_plan(snapshot: WorldSnapshot, route: tuple[str, ...],
                 compact: tuple[str, ...], cmds, use_start: str, use_goal: str,
                 turns: int, length: float, out_path: Path) -> None:
    from PIL import Image, ImageDraw, ImageFont
    CELL_PX, PAD = 60, 50
    W, H = PAD * 2 + COLS * CELL_PX, PAD * 2 + ROWS * CELL_PX

    def cell(n: str) -> CellRef:
        c, r = parse_cell(n)
        return CellRef(c, r)

    img = Image.new("RGB", (W, H + 34), (30, 30, 30))
    dr = ImageDraw.Draw(img)
    try:
        f14 = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 14)
        f10 = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 10)
    except Exception:
        f14 = f10 = ImageFont.load_default()

    def xy(c: CellRef) -> tuple[int, int]:
        return PAD + c.col * CELL_PX, PAD + c.row * CELL_PX

    for r in range(ROWS + 1):
        dr.line([(PAD, PAD + r * CELL_PX), (PAD + COLS * CELL_PX, PAD + r * CELL_PX)], (80, 80, 80))
    for c in range(COLS + 1):
        dr.line([(PAD + c * CELL_PX, PAD), (PAD + c * CELL_PX, PAD + ROWS * CELL_PX)], (80, 80, 80))
    for c in range(COLS):
        dr.text((PAD + c * CELL_PX + CELL_PX // 2, PAD - 20), LETTERS[c], "white", f10, "mm")
    for r in range(ROWS):
        dr.text((PAD - 20, PAD + r * CELL_PX + CELL_PX // 2), str(r + 1), "white", f10, "mm")

    for name in [c.name for c in snapshot.obstacles]:
        x, y = xy(cell(name))
        dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (180, 40, 40))
        dr.text((x + CELL_PX // 2, y + CELL_PX // 2), "障", "white", f14, "mm")
    for r in range(ROWS):
        name = f"{BRIDGE_COL}{r + 1}"
        x, y = xy(cell(name))
        if name in {c.name for c in snapshot.passable}:
            dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (80, 60, 120))
            dr.text((x + CELL_PX // 2, y + CELL_PX // 2), "洞", "white", f14, "mm")
        else:
            dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (60, 60, 60))
            dr.text((x + CELL_PX // 2, y + CELL_PX // 2), "墙", (150, 150, 150), f10, "mm")
    for name in [c.name for c in snapshot.mines]:
        x, y = xy(cell(name))
        dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (220, 200, 30))
        dr.text((x + CELL_PX // 2, y + CELL_PX // 2 - 6), "雷", "black", f14, "mm")
    for t in snapshot.targets:
        x, y = xy(t.cell)
        dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (95, 75, 140))
        dr.text((x + CELL_PX // 2, y + CELL_PX // 2 - 6),
                "蓝" if t.is_blue else "红", "white", f14, "mm")

    def grad(t: float) -> tuple[int, int, int]:
        if t < 0.5:
            return (int(255 * t * 2), 255, 0)
        return (255, int(255 * (1 - (t - 0.5) * 2)), 0)

    rc = [cell(n) for n in route]
    for i in range(len(rc) - 1):
        x1, y1 = xy(rc[i]); x2, y2 = xy(rc[i + 1])
        dr.line([(x1 + CELL_PX // 2, y1 + CELL_PX // 2), (x2 + CELL_PX // 2, y2 + CELL_PX // 2)],
                grad(i / max(len(rc) - 2, 1)), 4)
    cset = set(compact)
    for i, n in enumerate(route):
        x, y = xy(cell(n))
        cx, cy = x + CELL_PX // 2, y + CELL_PX // 2
        r = 7 if n in cset else 3
        col = grad(i / max(len(route) - 1, 1)) if n in cset else (150, 150, 150)
        dr.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col, outline="white")
    for name in cmds.shoot_cells:
        x, y = xy(cell(name))
        cx, cy = x + CELL_PX // 2, y + CELL_PX // 2
        dr.ellipse([cx - 11, cy - 11, cx + 11, cy + 11], outline=(255, 200, 0), width=2)
        dr.line([(cx - 6, cy - 6), (cx + 6, cy + 6)], (255, 200, 0), 2)
        dr.line([(cx - 6, cy + 6), (cx + 6, cy - 6)], (255, 200, 0), 2)
    sx, sy = xy(cell(use_start)); gx, gy = xy(cell(use_goal))
    dr.rectangle([sx + 4, sy + 4, sx + CELL_PX - 4, sy + CELL_PX - 4], outline=(0, 255, 0), width=3)
    dr.text((sx + CELL_PX // 2, sy + CELL_PX - 8), "起点", (0, 255, 0), f10, "mm")
    dr.rectangle([gx + 4, gy + 4, gx + CELL_PX - 4, gy + CELL_PX - 4], outline=(255, 100, 0), width=3)
    dr.text((gx + CELL_PX // 2, gy + CELL_PX - 8), "终点", (255, 100, 0), f10, "mm")
    dr.text((PAD, H - 18), "绿→黄→红=路径先后  雷=视频识别  障/墙/洞/靶=态势  ⊕=停靠射击", "white", f10)
    dr.text((PAD, H + 2), f"路线 {len(route)} 格 / 压缩 {len(compact)} 点 / 转向 {turns} 次"
            f" / 直线长 {length:.1f} 格  起点{use_start}→终点{use_goal}", (255, 230, 130), f10)
    img.save(out_path)
    print(f"[render] {out_path}")


# ---------------------------------------------------------------- CLI
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="赛前视频场地侦察：D13+v4d 识别 → 态势 → 路线规划（全机械化）")
    ap.add_argument("video", type=str)
    ap.add_argument("--stride", type=int, default=15)
    ap.add_argument("--out", type=str, default=None)
    ap.add_argument("--max-frames", type=int, default=0, help="调试：只处理前 N 个采样帧")
    ap.add_argument("--d13", type=str, default=None, help="hbb_d13.onnx 路径")
    ap.add_argument("--v4d", type=str, default=None, help="cell_classifier_v4d.onnx 路径")
    args = ap.parse_args(argv)

    out_dir = Path(args.out) if args.out else Path(
        f"视频侦察_{datetime.now().strftime('%Y%m%d')}")
    state = recon_video(args.video, out_dir, args.d13, args.v4d,
                        args.stride, args.max_frames)
    plan_from_state(state, out_dir)
    print(f"[done] 全部产物在 {Path.cwd() / out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
