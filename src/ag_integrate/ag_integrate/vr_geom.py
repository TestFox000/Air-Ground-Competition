# vr_geom —— 由 video_recon.py 拆出（格号/仿射对齐/幻影消除/冲突裁决几何）。与原文件逐行一致（AST 校验过）。
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime
from itertools import combinations
from pathlib import Path

import cv2
import numpy as np

try:                                    # 包内导入（python -m ag_integrate.video_recon）
    from .cjk_font import load_cjk_font
    from .field_frame import CellRef, COLS, LETTERS, ROWS
    from .field_rules import MAX_MINES
    from .world_model import TargetRef, WorldSnapshot
    from .mission_router import _BARRIER_COLS, _MID_BOUNDARY, plan_route
    from .car_adapter import compact_and_adapt
except ImportError:                     # 直接以文件路径运行（比赛现场快捷用法）
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from ag_integrate.cjk_font import load_cjk_font
    from ag_integrate.field_frame import CellRef, COLS, LETTERS, ROWS
    from ag_integrate.field_rules import MAX_MINES
    from ag_integrate.world_model import TargetRef, WorldSnapshot
    from ag_integrate.mission_router import _BARRIER_COLS, _MID_BOUNDARY, plan_route
    from ag_integrate.car_adapter import compact_and_adapt
from vr_const import ANCHOR_GATE, BRIDGE_COL, BRIDGE_COL_IDX, CARRY_INLIER, FIT_INLIER, GRID_MAE_MAX, LABEL_OVERLAP_IOU, MAX_VOTE_RESIDUAL, NAMES, NMS_IOU, PHANTOM_STRONG_RECALL_MIN, REVIEW_CONF_MIN, SAHI_OVERLAP, SAHI_TILE, VOTE_MIN

# 末两条是**机上**现役模型位置（glob 取最新，与 tools/orin_fix_pyvision.sh 的
# `ls -t ~/dzuav/models/...` 同一约定）：机上那份仓库拷贝不带 models/release
# （部署走 --skip-models，引擎由机上转换），没有这些就只能手传 --d13/--v4d。
# 前一条优先认 canonical 文件名（d13 检测器），后一条兜住 640/迭代变体——
# 否则 mtime 更新的 640 变体会无声接管分辨率（召回与已验收口径不同）。
# 选中哪份会打印出来；imgsz 不匹配由 D13Detector._sync_to_model 自动纠正。


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
                       vframes: dict[str, set],
                       coverage: dict[str, int] | None = None,
                       ) -> tuple[list[str], dict[str, str]]:
    """幻影消除：弱格(<=3票)若与某强格(>=6票且>=3x)从未同帧共现、且票期
    被强格夹芯包裹（强格在弱格前后都有票），判定为同一物理物体的误归属
    （透视偏移/v4d 误读锚点拽偏），剔除。同帧共现 = 两个独立物体，保留。

    召回前置门（2026-10-23 新增）：强格在自身覆盖帧里的检出率
    < PHANTOM_STRONG_RECALL_MIN 时不作幻影源。"从未同帧共现"只有在强格
    大概率会被检出时才是证据；强格自召回 18% 时，弱格那几帧里它恰好
    都没检出的概率过半，"未共现"纯属噪声（当天 G5/I8 两颗真雷被误杀）。"""
    phantoms: dict[str, str] = {}
    strong = [c for c in cells if votes.get(c, 0) >= 6]
    for c in cells:
        n = votes.get(c, 0)
        if n > 3:
            continue
        for s in strong:
            if votes.get(s, 0) < 3 * n:
                continue
            if coverage is not None:
                cov = coverage.get(s, 0)
                if cov and votes.get(s, 0) / cov < PHANTOM_STRONG_RECALL_MIN:
                    continue          # 强格自召回低：未共现不构成证据
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
                    phantom_kill: bool = False,
                    strong_votes: int = 1 << 30,
                    coverage: dict[str, int] | None = None,
                    ) -> list[str]:
    """跨类合并投票 + 票数/均值置信度门控 + 可选幻影消除。

    mean_conf 门**只拦"票数刚好够"的弱格**（v < strong_votes）：多帧独立同意的
    证据强度已经足够，再拿均值卡它会把真正的小目标卡掉。2026-10-21 实测事故：
    把单框阈值从 0.22 放宽到 0.15 后，G9 拿到 3 票、均值置信度 0.336 < 0.35，
    被门卡掉——同一段视频上一版（阈值 0.22）反而检得到它。放宽单框阈值与收紧
    均值门互相打架，属于"参数联动"而非模型变差。
    """
    merged: Counter = Counter()
    for cls in cls_list:
        merged.update(votes.get(cls, Counter()))
    cells = []
    for c, v in merged.items():
        if v < min_votes:
            continue
        if min_mean_conf and v < strong_votes:
            mconf = sum(vconf.get(k, {}).get(c, 0.0) for k in cls_list) / max(v, 1)
            if mconf < min_mean_conf:
                continue
        cells.append(c)
    if phantom_kill and cells:
        merged_frames: dict[str, set] = defaultdict(set)
        for cls in cls_list:
            for c, fs in vframes.get(cls, {}).items():
                merged_frames[c] |= fs
        cells, _ph = eliminate_phantoms(cells, merged, merged_frames,
                                        coverage=coverage)
    return sorted(cells)
def mine_review_candidates(votes: Counter, vconf: dict[str, float],
                           claimed: set[str]) -> list[dict]:
    """高conf单票 → "待人工复核"清单（只提示，不进雷表、不挡路线）。

    为什么（2026-10-23 尸检）：I4(0.88)/I5(0.85)/J10(0.89) 单票被票数门
    静默丢弃。0.85+ 的 paper 单框极少是人体/阴影误检（那些是低conf），
    扔掉等于扔真雷证据；但单票直接确认又是在最薄证据上硬判决（stride1
    弱误检累积的教训）。折中：列出来，现场抽 label_crops 十秒定真假。
    claimed = 已被其它语义（障碍/靶）裁决占用的格，不重复提示。"""
    out = []
    for c, v in votes.items():
        if v >= VOTE_MIN or c[0] == BRIDGE_COL or c in claimed:
            continue
        conf = vconf.get(c, 0.0) / max(v, 1)
        if conf >= REVIEW_CONF_MIN:
            out.append({"cell": c, "votes": v, "conf": round(conf, 3)})
    return sorted(out, key=lambda r: -r["conf"])
def resolve_cell_conflicts(roles: dict[str, list[str]],
                           role_votes: dict[str, dict[str, int]],
                           keep_roles: tuple[str, ...] = ("passable",),
                           ) -> tuple[dict[str, list[str]], list[dict]]:
    """同一格被多个语义命中时裁决成单一标签，返回 (过滤后的各角色格, 冲突明细)。

    为什么必须有（实测）：把 paper 阈值放宽到 0.15 后
    红桶/红靶、蓝桶/蓝靶在近距画面里互相串类，同一格同时进了
    target_blue 与 bucket_green（B7: 5 vs 31 票）、target_red 与 bucket_red
    （D5: 6 vs 19 票）。不裁决会产出自相矛盾的态势：靶格同时是障碍格 →
    规划器找不到可停靠的射击邻格 → 车命令里一个 `*` 都没有（本轮实测）。

    规则：票多者胜；平票按 靶 > 桶/障 > 雷 的语义优先级。K 列桥门（passable）
    不参与——门格被实物占用是规划器自己的告警路径（10-17 已修），
    在这里吞掉反而会让"门开/门堵"无从审计。
    冲突明细写进 field_state，供现场人工复核。
    """
    priority = {"target_blue": 4, "target_red": 4, "obstacle": 3, "mine": 2}
    claims: dict[str, list[tuple[int, int, str]]] = defaultdict(list)
    for role, cells in roles.items():
        if role in keep_roles:
            continue
        for c in cells:
            claims[c].append((role_votes.get(role, {}).get(c, 0),
                              priority.get(role, 1), role))

    winners: dict[str, str] = {}
    conflicts: list[dict] = []
    for c, items in claims.items():
        kinds = {r for _, _, r in items}
        if len(kinds) <= 1:
            continue
        items.sort(key=lambda t: (t[0], t[1]), reverse=True)
        winners[c] = items[0][2]
        conflicts.append({"cell": c, "winner": items[0][2],
                          "claims": {r: v for v, _, r in items}})

    out: dict[str, list[str]] = {}
    for role, cells in roles.items():
        out[role] = list(cells) if role in keep_roles \
            else [c for c in cells if winners.get(c, role) == role]
    return out, conflicts
def split_mines_passable(paper_cells: list[str],
                         bridge_col_idx: int = BRIDGE_COL_IDX
                         ) -> tuple[list[str], list[str]]:
    """K 列 paper = 桥洞箭头(可通行)，其余列 paper = 雷。"""
    mines = [c for c in paper_cells if LETTERS.index(c[0]) != bridge_col_idx]
    passable = [c for c in paper_cells if LETTERS.index(c[0]) == bridge_col_idx]
    return mines, passable
# 注：此处原有的 shoot_keep_cells() 已删除——它只保"有轨道的靶"的停靠格，
# 静态蓝靶的开火格保不住，压缩会删掉开火格导致车命令丢失 '*'（2026-10-21 实测）。
# KEEP 推导唯一真源是 mission_router.mission_keep，勿在本文件重建。


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
def slice_tiles(w: int, h: int, tile: int = SAHI_TILE,
                overlap: float = SAHI_OVERLAP) -> list[tuple[int, int, int, int]]:
    """SAHI 切片几何：tile×tile 瓦片 + overlap 重叠，返回覆盖全图的 (x1,y1,x2,y2)。

    步长 = tile*(1-overlap)；末瓦片钳到图边保证全覆盖。某边 ≤ tile 时该维
    只出一个满幅位置（1280x720 / tile 640 → x∈[0,480,640]、y∈[0,80] 共 6 片）。
    """
    def axis(n: int) -> list[int]:
        if n <= tile:
            return [0]
        step = max(1, int(tile * (1 - overlap)))
        starts = list(range(0, n - tile + 1, step))
        if starts[-1] != n - tile:
            starts.append(n - tile)
        return starts
    return [(x, y, min(x + tile, w), min(y + tile, h))
            for y in axis(h) for x in axis(w)]
def _nms_merge(dets: list[dict], iou_th: float = NMS_IOU) -> list[dict]:
    """跨瓦片框合并：同类内按 conf 贪心 NMS。

    重叠区的同一物体会被相邻瓦片各检出一次，靠本函数去重（同 detect() 的
    帧内 NMS 口径 NMS_IOU，只是合并域从"单瓦片"扩到"全图"）。"""
    out: list[dict] = []
    for name in NAMES:
        cand = sorted((d for d in dets if d["cls"] == name),
                      key=lambda d: -d["conf"])
        kept = [d for d in out if d["cls"] == name]
        for d in cand:
            if all(_iou(d["box"], k["box"]) <= iou_th for k in kept):
                out.append(d)
                kept.append(d)
    return out
