# -*- coding: utf-8 -*-
"""赛前视频场地侦察：D13 + v4d 全机械化识别 → 场地态势 → 路线规划。

默认 = **实测最优点**（不是最省时间，也不是最密采样）：
    python video_recon.py 场地巡拍.mp4                       # 默认 stride=5（8/8 基线）
    python video_recon.py 场地巡拍.mp4 --out 输出目录
    python video_recon.py 场地巡拍.mp4 --stride 1            # 逐帧（更慢，且误检更多）
⚠️ stride 两端都会掉精度，不是"越快越差"的单向关系。同一段视频实测（基线 8 雷）：
   stride 15 → 3 雷（采样漏检）；stride 5 → 8 雷；stride 1 → 14 雷（7 个多余，
   弱误检累积到 ≥3 票后混进来，且幻影消除把真雷 E2 误判成 C3 的误归属）。
   默认 5 是召回/精度同时达标的点。已有缓存要改门控时用 --reaggregate（秒级）。

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

# ---------------------------------------------------------------- 常量
NAMES = ["paper", "cell_label", "car", "target_red", "target_blue", "obstacle",
         "bucket_red", "bucket_green", "bucket_cyan",
         "car_start", "car_end", "uav_start", "uav_end"]
CONF = {  # 离线侦察工作点（paper 0.15 + 投票门控；默认 stride=5 实测最优点）
    # 与机载 C++ kDeployConf（paper=0.25、cell_label=0.50）**不同源**：机载是
    # 实时单帧无时序投票，必须收紧；离线有跨帧投票，放宽才不丢单帧可见的目标。
    # 实测教训（同一段视频，官方基线 8 雷）：stride 15→3 雷（漏检）、5→8 雷、
    # 1→14 雷（弱误检累积票后混入）。默认 5 是召回/精度的实测平衡点。
    "paper": 0.15, "cell_label": 0.35, "car": 0.25,
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
# 移动靶轨道的邻域半径（格）：落在这个邻域内的"静靶"检出 = 移动靶的误投影/误检
# （2026-10-18 E8 审计已裁定，2026-10-21 机上 E7 检出复现）。裁判地图规定
# **全场唯一靶 = 移动双面靶**（占 D7~E7 两格），别处检出的"靶"必是桶之类的
# 彩色物体被误判——两个独立结论同向，故按此半径做先验裁决。
TRACK_NEIGHBOR_R = 1

# 模型默认搜索路径（相对运行目录逐级向上找）。
# 第一顺位是 models/release/ 的 current 指针（文件内容 = 同目录下的模型文件名），
# 与 tools/orin_deploy.py / tools/preflight.py 同一真源——"现役模型"只有这一处
# 定义。训练目录路径留作兼容。
# 2026-10-21 实测教训：原实现只写训练目录（dk_train_pack_* / cloud_train_*），
# 那些目录一清理，默认调用就报"未找到模型"，逼人手传 --d13/--v4d——与"比赛日
# 无人值守一条命令"的目标相悖，且手传的路径无人校验版本（7 类旧模型照跑）。
D13_SEARCH = ("models/release/current_hbb_detector",
              "dk_train_pack_20261016/models/hbb_d13.onnx",
              "~/dzuav/models/hbb_detector_d13*.onnx",
              "~/dzuav/models/hbb*d13*.onnx")
V4D_SEARCH = ("models/release/current_cell_classifier",
              "cloud_train_20261015/deploy_pkg/cell_classifier_v4d_20261015.onnx",
              "~/dzuav/models/cell_classifier_v4d*.onnx",
              "~/dzuav/models/cell*v4d*.onnx")
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
                    phantom_kill: bool = False,
                    strong_votes: int = 1 << 30) -> list[str]:
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
        cells, _ph = eliminate_phantoms(cells, merged, merged_frames)
    return sorted(cells)


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
    """hbb_d13*.onnx（OpenCV DNN）：letterbox → [1,17,N] 解码 + 逐类 NMS。

    imgsz 由模型决定：1280 版输出 33600 anchors（160²+80²+40²），640 版输出
    8400（80²+40²+20²）。两版共用同一套权重，只是导出分辨率不同——
    2026-10-17 实测 640 版召回 6/8、1280 版 4/8（见 进度/2026-10-17_会话纪要）。
    """

    def __init__(self, onnx_path: str | Path, imgsz: int = IN_SIZE):
        self.net = cv2.dnn.readNetFromONNX(str(onnx_path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.imgsz = int(imgsz)
        # anchor-free：每个 stride 层每格 1 个点
        self.n_anchors = sum((self.imgsz // s) ** 2 for s in (8, 16, 32))
        self.model_path = str(onnx_path)
        self._probe_contract()

    def _probe_contract(self) -> None:
        """构造期契约自检：通道数必须是 13 类（17=4+13），分辨率自动对齐。

        为什么必须在这里查（2026-10-21 机上实测事故）：机上 `~/dzuav/models`
        同时躺着 7 类旧模型与 13 类模型，指针/glob 一旦选中 7 类（11 通道），
        `detect()` 每帧都在 `scores[7]` 索引越界；而**逐帧异常隔离**会把它当成
        "60 个坏帧"静默跳过 → 态势全空、路线照出、整跑 rc=0 的假绿。
        与机载 C++ `HbbDetector.cpp` 的 [1,17,N] 硬校验同一道门：模型不对就不许跑。
        """
        want = 4 + len(NAMES)
        tried: list[str] = []
        for S in (self.imgsz, 640 if self.imgsz == 1280 else 1280):
            self.net.setInput(np.zeros((1, 3, S, S), np.float32))
            try:
                out = self.net.forward()
            except Exception as exc:  # noqa: BLE001 - 静态 shape 不匹配会在这爆
                tried.append(f"{S}: {type(exc).__name__}: {exc}")
                continue
            ch = int(out.shape[1] if out.ndim == 3 else out.shape[0])
            n = int(out.shape[2] if out.ndim == 3 else out.shape[1])
            if ch != want:
                tried.append(f"{S}: 输出 {ch} 通道（need {want}）")
                continue
            if n != sum((S // s) ** 2 for s in (8, 16, 32)):
                tried.append(f"{S}: {n} anchors 与分辨率不符")
                continue
            if S != self.imgsz:
                print(f"[recon][提示] 模型实际是 {S} 推理（{n} anchors），"
                      f"已自动纠正 --imgsz")
            self.imgsz, self.n_anchors = S, n
            return
        raise ValueError(
            f"模型契约不符，拒绝运行: {self.model_path}\n"
            f"  期望 13 类（{want} 通道 = 4 框 + 13 类），实测：\n"
            + "\n".join(f"    {t}" for t in tried)
            + "\n  最常见原因：选中了 7 类旧模型（11 通道）。"
              "\n  修法：--d13 指向 13 类 onnx（机上 ~/dzuav/models/"
              "hbb_detector_d13_20261016.onnx，或在仓库根目录跑让 "
              "models/release/current_hbb_detector 指针生效）")

    def _sync_to_model(self, n: int) -> bool:
        """把 imgsz 对齐到模型真实输出（换模型时自动纠正，免手工 --imgsz）。

        这是 2026-10-21 的补强：640/1280 两版模型的 anchor 数不同，用户拿错
        --imgsz 时原实现会直接索引越界或静默解错。现在先按常见两档反查，
        对不上再报一条能直接照做的错误（含模型路径与实际 anchor 数）。
        """
        by_anchors = {sum((s // k) ** 2 for k in (8, 16, 32)): s for s in (640, 1280)}
        if n in by_anchors:
            self.imgsz = by_anchors[n]
            self.n_anchors = n
            print(f"[recon][提示] 模型实际是 {self.imgsz} 推理（{n} anchors），"
                  f"已自动纠正 --imgsz")
            return True
        return False

    def detect(self, img_bgr: np.ndarray) -> list[dict]:
        """返回 [{cls, conf, box=[x1,y1,x2,y2](原图坐标)}]，坐标已映射回原图。"""
        S = self.imgsz
        h, w = img_bgr.shape[:2]
        scale = S / max(h, w)
        nw, nh = round(w * scale), round(h * scale)
        resized = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((S, S, 3), 114, np.uint8)
        pad_x, pad_y = (S - nw) // 2, (S - nh) // 2
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
        self.net.setInput(cv2.dnn.blobFromImage(
            canvas, 1 / 255.0, (S, S), swapRB=True))
        out = self.net.forward()[0]              # (17, n_anchors)
        n = out.shape[1]
        if n != self.n_anchors:
            if self._sync_to_model(n):
                return self.detect(img_bgr)      # 用纠正后的尺寸重跑一次
            raise ValueError(
                f"模型输出 {n} 个 anchor，与 imgsz={S} 期望的 {self.n_anchors} 不符，"
                f"且不是已知的 640/1280 两档。\n"
                f"  模型: {self.model_path}\n"
                f"  处理: 用 --imgsz 指定与模型导出一致的分辨率，或换用 1280/640 版模型")
        boxes, scores = out[:4, :].T, out[4:, :]  # cx,cy,w,h | (13,N)
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
        self.model_path = str(onnx_path)
        self._probe_contract()

    def _probe_contract(self) -> None:
        """双头形状自检：letter 14 / row 11。

        与 D13 同一理由（2026-10-21）：机上模型目录混放多代产物，选错时
        逐帧隔离会把异常吞成"坏帧"，产出全空的态势却 rc=0。这里一次 96×96
        推理就能验明；形状不对直接拒绝，由调用方换下一个候选。
        """
        self.net.setInput(np.zeros((1, 3, V4D_IN, V4D_IN), np.float32))
        try:
            outs = self.net.forward(self.out_names)
        except Exception as exc:  # noqa: BLE001
            raise ValueError(
                f"格号模型无法前向: {self.model_path}\n  {type(exc).__name__}: {exc}"
                f"\n  修法：--v4d 指向双头 v4d onnx（letter14/row11，96×96 输入）") from exc
        shapes = sorted(int(o.shape[-1]) for o in outs)
        if shapes != [11, 14]:
            raise ValueError(
                f"格号模型契约不符: {self.model_path}\n"
                f"  期望双头输出 [14, 11]，实测 {shapes}"
                f"\n  修法：--v4d 指向 cell_classifier_v4d*.onnx")

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
def _model_from_entry(entry: Path) -> Path | None:
    """搜索项 -> 真实 ONNX。搜索项可以是指针文件（内容 = 同目录模型文件名，
    与 tools/orin_deploy.py 的 current 指针同一约定）。"""
    if not entry.is_file():
        return None
    if entry.suffix.lower() == ".onnx":
        return entry
    target = entry.parent / entry.read_text(encoding="utf-8").strip()
    return target if target.is_file() else None


def _newest_by_glob(pattern: str) -> Path | None:
    """glob 搜索项取 mtime 最新的一份（多份变体并存时 = 最后部署/转换的那份）。"""
    hits = [Path(p) for p in glob.glob(os.path.expanduser(pattern))]
    hits = [p for p in hits if p.is_file()]
    return max(hits, key=lambda p: p.stat().st_mtime) if hits else None


def _iter_model_candidates(search: tuple[str, ...], explicit: str | None,
                           kind: str) -> list[Path]:
    """按优先级给出候选模型路径列表（指针 → 固定路径 → 机上 glob），去重保序。

    报告"找过哪些位置"用的是本函数的完整列表；真正选谁由调用方逐个做契约
    自检（选中的模型 7 类/13 类必须当场验明，不许等到逐帧索引越界）。
    """
    if explicit:
        p = Path(explicit)
        if not p.is_file():
            raise FileNotFoundError(
                f"--{kind} 指定的模型不存在: {p.resolve()}\n"
                f"  机上模型在 ~/dzuav/models/ 下（d13: hbb*d13*.onnx，"
                f"v4d: cell*v4d*.onnx）；确认文件名后再传")
        return [p]
    out: list[Path] = []
    tried: list[str] = []
    for base in [Path.cwd(), *Path.cwd().parents[:3]]:
        for rel in search:
            if any(ch in rel for ch in "*?["):        # glob 项与工作目录无关
                got = _newest_by_glob(rel)
                if got is None:
                    tried.append(f"{os.path.expanduser(rel)}（glob 无匹配）")
                else:
                    out.append(got)
                continue
            entry = base / rel
            if not entry.is_file():
                tried.append(str(entry))
                continue
            got = _model_from_entry(entry)
            if got is None:
                tried.append(f"{entry}（指针指向的文件不存在）")
            else:
                out.append(got)
    seen: set[str] = set()
    uniq = [p for p in out if not (str(p) in seen or seen.add(str(p)))]
    if not uniq:
        raise FileNotFoundError(
            f"未找到 {kind} 模型（{kind}：d13 检测器 / v4d 格号分类器）。找过：\n"
            + "\n".join(f"    {t}" for t in tried)
            + f"\n  修法：① --{kind} <模型.onnx> 显式指定；"
              f"② 在仓库根目录运行（models/release/current_* 指针决定现役模型，"
              f"指针缺失先跑 tools/release_models.py 发布）")
    return uniq


def _find_model(search: tuple[str, ...], explicit: str | None,
                kind: str = "d13") -> Path:
    """首个候选（不校验契约；需要"跳过坏候选"的调用方用 _iter_model_candidates）。"""
    return _iter_model_candidates(search, explicit, kind)[0]


def _open_first_valid(cands: list[Path], opener, label: str):
    """逐个尝试候选，返回第一个通过契约自检的模型对象。

    为什么必须逐个试（2026-10-21 机上实测）：机上 `models/release` 里留着早期
    部署的 7 类指针副本，优先级高于 `~/dzuav/models` 的 13 类模型；`D13Detector`
    契约自检会拒绝它，此时应**继续往下找**而不是整跑失败——但每一次拒绝都要
    打印出来，否则"为什么没用指针那份"永远查不清。
    """
    problems: list[str] = []
    for p in cands:
        try:
            return opener(p)
        except ValueError as exc:
            problems.append(str(exc))
            print(f"[recon][警告] 候选{label}未通过契约自检，继续找下一份:\n{exc}")
    raise ValueError(
        f"所有候选{label}都未通过契约自检（共 {len(cands)} 份）：\n"
        + "\n".join(f"  - {p}" for p in cands)
        + "\n处理：用显式参数指向正确模型，或清理机上过期的指针副本")


def _write_annot(out_dir: Path, fidx: int, frame: np.ndarray,
                 reads: list, objs: list) -> None:
    """落一张标注抽样帧（检测框 + 格号读数）。纯诊断产物，调用点已包异常隔离。"""
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


def recon_video(video: str | Path, out_dir: str | Path,
                d13_onnx: str | Path | None = None,
                v4d_onnx: str | Path | None = None,
                stride: int = 5, max_frames: int = 0,
                imgsz: int = IN_SIZE,
                keep_annots: int = 8, keep_crops: int = 40) -> dict:
    """视频 → 场地态势。返回 field_state dict（同时落盘 frames_meta.json）。"""
    video, out_dir = Path(video), Path(out_dir)
    if int(stride) < 1:
        raise ValueError(f"--stride 必须 >= 1，收到 {stride}")
    if not video.is_file():
        raise FileNotFoundError(
            f"视频不存在: {video.resolve()}\n"
            f"  检查路径/盘符；机上常见是文件名带空格或中文未加引号")
    # 模型先解析、目录后建：解析失败时不留空目录（报错里"产物目录（若已建）"
    # 才说得准，现场也不会误以为跑过半程）。
    # 逐个候选做契约自检（13 类 17 通道 / v4d 双头 14+11）：机上 pointers 过期
    # 副本会被拒绝并继续往下找，绝不让 7 类模型跑出"全坏帧"的假绿。
    d13 = _open_first_valid(
        _iter_model_candidates(D13_SEARCH, d13_onnx and str(d13_onnx), "d13"),
        lambda p: D13Detector(p, imgsz=imgsz), " d13 ")
    v4d = _open_first_valid(
        _iter_model_candidates(V4D_SEARCH, v4d_onnx and str(v4d_onnx), "v4d"),
        V4dReader, " v4d ")
    # 用哪份模型必须打印：机上 ~/dzuav/models 常同时躺着 640/1280/迭代变体，
    # 静默选中会让"同一个视频跑出不同答案"无从溯源（2026-10-21 实测）。
    print(f"[recon] d13 = {d13.model_path}（{d13.imgsz} 推理）")
    print(f"[recon] v4d = {v4d.model_path}")
    (out_dir / "annotated").mkdir(parents=True, exist_ok=True)
    (out_dir / "label_crops").mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(
            f"OpenCV 打不开视频: {video.resolve()}\n"
            f"  常见原因：文件损坏 / 编码不受 OpenCV 支持（试转 h264 mp4）/ "
            f"路径含中文且系统编码不匹配")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if total <= 0:
        cap.release()
        raise RuntimeError(
            f"视频帧数为 {total}，无法解码出任何帧: {video.resolve()}\n"
            f"  用 ffprobe 或播放器确认文件完整；0 字节文件在机上录像里出现过")
    idxs = list(range(0, total, stride))
    if max_frames:
        idxs = idxs[:max_frames]
    print(f"[recon] 视频总帧 {total}，采样 {len(idxs)} 帧 (stride={stride})")
    if len(idxs) > 60:                 # 逐帧默认下这次要跑很久，先把时间说清
        print(f"[recon] 预计约 {len(idxs) * 1.5 / 60:.0f} 分钟（实测约 1.5 秒/帧）")

    frames_meta: list[dict] = []
    cont_pos: dict[str, list[tuple[float, float, float]]] = defaultdict(list)
    crop_saved = 0
    t0 = time.time()
    last_aff: np.ndarray | None = None    # 最近成功仿射（时间外推缓存）
    last_step = -10 ** 9                  # 它对应的已处理帧号
    # 坏帧/坏读数的计数与明细：一次整视频侦察要跑 5~7 分钟，任何一帧异常都
    # 不该终止整次跑（与机载 C++ main.cpp 的 kMaxFrameErrors 同一套思路——
    # 那边 2026-10-20 已修"坏帧烧掉整局"，Python 这条链路此前还是裸循环）。
    frame_errors = 0
    error_samples: list[str] = []
    frames_detected = 0                # 成功跑完检测的帧数（全 0 = 链路真坏了）
    for fi, fidx in enumerate(idxs):
        cap.set(cv2.CAP_PROP_POS_FRAMES, fidx)
        ok, frame = cap.read()
        if not ok:
            continue
        try:
            dets = d13.detect(frame)
        except Exception as exc:  # noqa: BLE001 - 见上方 frame_errors 说明
            frame_errors += 1
            _note = f"f{fidx} 检测异常: {type(exc).__name__}: {exc}"
            if len(error_samples) < 5:
                error_samples.append(_note)
            print(f"[recon][坏帧] {_note}（跳过该帧，继续）")
            continue
        frames_detected += 1
        labels = [d for d in dets if d["cls"] == "cell_label"]
        objs = [d for d in dets if d["cls"] != "cell_label"]

        # v4d 读格号（正方形裁剪 + TTA + 双头置信门控）：逐框隔离——
        # 一个坏裁剪不该把整帧的其余读数一起丢掉。
        reads = []
        for d in labels:
            try:
                crop = v4d.crop_square(frame, d["box"])
                if crop is None:
                    continue
                li, ri, lc, rc = v4d.read(crop)
            except Exception as exc:  # noqa: BLE001
                frame_errors += 1
                _note = f"f{fidx} 格号读取异常: {type(exc).__name__}: {exc}"
                if len(error_samples) < 5:
                    error_samples.append(_note)
                continue
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
            try:
                px = np.array([[(b[0] + b[2]) / 2, (b[1] + b[3]) / 2] for *_, b in reads])
                grid = np.array([[li, ri] for li, ri, *_ in reads], float)
                aff, inl, mae = fit_consensus_affine(px, grid)
            except Exception as exc:  # noqa: BLE001 - 退化输入不该毁掉整帧
                frame_errors += 1
                _note = f"f{fidx} 网格拟合异常: {type(exc).__name__}: {exc}"
                if len(error_samples) < 5:
                    error_samples.append(_note)
                aff, inl = None, None
            grid_ok = aff is not None
            if grid_ok and len(reads) == 3 and last_aff is not None \
                    and fi - last_step <= CARRY_MAX_STEPS:
                # n=3 拟合无冗余（3 点必精确过定、单误读不可检出 = 静默毒化，
                # 见 test_video_recon_fit.py 特征测试）：强制过验收门——用最近
                # 成功仿射当仲裁者，≥2 读数与它一致才放行，且仿射换用仲裁者
                # 本身（新鲜 3 点解可能含误读）；仲裁失败宁丢帧不投毒票。
                gated = validate_affine_carry(last_aff, reads)
                if gated is not None:
                    keep, mae = gated
                    aff, grid_src = last_aff, "gate3"
                    reads = [reads[i] for i in keep]
                else:
                    grid_ok, mae = False, 9.9
            elif grid_ok:
                reads = [r for r, k in zip(reads, inl) if k]
        if grid_ok:
            if grid_src is None:
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

        if fi % keep_annots == 0:      # 标注帧抽样（纯诊断产物，失败不影响结论）
            try:
                _write_annot(out_dir, fidx, frame, reads, objs)
            except Exception as exc:  # noqa: BLE001
                frame_errors += 1
                _note = f"f{fidx} 标注图写入失败: {type(exc).__name__}: {exc}"
                if len(error_samples) < 5:
                    error_samples.append(_note)

        if (fi + 1) % 20 == 0:
            print(f"[recon] {fi + 1}/{len(idxs)} 帧  网格OK "
                  f"{sum(1 for m in frames_meta if m['grid_ok'])}  {time.time() - t0:.0f}s")
    cap.release()

    n_ok = sum(1 for m in frames_meta if m["grid_ok"])
    # "整段全坏"必须是硬失败：逐帧隔离是为了扛偶发坏帧，不是为了把系统级故障
    # （模型选错/解码器坏）粉饰成 0 检测的"成功"。2026-10-21 机上实测：7 类模型
    # 让每帧都 IndexError，隔离逻辑把 60 帧全吞掉，态势全空却 rc=0。
    if frames_detected == 0:
        raise RuntimeError(
            f"{len(frames_meta) + frame_errors} 个采样帧**没有任何一帧检测成功**——"
            f"不是偶发坏帧，是链路级故障。\n"
            + "\n".join(f"  {s}" for s in error_samples)
            + "\n  最常见原因：模型选错（7 类旧模型）/ 视频编码不受支持。\n"
              "  处理：核对启动时打印的 d13 路径，必要时用 --d13 显式指定 13 类模型")
    print(f"[recon] 检测完成：{len(frames_meta)} 帧中 {n_ok} 帧网格拟合成功，"
          f"耗时 {time.time() - t0:.0f}s")
    if frames_meta and n_ok == 0:
        print("[recon][注意] 全视频 0 帧网格拟合成功——态势将全空、路线退化为"
              "纯默认值。先确认视频里能否看清格号（太小/太糊/逆光），再看标注帧。")
    if frame_errors:
        # 坏帧数必须显式报出：静默跳过会让"这盘只识别到 2 个雷"看起来像模型问题
        print(f"[recon][注意] 共 {frame_errors} 处单帧/单框异常被跳过（不影响其余帧）")
        for _s in error_samples:
            print(f"[recon][注意]   {_s}")
        if len(error_samples) < frame_errors:
            print(f"[recon][注意]   …另有 {frame_errors - len(error_samples)} 处同类")

    field_state = aggregate_state(frames_meta, video=str(video), stride=stride)
    (out_dir / "field_state.json").write_text(
        json.dumps(field_state, ensure_ascii=False, indent=2), encoding="utf-8")
    (out_dir / "frames_meta.json").write_text(
        json.dumps(frames_meta, ensure_ascii=False), encoding="utf-8")
    return field_state


def aggregate_state(frames_meta: list[dict], video: str = "",
                    stride: int = 0) -> dict:
    """逐帧检测明细 → 场地态势（落格投票 → 门控 → 幻影 → 单格裁决）。

    抽成独立函数的原因：frames_meta.json 已落盘"投影后的连续格坐标"，
    门控/阈值/裁决这类**纯聚合规则**的改动不必重跑逐帧推理（机上 stride5
    整段要 17 分钟）。CLI `--reaggregate` 走的就是这条路径——"重算"与
    "重跑"共用同一份门控代码，不会长出第二套阈值。
    """
    # 全局分数校准：标签在格内位置一致 → paper 落格心分数众数即偏移
    cont_pos: dict[str, list[tuple[float, float, float]]] = defaultdict(list)
    for m in frames_meta:
        if not m.get("grid_ok"):
            continue
        for o in m.get("objs", []):
            q = o.get("q") or []
            if len(q) == 2:
                cont_pos[o["cls"]].append((q[0], q[1], o["conf"]))
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
    # paper 用 strong_votes=3：≥3 帧独立同意的格不再受均值置信度门限制（见
    # confirmed_cells 注释的 G9 事故）；其余类别维持原门控不变。
    paper_cells = confirmed_cells(["paper"], votes, vconf, vframes,
                                  VOTE_MIN, PAPER_MEAN_CONF_MIN, phantom_kill=True,
                                  strong_votes=3)
    mines, passable = split_mines_passable(paper_cells)
    obstacles = confirmed_cells(["obstacle", "bucket_red", "bucket_green", "bucket_cyan"],
                                votes, vconf, vframes)
    tgt_red = confirmed_cells(["target_red"], votes, vconf, vframes,
                              TARGET_MIN_VOTES, TARGET_MEAN_CONF_MIN)
    tgt_blue = confirmed_cells(["target_blue"], votes, vconf, vframes,
                               TARGET_MIN_VOTES, TARGET_MEAN_CONF_MIN)

    # 单格单标签裁决：靶 vs 桶/障 vs 雷（K 列桥门不参与，见函数说明）
    _obst_votes: Counter = Counter()
    for _cls in ("obstacle", "bucket_red", "bucket_green", "bucket_cyan"):
        _obst_votes.update(votes.get(_cls, Counter()))
    _roles, _conflicts = resolve_cell_conflicts(
        {"mine": mines, "passable": passable, "obstacle": obstacles,
         "target_red": tgt_red, "target_blue": tgt_blue},
        {"mine": dict(votes.get("paper", Counter())),
         "obstacle": dict(_obst_votes),
         "target_red": dict(votes.get("target_red", Counter())),
         "target_blue": dict(votes.get("target_blue", Counter()))})
    mines, passable = _roles["mine"], _roles["passable"]
    obstacles, tgt_red, tgt_blue = (_roles["obstacle"], _roles["target_red"],
                                    _roles["target_blue"])
    for _c in _conflicts:
        print(f"[recon][冲突] {_c['cell']} 多类命中 -> 判为 {_c['winner']} "
              f"（{_c['claims']}）")
    car_start = confirmed_cells(["car_start"], votes, vconf, vframes, 2)
    car_end = confirmed_cells(["car_end"], votes, vconf, vframes, 2)
    uav_marks = {cls: confirmed_cells([cls], votes, vconf, vframes, 2)
                 for cls in ("uav_start", "uav_end")}

    _, phantom_log = eliminate_phantoms(
        [c for c, v in votes["paper"].items() if v >= VOTE_MIN],
        votes["paper"], vframes.get("paper", {}))

    n_ok = sum(1 for m in frames_meta if m["grid_ok"])
    field_state = {
        "video": video, "stride": stride,
        "frames_sampled": len(frames_meta), "frames_grid_ok": n_ok,
        "frac_offset": [round(fc, 3), round(fr, 3)],
        "mines_video": mines, "passable_video": passable,
        "phantoms_eliminated": phantom_log,
        "obstacles_video": obstacles,
        "targets_video": {"red": tgt_red, "blue": tgt_blue},
        "car_start_video": car_start, "car_end_video": car_end,
        "uav_marks_video": uav_marks,
        "conflicts": _conflicts,          # 单格多类命中的人工复核明细
        "votes_detail": {cls: dict(cnt.most_common()) for cls, cnt in votes.items()},
        "coverage": dict(coverage.most_common()),
    }
    print(f"[recon] 态势: 雷 {mines} | K列箭头 {passable} | 障碍 {obstacles}"
          f" | 靶 红{tgt_red} 蓝{tgt_blue} | 幻影 {phantom_log}")
    return field_state


# ---------------------------------------------------------------- 规划
def merge_passable_with_default(detected) -> tuple[list[str], list[str]]:
    """桥门按段补齐裁判默认值，返回 (合并后的通行格, 其中来自默认值的格)。

    为什么必须按段补（2026-10-21 机上实测事故）：桥洞规则是 K 列分两段
    （K1~K6 与 K7~K11）各有且只有一个箭头。原先的 `passable or PASSABLE_DFT`
    只在**全空**时回退——视频只拍到一个段的箭头时，另一段没有任何通行格，
    被当成整段墙: 起点 N10 的连通域被切掉一半，`plan_route` 直接抛
    "goal N5 不可达"，整条视频侦察在最后一步失败（172 帧识别全对也没用）。
    正确语义是"**未覆盖要素**用默认值"——逐段判断覆盖，没覆盖的段才回退。

    解析不了的行号一律跳过（宁可用默认，也不拿坏名字去判连通）。
    """
    segs = ((1, 6), (7, ROWS))          # 两段的显示行号范围
    got: set[str] = set()
    for name in detected or ():
        s = str(name).strip().upper()
        if not s.startswith(BRIDGE_COL):
            continue
        try:
            row = int(s[len(BRIDGE_COL):])
        except ValueError:
            continue
        if 1 <= row <= ROWS:
            got.add(s)

    merged: list[str] = []
    from_default: list[str] = []
    for lo, hi in segs:
        mine = sorted(c for c in got if lo <= int(c[len(BRIDGE_COL):]) <= hi)
        if mine:
            merged.extend(mine)
        else:
            dft = [c for c in PASSABLE_DFT if lo <= int(c[len(BRIDGE_COL):]) <= hi]
            merged.extend(dft)
            from_default.extend(dft)
    return merged, from_default


def arbitrate_targets(tgt_blue, tgt_red, static_is_bucket: bool = True
                      ) -> tuple[list[str], list[str], list[str], dict[str, str]]:
    """靶先验裁决，返回 (静靶蓝, 静靶红, 改判为桶, 轨道靶观测{格号: 'red'|'blue'})。

    判据来源（三方一致）：
      1. **裁判系统地图**：全场唯一靶 = 移动双面靶，占轨道 D7~E7 两格
         （`match_params.yaml` target_track、"靶标两格双面蓝敌红友"）。
         因此除轨道自身外，视频在别的格子上检出的"靶"必是别的东西。
      2. **场地真值（2026-10-21 用户核对裁图后确认）**：
         `D5`/`B7`/`D11` 是**桶**；`E7` 是**靶**（移动双靶本体），且
         **红面 = 友方**（禁火）。
      3. **物证**：桶的检测框尺寸 22~25% 帧宽（与"障碍桶"检出同级）、俯视
         同心圆；真靶是 0.65×0.025m 立式薄板。E7 的框仅 3% 帧宽 = 远处本体。

    两类处理（**不能合并**，2026-10-21 首版把两者都当"剔除"是错的）：
      · 轨道 ±R 格内的检出 = **移动靶本体的瞬时投影** → 保留为轨道靶观测，
        并按可见面定敌我：观测到红面 → 友方，一律禁火、不派射击任务。
      · 其余静靶 = 分类头把桶读成靶 → 改判为桶并入障碍（用户已逐个确认）。

    红蓝同格冲突时**红优先**：红=友方，误判为蓝会去误击友方（rules −5），
    误判为红最多少打一个（fail-safe 方向）。

    真要在练习场摆了静靶（非官方布置），用 --allow-static-targets 关掉
    "静靶→桶"这一段（轨道本体的敌我判定不关）。
    """
    track = [parse_cell(c) for c in TRACK_DFT]

    def near_track(name: str) -> bool:
        c, r = parse_cell(name)
        return any(abs(c - tc) <= TRACK_NEIGHBOR_R and abs(r - tr) <= TRACK_NEIGHBOR_R
                   for tc, tr in track)

    obs: dict[str, str] = {}
    for name in tgt_red:                      # 红面先入：同格冲突时红优先（fail-safe）
        if near_track(name):
            obs[name] = "red"
    for name in tgt_blue:
        if near_track(name) and name not in obs:
            obs[name] = "blue"
    keep_blue = [c for c in tgt_blue if not near_track(c)]
    keep_red = [c for c in tgt_red if not near_track(c)]
    if not static_is_bucket:
        return keep_blue, keep_red, [], obs
    as_bucket = sorted(keep_blue + keep_red)
    return [], [], as_bucket, obs


def plan_from_state(field_state: dict, out_dir: str | Path,
                    static_target_is_bucket: bool = True,
                    replay_safe: bool = False) -> dict:
    """场地态势 → 路线规划 + 车端命令 + 渲染。视频未覆盖要素用裁判系统默认。

    static_target_is_bucket：静靶→桶的裁决开关（见 arbitrate_targets）。
    默认 True = "全场唯一靶"口径；练习场确实摆了静靶时用 CLI
    `--allow-static-targets` 关掉（移动靶的敌我判定不受影响）。

    replay_safe：静态回放安全开关（CLI `--replay-safe`）。两段式打击的
    前提是"在飞 fire_guard 实时复判红蓝"；静态回放链路没有实时感知，
    轨道靶停靠位的 '*' 会照打可见面——观测为红面即误击友方（−5/次）。
    开启后不排轨道靶停靠/射击位（机器级拦截，替代"手动去掉 '*'"的人肉
    约定），source.attack_requires_live_gate 同步写 False 供下游判定。
    """
    out_dir = Path(out_dir)

    def cell(n: str) -> CellRef:
        c, r = parse_cell(n)
        return CellRef(c, r)

    mines = field_state["mines_video"]
    passable = field_state["passable_video"]
    obstacles = list(field_state["obstacles_video"])
    tgt_blue = field_state["targets_video"]["blue"]
    tgt_red = field_state["targets_video"]["red"]
    car_start = field_state["car_start_video"]
    car_end = field_state["car_end_video"]

    use_passable, passable_from_default = merge_passable_with_default(passable)
    use_start = car_start[0] if len(car_start) == 1 else START_DFT
    use_goal = car_end[0] if len(car_end) == 1 else GOAL_DFT

    tgt_blue, tgt_red, bucket_from_target, track_obs = arbitrate_targets(
        tgt_blue, tgt_red, static_target_is_bucket)
    if bucket_from_target:
        print(f"[plan][裁决] 视频静靶 {bucket_from_target} 判为桶（场地真值已确认："
              f"格内是彩色圆桶，框尺寸与障碍桶同级）→ 并入障碍")
        obstacles = sorted(set(obstacles) | set(bucket_from_target))
    if track_obs:
        _faces = "、".join(f"{c}={('蓝面(敌，可打)' if f == 'blue' else '红面(友，禁火)')}"
                          for c, f in sorted(track_obs.items()))
        print(f"[plan][裁决] 轨道靶本体观测：{_faces}")

    targets: list[TargetRef] = []
    for c in tgt_blue:
        targets.append(TargetRef(cell(c), True))
    for c in tgt_red:
        targets.append(TargetRef(cell(c), False))
    # 移动靶（全场唯一靶）：**优先用视频观测**——观测格 = 本体瞬时位置。
    # 打击采用**两段式**（2026-10-18 审计口径 + 2026-10-21 用户要求"要有标靶
    # 打击"）：计划里排一个蓝面侧确认/射击停靠位，实弹由在飞 fire_guard 按实时
    # 画面复判红蓝（红面=友 / 未辨 → 禁火 fail-safe）。
    # 因此这里恒置 is_blue=True 让 mission_keep 保住停靠格——**它是"可打击性"，
    # 不是"当前可见面"**；可见面另在 targets_track_observed 里如实记录。
    # ⚠️ 只在"在飞感知链路开着"时才安全：静态回放（无实时感知）会照着 '*' 对
    #    红面开火 → 误击友方，此时用 --replay-safe 机器级剔除（不排停靠位）。
    if not replay_safe:
        for name, face in sorted(track_obs.items()):
            targets.append(TargetRef(cell(name), True,
                                     track=(cell(TRACK_DFT[0]), cell(TRACK_DFT[1])),
                                     blue_face=BLUE_FACE_DFT))
        if not track_obs:
            targets.append(TargetRef(cell(TRACK_DFT[0]), True,
                                     track=(cell(TRACK_DFT[0]), cell(TRACK_DFT[1])),
                                     blue_face=BLUE_FACE_DFT))

    snapshot = WorldSnapshot(
        timestamp_ms=0,
        mines=tuple(cell(c) for c in mines),
        passable=tuple(cell(c) for c in use_passable),
        obstacles=tuple(cell(c) for c in obstacles),
        targets=tuple(targets),
    )
    route = plan_route(snapshot, cell(use_start), cell(use_goal),
                       start_heading="y", final_heading="y")

    # 压缩 + 车端命令：必须走 car_adapter.compact_and_adapt（内部用 mission_keep
    # 推导 KEEP）。2026-10-21 实测事故：这里原先手拼 KEEP，只保"有轨道的靶"的
    # 停靠格，**静态蓝靶的开火格没保住** → 压缩把 D10（D11 蓝靶的正交邻格）当共线
    # 中间点删掉 → adapt 看不到它 → 车命令一个 '*' 都没有，整局不开火。
    # mission_keep 的注释明写"全仓只能有这一个 keep 推导"，此处即违规点。
    compact, cmds = compact_and_adapt(snapshot, route, use_start, "y", "y")

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
    # "排雷覆盖✓"只证明**已检出的雷**都在路线上，不证明没漏检。官方基线 8 颗
    # （field_rules.MAX_MINES：地图上不可能超过 8），检出数不等于基线时必须显式
    # 说出来，否则 2/8 的态势也会显示"全部覆盖"——这是 2026-10-22 实测踩到的假绿。
    _n_mines = len(mines)
    _mine_note = ("全部覆盖" if not missing else f"缺失{missing}")
    if _n_mines < MAX_MINES:
        _mine_note += (f"（仅检出 {_n_mines}/{MAX_MINES} 颗雷：漏检风险，"
                       f"用默认逐帧重跑或人工复核未检出区）")
    elif _n_mines > MAX_MINES:
        _mine_note += (f"（检出 {_n_mines} 颗 > 官方基线 {MAX_MINES}：必有 "
                       f"{_n_mines - MAX_MINES} 颗误检，先人工复核再执行）")

    route_plan = {
        "source": {"mines/obstacles/targets": "视频模型识别",
                   "未覆盖要素": "裁判系统默认",
                   "passable": use_passable,
                   # 明确标注哪些桥门来自默认值——现场一眼能看出"这段视频没拍到"
                   "passable_from_default": passable_from_default,
                   "start": use_start, "goal": use_goal,
                   # 靶裁决留痕（可追溯、可复盘）。字段名沿用 2026-10-18 审计口径：
                   # 轨道邻域内的"静靶"声明被改判为移动靶本体观测（不是误检）
                   "targets_reclassified_as_bucket": bucket_from_target,
                   "targets_track_observed": track_obs,
                   "targets_reclassified_referee_conflict": sorted(track_obs),
                   "target_attack_mode": (
                       ("回放安全（--replay-safe）：未排轨道靶停靠/射击位——"
                        "静态回放无实时红蓝复判，实弹任务必须由在飞链路重新规划")
                       if replay_safe else
                       "两段式：计划排蓝面侧确认/射击停靠位；实弹由在飞 fire_guard "
                       "按实时画面复判红蓝（红面/未辨 → 禁火）"
                       + (f"；本次视频观测到 {track_obs} → 该停靠位仅作确认，"
                          f"静态回放（无实时感知）时用 --replay-safe 剔除" if track_obs else "")),
                   # 机器级契约：True = 路线含轨道靶射击停靠位（'*'），必须由
                   # 在飞 fire_guard 实时复判红蓝后才能实弹；下游（机载/车端/
                   # 回放工具）见到 True 且自身无实时感知时必须拒绝执行实弹。
                   "attack_requires_live_gate": any(t.track for t in targets),
                   "static_target_arbitration": ("单靶先验（静靶→桶）"
                                                 if static_target_is_bucket
                                                 else "关闭（--allow-static-targets）")},
        "snapshot": {"mines": mines, "passable": use_passable, "obstacles": obstacles,
                     "targets": [
                         (f"轨道靶@{t.cell.name}（可打击=是；视频观测"
                          f"{'蓝面(敌)' if track_obs.get(t.cell.name) == 'blue' else '红面(友)'}）"
                          if t.track and t.cell.name in track_obs else
                          f"轨道靶@{t.cell.name}（视频未观测，用裁判先验）" if t.track else
                          f"{'蓝' if t.is_blue else '红'}@{t.cell.name}")
                         for t in targets]},
        "route": list(route), "route_len_cells": len(route),
        "route_len_diag": round(length, 2), "turns": turns,
        "compact": list(compact),
        "car_cmds": {"reset": cmds.reset, "route": cmds.route,
                     "shoot_cells": list(cmds.shoot_cells)},
        "check": {"排雷覆盖": _mine_note,
                  "障碍避让": "通过" if not viol else f"穿障{viol}"},
    }
    # 安全提示：两段式口径下射击停靠位是"确认位"，实弹由在飞 fire_guard 复判。
    # 视频观测到红面（友方）时必须显式提醒——静态回放这条链路没有实时感知，
    # 会照着 '*' 对红面开火（误击友方 −5）。replay_safe 下没有 '*'，直接通过。
    if replay_safe:
        route_plan["check"]["友方面禁火"] = (
            "通过（--replay-safe：未排轨道靶停靠/射击位，无 '*' 可走火）")
    elif any(f == "red" for f in track_obs.values()):
        route_plan["check"]["友方面禁火"] = (
            f"两段式：射击位 {list(cmds.shoot_cells)} 为确认位，"
            "实弹由在飞 fire_guard 复判（红面/未辨 → 禁火）；"
            "静态回放请手动去掉该 '*'")
    (out_dir / "route_plan.json").write_text(
        json.dumps(route_plan, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[plan] 路线 {len(route)} 格: {' → '.join(route)}")
    print(f"[plan] 压缩 {len(compact)} 点 | 排雷覆盖 {'✓' if not missing else missing}"
          f" | 避障 {'✓' if not viol else viol}")
    # 终端摘要必须与 check 字段同口径：只打 "排雷覆盖 ✓" 会让 2/8 的态势看起来
    # 完全没问题（用户 2026-10-22 就是看这行 + 渲染图问"这航点缺雷"）。
    if _n_mines < MAX_MINES:
        print(f"[plan][注意] 视频只检出 {_n_mines}/{MAX_MINES} 颗雷（官方基线 "
              f"{MAX_MINES}）——未检出的雷不在路线上，属于漏检风险；"
              f"正式用前请用默认逐帧重跑或人工复核未检出区")
    elif _n_mines > MAX_MINES:
        print(f"[plan][注意] 视频检出 {_n_mines} 颗雷 > 官方基线 {MAX_MINES}——"
              f"必有 {_n_mines - MAX_MINES} 颗误检，先复核再执行车命令")
    if "友方面禁火" in route_plan["check"]:
        print(f"[plan] 友方面禁火：{route_plan['check']['友方面禁火']}")
    _render_plan(snapshot, route, compact, cmds, use_start, use_goal,
                 turns, length, out_dir / "route_render.png")
    return route_plan


#: 渲染字体走 ag_integrate.cjk_font（跨平台候选表 + cjk_ok 降级判定，全仓一份）。
#: 2026-10-21 机上实测：原实现写死 C:/Windows/Fonts/msyh.ttc，在 Orin 上必然落进
#: ImageFont.load_default()（PIL 位图字体，latin-1），画第一个汉字 "墙" 就
#: UnicodeEncodeError——视频侦察在最后一步渲染崩掉，产不出 route_render.png。
#: Noto Sans CJK 是 JetPack 自带（/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc）。


def _render_plan(snapshot: WorldSnapshot, route: tuple[str, ...],
                 compact: tuple[str, ...], cmds, use_start: str, use_goal: str,
                 turns: int, length: float, out_path: Path) -> None:
    from PIL import Image, ImageDraw
    CELL_PX, PAD = 60, 50
    W, H = PAD * 2 + COLS * CELL_PX, PAD * 2 + ROWS * CELL_PX

    def cell(n: str) -> CellRef:
        c, r = parse_cell(n)
        return CellRef(c, r)

    img = Image.new("RGB", (W, H + 34), (30, 30, 30))
    dr = ImageDraw.Draw(img)
    f14, cjk_ok = load_cjk_font(14)
    f10, _ = load_cjk_font(10)
    # 无 CJK 字体时的 ASCII 标签（含义与中文一一对应，图例同步切换）
    T = (lambda zh, en: zh) if cjk_ok else (lambda zh, en: en)

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

    # 中场挡板（第 6/7 行之间、D~N 列；A/B/C 西走廊可自由穿越）——画出来，
    # 否则"路线为什么绕这么大圈"在图上无从核对（2026-10-21 用户提问）。
    # 真源是 mission_router._MID_BOUNDARY / _BARRIER_COLS，import 而非另写一份。
    _by = PAD + _MID_BOUNDARY * CELL_PX
    _bx0 = PAD + min(_BARRIER_COLS) * CELL_PX
    _bx1 = PAD + (max(_BARRIER_COLS) + 1) * CELL_PX
    dr.rectangle([_bx0, _by - 3, _bx1, _by + 3], fill=(120, 120, 130))
    for _x in range(_bx0, _bx1, 14):                       # 斜线 = 实体挡板
        dr.line([(_x, _by + 3), (_x + 6, _by - 3)], (170, 170, 180), 1)
    dr.text((_bx1 - 88, _by + 8), T("中场挡板 D~N", "barrier D~N"),
            (190, 190, 200), f10)

    for name in [c.name for c in snapshot.obstacles]:
        x, y = xy(cell(name))
        dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (180, 40, 40))
        dr.text((x + CELL_PX // 2, y + CELL_PX // 2), T("障", "OBS"), "white", f14, "mm")
    for r in range(ROWS):
        name = f"{BRIDGE_COL}{r + 1}"
        x, y = xy(cell(name))
        if name in {c.name for c in snapshot.passable}:
            dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (80, 60, 120))
            dr.text((x + CELL_PX // 2, y + CELL_PX // 2), T("洞", "GATE"), "white", f14, "mm")
        else:
            dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (60, 60, 60))
            dr.text((x + CELL_PX // 2, y + CELL_PX // 2), T("墙", "WALL"),
                    (150, 150, 150), f10, "mm")
    for name in [c.name for c in snapshot.mines]:
        x, y = xy(cell(name))
        dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (220, 200, 30))
        dr.text((x + CELL_PX // 2, y + CELL_PX // 2 - 6), T("雷", "MINE"), "black", f14, "mm")
    for t in snapshot.targets:
        x, y = xy(t.cell)
        dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (95, 75, 140))
        dr.text((x + CELL_PX // 2, y + CELL_PX // 2 - 6),
                T("蓝", "BLU") if t.is_blue else T("红", "RED"), "white", f14, "mm")

    def grad(t: float) -> tuple[int, int, int]:
        if t < 0.5:
            return (int(255 * t * 2), 255, 0)
        return (255, int(255 * (1 - (t - 0.5) * 2)), 0)

    rc = [cell(n) for n in route]
    for i in range(len(rc) - 1):
        x1, y1 = xy(rc[i]); x2, y2 = xy(rc[i + 1])
        dr.line([(x1 + CELL_PX // 2, y1 + CELL_PX // 2), (x2 + CELL_PX // 2, y2 + CELL_PX // 2)],
                grad(i / max(len(rc) - 2, 1)), 4)
    # 跨中场挡板的那几步单独圈出来（合法性一眼可查：只许西走廊 A/B/C 或桥洞）
    for i in range(len(rc) - 1):
        a, b = rc[i], rc[i + 1]
        if ((a.row < _MID_BOUNDARY <= b.row) or (b.row < _MID_BOUNDARY <= a.row)):
            x1, y1 = xy(a); x2, y2 = xy(b)
            mx = (x1 + x2) // 2 + CELL_PX // 2
            my = (y1 + y2) // 2 + CELL_PX // 2
            dr.ellipse([mx - 9, my - 9, mx + 9, my + 9], outline=(0, 255, 255), width=2)
            dr.text((mx, my - 20), T(f"跨挡板 {a.name}→{b.name}", f"cross {a.name}->{b.name}"),
                    (0, 255, 255), f10, "mm")
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
    dr.text((sx + CELL_PX // 2, sy + CELL_PX - 8), T("起点", "START"), (0, 255, 0), f10, "mm")
    dr.rectangle([gx + 4, gy + 4, gx + CELL_PX - 4, gy + CELL_PX - 4], outline=(255, 100, 0), width=3)
    dr.text((gx + CELL_PX // 2, gy + CELL_PX - 8), T("终点", "GOAL"), (255, 100, 0), f10, "mm")
    if cjk_ok:
        dr.text((PAD, H - 18),
                "绿→黄→红=路径先后  雷=视频识别  障/墙/洞/靶=态势  ⊕=停靠射击", "white", f10)
        dr.text((PAD, H + 2), f"路线 {len(route)} 格 / 压缩 {len(compact)} 点 / 转向 {turns} 次"
                f" / 直线长 {length:.1f} 格  起点{use_start}→终点{use_goal}", (255, 230, 130), f10)
    else:
        # 无 CJK 字体（本机未装/精简镜像）：整图退化成 ASCII，绝不半中半英。
        dr.text((PAD, H - 18),
                "G->Y->R=path order  MINE=from video  OBS/WALL/GATE/TGT=state  +=shoot stop",
                "white", f10)
        dr.text((PAD, H + 2), f"route {len(route)} cells / {len(compact)} keep pts / "
                f"{turns} turns / len {length:.1f} cells  {use_start}->{use_goal}",
                (255, 230, 130), f10)
    img.save(out_path)
    print(f"[render] {out_path}")


# ---------------------------------------------------------------- CLI
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="赛前视频场地侦察：D13+v4d 识别 → 态势 → 路线规划（全机械化）")
    ap.add_argument("video", nargs="?", default=None, type=str)
    ap.add_argument("--stride", type=int, default=5,
                    help="采样帧距（默认 5 = 实测最优点：8/8 基线；15 漏检到 3 雷，"
                         "1 过密到 14 雷含误检——两端都会掉精度）")
    # 门控/裁决规则改动后不必重跑推理：frames_meta.json 已含逐帧投影结果，
    # 用它重算态势（秒级）。这既避免整段重跑，也保证"重算"与"重跑"用的是
    # 同一份门控代码。
    ap.add_argument("--reaggregate", type=str, default=None, metavar="DIR",
                    help="不跑识别：用 DIR/frames_meta.json 按当前门控重算态势并重出规划")
    ap.add_argument("--imgsz", type=int, default=IN_SIZE, choices=(640, 1280),
                    help="推理分辨率，必须与模型导出一致（1280 版 33600 anchors，"
                         "640 版 8400）；640 是 10-17 实测召回更高的离线配置")
    ap.add_argument("--out", type=str, default=None)
    ap.add_argument("--max-frames", type=int, default=0, help="调试：只处理前 N 个采样帧")
    ap.add_argument("--d13", type=str, default=None, help="hbb_d13.onnx 路径")
    ap.add_argument("--v4d", type=str, default=None, help="cell_classifier_v4d.onnx 路径")
    # 必须真注册：此前失败提示里写着"加 --debug"，但 argparse 不认这个参数，
    # 照做只会得到 "unrecognized arguments: --debug"（2026-10-21 实测）。
    ap.add_argument("--debug", action="store_true",
                    help="报错时打印完整堆栈（默认只给结论 + 处理建议）")
    # 默认按裁判地图"全场唯一靶=移动双靶"裁决：视频检出的静靶判为桶（物证见
    # reclassify_targets）。练习场若真摆了静靶（非官方布置）再用本开关关掉。
    ap.add_argument("--allow-static-targets", action="store_true",
                    help="不把视频检出的静靶改判为桶（练习场确实摆了静靶时才用）")
    # 静态回放安全：两段式的 '*' 依赖在飞 fire_guard 实时复判红蓝；回放链路
    # 没有实时感知，必须机器级剔除轨道靶射击停靠位（替代"手动去掉 '*'"）。
    ap.add_argument("--replay-safe", action="store_true",
                    help="静态回放模式：不排轨道靶停靠/射击位（无实时红蓝复判，"
                         "照 '*' 开火会误击红面友方）")
    args = ap.parse_args(argv)
    if args.stride < 1:
        ap.error(f"--stride 必须 >= 1（收到 {args.stride}）")
    if not args.reaggregate and not args.video:
        ap.error("缺少 video（或用 --reaggregate DIR 从已有识别结果重算）")

    out_dir = Path(args.out) if args.out else Path(
        f"视频侦察_{datetime.now().strftime('%Y%m%d')}")
    # 现场用法：报错要能照着做，不要甩一屏 traceback（机上 VNC 里翻不动）。
    # 保留原始异常链：--debug 时打印完整堆栈。
    try:
        if args.reaggregate:
            out_dir = Path(args.reaggregate)
            meta_path = out_dir / "frames_meta.json"
            if not meta_path.is_file():
                raise FileNotFoundError(
                    f"--reaggregate 需要 {meta_path}（逐帧识别明细），该目录没有。\n"
                    f"  它是每次识别都会落盘的产物；没有说明这个目录不是识别结果目录，"
                    f"或来自旧版本脚本——请重跑一次识别：video_recon.py <视频>")
            frames_meta = json.loads(meta_path.read_text(encoding="utf-8"))
            # 沿用原态势记录的来源信息（视频路径/帧距），避免重算后来源字段失真
            prev_path = out_dir / "field_state.json"
            prev = (json.loads(prev_path.read_text(encoding="utf-8"))
                    if prev_path.is_file() else {})
            print(f"[reagg] 用 {meta_path}（{len(frames_meta)} 帧）按当前门控重算态势")
            state = aggregate_state(frames_meta,
                                    video=prev.get("video", str(args.video or "")),
                                    stride=int(prev.get("stride", args.stride)))
            prev_path.write_text(json.dumps(state, ensure_ascii=False, indent=2),
                                 encoding="utf-8")
        else:
            state = recon_video(args.video, out_dir, args.d13, args.v4d,
                                args.stride, args.max_frames, imgsz=args.imgsz)
        plan_from_state(state, out_dir,
                        static_target_is_bucket=not args.allow_static_targets,
                        replay_safe=args.replay_safe)
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(f"\n[失败] {exc}", file=sys.stderr)
        # 用 args.debug 而不是 "--debug" in sys.argv：main(argv) 被显式传参调用时
        # （测试、脚本内调用）sys.argv 里根本没有这个 token，判定会静默失效。
        if args.debug or os.environ.get("AG_DEBUG"):
            raise
        print("  产物目录（若已建）: "
              f"{out_dir.resolve() if out_dir.exists() else '(未创建)'}",
              file=sys.stderr)
        print("  加 --debug 或设 AG_DEBUG=1 可看完整堆栈", file=sys.stderr)
        return 2
    print(f"[done] 全部产物在 {Path.cwd() / out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
