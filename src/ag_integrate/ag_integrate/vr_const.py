# vr_const —— 由 video_recon.py 拆出（检测类别/阈值/场地默认值常量）。与原文件逐行一致（AST 校验过）。
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
         "car_start", "car_end", "uav_start", "uav_end", "target"]
# 2026-10-26 ft_v1 起第 14 类：target=靶板+导轨整体（id 13，与协议 TARGET=13 对齐）；
# target_red/blue 语义改为"可见靶面"，继续驱动敌我判定投票。
CONF = {  # 离线侦察工作点（paper 0.15 + 投票门控；默认 stride=5 实测最优点）
    # 与机载 C++ kDeployConf（paper=0.25、cell_label=0.50）**不同源**：机载是
    # 实时单帧无时序投票，必须收紧；离线有跨帧投票，放宽才不丢单帧可见的目标。
    # 实测教训（同一段视频，官方基线 8 雷）：stride 15→3 雷（漏检）、5→8 雷、
    # 1→14 雷（弱误检累积票后混入）。默认 5 是召回/精度的实测平衡点。
    "paper": 0.15, "cell_label": 0.35, "car": 0.25,
    "target_red": 0.25, "target_blue": 0.25, "target": 0.25, "obstacle": 0.40,
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
# 幻影源强格的最小自召回：召回低于它时"从未同帧共现"不构成证据
# （2026-10-23 尸检：G9 召回 13/71=18%，弱格 2 帧里 G9 恰好没检出的
#  概率 ~67%；当天 G5(2票 0.37)/I8(2票 0.46) 因此被误杀，8 雷只剩 6）
PHANTOM_STRONG_RECALL_MIN = 0.50
# 单票进入"待人工复核"的置信度下限（0.85+ 的 paper 单框极少是人体/阴影
# 误检；静默丢弃等于扔真雷证据，直接确认又是最薄证据上的硬判决）
REVIEW_CONF_MIN = 0.80
# --sahi 切片推理：瓦片 640 → 1280 推理 = 2× 有效放大（2026-10-23 引入。
# 该航拍域全帧 1280x720→1280 推理 scale=1.0 无放大，单帧召回仅 10~20%；
# 瓦片放大针对小目标，SAHI 口径 +12~25% AP。代价：推理次数 ×瓦片数，
# 只给离线链，机载实时链不用）
SAHI_TILE = 640
SAHI_OVERLAP = 0.25
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
LABEL_OVERLAP_IOU = 0.30
