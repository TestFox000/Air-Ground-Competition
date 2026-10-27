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
     剔除。同帧共现 = 两个独立物体，保留。强格自召回 <50% 时禁用
     （2026-10-23 尸检：召回 18% 时"未共现"是噪声，G5/I8 真雷被误杀）。
  5b. 高conf(>=0.80)单票不静默丢弃 → "待人工复核"清单（只提示不进雷表）。
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

# 2026-10-27 模块化拆分：实现移入同目录 vr_const/vr_geom/vr_models/vr_pipeline（扁平互导）。
# 双模式兼容：包导入（from ag_integrate import video_recon）与按路径直跑（python3 .../video_recon.py，机载 orin_*.sh 用）都成立——
# 门面先把本目录推上 sys.path，再用扁平导入再导出全部名字（含 _ 私有名）。
import os as _os, sys as _sys
_here = _os.path.dirname(_os.path.abspath(__file__))
if _here not in _sys.path:
    _sys.path.insert(0, _here)  # 扁平互导在包模式下也可解析

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
from vr_pipeline import _BARRIER_COLS, _MID_BOUNDARY, COLS, CellRef, LETTERS, MAX_MINES, ROWS, TargetRef, WorldSnapshot, compact_and_adapt, load_cjk_font, plan_route  # noqa: F401  # 原头部名再导出（旧代码可能借道 vr.plan_route 等）

from vr_const import ANCHOR_GATE, BLUE_FACE_DFT, BRIDGE_COL, BRIDGE_COL_IDX, CARRY_INLIER, CARRY_MAX_STEPS, CONF, FIT_INLIER, GOAL_DFT, GRID_MAE_MAX, IN_SIZE, LABEL_OVERLAP_IOU, MAX_VOTE_RESIDUAL, NAMES, NMS_IOU, OBJ_CLASSES, PAPER_MEAN_CONF_MIN, PASSABLE_DFT, PHANTOM_STRONG_RECALL_MIN, REVIEW_CONF_MIN, SAHI_OVERLAP, SAHI_TILE, START_DFT, TARGET_MEAN_CONF_MIN, TARGET_MIN_VOTES, TRACK_DFT, TRACK_NEIGHBOR_R, V4D_IN, V4D_MIN_CONF, V4D_PAD, VOTE_MIN  # noqa: F401
from vr_geom import _iou, _nms_merge, anchor_within, cell_name, circular_mode, confirmed_cells, eliminate_phantoms, filter_label_overlap, fit_consensus_affine, imwrite_u, mine_review_candidates, parse_cell, resolve_cell_conflicts, slice_tiles, snap_cell, split_mines_passable, validate_affine_carry  # noqa: F401
from vr_models import D13Detector, D13_SEARCH, V4D_SEARCH, V4dReader, _find_model, _iter_model_candidates, _model_from_entry, _newest_by_glob, _open_first_valid  # noqa: F401
from vr_pipeline import _render_plan, _write_annot, aggregate_state, arbitrate_targets, main, merge_passable_with_default, plan_from_state, recon_video  # noqa: F401

if __name__ == "__main__":
    raise SystemExit(main())
