# vr_models —— 由 video_recon.py 拆出（D13/V4D 推理引擎与模型发现）。与原文件逐行一致（AST 校验过）。
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
from vr_const import CONF, IN_SIZE, NAMES, NMS_IOU, SAHI_OVERLAP, SAHI_TILE, V4D_IN, V4D_PAD
from vr_geom import _nms_merge, filter_label_overlap, slice_tiles

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

    def detect_sahi(self, img_bgr: np.ndarray) -> list[dict]:
        """切片推理（--sahi）：每瓦片独立 detect 后平移回全图坐标，跨瓦片 NMS。

        瓦片 640 letterbox 到 imgsz=1280 = 2× 有效放大——全帧推理对 720p 是
        scale=1.0 原尺寸直推，小目标（雷贴纸/远距格号）特征不足是单帧召回
        10~20% 的主因之一。代价：推理次数 ×瓦片数（约 ×3~4 秒/帧），只给
        离线侦察用。图比瓦片还小（≤1 片）时自动退回全帧。"""
        h, w = img_bgr.shape[:2]
        tiles = slice_tiles(w, h, SAHI_TILE, SAHI_OVERLAP)
        if len(tiles) <= 1:
            return self.detect(img_bgr)
        merged: list[dict] = []
        for x1, y1, x2, y2 in tiles:
            for d in self.detect(img_bgr[y1:y2, x1:x2]):
                d["box"] = [d["box"][0] + x1, d["box"][1] + y1,
                            d["box"][2] + x1, d["box"][3] + y1]
                merged.append(d)
        # 合并后再过一遍标签重叠过滤：瓦片级的 label 框平移回来后，可能
        # 与另一瓦片检出的 paper 框构成"标签文字误检"关系（全帧推理里这层
        # 在 detect() 内部完成，切片链必须在合并后重做）。
        return filter_label_overlap(_nms_merge(merged))
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
