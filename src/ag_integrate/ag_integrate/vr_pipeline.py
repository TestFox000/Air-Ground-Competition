# vr_pipeline —— 由 video_recon.py 拆出（视频→态势→规划→渲染主管线与 CLI main）。与原文件逐行一致（AST 校验过）。
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
from vr_const import BLUE_FACE_DFT, BRIDGE_COL, CARRY_MAX_STEPS, GOAL_DFT, IN_SIZE, OBJ_CLASSES, PAPER_MEAN_CONF_MIN, PASSABLE_DFT, SAHI_OVERLAP, SAHI_TILE, START_DFT, TARGET_MEAN_CONF_MIN, TARGET_MIN_VOTES, TRACK_DFT, TRACK_NEIGHBOR_R, V4D_MIN_CONF, VOTE_MIN
from vr_geom import anchor_within, cell_name, circular_mode, confirmed_cells, eliminate_phantoms, fit_consensus_affine, imwrite_u, mine_review_candidates, parse_cell, resolve_cell_conflicts, snap_cell, split_mines_passable, validate_affine_carry
from vr_models import D13Detector, D13_SEARCH, V4D_SEARCH, V4dReader, _iter_model_candidates, _open_first_valid

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
                keep_annots: int = 8, keep_crops: int = 40,
                sahi: bool = False) -> dict:
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
    detect_fn = d13.detect_sahi if sahi else d13.detect
    if sahi:
        print(f"[recon] 切片推理: 瓦片 {SAHI_TILE} 重叠 {int(SAHI_OVERLAP*100)}%"
              f"（≈{d13.imgsz}/{SAHI_TILE:.0f}× 有效放大，慢但小目标召回高）")
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
    seq_mode = False
    if total <= 0 or total > 10**9:
        # 机载 MKV（MJPEG 直存无索引）会让 FRAME_COUNT 返回垃圾值（实测
        # -2.7e17，2026-10-25）。回退：grab() 顺序数一遍帧数，且整跑改为
        # 顺序采样——这类 MKV 上 POS_FRAMES 定位会取到错误帧（同日抽帧
        # 验证过 seek 错位）；索引正常的 MP4 仍走原 seek 路径。
        cap.release()
        cap = cv2.VideoCapture(str(video))
        total = 0
        while cap.grab():
            total += 1
        cap.release()
        cap = cv2.VideoCapture(str(video))
        seq_mode = total > 0
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
        per = 4.5 if sahi else 1.5     # 实测全帧约 1.5 秒/帧；切片 6 瓦片约 4.5
        print(f"[recon] 预计约 {len(idxs) * per / 60:.0f} 分钟（约 {per} 秒/帧）")

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
    next_read = 0                       # 顺序模式：已读到的下一帧号
    for fi, fidx in enumerate(idxs):
        if seq_mode:
            # 顺序读到采样点（无索引 MKV 上 seek 不可靠，见上）
            ok, frame = cap.read()
            while ok and next_read < fidx:
                next_read += 1
                ok, frame = cap.read()
            if ok:
                next_read += 1
        else:
            cap.set(cv2.CAP_PROP_POS_FRAMES, fidx)
            ok, frame = cap.read()
        if not ok:
            continue
        try:
            dets = detect_fn(frame)
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
    # confirmed_cells 注释的 G9 事故）；幻影消除带 coverage（召回前置门，
    # 见 eliminate_phantoms 注释的 G5/I8 误杀事故）。
    paper_cells = confirmed_cells(["paper"], votes, vconf, vframes,
                                  VOTE_MIN, PAPER_MEAN_CONF_MIN, phantom_kill=True,
                                  strong_votes=3, coverage=dict(coverage))
    mines, passable = split_mines_passable(paper_cells)
    obstacles = confirmed_cells(["obstacle", "bucket_red", "bucket_green", "bucket_cyan"],
                                votes, vconf, vframes)
    tgt_red = confirmed_cells(["target_red"], votes, vconf, vframes,
                              TARGET_MIN_VOTES, TARGET_MEAN_CONF_MIN)
    tgt_blue = confirmed_cells(["target_blue"], votes, vconf, vframes,
                               TARGET_MIN_VOTES, TARGET_MEAN_CONF_MIN)
    # 整体板票（ft_v1 第 14 类）：只记录供审计/打击点定位，不参与单格裁决——
    # 敌我判定与静靶仲裁仍由面类票驱动，避免在 14 类切换日改变规划输入。
    tgt_asm = confirmed_cells(["target"], votes, vconf, vframes,
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
    # 高conf单票待复核（只提示，不进雷表；见 mine_review_candidates 注释）
    mines_review = mine_review_candidates(
        votes.get("paper", Counter()), vconf.get("paper", {}),
        set(obstacles) | set(tgt_red) | set(tgt_blue))
    car_start = confirmed_cells(["car_start"], votes, vconf, vframes, 2)
    car_end = confirmed_cells(["car_end"], votes, vconf, vframes, 2)
    uav_marks = {cls: confirmed_cells([cls], votes, vconf, vframes, 2)
                 for cls in ("uav_start", "uav_end")}

    _, phantom_log = eliminate_phantoms(
        [c for c, v in votes["paper"].items() if v >= VOTE_MIN],
        votes["paper"], vframes.get("paper", {}), coverage=dict(coverage))

    n_ok = sum(1 for m in frames_meta if m["grid_ok"])
    field_state = {
        "video": video, "stride": stride,
        "frames_sampled": len(frames_meta), "frames_grid_ok": n_ok,
        "frac_offset": [round(fc, 3), round(fr, 3)],
        "mines_video": mines, "passable_video": passable,
        "mines_review": mines_review,   # 高conf单票待人工复核（非雷、不挡路线）
        "phantoms_eliminated": phantom_log,
        "obstacles_video": obstacles,
        "targets_video": {"red": tgt_red, "blue": tgt_blue, "assembly": tgt_asm},
        "car_start_video": car_start, "car_end_video": car_end,
        "uav_marks_video": uav_marks,
        "conflicts": _conflicts,          # 单格多类命中的人工复核明细
        "votes_detail": {cls: dict(cnt.most_common()) for cls, cnt in votes.items()},
        "coverage": dict(coverage.most_common()),
    }
    print(f"[recon] 态势: 雷 {mines} | K列箭头 {passable} | 障碍 {obstacles}"
          f" | 靶 红{tgt_red} 蓝{tgt_blue} | 幻影 {phantom_log}")
    if mines_review:
        print(f"[recon][复核] 高conf单票待人工复核（不是雷、不挡路线，抽 "
              f"label_crops 定真假）: "
              f"{[(r['cell'], r['conf']) for r in mines_review]}")
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
    # 切片推理（2026-10-23）：720p 全帧 1280 推理无放大，小目标召回低；
    # 瓦片 640→1280 = 2× 放大换召回（代价约 3 倍耗时）。缓存目录要加 -sahi
    # 后缀（orin_video_recon.sh 已按本 flag 区分），与全帧缓存不混。
    ap.add_argument("--sahi", action="store_true",
                    help="切片推理：瓦片放大换小目标召回（慢约 3 倍，离线用）")
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
                                args.stride, args.max_frames, imgsz=args.imgsz,
                                sahi=args.sahi)
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
