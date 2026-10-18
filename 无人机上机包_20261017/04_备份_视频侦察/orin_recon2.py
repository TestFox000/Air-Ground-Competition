# -*- coding: utf-8 -*-
"""Orin 一键视频侦察 v2：四杠杆集成。

① 引擎优先级 int8_640 > fp16_640 > fp16_1280（输入尺寸从引擎动态读取）
② stride 默认 15（125 帧）
③ v4d 门控：仅对 ≥3 个标签的帧读格号
④ INT8 引擎支持

用法（Orin 开机后）：
  set AG_ORIN_PASS=xxx
  python orin_recon2.py 视频.mp4 --host <无人机IP> [--stride 15]
"""
from __future__ import annotations

import argparse
import json
import os
import time
from collections import Counter, defaultdict
from itertools import combinations
from pathlib import Path

import cv2
import numpy as np
import paramiko

BASE = Path(__file__).resolve().parent
V4D_ONNX = str(BASE / "cloud_train_20261015" / "deploy_pkg" / "cell_classifier_v4d_20261015.onnx")
ENGINES = ["~/dzuav/engines/hbb_d13_int8_640.engine",
           "~/dzuav/engines/hbb_d13_fp16_640.engine",
           "~/dzuav/engines/hbb_d13_fp16.engine"]
NAMES = ["paper", "cell_label", "car", "target_red", "target_blue", "obstacle",
         "bucket_red", "bucket_green", "bucket_cyan",
         "car_start", "car_end", "uav_start", "uav_end"]
CONF = [0.22, 0.35, 0.25, 0.25, 0.25, 0.40, 0.40, 0.40, 0.40, 0.25, 0.25, 0.25, 0.25]
NMS_IOU = 0.45
LETTERS = "ABCDEFGHIJKLMN"
COLS, ROWS = 14, 11
V4D_IN, V4D_MIN_CONF, V4D_PAD = 96, 0.65, 1.3
FIT_INLIER, GRID_MAE_MAX, ANCHOR_GATE, MAX_VOTE_RESIDUAL = 0.45, 0.45, 1.5, 0.35
OFFICIAL = {"mines": {"B1", "A2", "I2", "C3", "E3", "F5", "H8", "G9"},
            "obstacles": {"D5", "I6", "B7", "F8", "C9", "E10", "G10", "A11"},
            "passable": {"K4", "K10"}}
BRIDGE_COL_IDX = LETTERS.index("K")

REMOTE = r'''#!/usr/bin/env python3
# Orin 端：环境探测 + 引擎优先级选择 + D13 推理。输出 JSONL。
import json, os, sys, time

ENV = {}
def probe(name, imp):
    try:
        m = __import__(imp)
        ENV[name] = getattr(m, "__version__", "ok")
        return m
    except Exception:
        ENV[name] = None
        return None

trt = probe("tensorrt", "tensorrt")
cuda = probe("cuda_python", "cuda")
pycuda = None
if not cuda:
    pycuda = probe("pycuda", "pycuda.driver")
cv2 = probe("opencv", "cv2")

CAND = ["/".join([os.path.expanduser("~"), "dzuav", "engines", n]) for n in
        ["hbb_d13_int8_640.engine", "hbb_d13_fp16_640.engine", "hbb_d13_fp16.engine"]]
ENGINE = next((e for e in CAND if os.path.isfile(e)), None)
ENV["engine"] = ENGINE and ENGINE.split("/")[-1]

if not (trt and (cuda or pycuda) and cv2 and ENGINE):
    print(json.dumps({"env": ENV, "fatal": "环境/引擎缺口见 env"}))
    sys.exit(3)

import numpy as np, cv2, tensorrt as trt

if cuda:
    from cuda import cuda as cu
    cu.cuInit(0)
    class S: pass
    s = S()
    s.malloc = lambda n: cu.cuMemAlloc(n)[1]
    s.free = lambda p: cu.cuMemFree(p)
    s.h2d = lambda h, d, n: cu.cuMemcpyHtoD(d, h, n)
    s.d2h = lambda d, h, n: cu.cuMemcpyDtoH(h, d, n)
else:
    import pycuda.driver as drv
    drv.init()
    class S: pass
    s = S()
    s.malloc = lambda n: drv.mem_alloc(n)
    s.free = lambda p: p.free()
    s.h2d = lambda h, d, n: drv.memcpy_htod(d, h, n)
    s.d2h = lambda d, h, n: drv.memcpy_dtoh(h, d, n)

runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
with open(ENGINE, "rb") as f:
    engine = runtime.deserialize_cuda_engine(f.read())
ctx = engine.create_execution_context()

try:  # TRT 8/9 兼容
    names = [engine.get_binding_name(i) for i in range(engine.num_bindings)]
    inp, outp = names[0], names[-1]
    oshape = tuple(engine.get_binding_shape(outp))
    ishape = tuple(engine.get_binding_shape(inp))
except AttributeError:
    names = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)]
    inp = [n for n in names if engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT][0]
    outp = [n for n in names if engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT][0]
    oshape = tuple(engine.get_tensor_shape(outp))
    ishape = tuple(engine.get_tensor_shape(inp))
IN_SIZE = ishape[-1]
ENV["in_size"] = IN_SIZE

IN = np.empty(ishape, dtype=np.float32)
OUT = np.empty(oshape, dtype=np.float32)
d_in, d_out = s.malloc(IN.nbytes), s.malloc(OUT.nbytes)

def run(blob):
    s.h2d(blob.tobytes(), d_in, IN.nbytes)
    try:
        ctx.execute_v2([int(d_in), int(d_out)])
    except AttributeError:
        ctx.set_tensor_address(inp, int(d_in)); ctx.set_tensor_address(outp, int(d_out))
        ctx.execute_async_v3()
    s.d2h(d_out, OUT, OUT.nbytes)

NAMES13 = NAMES
CONF13 = CONF

video, stride = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 15
cap = cv2.VideoCapture(video)
total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
t0 = time.time()
print(json.dumps({"env": ENV, "start": True, "total": total}))
n = 0
for fidx in range(0, total, stride):
    cap.set(cv2.CAP_PROP_POS_FRAMES, fidx)
    ok, frame = cap.read()
    if not ok:
        continue
    h, w = frame.shape[:2]
    sc = IN_SIZE / max(h, w)
    nw, nh = round(w * sc), round(h * sc)
    canvas = np.full((IN_SIZE, IN_SIZE, 3), 114, np.uint8)
    px, py = (IN_SIZE - nw) // 2, (IN_SIZE - nh) // 2
    canvas[py:py + nh, px:px + nw] = cv2.resize(frame, (nw, nh))
    IN[0] = cv2.dnn.blobFromImage(canvas, 1 / 255.0, (IN_SIZE, IN_SIZE), swapRB=True)[0]
    run(IN)
    o = OUT.reshape(17, -1)
    boxes, scores = o[:4, :].T, o[4:, :]
    dets = []
    for c in range(13):
        th = CONF13[c]
        sv = scores[c]
        m = sv > th
        if not m.any():
            continue
        b, scc = boxes[m], sv[m]
        xywh = np.stack([(b[:, 0] - b[:, 2] / 2 - px) / sc,
                         (b[:, 1] - b[:, 3] / 2 - py) / sc,
                         b[:, 2] / sc, b[:, 3] / sc], axis=1)
        keep = cv2.dnn.NMSBoxes(xywh.tolist(), scc.tolist(), th, 0.45)
        for i in np.array(keep).flatten():
            x, y, ww, hh = xywh[i]
            dets.append([NAMES13[c], float(scc[i]),
                         float(x), float(y), float(x + ww), float(y + hh)])
    print(json.dumps({"frame": fidx, "dets": dets}))
    n += 1
cap.release()
print(json.dumps({"env": ENV, "done": n, "seconds": round(time.time() - t0, 1)}))
'''


def fit_consensus(px, grid):
    n = len(px)
    if n < 3:
        return None, np.zeros(n, bool), 9.9
    x = np.hstack([px, np.ones((n, 1))])
    best = None
    for combo in combinations(range(n), 3):
        idx = list(combo)
        if np.linalg.matrix_rank(x[idx]) < 3:
            continue
        try:
            a, *_ = np.linalg.lstsq(x[idx], grid[idx], rcond=None)
        except np.linalg.LinAlgError:
            continue
        r = np.linalg.norm(x @ a - grid, axis=1)
        inl = r < FIT_INLIER
        if inl.sum() < 3:
            continue
        try:
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--host", required=True)
    ap.add_argument("--user", default="duzhong")
    ap.add_argument("--stride", type=int, default=15)
    ap.add_argument("--out", default="全场侦察_Orin")
    args = ap.parse_args()
    pw = os.environ.get("AG_ORIN_PASS")
    if not pw:
        print("[err] 先 set AG_ORIN_PASS=...")
        return 1
    out_dir = BASE / args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()

    print(f"[orin] 连接 {args.user}@{args.host} ...")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(args.host, username=args.user, password=pw, timeout=15)
    sftp = c.open_sftp()
    rvideo = f"/tmp/recon_{int(time.time())}.mp4"
    print(f"[orin] 上传 {args.video} -> {rvideo}")
    sftp.put(str(args.video), rvideo)
    with sftp.open("/tmp/orin_recon2_remote.py", "w") as f:
        f.write(REMOTE)
    sftp.close()

    print("[orin] 推理（引擎优先级 int8_640 > fp16_640 > fp16_1280）...")
    _, stdout, stderr = c.exec_command(
        f"python3 /tmp/orin_recon2_remote.py {rvideo} {args.stride}", timeout=1800)
    lines = stdout.read().decode("utf-8", "replace").splitlines()
    c.close()

    env_info, frames = None, {}
    for ln in lines:
        try:
            j = json.loads(ln)
        except json.JSONDecodeError:
            continue
        if "env" in j:
            env_info = j
        if "frame" in j:
            frames[j["frame"]] = j["dets"]
    if not env_info:
        print(f"[err] 无输出: {stderr.read().decode('utf-8', 'replace')[:400]}")
        return 2
    env = env_info.get("env", {})
    print(f"[orin] 引擎 {env.get('engine')} 输入{env.get('in_size')} | "
          f"{env_info.get('done')} 帧 / {env_info.get('seconds')}s")
    if env_info.get("fatal"):
        print(f"[可行性缺口] {env_info['fatal']}")
        return 3

    # ---- 本地：v4d 门控读格号（杠杆③：只读 ≥3 标签的帧）+ 聚合 ----
    print("[local] v4d（门控）+ 共识几何 ...")
    vnet = cv2.dnn.readNetFromONNX(V4D_ONNX)
    vouts = vnet.getUnconnectedOutLayersNames()

    def sm(x):
        e = np.exp(x - x.max())
        return e / e.sum()

    def v4d_read(crop):
        best = None
        for k in range(4):
            cc = np.rot90(crop, k) if k else crop
            vnet.setInput(cv2.dnn.blobFromImage(
                cv2.resize(cc, (V4D_IN, V4D_IN)), 1 / 255.0,
                (V4D_IN, V4D_IN), swapRB=True))
            lg, rg = vnet.forward(vouts)
            lp, rp = sm(lg[0]), sm(rg[0])
            li, ri = int(lp.argmax()), int(rp.argmax())
            cand = (min(float(lp[li]), float(rp[ri])), li, ri)
            if best is None or cand[0] > best[0]:
                best = cand
        return best

    cap = cv2.VideoCapture(args.video)
    frames_meta = []
    n_skip = 0
    for fidx in sorted(frames):
        dets = frames[fidx]
        if sum(1 for d in dets if d[0] == "cell_label") < 3:  # 门控
            n_skip += 1
            continue
        cap.set(cv2.CAP_PROP_POS_FRAMES, fidx)
        ok, frame = cap.read()
        if not ok:
            continue
        reads = []
        for cls, conf, x1, y1, x2, y2 in dets:
            if cls != "cell_label":
                continue
            ccx, ccy = (x1 + x2) / 2, (y1 + y2) / 2
            side = max(x2 - x1, y2 - y1) * V4D_PAD
            crop = frame[max(0, int(ccy - side / 2)):int(ccy + side / 2),
                         max(0, int(ccx - side / 2)):int(ccx + side / 2)]
            if crop.size == 0:
                continue
            r = v4d_read(crop)
            if r and r[0] >= V4D_MIN_CONF:
                reads.append((r[1], r[2], [x1, y1, x2, y2]))
        frames_meta.append({"frame": fidx, "reads": reads, "dets": dets})
    cap.release()
    print(f"[local] v4d 门控：读取 {len(frames_meta)} 帧 / 跳过 {n_skip} 帧")

    def circular_mode(vals, bins=20):
        if not vals:
            return 0.5
        hist, edges = np.histogram(np.array(vals) % 1.0, bins=bins, range=(0, 1))
        ext = np.concatenate([hist[-1:], hist, hist[:1]])
        k = int(np.argmax(ext[1:-1]))
        return (edges[k] + edges[k + 1]) / 2

    metas, cont = [], defaultdict(list)
    for m in frames_meta:
        reads = m["reads"]
        A = inl = None
        if len(reads) >= 3:
            px = np.array([[(b[0] + b[2]) / 2, (b[1] + b[3]) / 2] for _, _, b in reads])
            grid = np.array([[li, ri] for li, ri, _ in reads], float)
            A, inl, _ = fit_consensus(px, grid)
            if A is not None:
                reads = [r for r, k in zip(reads, inl) if k]
        fobjs = []
        if A is not None:
            for cls, conf, x1, y1, x2, y2 in m["dets"]:
                if cls == "cell_label":
                    continue
                p = np.array([(x1 + x2) / 2, (y1 + y2) / 2, 1.0]) @ A
                qc, qr = float(p[0]), float(p[1])
                if -0.5 <= qc < COLS + 0.5 and -0.5 <= qr < ROWS + 0.5:
                    cont[cls].append((qc, qr, conf))
                    fobjs.append({"cls": cls, "conf": conf,
                                  "q": [round(qc, 3), round(qr, 3)]})
        metas.append({"frame": m["frame"], "grid_ok": A is not None,
                      "labels": [f"{LETTERS[li]}{ri + 1}" for li, ri, _ in reads],
                      "objs": fobjs})

    fc = circular_mode([q[0] for q in cont["paper"]] or
                       [q[0] for v in cont.values() for q in v])
    fr = circular_mode([q[1] for q in cont["paper"]] or
                       [q[1] for v in cont.values() for q in v])
    votes = defaultdict(Counter)
    vframes = defaultdict(lambda: defaultdict(set))
    vconf = defaultdict(lambda: defaultdict(float))
    for m in metas:
        if not m["grid_ok"]:
            continue
        anchors = [(LETTERS.index(n[0]), int(n[1:]) - 1) for n in m["labels"]]
        seen = set()
        for o in m["objs"]:
            qc, qr = o["q"]
            if not any(max(abs(qc - a0), abs(qr - a1)) <= ANCHOR_GATE
                       for a0, a1 in anchors):
                continue
            col, row = round(qc - fc), round(qr - fr)
            if max(abs(qc - fc - col), abs(qr - fr - row)) > MAX_VOTE_RESIDUAL:
                continue
            if not (0 <= col < COLS and 0 <= row < ROWS):
                continue
            name = f"{LETTERS[col]}{row + 1}"
            if (o["cls"], name) in seen:
                continue
            seen.add((o["cls"], name))
            votes[o["cls"]][name] += 1
            vframes[o["cls"]][name].add(m["frame"])
            vconf[o["cls"]][name] += o["conf"]

    def clusters(fs, gap=300):
        fs = sorted(fs)
        return 1 + sum(1 for a, b in zip(fs, fs[1:]) if b - a > gap) if fs else 0

    paper_cells = []
    for c_, v in votes["paper"].items():
        if v < 2:
            continue
        if clusters(vframes["paper"][c_]) >= 2 or vconf["paper"].get(c_, 0) / v >= 0.35:
            paper_cells.append(c_)
    mines = [c for c in paper_cells if LETTERS.index(c[0]) != BRIDGE_COL_IDX]
    passable = [c for c in paper_cells if LETTERS.index(c[0]) == BRIDGE_COL_IDX]
    obstacles = sorted(c for c, v in Counter(
        {**votes["obstacle"], **votes["bucket_red"], **votes["bucket_green"],
         **votes["bucket_cyan"]}).items() if v >= 2)

    vm = set(mines)
    print(f"[done] 总耗时 {time.time() - t0:.0f}s | "
          f"网格OK {sum(1 for m in metas if m['grid_ok'])}/{len(metas)}")
    print(f"[cmp] 雷 {sorted(vm)} 命中{len(vm & OFFICIAL['mines'])}/8"
          f" 漏{sorted(OFFICIAL['mines'] - vm)} 多{sorted(vm - OFFICIAL['mines'])}")
    print(f"[cmp] 障碍 {obstacles} | 门 {passable}")
    (out_dir / "field_state.json").write_text(json.dumps({
        "engine": env, "total_s": round(time.time() - t0, 1),
        "mines": mines, "passable": passable, "obstacles": obstacles,
        "votes": {k: dict(v.most_common()) for k, v in votes.items()},
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[out] {out_dir / 'field_state.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
