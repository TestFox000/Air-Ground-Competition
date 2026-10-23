#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从机上视频抽出可疑格（B7/D11/E7）的检测框裁剪，供人工核实类别。

用法（机上）：python3 tools/orin_probe_cells.py <视频> <frames_meta.json> <输出目录>
每个目标格取前 3 个命中帧，按检测框外扩 40% 裁剪。
"""
import json
import sys
from pathlib import Path

import cv2


def main() -> int:
    video, meta_path, out_dir = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])
    out_dir.mkdir(parents=True, exist_ok=True)
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    # 帧内连续格坐标 -> 格号（与 video_recon 的落格规则一致：round(q-frac)）
    fc, fr = 0.33, 0.38                    # 本次全量跑的分数校准
    wants = {"B7", "D11", "E7", "D5"}
    cap = cv2.VideoCapture(video)
    got = {k: 0 for k in wants}
    for m in meta:
        if not m.get("grid_ok"):
            continue
        hits = []
        for o in m.get("objs", []):
            qc, qr = o["q"]
            col, row = int(round(qc - fc)), int(round(qr - fr))
            name = f"{chr(ord('A') + col)}{row + 1}"
            if name in wants and got[name] < 3:
                hits.append((name, o))
        if not hits:
            continue
        cap.set(cv2.CAP_PROP_POS_FRAMES, m["frame"])
        ok, frame = cap.read()
        if not ok:
            continue
        for name, o in hits:
            x1, y1, x2, y2 = o["box"]
            w, h = x2 - x1, y2 - y1
            pad = 0.4 * max(w, h)
            X1, Y1 = int(max(0, x1 - pad)), int(max(0, y1 - pad))
            X2, Y2 = int(min(frame.shape[1], x2 + pad)), int(min(frame.shape[0], y2 + pad))
            crop = frame[Y1:Y2, X1:X2]
            if crop.size == 0:
                continue
            fn = out_dir / f"{name}_f{m['frame']:05d}_{o['cls']}_{o['conf']:.2f}.jpg"
            ok, buf = cv2.imencode(".jpg", crop)
            if ok:
                fn.write_bytes(buf.tobytes())
            got[name] += 1
    cap.release()
    print("裁剪完成:", got)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
