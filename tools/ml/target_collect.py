# -*- coding: utf-8 -*-
"""标靶数据自动采集流水线：视频→抽帧→质量过滤→去重→预标注

用法:
    python target_collect.py --videos 视频文件夹 --out E:\\ml_datasets\\target_raw

你只需把手机录的视频(mp4/mov)放进一个文件夹，跑这个脚本，
产出 images/ + labels/(YOLO-pose预标注) + preview/（抽帧预览，检查覆盖度）
"""
import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np

# ---------- 质量过滤 ----------

def blur_score(gray):
    """拉普拉斯方差，<80 视为模糊。"""
    return cv2.Laplacian(gray, cv2.CV_64F).var()

def exposure_ok(gray):
    """过曝/欠曝过滤：均值在 [40, 220]，且非全黑全白。"""
    m = gray.mean()
    return 40 <= m <= 220

def phash(img, size=16):
    """简易感知哈希（DCT太重，用缩放+均值二值化，够用）。"""
    g = cv2.resize(img, (size, size))
    return (g > g.mean()).flatten()

def hamming(a, b):
    return int(np.sum(a != b))

# ---------- 预标注 ----------

class TargetPrelabeler:
    """用已有 best(1).onnx (pose版[1,9,8400]) 预标注：框+靶心关键点。"""

    def __init__(self, onnx_path):
        import onnxruntime as ort
        self.sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
        self.iname = self.sess.get_inputs()[0].name
        self.imgsz = self.sess.get_inputs()[0].shape[-1]  # 通常640

    def predict(self, img_bgr, conf=0.35):
        H, W = img_bgr.shape[:2]
        rs = min(self.imgsz / W, self.imgsz / H)
        rw, rh = int(W * rs), int(H * rs)
        r = cv2.resize(img_bgr, (rw, rh))
        pad_w, pad_h = self.imgsz - rw, self.imgsz - rh
        if pad_w > 0 or pad_h > 0:
            r = cv2.copyMakeBorder(r, pad_h // 2, pad_h - pad_h // 2,
                                   pad_w // 2, pad_w - pad_w // 2, cv2.BORDER_CONSTANT)
        x = cv2.cvtColor(r, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        x = np.transpose(x, (2, 0, 1))[None]
        out = self.sess.run(None, {self.iname: x})[0][0]  # [9, N]
        # 输出布局: 4bbox + 2cls(blue,red) + 3kpt(x,y,conf)
        b, c, k = out[:4], out[4:6], out[6:9]
        ids = np.argmax(c, axis=0)
        scores = c[ids, np.arange(c.shape[1])]
        keep = scores > conf
        b, ids, scores, k = b[:, keep], ids[keep], scores[keep], k[:, keep]
        # letterbox逆变换
        sx = W / rw
        sy = H / rh
        ox = -pad_w / 2 * sx
        oy = -pad_h / 2 * sy
        dets = []
        for i in range(b.shape[1]):
            x1, y1, x2, y2 = b[:, i]
            x1, x2 = (x1 * sx + ox) / W, (x2 * sx + ox) / W
            y1, y2 = (y1 * sy + oy) / H, (y2 * sy + oy) / H
            kx, ky = (k[0, i] * sx + ox) / W, (k[1, i] * sy + oy) / H
            dets.append((int(ids[i]), x1, y1, x2, y2, kx, ky, float(scores[i]), float(k[2, i])))
        return dets

def to_yolo_pose(W, H, det):
    """(cls, x1,y1,x2,y2, kx,ky, conf, kconf) → YOLO-pose标签行"""
    cls, x1, y1, x2, y2, kx, ky, _, _ = det
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    w, h = x2 - x1, y2 - y1
    return f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} {kx:.6f} {ky:.6f} 2"

# ---------- 主流程 ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos", required=True, help="视频文件或文件夹")
    ap.add_argument("--out", default=r"E:\ml_datasets\target_raw")
    ap.add_argument("--fps", type=float, default=2.0, help="抽帧率")
    ap.add_argument("--min-blur", type=float, default=80.0)
    ap.add_argument("--hamming-thresh", type=int, default=30, help="感知哈希去重阈值")
    ap.add_argument("--onnx", default=r"E:\OneDrive - mails.qust.edu.cn\xwechat_files\wxid_esyua6tojf0x22_b065\msg\file\2026-09\best(1).onnx")
    ap.add_argument("--min-conf", type=float, default=0.35, help="预标注置信度下限")
    args = ap.parse_args()

    src = Path(args.videos)
    videos = ([src] if src.is_file() else
              sorted(p for p in src.iterdir() if p.suffix.lower() in {".mp4", ".mov", ".avi", ".mkv"}))
    out = Path(args.out)
    for d in ["images", "labels", "preview", "rejected"]:
        (out / d).mkdir(parents=True, exist_ok=True)

    pre = TargetPrelabeler(args.onnx)
    hashes, n_kept, n_reject = [], 0, 0

    for vid in videos:
        cap = cv2.VideoCapture(str(vid))
        fps = cap.get(cv2.CAP_PROP_FPS) or 30
        step = max(1, int(fps / args.fps))
        i, stem = 0, vid.stem
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if i % step == 0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # 1) 质量过滤
                if not exposure_ok(gray) or blur_score(gray) < args.min_blur:
                    cv2.imwrite(str(out / "rejected" / f"{stem}_f{i:06d}_blur.jpg"), frame)
                    n_reject += 1
                    i += 1
                    continue
                # 2) 去重
                h = phash(gray)
                if any(hamming(h, ph) < args.hamming_thresh for ph in hashes):
                    i += 1
                    continue
                hashes.append(h)
                # 3) 预标注
                dets = pre.predict(frame, conf=args.min_conf)
                name = f"{stem}_f{i:06d}"
                cv2.imwrite(str(out / "images" / f"{name}.jpg"), frame)
                with open(out / "labels" / f"{name}.txt", "w") as f:
                    for d in dets:
                        f.write(to_yolo_pose(frame.shape[1], frame.shape[0], d) + "\n")
                # 4) 预览图（画框）
                prev = frame.copy()
                for d in dets:
                    cls, x1, y1, x2, y2 = d[0], d[1], d[2], d[3], d[4]
                    W, H = frame.shape[1], frame.shape[0]
                    c = (255, 0, 0) if cls == 0 else (0, 0, 255)
                    cv2.rectangle(prev, (int(x1*W), int(y1*H)), (int(x2*W), int(y2*H)), c, 2)
                    cv2.circle(prev, (int(d[5]*W), int(d[6]*H)), 4, (0, 255, 0), -1)
                cv2.imwrite(str(out / "preview" / f"{name}.jpg"), prev)
                n_kept += 1
            i += 1
        cap.release()
        print(f"[{vid.name}] 完成，累计保留 {n_kept} 帧")

    print(f"\n=== 采集完成 ===")
    print(f"保留: {n_kept} 帧 (含预标注) | 拒绝(模糊/过曝): {n_reject} 帧")
    print(f"输出: {out}")
    print(f"下一步: 打开 {out}\\preview\\ 抽查预标注质量，然后交 target_train.py 训练")

if __name__ == "__main__":
    main()
