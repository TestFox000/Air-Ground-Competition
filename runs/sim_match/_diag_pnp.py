# -*- coding: utf-8 -*-
"""PnP 内点少的根因诊断：格号分配错 or 内参错（对 frame_00170，78 框人工真值）。

实验设计（全部同机上 RANSAC 参数 100 迭代/5px/0.995）：
  1) 格号一致率：direct 识别 / lattice 几何分配 / 最终合并 vs 真值格号
  2) PnP 三组对照：
       A = 链路现状（识别格号 + YOLO 框中心）
       D = 真值格号 + YOLO 框中心   （只剔除"格号错"因素）
       B = 真值格号 + 真值框中心     （再剔除"检测中心偏差"因素）
     各跑 ITERATIVE / P3P / AP3P，统计内点数 + 全点重投影误差分布
  3) Homography 参照（不依赖内参）：真值格号平面点 -> 真值框中心
       残差小 => "格号↔像素"对应是良好平面投影，PnP 再失败则指向内参错
       残差大 => 标签中心相对格心偏移不一致 / 格号真值本身有问题
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

REPO = Path(__file__).resolve().parents[2]
for p in ("tools", "tools/ml", "reference/onboard/dzuav/src", "src/ag_integrate"):
    sys.path.insert(0, str(REPO / p))

from sim_match import load_intrinsics, world_of            # noqa: E402
from eval_cell_frames import design, fit_lattice, to_cid, to_grid   # noqa: E402
from cell_classifier.predictor import CellIdRecognizer      # noqa: E402

IMG = REPO / "capture/aerial_field/images/frame_00170.jpg"
GT = REPO / "runs/cell_recog_render/label/frame_00170"
YOLO_PT = REPO / "runs/detect/runs/detect/aerial_field_finetuned_v2/weights/best.pt"
CELL_PT = REPO / "runs/cell_classifier_v4c/best_snapshot.pt"
CONF, ANCHOR_CONF = 0.45, 0.80
MATCH_PX = 80.0          # YOLO 框 <-> 真值框 中心匹配阈值（3840 宽下相邻标签 >200px）
FLAGS = {"ITERATIVE": cv2.SOLVEPNP_ITERATIVE,
         "P3P": cv2.SOLVEPNP_P3P, "AP3P": cv2.SOLVEPNP_AP3P}


def pnp_report(tag, world, pts, K, dist_coeffs=None):
    print(f"\n== PnP [{tag}] n={len(world)}")
    world = world.astype(np.float32).reshape(-1, 1, 3)
    pts = pts.astype(np.float32).reshape(-1, 1, 2)
    for name, flag in FLAGS.items():
        ok, rvec, tvec, inl = cv2.solvePnPRansac(
            world, pts, K, dist_coeffs, iterationsCount=100, reprojectionError=5.0,
            confidence=0.995, flags=flag)
        if not ok:
            print(f"  {name:9s} 无解")
            continue
        proj, _ = cv2.projectPoints(world, rvec, tvec, K, dist_coeffs)
        err = np.linalg.norm(proj[:, 0, :] - pts[:, 0, :], axis=1)
        q = np.percentile(err, [50, 75, 90])
        print(f"  {name:9s} 内点={len(inl):3d}/{len(world)}   "
              f"全点误差px 中位={q[0]:6.1f} p75={q[1]:6.1f} p90={q[2]:6.1f} "
              f"max={err.max():7.1f}   <=5px: {int((err <= 5).sum())}/{len(err)}")
    # 用 P3P 的解展开逐点误差明细（诊断"少数点错"还是"全体系统错"）
    ok, rvec, tvec, inl = cv2.solvePnPRansac(
        world, pts, K, dist_coeffs, iterationsCount=100, reprojectionError=5.0,
        confidence=0.995, flags=cv2.SOLVEPNP_P3P)
    if ok:
        proj, _ = cv2.projectPoints(world, rvec, tvec, K, dist_coeffs)
        err = np.linalg.norm(proj[:, 0, :] - pts[:, 0, :], axis=1)
        tvec_flat = tvec.ravel()
        print(f"  [P3P 解] t=({tvec_flat[0]:.2f},{tvec_flat[1]:.2f},{tvec_flat[2]:.2f})m "
              f"误差分桶: <5px={int((err < 5).sum())} 5-20px={int(((err >= 5) & (err < 20)).sum())} "
              f"20-100px={int(((err >= 20) & (err < 100)).sum())} >=100px={int((err >= 100).sum())}")


def main():
    frame = cv2.imdecode(np.fromfile(str(IMG), dtype=np.uint8), cv2.IMREAD_COLOR)
    H, W = frame.shape[:2]
    K, dist, base = load_intrinsics()
    Kf = K.copy()
    Kf[0] *= W / base[0]
    Kf[1] *= H / base[1]
    img = cv2.undistort(frame, Kf, dist, None, Kf)   # 修正后的同款链路
    print(f"frame {W}x{H}  K.fx={Kf[0,0]:.1f} fy={Kf[1,1]:.1f} cx={Kf[0,2]:.1f} cy={Kf[1,2]:.1f}")

    # ---- 真值 ----
    boxes = json.loads((GT / "boxes.json").read_text(encoding="utf-8"))["boxes"]
    labels = json.loads((GT / "labels_final.json").read_text(encoding="utf-8"))
    gt_idx = [i for i in range(len(boxes)) if str(i) in labels]   # 部分框未人工确认
    gt_raw = np.array([[boxes[i]["cx"], boxes[i]["cy"]] for i in gt_idx], np.float64)
    gt_cids = [labels[str(i)]["cell_id"] for i in gt_idx]
    gt_u = cv2.undistortPoints(gt_raw.reshape(-1, 1, 2), Kf, dist, None, Kf) \
        .reshape(-1, 2)
    qual = [labels[str(i)].get("quality", "?") for i in gt_idx]
    print(f"真值 {len(gt_idx)}/{len(boxes)} 框有人工格号  quality: " +
          " ".join(f"{q}={qual.count(q)}" for q in sorted(set(qual))))

    # ---- YOLO 检测（同 sim_match 参数）----
    from ultralytics import YOLO
    yolo = YOLO(str(YOLO_PT))
    pred = yolo.predict(img, conf=0.25, iou=0.45, imgsz=1280,
                        verbose=False, device=0)[0]
    dets = [b for b, c in zip(pred.boxes.xyxy.cpu().numpy(),
                              pred.boxes.cls.cpu().numpy().astype(int))
            if yolo.names[c] == "cell_label"]
    det_pts = np.array([[(b[0] + b[2]) / 2, (b[1] + b[3]) / 2] for b in dets],
                       np.float64)

    # ---- 直接识别 + lattice（同 sim_match 逻辑）----
    rec = CellIdRecognizer(str(CELL_PT), device="cuda:0", half=True,
                           rotation_tta=True, confidence_threshold=0.0)
    crops = []
    for b in dets:
        ccx, ccy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
        side = max(b[2] - b[0], b[3] - b[1]) * 1.3
        crops.append(Image.fromarray(cv2.cvtColor(
            img[max(0, int(ccy - side / 2)):int(ccy + side / 2),
                max(0, int(ccx - side / 2)):int(ccx + side / 2)],
            cv2.COLOR_BGR2RGB)))
    direct = {}
    for j, pr in enumerate(rec.predict(crops)):
        if pr is not None and pr.joint_confidence >= CONF:
            direct[j] = (pr.cell_id, pr.joint_confidence)
    n_anchor = sum(1 for _, cf in direct.values() if cf >= ANCHOR_CONF)
    amap = {}
    fit_ok = False
    if len(direct) >= 8 and n_anchor >= 8:
        fit = fit_lattice([det_pts[j] for j, (_, cf) in direct.items()
                           if cf >= ANCHOR_CONF],
                          [cid for j, (cid, cf) in direct.items()
                           if cf >= ANCHOR_CONF])
        if fit:
            fit_ok = True
            cx, cy, order = fit
            grid_all = np.array([[c, r] for r in range(11) for c in range(14)], float)
            Xa = design(grid_all, order)
            pos = np.stack([Xa @ cx, Xa @ cy], 1)
            cand = []
            for j in range(len(det_pts)):
                d_ = np.linalg.norm(pos - det_pts[j], axis=1)
                for g in np.argsort(d_)[:2]:
                    cand.append((float(d_[g]), j, int(g)))
            cand.sort()
            used = set()
            for dist_, j, g in cand:
                if j in amap or g in used or dist_ > 80:
                    continue
                used.add(g)
                amap[j] = to_cid(*grid_all[g])
    print(f"检测 cell_label={len(dets)}  direct(>= {CONF})={len(direct)}  "
          f"anchors(>= {ANCHOR_CONF})={n_anchor}  lattice_fit={'ok' if fit_ok else 'FAIL'}  "
          f"几何分配={len(amap)}")

    # ---- 匹配：det idx -> gt idx ----
    match = {}
    for j, p in enumerate(det_pts):
        d_ = np.linalg.norm(gt_u - p, axis=1)
        k = int(np.argmin(d_))
        if d_[k] < MATCH_PX:
            match[j] = k
    print(f"YOLO 框匹配真值: {len(match)}/{len(dets)} "
          f"(平均中心偏移 {np.mean([np.linalg.norm(gt_u[k] - det_pts[j]) for j, k in match.items()]):.1f}px)")

    def agree_rate(idmap, tag):
        hit, tot, bad = 0, 0, []
        for j, cid in idmap.items():
            if j not in match:
                continue
            tot += 1
            if cid == gt_cids[match[j]]:
                hit += 1
            else:
                bad.append(f"{cid}!={gt_cids[match[j]]}")
        r = hit / tot * 100 if tot else 0
        print(f"  {tag}: {hit}/{tot} = {r:.1f}%" + (f"  错例: {bad[:8]}" if bad else ""))
        return r

    print("\n== 格号一致率（vs 真值）")
    agree_rate({j: cid for j, (cid, _) in direct.items()}, "direct 识别          ")
    agree_rate(amap, "lattice 几何分配      ")
    final = dict(amap)
    for j, (cid, _) in direct.items():
        final[j] = cid                    # 冲突信直接识别（同 sim_match）
    agree_rate(final, "最终合并地标(=A 用)  ")

    # ---- PnP 三组 ----
    a_idx = sorted(final)
    pnp_report("A 链路: 识别格号 + YOLO中心",
               np.array([world_of(final[j]) for j in a_idx]),
               det_pts[a_idx], Kf)
    d_idx = [j for j in a_idx if j in match]
    pnp_report(f"D 真值格号 + YOLO中心 (匹配上 {len(d_idx)} 个)",
               np.array([world_of(gt_cids[match[j]]) for j in d_idx]),
               det_pts[d_idx], Kf)
    pnp_report("B 真值格号 + 真值中心 (78 框全量)",
               np.array([world_of(c) for c in gt_cids]), gt_u, Kf)

    # ---- 完整标签过滤：被图像边缘裁切的半标签，其"框中心"系统性偏离格心 ----
    MARGIN = 15
    full = [k for k, i in enumerate(gt_idx)
            if boxes[i]["cx"] - boxes[i]["w"] / 2 > MARGIN
            and boxes[i]["cy"] - boxes[i]["h"] / 2 > MARGIN
            and boxes[i]["cx"] + boxes[i]["w"] / 2 < W - MARGIN
            and boxes[i]["cy"] + boxes[i]["h"] / 2 < H - MARGIN]
    print(f"\n完整标签 {len(full)}/{len(gt_idx)}（其余贴边被裁，框中心不可当格心用）")
    cidf = [gt_cids[k] for k in full]
    wf = np.array([world_of(c) for c in cidf])
    gf = gt_u[full]
    pnp_report("B' 真值格号 + 真值中心 (仅完整标签)",
               wf, gf, Kf)

    # ---- Homography 参照（不依赖内参）----
    w2 = np.array([[world_of(c)[0], world_of(c)[1]] for c in cidf], np.float64)
    Hm, mask = cv2.findHomography(w2, gf, cv2.RANSAC, 5.0)
    hp = (Hm @ np.hstack([w2, np.ones((len(w2), 1))]).T).T
    hp = hp[:, :2] / hp[:, 2:3]
    err = np.linalg.norm(hp - gf, axis=1)
    print(f"\n== Homography 真值格号->真值中心（完整标签, 不依赖内参）")
    print(f"  内点={int(mask.sum())}/{len(w2)}  残差px 中位={np.median(err):.2f} "
          f"p90={np.percentile(err, 90):.2f} max={err.max():.2f}  "
          f"<=3px: {int((err <= 3).sum())}  <=5px: {int((err <= 5).sum())}  "
          f"<=10px: {int((err <= 10).sum())}")
    if int(mask.sum()) >= 10:
        inw = np.array(err)[mask.ravel().astype(bool)]
        print(f"  内点残差px 中位={np.median(inw):.2f} p90={np.percentile(inw, 90):.2f} "
              f"max={inw.max():.2f}")

    # ---- 原始像素对照：不做去畸变，检验"错误的 K/dist 去畸变"是否为崩坏根因 ----
    gf_raw = gt_raw[full]
    pnp_report("B'' 真值格号 + 原始像素, dist 交给 PnP",
               wf, gf_raw, Kf, dist_coeffs=dist)
    w2r = w2.copy()
    Hr, mkr = cv2.findHomography(w2r, gf_raw, cv2.RANSAC, 5.0)
    hrp = (Hr @ np.hstack([w2r, np.ones((len(w2r), 1))]).T).T
    hrp = hrp[:, :2] / hrp[:, 2:3]
    rerr = np.linalg.norm(hrp - gf_raw, axis=1)
    print(f"\n== Homography 真值格号->原始像素（不去畸变, 不依赖内参/dist）")
    print(f"  内点={int(mkr.sum())}/{len(w2r)}  残差px 中位={np.median(rerr):.2f} "
          f"p90={np.percentile(rerr, 90):.2f} max={rerr.max():.2f}  "
          f"<=3px: {int((rerr <= 3).sum())}  <=5px: {int((rerr <= 5).sum())}  "
          f"<=10px: {int((rerr <= 10).sum())}")
    print(f"  [对照] 去畸变图上单应内点 13/53  vs  原始像素内点 {int(mkr.sum())}/53 —— "
          f"若原始像素内点显著更多, 则根因是内参/dist 与该相机不符, 而非格号/标签问题")

    # ---- 多项式格网映射精度（lattice 同款, 畸变免疫, 不依赖内参）----
    grid_f = np.array([to_grid(c) for c in cidf], float)
    order = 3 if len(grid_f) >= 14 else 2
    Xa = design(grid_f, order)
    cxp = np.linalg.lstsq(Xa, gf_raw[:, 0], rcond=None)[0]
    cyp = np.linalg.lstsq(Xa, gf_raw[:, 1], rcond=None)[0]
    pred = np.stack([Xa @ cxp, Xa @ cyp], 1)
    perr = np.linalg.norm(pred - gf_raw, axis=1)
    # 留一验证：预测能力（目标像素->格坐标定位的实际精度上界）
    loo = []
    for k in range(len(grid_f)):
        m = np.ones(len(grid_f), bool)
        m[k] = False
        lk = np.linalg.lstsq(Xa[m], gf_raw[m, 0], rcond=None)[0]
        rk = np.linalg.lstsq(Xa[m], gf_raw[m, 1], rcond=None)[0]
        loo.append(np.hypot(Xa[k] @ lk - gf_raw[k, 0], Xa[k] @ rk - gf_raw[k, 1]))
    loo = np.array(loo)
    # 米/像素 换算：取多项式映射在场地中心的梯度
    g0 = np.array([[7.0, 5.0]])
    g1 = np.array([[8.0, 5.0]])
    dx = float((design(g1, order) @ cxp - design(g0, order) @ cxp)[0])
    dy = float((design(g1, order) @ cyp - design(g0, order) @ cyp)[0])
    px_per_m = float(np.hypot(dx, dy)) / 0.35
    print(f"\n== 三阶多项式 格号->原始像素（{len(grid_f)} 个完整真值标签）")
    print(f"  全拟合残差px 中位={np.median(perr):.2f} p90={np.percentile(perr, 90):.2f} max={perr.max():.2f}")
    print(f"  留一预测px 中位={np.median(loo):.2f} p90={np.percentile(loo, 90):.2f} max={loo.max():.2f}"
          f"   (场地中心 ~{px_per_m:.0f}px/m, 留一中位 ≈ {np.median(loo) / px_per_m * 100:.1f}cm)")
    print("  => 多项式对畸变免疫、不依赖内参; 若留一 < ~20px(约 3cm), 可作为定位主/兜底方案")


if __name__ == "__main__":
    main()
