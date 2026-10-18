"""模拟比赛驱动器：本地复刻机载 dzuav 全链路，真 UDP 广播驱动车端彩排。

无人机未上线时的"机载替身"——与 onboard/dzuav/src/main.cpp 逐段对应：

    undistort(去畸变, 内参从 MapLocalizer.cpp 解析, 标定后自动同步)
      -> HbbDetector   (ultralytics .pt, conf=.25/iou=.45 同 C++)
      -> CellClassifier(CellIdRecognizer TTA, conf>=.45)
      -> lattice 几何扩散(高置信锚点拟合 -> 全场格号, 机上 C++ 没有的增强, --no-lattice 可关)
      -> solvePnPRansac(>=4 地标, ITERATIVE/100 迭代/5px/.995 同 C++)
      -> 投影目标到世界坐标 -> intel_protocol.build 广播 UDP:50000
      -> 车顶红色标记 -> car_yaw -> build_car_pose 广播 0x4002

用法:
  # 录像/图片回放（彩排主用）
  python tools/sim_match.py --video capture/xxx.mp4 --rate 10
  python tools/sim_match.py --image capture/frame_00124.jpg --once
  # USB 相机实时（同款 AR0234 直连）
  python tools/sim_match.py --camera 1 --rate 15
  # A/B: 关掉 lattice 看"纯机上链路"的定位成功率
  python tools/sim_match.py --video ... --no-lattice
车端同时跑: ros2 launch ag_integrate integrate.launch.py  (dry_run=true 看路线)
"""
from __future__ import annotations

import argparse
import json
import re
import socket
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "tools" / "ml"))
sys.path.insert(0, str(REPO / "reference" / "onboard" / "dzuav" / "src"))
sys.path.insert(0, str(REPO / "src" / "ag_integrate"))
from eval_cell_frames import design, fit_lattice, to_cid, to_grid  # noqa: E402
from ag_integrate.intel_protocol import (  # noqa: E402
    CarPose, IntelFrame, IntelObject, ObjectType, build, build_car_pose,
)
from cell_classifier.predictor import CellIdRecognizer  # noqa: E402

# 与 C++ className/ObjectType 顺序一致的类别名映射。
# D13 统一模型（hbb_d13_best.pt，2026-10-16）：类别名 = ObjectType 枚举名小写，
# 由枚举直接推导、永不漂移；雷/桥洞不再分两类（都是 paper，语义由车端
# world_model 按落格判定：K 列=通行箭头，其余=雷），本链路只透传类型。
NAME2TYPE = {t.name.lower(): t for t in ObjectType}
LETTERS = "ABCDEFGHIJKLMN"
CELL, X0, Y0 = 0.35, 0.065, 0.295


def world_of(cid: str) -> np.ndarray:
    """格号 -> 世界坐标（同 MapLocalizer::world, id.row 1 基）。"""
    col = ord(cid[0]) - ord("A")
    row = int(cid[1:])
    return np.array([X0 + col * CELL, Y0 + (11 - row) * CELL, 0.0], np.float32)


def load_intrinsics() -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    """从 onboard/dzuav/src/MapLocalizer.cpp 解析 K/dist/基准分辨率。

    C++ 是内参唯一真源（calibrate_camera.py inject 写进去），模拟器跟它走，
    保证"彩排看到的"="机上跑出来的"。
    """
    src = (REPO / "onboard/dzuav/src/MapLocalizer.cpp").read_text(encoding="utf-8")
    m = re.search(r"K_ = \(cv::Mat_<double>\(3, 3\) <<\s*"
                  r"([-\d.]+),\s*0,\s*([-\d.]+),\s*0,\s*([-\d.]+),\s*([-\d.]+),\s*0,\s*0,\s*1", src)
    d = re.search(r"dist_ = \(cv::Mat_<double>\(1, 5\) <<\s*([-\d.e]+),\s*([-\d.e]+),\s*"
                  r"([-\d.e]+),\s*([-\d.e]+),\s*([-\d.e]+)", src)
    if not (m and d):
        raise SystemExit("MapLocalizer.cpp 内参解析失败，检查 K_/dist_ 格式")
    K = np.array([[float(m.group(1)), 0, float(m.group(2))],
                  [0, float(m.group(3)), float(m.group(4))],
                  [0, 0, 1]])
    dist = np.array([float(x) for x in d.groups()], np.float64)
    bw = int(re.search(r"sz\.width\) / (\d+)", src).group(1))
    bh = int(re.search(r"sz\.height\) / (\d+)", src).group(1))
    return K, dist, (bw, bh)


class SimChain:
    """一帧图像 -> (objects, car_pose, 可视化图)。"""

    def __init__(self, yolo_pt: Path, cell_pt: Path, args):
        from ultralytics import YOLO
        # 设备选择：默认自动（显存够用走 GPU，被训练占满时回落 CPU）
        if getattr(args, "device", "auto") == "auto":
            try:
                import torch
                free = torch.cuda.mem_get_info()[0] / 2**20 if torch.cuda.is_available() else 0
                args.device = 0 if free > 2500 else "cpu"
            except Exception:
                args.device = "cpu"
        # 归一化: YOLO 用 int(0)/"cpu"; torch 分类器用 "cuda:0"/"cpu"
        self.device = args.device
        dev_torch = f"cuda:{args.device}" if str(args.device).isdigit() else "cpu"
        self.yolo = YOLO(str(yolo_pt))
        self.rec = CellIdRecognizer(str(cell_pt), device=dev_torch,
                                     half=(dev_torch != "cpu"),
                                     rotation_tta=True, confidence_threshold=0.0)
        self.K, self.dist, self.base = load_intrinsics()
        self.args = args
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def _scaled_k(self, size):
        K = self.K.copy()
        K[0] *= size[0] / self.base[0]
        K[1] *= size[1] / self.base[1]
        K[2, 2] = 1
        return K

    def process(self, frame: cv2.Mat, render: cv2.Mat | None):
        # 同 C++ MapLocalizer::undistort：先按帧分辨率缩放基准内参再去畸变。
        # （此前误用基准 1920x1080 的 K 处理任意分辨率帧，归一化坐标尺度错误，
        #   边缘标签中心可偏移上百像素——已在诊断 PnP 内点问题时定位。）
        K = self._scaled_k((frame.shape[1], frame.shape[0]))
        img = cv2.undistort(frame, K, self.dist, None, K)
        pred = self.yolo.predict(img, conf=0.25, iou=0.45, imgsz=1280,
                                 verbose=False, device=self.device)[0]
        boxes, clss, confs = [], [], []
        for b, c, k in zip(pred.boxes.xyxy.cpu().numpy(),
                           pred.boxes.cls.cpu().numpy().astype(int),
                           pred.boxes.conf.cpu().numpy()):
            name = self.yolo.names[c]
            if name not in NAME2TYPE:
                continue
            boxes.append(b)
            clss.append(NAME2TYPE[name])
            confs.append(float(k))

        # ---- 格号识别 + lattice 扩散 ----
        cell_idx = [i for i, t in enumerate(clss) if t == ObjectType.CELL_LABEL]
        cell_ids: dict[int, tuple[str, float]] = {}   # 框索引 -> (格号, 置信)
        crops, centers_all = [], []
        for i in cell_idx:
            x1, y1, x2, y2 = boxes[i]
            ccx, ccy = (x1 + x2) / 2, (y1 + y2) / 2
            side = max(x2 - x1, y2 - y1) * 1.3
            sx1, sy1 = max(0, int(ccx - side / 2)), max(0, int(ccy - side / 2))
            sx2 = min(img.shape[1], int(ccx + side / 2))
            sy2 = min(img.shape[0], int(ccy + side / 2))
            crops.append(Image.fromarray(cv2.cvtColor(img[sy1:sy2, sx1:sx2],
                                                       cv2.COLOR_BGR2RGB)))
            centers_all.append((ccx, ccy))
        direct: dict[int, tuple[str, float]] = {}
        for j, pr in enumerate(self.rec.predict(crops) if crops else []):
            if pr is not None and pr.joint_confidence >= self.args.conf:
                direct[cell_idx[j]] = (pr.cell_id, pr.joint_confidence)

        fit = None
        if not self.args.no_lattice and len(direct) >= 8:
            anchors = [(centers_all[cell_idx.index(i)], cid)
                       for i, (cid, cf) in direct.items() if cf >= self.args.anchor_conf]
            if len(anchors) >= 8:
                fit = fit_lattice([a[0] for a in anchors], [a[1] for a in anchors])
                if fit:
                    cx, cy, order = fit
                    grid_all = np.array([[c, r] for r in range(11) for c in range(14)], float)
                    Xa = design(grid_all, order)
                    pos = np.stack([Xa @ cx, Xa @ cy], 1)
                    cen = np.array(centers_all)
                    cand = []
                    for j in range(len(cen)):
                        d_ = np.linalg.norm(pos - cen[j], axis=1)
                        for g in np.argsort(d_)[:2]:
                            cand.append((float(d_[g]), j, int(g)))
                    cand.sort()
                    used, amap = set(), {}
                    for dist_, j, g in cand:
                        if j in amap or g in used or dist_ > 80:
                            continue
                        used.add(g)
                        amap[j] = to_cid(*grid_all[g])
                    for j, cid in amap.items():
                        i = cell_idx[j]
                        if i in direct:            # 直接识别与几何一致 -> 取更高置信
                            d_cid, d_cf = direct[i]
                            cell_ids[i] = (d_cid, max(d_cf, 0.9)) if d_cid == cid \
                                else (d_cid, d_cf)   # 冲突信直接识别
                        else:
                            cell_ids[i] = (cid, 0.7)  # 几何扩散的给中等置信
        cell_ids.update({i: v for i, v in direct.items() if i not in cell_ids})

        # ---- PnP（与机上修复版一致：P3P + 贴边半标签过滤）----
        # 诊断结论（runs/sim_match/_diag_pnp.py, frame_00170 真值对照）：
        #   * SOLVEPNP_ITERATIVE 对纯平面点集直接无解 -> 必须 P3P
        #   * 贴边被裁的半标签, 框中心系统性偏离格心 -> 弃用
        #   * 内参未标定时 PnP 内点过少 -> 多项式格网兜底（对畸变免疫, 不依赖内参）
        MARGIN = 8
        marks = [(cell_ids[i][0],
                  ((boxes[i][0] + boxes[i][2]) / 2, (boxes[i][1] + boxes[i][3]) / 2))
                 for i in cell_ids
                 if MARGIN < (boxes[i][0] + boxes[i][2]) / 2 < img.shape[1] - MARGIN
                 and MARGIN < (boxes[i][1] + boxes[i][3]) / 2 < img.shape[0] - MARGIN]
        method = "none"
        rvec = tvec = None
        if len(marks) >= 4:
            pts = np.array([p for _, p in marks], np.float32)
            hull = cv2.convexHull(pts)
            if abs(cv2.contourArea(hull)) >= 0.02 * img.shape[1] * img.shape[0]:
                w = np.array([world_of(c) for c, _ in marks], np.float32)
                ok, rvec, tvec, inl = cv2.solvePnPRansac(
                    w, pts, K, None, iterationsCount=100, reprojectionError=5.0,
                    confidence=0.995, flags=cv2.SOLVEPNP_P3P)
                # 解质量门槛：错误内参下 P3P 会"7 内点假成功"（诊断实测全点中位
                # 误差 ~250px vs 正确内参 <3px），垃圾解的投影会把目标投到场外。
                if ok and len(inl) >= 4:
                    proj, _ = cv2.projectPoints(w, rvec, tvec, K, None)
                    err = np.linalg.norm(proj[:, 0, :] - pts, axis=1)
                    if len(inl) >= max(12, 0.3 * len(marks)) and np.median(err) <= 10.0:
                        method = "pnp"
        project = None
        if method == "pnp":
            R, _ = cv2.Rodrigues(rvec)
            Hinv = np.linalg.inv(K @ np.hstack([R[:, :2], tvec]))

            def project(px, py, _H=Hinv):
                v = _H @ np.array([px, py, 1.0])
                return v[0] / v[2], v[1] / v[2]
        elif fit is not None:
            # 多项式格网兜底：lattice 三阶多项式描述"格坐标->像素"（含畸变），
            # 细分网格后取最近点逆映射（留一精度中位 ~1.4cm, 见诊断脚本）。
            cx, cy, order = fit
            cs = np.linspace(0, 13, 14 * 16 + 1)
            rs = np.linspace(0, 10, 11 * 16 + 1)
            grid_fine = np.array([[c, r] for r in rs for c in cs], float)
            Xf = design(grid_fine, order)
            pos_fine = np.stack([Xf @ cx, Xf @ cy], 1)
            method = "poly"

            def project(px, py):
                d_ = np.linalg.norm(pos_fine - (px, py), axis=1)
                c, r = grid_fine[int(np.argmin(d_))]
                return X0 + c * CELL, Y0 + (10 - r) * CELL

        objects, car_pose = [], None
        if project is not None:
            for i, (t, cf) in enumerate(zip(clss, confs)):
                if t == ObjectType.CELL_LABEL:
                    continue
                x, y = project((boxes[i][0] + boxes[i][2]) / 2,
                               (boxes[i][1] + boxes[i][3]) / 2)
                objects.append(IntelObject(t, x, y,
                                           float(boxes[i][2] - boxes[i][0]),
                                           float(boxes[i][3] - boxes[i][1])))
            # 车顶红色标记 -> 车头朝向（同 estimateCarYaw）
            cars = [i for i, t in enumerate(clss) if t == ObjectType.CAR]
            if cars:
                best = max(cars, key=lambda i: confs[i])
                x1, y1, x2, y2 = boxes[best]
                roi = img[int(y1):int(y2), int(x1):int(x2)]
                if roi.size > 16:
                    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)) | \
                           cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
                    mom = cv2.moments(mask, True)
                    if mom["m00"] >= 50:
                        mx = x1 + mom["m10"] / mom["m00"]
                        my = y1 + mom["m01"] / mom["m00"]
                        wx, wy = project(mx, my)
                        cx_, cy_ = project((x1 + x2) / 2, (y1 + y2) / 2)
                        car_pose = CarPose(0, cx_, cy_, float(np.arctan2(wy - cy_, wx - cx_)))

        # ---- 可视化 ----
        # last_project: 暴露本帧投影函数（像素->世界），供测试/核对工具复用
        self.last_project = project
        if render is not None:
            render[:] = img
            for i, (t, cf) in enumerate(zip(clss, confs)):
                x1, y1, x2, y2 = map(int, boxes[i])
                color = (0, 255, 255) if t == ObjectType.CELL_LABEL else (0, 255, 0)
                cv2.rectangle(render, (x1, y1), (x2, y2), color, 2)
                tag = t.name.lower()
                if i in cell_ids:
                    tag = f"{cell_ids[i][0]} {cell_ids[i][1]:.2f}"
                cv2.putText(render, f"{tag} {cf:.2f}", (x1, max(18, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            cv2.putText(render, f"locate={method} landmarks={len(marks)}",
                        (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        return objects, car_pose, method, len(marks)

    def broadcast(self, objects, car_pose) -> None:
        ts = int(time.time() * 1000)
        if objects:
            data = build(IntelFrame(ts, tuple(objects)))
            self.sock.sendto(data, ("255.255.255.255", self.args.port))
        if car_pose is not None:
            self.sock.sendto(build_car_pose(CarPose(ts, car_pose.x, car_pose.y, car_pose.yaw)),
                             ("255.255.255.255", self.args.port))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--video", type=Path)
    src.add_argument("--image", type=Path)
    src.add_argument("--camera", type=int)
    ap.add_argument("--weights-yolo", type=Path,
                    default=REPO / "runs/detect/runs/detect/aerial_field_finetuned_v2/weights/best.pt")
    ap.add_argument("--weights-cell", type=Path,
                    default=REPO / "runs/cell_classifier_v4c/best.pt")
    ap.add_argument("--port", type=int, default=50000)
    ap.add_argument("--rate", type=float, default=10.0, help="广播帧率 Hz")
    ap.add_argument("--conf", type=float, default=0.45, help="格号识别阈值")
    ap.add_argument("--anchor-conf", type=float, default=0.80, help="lattice 锚点阈值")
    ap.add_argument("--no-lattice", action="store_true", help="关 lattice（复刻纯 C++ 链路）")
    ap.add_argument("--once", action="store_true", help="单帧后退出")
    ap.add_argument("--render", type=Path, help="落盘可视化目录")
    ap.add_argument("--show", action="store_true", help="弹窗实时显示")
    ap.add_argument("--log", action="store_true", help="JSONL 落盘 runs/sim_match/")
    args = ap.parse_args()

    chain = SimChain(args.weights_yolo, args.weights_cell, args)
    print(f"[sim] yolo={args.weights_yolo.name} cell={args.weights_cell.name} "
          f"lattice={'off' if args.no_lattice else 'on'} port={args.port}")

    log_fh = None
    if args.log:
        d = REPO / "runs/sim_match"
        d.mkdir(parents=True, exist_ok=True)
        log_fh = open(d / f"sim_{datetime.now():%H%M%S}.jsonl", "a", encoding="utf-8")

    cap = None
    if args.camera is not None:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)  # AR0234 USB 用 DSHOW
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    elif args.video:
        cap = cv2.VideoCapture(str(args.video))

    fi = 0
    period = 1.0 / max(args.rate, 0.1)
    try:
        while True:
            if args.image:
                frame = cv2.imdecode(np.fromfile(str(args.image), dtype=np.uint8),
                                     cv2.IMREAD_COLOR)
                if frame is None:
                    raise SystemExit(f"读不了图: {args.image}")
            else:
                ok, frame = cap.read()
                if not ok:
                    if args.video:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # 录像循环
                        continue
                    break

            render = frame.copy() if (args.render or args.show) else None
            objects, car_pose, method, n_land = chain.process(frame, render)
            chain.broadcast(objects, car_pose)
            msg = (f"[sim] #{fi:04d} locate={method} landmarks={n_land} "
                   f"objs={[f'{o.type.name}:{o.x:.2f},{o.y:.2f}' for o in objects]}")
            print(msg)
            if log_fh:
                log_fh.write(json.dumps({
                    "ts": time.time(), "frame": fi, "locate": method,
                    "landmarks": n_land,
                    "objects": [{"cls": o.type.name, "x": round(o.x, 3),
                                 "y": round(o.y, 3), "conf": None}
                                for o in objects],
                    "car_pose": car_pose and {"x": round(car_pose.x, 3),
                                              "y": round(car_pose.y, 3),
                                              "yaw": round(car_pose.yaw, 3)},
                }, ensure_ascii=False) + "\n")
                log_fh.flush()
            if args.render and fi % 10 == 0:
                args.render.mkdir(parents=True, exist_ok=True)
                (args.render / f"frame_{fi:04d}.jpg").write_bytes(
                    cv2.imencode(".jpg", render, [cv2.IMWRITE_JPEG_QUALITY, 80])[1])
            if args.show:
                cv2.imshow("sim_match (ESC=quit)", render)
                if cv2.waitKey(1) == 27:
                    break
            fi += 1
            if args.once:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        if cap:
            cap.release()
        cv2.destroyAllWindows()
        if log_fh:
            log_fh.close()
    print(f"[sim] 结束，共 {fi} 帧")
    return 0


if __name__ == "__main__":
    sys.exit(main())
