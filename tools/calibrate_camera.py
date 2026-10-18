"""相机棋盘格标定：USB 采集 -> cv2.calibrateCamera -> 自动注入 MapLocalizer.cpp。

PnP 定位的精度根基是相机内参。无人机上的 AR0234 从未标定过（无法接电脑），
现同款相机 USB 直连本机（OpenCV 后端一律 CAP_DSHOW，MSMF 在 cv2 5.0 有 bug），
标定一次，注入机载代码，无人机修复后直接受益。

用法（三步，也可 --all 一条龙）:
  python tools/calibrate_camera.py board                       # 1. 生成棋盘格图，打印或手机屏显
  python tools/calibrate_camera.py capture --camera 1 --size 1920x1200
      #  2. 实时找角点自动采集 >=20 张（移动棋盘覆盖画面各区域），空格手动拍，ESC 结束
  python tools/calibrate_camera.py calibrate --square 25       # 3. 标定（方格实测边长 mm），自动剔坏图
  python tools/calibrate_camera.py inject                      # 4. 注入 onboard/dzuav/src/MapLocalizer.cpp

注意:
  - 标定分辨率 = 部署运行分辨率（默认 1920x1200 原生）。机上启动用同规格
    `orin_run_dzuav.sh camera /dev/video0 1920x1200@30`，K 1:1 生效不做缩放。
  - 打印的方格务必用尺量实际边长填 --square，纸机缩放会让焦距差出百分之几。
"""
from __future__ import annotations

import argparse
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO = Path(__file__).resolve().parent.parent
CALIB_DIR = REPO / "runs" / "camera_calib"
SHOTS = CALIB_DIR / "shots"
CPP = REPO / "onboard" / "dzuav" / "src" / "MapLocalizer.cpp"


def imwrite_cn(path: Path, img: np.ndarray) -> None:
    """cv2.imwrite 不支持中文路径，用 imencode + tofile。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imencode(".png", img)[1].tofile(str(path))


def imread_cn(path: Path) -> np.ndarray:
    return cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)


def gen_board(args) -> int:
    """生成打印用棋盘格 PNG（A4 300dpi，方格 25mm，内角点 9x6）。"""
    cols, rows = args.pattern_split
    sq_px = int(args.square_mm / 25.4 * 300)
    margin = int(15 / 25.4 * 300)
    w = margin * 2 + sq_px * (cols + 1)
    h = margin * 2 + sq_px * (rows + 1)
    img = np.full((h, w), 255, np.uint8)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                y0, x0 = margin + r * sq_px, margin + c * sq_px
                img[y0:y0 + sq_px, x0:x0 + sq_px] = 0
    cv2.rectangle(img, (margin - 2, margin - 2), (w - margin + 1, h - margin + 1), 0, 2)
    out = CALIB_DIR / f"chessboard_{cols}x{rows}_{args.square_mm}mm.png"
    imwrite_cn(out, img)
    print(f"棋盘格已生成: {out} ({out.stat().st_size / 1e6:.1f} MB, 300dpi)")
    print(f"  内角点 {cols}x{rows}，方格 {args.square_mm}mm")
    print("  打印选'实际大小/100%'，打印后用尺量方格边长，量得多少 --square 就填多少")
    print("  手机屏显亦可：图片按 1:1 全屏显示，勿缩放")
    return 0


def capture(args) -> int:
    """实时棋盘格采集：姿态变化自动存图，>=20 张、覆盖画面各区域后提示可结束。"""
    w, h = args.size_split
    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)  # MSMF 有 bug，一律 DSHOW
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if not cap.isOpened():
        raise SystemExit(f"打不开相机 index={args.camera}（换 --camera 试试）")
    rw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    rh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    warn = "" if (rw, rh) == (w, h) else f" !! 分辨率不符({rw}x{rh})，K 基准以实际为准"
    print(f"相机已开: {rw}x{rh}（要求 {w}x{h}{warn}）")

    SHOTS.mkdir(parents=True, exist_ok=True)
    for old in SHOTS.glob("shot_*.png"):
        old.unlink()
    cols, rows = args.pattern_split
    last_pts: np.ndarray | None = None
    last_t = 0.0
    n = 0
    cover = set()  # 3x3 区域覆盖
    print("移动棋盘到画面不同位置（尤其四角和边缘），自动采集；空格=手动拍，ESC=结束")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("取帧失败")
            break
        found, pts = cv2.findChessboardCornersSB(frame, (cols, rows), 0)
        disp = frame.copy()
        if found:
            cv2.drawChessboardCorners(disp, (cols, rows), pts, found)
            moved = last_pts is None or np.linalg.norm(
                pts.reshape(-1, 2) - last_pts.reshape(-1, 2), axis=1).mean() > 40
            if moved and time.time() - last_t > 1.5 and n < 60:
                imwrite_cn(SHOTS / f"shot_{n:02d}.png", frame)
                cx_, cy_ = pts.reshape(-1, 2).mean(0)
                cover.add((min(2, int(cx_ / rw * 3)), min(2, int(cy_ / rh * 3))))
                n += 1
                last_pts, last_t = pts, time.time()
                print(f"  采集 #{n:02d}  覆盖{len(cover)}/9区  ({datetime.now():%H:%M:%S})")
        msg = f"shots={n}/20+  ESC=finish" + ("" if n < 20 else "  [够了，可 ESC]")
        cv2.putText(disp, msg, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.imshow("calib capture (SPACE=shot, ESC=done)", disp)
        key = cv2.waitKey(30) & 0xFF
        if key == 27:
            break
        if key == ord(" ") and ok:
            imwrite_cn(SHOTS / f"shot_{n:02d}.png", frame)
            n += 1
            print(f"  手动拍 #{n:02d}")
    cap.release()
    cv2.destroyAllWindows()
    print(f"共 {n} 张 -> {SHOTS}")
    print("下一步: python tools/calibrate_camera.py calibrate --square <实测方格边长mm>")
    return 0 if n >= 10 else 1


def calibrate(args) -> int:
    """标定：逐图重投影误差剔除坏图后重标，写 calibration.yaml。"""
    cols, rows = args.pattern_split
    shots = sorted(SHOTS.glob("shot_*.png"))
    if len(shots) < 10:
        raise SystemExit(f"采集图太少（{len(shots)}），至少 10 张")
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square
    size = imread_cn(shots[0]).shape[1::-1]

    def run(idxs):
        objs, imgs = [], []
        for i in idxs:
            im = imread_cn(shots[i])
            found, pts = cv2.findChessboardCornersSB(im, (cols, rows), 0)
            if not found:
                print(f"  [跳过] {shots[i].name} 找不到角点")
                return None
            objs.append(objp)
            imgs.append(pts)
        rms, K, dist, _, _ = cv2.calibrateCamera(objs, imgs, size)
        per = []
        for o, p in zip(objs, imgs):
            _, rvec, tvec = cv2.solvePnP(o, p, K, dist, flags=cv2.SOLVEPNP_SQPNP)
            proj, _ = cv2.projectPoints(o, rvec, tvec, K, dist)
            per.append(float(np.linalg.norm(proj - p, axis=2).mean()))
        return rms, K, dist, per

    idxs = list(range(len(shots)))
    res = run(idxs)
    if res is None:
        raise SystemExit("首图即找不到角点，重拍")
    rms, K, dist, per = res
    for it in range(4):  # 迭代剔坏图
        med = float(np.median(per))
        bad = {idxs[i] for i, e in enumerate(per) if e > max(1.5 * med, med + 0.15)}
        print(f"迭代{it + 1}: rms={rms:.4f}px 中位={med:.3f}px "
              f"剔坏图{len(bad)}张 剩{len(idxs) - len(bad)}张")
        if not bad or len(idxs) - len(bad) < 10:
            break
        idxs = [i for i in idxs if i not in bad]
        res = run(idxs)
        if res is None:
            break
        rms, K, dist, per = res

    CALIB_DIR.mkdir(parents=True, exist_ok=True)
    data = {
        "fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]),
        "cy": float(K[1, 2]), "dist": [float(x) for x in np.ravel(dist)[:5]],
        "width": int(size[0]), "height": int(size[1]),
        "rms_px": float(rms), "n_views": len(idxs),
        "pattern": f"{cols}x{rows}", "square_mm": args.square,
        "date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }
    out = CALIB_DIR / "calibration.yaml"
    out.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
                   encoding="utf-8")
    print(f"\n内参: fx={data['fx']:.2f} fy={data['fy']:.2f} "
          f"cx={data['cx']:.2f} cy={data['cy']:.2f}")
    print(f"畸变: {[f'{x:.6f}' for x in data['dist']]}")
    print(f"RMS={data['rms_px']:.3f}px（<0.5 优秀, <1 可用）  {len(idxs)} 张")
    print(f"已写 {out}")
    print("下一步: python tools/calibrate_camera.py inject")
    return 0


def inject(_args) -> int:
    """把标定结果写进 onboard/dzuav/src/MapLocalizer.cpp（K_/dist_/基准分辨率）。"""
    if not CPP.is_file():
        raise SystemExit(f"找不到 {CPP}")
    data = yaml.safe_load((CALIB_DIR / "calibration.yaml").read_text(encoding="utf-8"))
    src = CPP.read_text(encoding="utf-8")
    k_new = (f"    K_ = (cv::Mat_<double>(3, 3) << {data['fx']}, 0, {data['cx']}, "
             f"0, {data['fy']}, {data['cy']}, 0, 0, 1);")
    d_new = (f"    dist_ = (cv::Mat_<double>(1, 5) << "
             f"{', '.join(repr(float(x)) for x in data['dist'])});")
    src2, n1 = re.subn(r"K_ = \(cv::Mat_<double>\(3, 3\) << [^;]+;", k_new, src)
    src2, n2 = re.subn(r"dist_ = \(cv::Mat_<double>\(1, 5\) << [^;]+;", d_new, src2)
    # 基准分辨率（scaledIntrinsics 的缩放分母）改为标定分辨率
    src2, n3 = re.subn(r"sz\.width\) / \d+", f"sz.width) / {data['width']}", src2)
    src2, n4 = re.subn(r"sz\.height\) / \d+", f"sz.height) / {data['height']}", src2)
    if not (n1 and n2 and n3 and n4):
        raise SystemExit(f"patch 点没找全 K={n1} dist={n2} W={n3} H={n4}，检查 MapLocalizer.cpp")
    CPP.write_text(src2, encoding="utf-8")
    print(f"已注入 {CPP}")
    print(f"  K 基准 {data['width']}x{data['height']}  (rms {data['rms_px']:.3f}px, "
          f"{data['n_views']}图, {data['date']})")
    print("机上生效: git pull && tools/orin_build_verify.sh && tools/orin_convert_engines.sh")
    print(f"提示: 机上相机按 {data['width']}x{data['height']} 运行可免缩放误差")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("board", help="生成打印用棋盘格")
    p.add_argument("--pattern", default="9x6", help="内角点数（默认 9x6）")
    p.add_argument("--square-mm", type=float, default=25.0, help="方格边长 mm（默认 25）")
    p.set_defaults(func=gen_board)

    p = sub.add_parser("capture", help="实时采集棋盘格图")
    p.add_argument("--camera", type=int, default=1, help="相机 index（USB 同款=1）")
    p.add_argument("--size", default="1920x1200", help="采集分辨率")
    p.add_argument("--pattern", default="9x6")
    p.set_defaults(func=capture)

    p = sub.add_parser("calibrate", help="标定并写 calibration.yaml")
    p.add_argument("--pattern", default="9x6")
    p.add_argument("--square", type=float, required=True, help="方格实测边长 mm")
    p.set_defaults(func=calibrate)

    sub.add_parser("inject", help="注入 MapLocalizer.cpp").set_defaults(func=inject)

    args = ap.parse_args()
    if getattr(args, "pattern", None):
        c, r = args.pattern.lower().split("x")
        args.pattern_split = (int(c), int(r))
    if getattr(args, "size", None):
        w, h = args.size.lower().split("x")
        args.size_split = (int(w), int(h))
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
