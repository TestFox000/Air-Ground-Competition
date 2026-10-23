"""相机棋盘格标定：USB 采集 -> cv2.calibrateCamera -> 自动注入 MapLocalizer.cpp。

PnP 定位的精度根基是相机内参。无人机上的 AR0234 从未标定过（无法接电脑），
现同款相机 USB 直连本机（OpenCV 后端一律 CAP_DSHOW，MSMF 在 cv2 5.0 有 bug），
标定一次，注入机载代码，无人机修复后直接受益。

用法（四步，按顺序跑；没有一条龙开关）:
  python tools/calibrate_camera.py board                       # 1. 生成棋盘格图，打印或手机屏显
  python tools/calibrate_camera.py capture --camera 1
      #  2. 实时找角点自动采集 >=20 张（移动棋盘覆盖画面各区域），空格手动拍，ESC 结束
      #     --size 默认已取机上采集分辨率，一般不用传
  python tools/calibrate_camera.py calibrate --square 25       # 3. 标定（方格实测边长 mm），自动剔坏图
  python tools/calibrate_camera.py inject                      # 4. 注入 onboard/dzuav/src/MapLocalizer.cpp

注意:
  - **标定分辨率必须 = 机上实际采集分辨率**。`capture --size` 的默认值直接取
    `orin_bringup.sh` 的 `DZUAV_SPEC` 与 `orin_run_dzuav.sh` 的 `size_fps`
    （当前均为 1920x1080），此时 K 1:1 生效不触发缩放。`inject` 会核对这一条，
    不一致时拒绝注入（需 `--allow-basis-mismatch` 强行覆盖）。
    ⚠️ 历史坑（2026-10-20 查出）：本脚本曾默认 1920x1200，而部署线是 1080。
    照旧默认标定 + inject 会把 MapLocalizer 的基准分母改成 1200，机上仍按
    1080 采集，scaledIntrinsics 就把 fy/cy 乘上 1080/1200=0.9 —— 定位系统性
    偏 11%，而 L1/L2/L3 全绿（引擎和协议都不涉及内参）。
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
BRINGUP = REPO / "tools" / "orin_bringup.sh"
RUN_DZUAV = REPO / "tools" / "orin_run_dzuav.sh"


def deploy_frame_sizes() -> dict[str, tuple[int, int]]:
    """机上实际会请求的采集分辨率，从启动脚本的默认值里读（不另抄常量）。

    标定基准与采集分辨率不一致时，scaledIntrinsics 会按比例改 fy/cy；而
    1920x1200 -> 1920x1080 在 USB 相机上通常是**裁切**不是缩放，按比例改
    就是错的。所以这两个数必须核对，不能靠注释提醒。
    """
    out: dict[str, tuple[int, int]] = {}
    for path, pat, key in (
        (BRINGUP, r'^DZUAV_SPEC="\$\{DZUAV_SPEC:-(\d+)x(\d+)@', "DZUAV_SPEC"),
        (RUN_DZUAV, r'^\s*size_fps="\$\{3:-(\d+)x(\d+)@', "size_fps"),
    ):
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        m = re.search(pat, text, re.M)
        if m:
            out[key] = (int(m.group(1)), int(m.group(2)))
    return out


def _default_capture_size() -> str:
    """capture --size 的默认值 = 机上采集分辨率（从启动脚本派生，不另抄第四份）。

    两个脚本给的数不一致时退回 1920x1080；那种情况下 inject 的基准核对也会
    拦住，不会静默产出一份谁都对不上的内参。
    """
    sizes = set(deploy_frame_sizes().values())
    if len(sizes) == 1:
        w, h = next(iter(sizes))
        return f"{w}x{h}"
    return "1920x1080"


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


def inject(args) -> int:
    """把标定结果写进 onboard/dzuav/src/MapLocalizer.cpp（K_/dist_/基准分辨率）。"""
    if not CPP.is_file():
        raise SystemExit(f"找不到 {CPP}")
    data = yaml.safe_load((CALIB_DIR / "calibration.yaml").read_text(encoding="utf-8"))

    # 基准分辨率 = 机上采集分辨率才叫"不缩放"。注入会把 scaledIntrinsics 的分母
    # 一起改成标定分辨率，于是两边不一致时不会报错、只会让定位静默偏掉。
    basis = (int(data["width"]), int(data["height"]))
    deployed = deploy_frame_sizes()
    if not deployed:
        print("[警告] 读不到 orin_bringup.sh / orin_run_dzuav.sh 的采集分辨率默认值，"
              "基准一致性这项没法核对——请手工确认")
    else:
        bad = {k: v for k, v in deployed.items() if v != basis}
        if bad and not args.allow_basis_mismatch:
            detail = "、".join(f"{k}={v[0]}x{v[1]}" for k, v in bad.items())
            want_key, (want_w, want_h) = next(iter(bad.items()))
            raise SystemExit(
                f"标定基准 {basis[0]}x{basis[1]} 与机上采集分辨率不一致（{detail}）。\n"
                f"  注入后运行时 scaledIntrinsics 会把 fx 乘 {want_w}/{basis[0]}"
                f"={want_w / basis[0]:.3f}、fy 乘 {want_h}/{basis[1]}"
                f"={want_h / basis[1]:.3f}；\n"
                f"  而 USB 相机换分辨率通常是**裁切**不是缩放，按比例改内参是错的，\n"
                f"  定位会系统性偏移且 L1/L2/L3 全绿（那三级都不涉及内参）。\n"
                f"  正解：capture --size {want_w}x{want_h} 重采重标；\n"
                f"  确要保留本基准就加 --allow-basis-mismatch，并同步把 {want_key} "
                f"改成 {basis[0]}x{basis[1]}@… 让机上按标定分辨率采集。")

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
    # 函数上方那句「基准 1920x1080」注释也要跟着走，否则代码改了注释还在撒谎
    src2, n5 = re.subn(r"（基准 \d+x\d+）",
                       f"（基准 {data['width']}x{data['height']}）", src2)
    if not (n1 and n2 and n3 and n4 and n5):
        raise SystemExit(
            f"patch 点没找全 K={n1} dist={n2} W={n3} H={n4} 注释={n5}，检查 MapLocalizer.cpp")
    CPP.write_text(src2, encoding="utf-8")
    print(f"已注入 {CPP}")
    print(f"  K 基准 {data['width']}x{data['height']}  (rms {data['rms_px']:.3f}px, "
          f"{data['n_views']}图, {data['date']})")
    print("机上生效（改的是 C++ 源码，必须重推源码再编译；转引擎与此无关）:")
    print("  python tools/orin_deploy.py --host <Orin IP> --user <用户> --skip-models --verify")
    print(f"提示: 机上采集分辨率必须就是 {data['width']}x{data['height']}"
          "（orin_bringup.sh 的 DZUAV_SPEC / orin_run_dzuav.sh 的 size_fps），"
          "否则 scaledIntrinsics 会按比例改内参")
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
    p.add_argument("--size", default=_default_capture_size(),
                   help="采集分辨率（默认取机上 DZUAV_SPEC = %(default)s；"
                        "与它不等时 inject 拒绝注入）")
    p.add_argument("--pattern", default="9x6")
    p.set_defaults(func=capture)

    p = sub.add_parser("calibrate", help="标定并写 calibration.yaml")
    p.add_argument("--pattern", default="9x6")
    p.add_argument("--square", type=float, required=True, help="方格实测边长 mm")
    p.set_defaults(func=calibrate)

    p = sub.add_parser("inject", help="注入 MapLocalizer.cpp")
    p.add_argument("--allow-basis-mismatch", action="store_true",
                   help="标定基准与机上采集分辨率不一致时仍注入（默认拒绝）")
    p.set_defaults(func=inject)

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
