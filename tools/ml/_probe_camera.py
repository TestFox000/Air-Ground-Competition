"""探测 USB 相机 (LRCP AR0234) 能力: 分辨率/曝光/增益/对焦/白平衡 + 测试帧.

用法: python _probe_camera.py
输出: runs/camera_probe/test_{i}.png + 控制台报告
"""
import cv2
import numpy as np
from pathlib import Path

OUT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\camera_probe")
OUT.mkdir(parents=True, exist_ok=True)

PROPS = [
    (cv2.CAP_PROP_FPS, "fps"),
    (cv2.CAP_PROP_FOURCC, "fourcc"),
    (cv2.CAP_PROP_BRIGHTNESS, "brightness"),
    (cv2.CAP_PROP_CONTRAST, "contrast"),
    (cv2.CAP_PROP_SATURATION, "saturation"),
    (cv2.CAP_PROP_EXPOSURE, "exposure"),
    (cv2.CAP_PROP_AUTO_EXPOSURE, "auto_exposure"),
    (cv2.CAP_PROP_GAIN, "gain"),
    (cv2.CAP_PROP_FOCUS, "focus"),
    (cv2.CAP_PROP_AUTOFOCUS, "autofocus"),
    (cv2.CAP_PROP_ZOOM, "zoom"),
    (cv2.CAP_PROP_AUTO_WB, "auto_wb"),
    (cv2.CAP_PROP_TEMPERATURE, "wb_temp"),
    (cv2.CAP_PROP_SHARPNESS, "sharpness"),
    (cv2.CAP_PROP_GAMMA, "gamma"),
    (cv2.CAP_PROP_BACKLIGHT, "backlight"),
    (cv2.CAP_PROP_ROLL, "roll"),
]

RES_CANDIDATES = [(1920, 1200), (1920, 1080), (3840, 2160), (1280, 720), (2048, 1536)]


def probe(idx, backend):
    cap = cv2.VideoCapture(idx, backend)
    if not cap.isOpened():
        return None
    ok, frame = cap.read()
    if not ok:
        cap.release()
        return None
    # 预热: 连拍20帧让自动曝光/白平衡稳定
    for _ in range(20):
        cap.read()
    ok, frame = cap.read()
    h, w = frame.shape[:2]
    print(f"\n=== index {idx}: {w}x{h} ===")
    for prop, name in PROPS:
        try:
            print(f"  {name}: {cap.get(prop)}")
        except Exception:
            pass

    # 探测支持的最大分辨率
    print("  -- 分辨率候选 --")
    for tw, th in RES_CANDIDATES:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, tw)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, th)
        ok2, fr = cap.read()
        if ok2:
            print(f"  set {tw}x{th} -> actual {fr.shape[1]}x{fr.shape[0]}")
        else:
            print(f"  set {tw}x{th} -> FAIL")

    # 恢复默认分辨率并拍清晰度测试帧
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    for _ in range(10):
        cap.read()
    ok3, frame2 = cap.read()
    if ok3:
        gray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
        sharp = cv2.Laplacian(gray, cv2.CV_64F).var()
        print(f"  测试帧清晰度(拉普拉斯方差): {sharp:.1f}")
        cv2.imwrite(str(OUT / f"test_{idx}.png"), frame2)
        print(f"  已存 {OUT / f'test_{idx}.png'}")
        # 上下半场清晰度对比(检查对焦平面)
        h2 = gray.shape[0] // 2
        print(f"  上半场: {cv2.Laplacian(gray[:h2], cv2.CV_64F).var():.1f}  "
              f"下半场: {cv2.Laplacian(gray[h2:], cv2.CV_64F).var():.1f}")
    cap.release()
    return (w, h)


print(f"OpenCV {cv2.__version__}")
found = {}
for backend, bname in [(cv2.CAP_DSHOW, "DSHOW"), (cv2.CAP_MSMF, "MSMF")]:
    print(f"\n########## 后端 {bname} ##########")
    for i in range(3):
        r = probe(i, backend)
        if r:
            found[(bname, i)] = r
print("\n汇总:", found)
