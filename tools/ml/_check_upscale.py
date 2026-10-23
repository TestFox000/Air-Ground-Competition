"""验证航拍4K图是否为1920x1200插值: 往返压缩(4K→1200p→4K)若几乎无损则为插值图."""
import cv2
import numpy as np

FRAMES = [
    r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\datasets\aerial_field_mixed\images\val\frame_00099.jpg",
    r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\datasets\aerial_field_mixed\images\val\frame_00170.jpg",
    r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\datasets\aerial_field_mixed\images\train\frame_00124.jpg",
]

def imread_unicode(p):
    return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)


for fp in FRAMES:
    img = imread_unicode(fp)
    h, w = img.shape[:2]
    # 往返: 4K -> 1920x1200 (传感器原生) -> 4K
    small = cv2.resize(img, (1920, 1200), interpolation=cv2.INTER_AREA)
    back = cv2.resize(small, (w, h), interpolation=cv2.INTER_LINEAR)
    diff = img.astype(np.float32) - back.astype(np.float32)
    mse = float((diff ** 2).mean())
    psnr = 10 * np.log10(255 ** 2 / mse) if mse > 0 else 99.0
    # JPEG噪声基线: 换成 无损往返 (原尺寸直接下上采样一个不可察觉的微小尺寸?) 用局部方差对比
    lap = cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
    name = fp.split("\\")[-1]
    print(f"{name}: {w}x{h}  往返PSNR={psnr:.2f}dB  原图清晰度={lap:.0f}")
    # PSNR>30dB => 原图几乎不含1200p以上的高频信息 => 插值图
print("\n判据: PSNR>30dB 基本可断定4K为插值; <25dB 说明含真实4K高频")
