"""渲染 lattice RANSAC 效果图（供 inline 展示）：检测框 + 最终格号 + 来源着色。

颜色：绿=v4c确认  蓝=lattice补全  黄=lattice纠正  橙=冲突保留v4c  灰=重复剔除  红=未定/剔除
用法：python tools/ml/render_lattice.py
"""
from __future__ import annotations

import base64
import io
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
RESULTS = ROOT / "runs\lattice_ransac\lattice_ransac_results.json"
VAL_DIR = ROOT / r"capture\datasets\aerial_field_mixed\images\val"
OUT_DIR = ROOT / "runs\lattice_ransac"

FRAMES = ["frame_00175.jpg", "frame_00168.jpg"]  # 覆盖提升最大 + 检出最多
MAX_W = 900

SOURCE_COLOR = {
    "v4c+confirmed": (30, 180, 60),
    "lattice_filled": (40, 120, 250),
    "lattice_corrected": (240, 180, 0),
    "v4c_conflict(lattice_inlier)": (255, 130, 0),
    "v4c_conflict(high_conf)": (255, 130, 0),
    "v4c": (255, 130, 0),
    "dup_dropped": (160, 160, 160),
    "rejected": (220, 50, 50),
    "none": (220, 50, 50),
}


def load_font(size: int):
    for p in [r"C:\Windows\Fonts\msyhbd.ttc", r"C:\Windows\Fonts\msyh.ttc",
              r"C:\Windows\Fonts\arial.ttf"]:
        if Path(p).is_file():
            try:
                return ImageFont.truetype(p, size)
            except OSError:
                continue
    return ImageFont.load_default()


def render_frame(fs: dict, out_path: Path) -> None:
    img = Image.open(VAL_DIR / fs["name"]).convert("RGB")
    scale = MAX_W / img.width
    img = img.resize((MAX_W, int(img.height * scale)), Image.LANCZOS)
    draw = ImageDraw.Draw(img)
    font = load_font(15)
    for d in fs.get("dets", []):
        src = d.get("source", "none")
        color = SOURCE_COLOR.get(src, (220, 50, 50))
        x1 = (d["u"] - d["box_w"] / 2) * scale
        y1 = (d["v"] - d["box_h"] / 2) * scale
        x2 = (d["u"] + d["box_w"] / 2) * scale
        y2 = (d["v"] + d["box_h"] / 2) * scale
        draw.rectangle([x1, y1, x2, y2], outline=color, width=2)
        label = d.get("final_id") or d.get("v4c_id") or "?"
        if src == "lattice_filled":
            label += "◆"
        elif src == "lattice_corrected":
            label += "✚"
        elif src.startswith("v4c_conflict"):
            label += "⚠"
        bbox = draw.textbbox((x1, max(0, y1 - 18)), label, font=font)
        draw.rectangle([bbox[0] - 1, bbox[1] - 1, bbox[2] + 1, bbox[3] + 1], fill=color)
        draw.text((x1, max(0, y1 - 18)), label, fill=(255, 255, 255), font=font)
    # 顶部统计条
    n_final = sum(1 for d in fs["dets"] if d.get("final_id"))
    bar = (f"{fs['name']}  检出{fs['detected']}  v4c {fs['v4c_ok']}→最终 {n_final}"
           f"  覆盖 {fs['coverage_v4c']}%→{fs['coverage_lattice']}%  rms={fs.get('rms')}")
    draw.rectangle([0, 0, MAX_W, 30], fill=(20, 20, 20))
    draw.text((10, 6), bar, fill=(255, 255, 255), font=load_font(17))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=72)
    out_path.write_bytes(buf.getvalue())
    print(f"写出 {out_path} ({len(buf.getvalue()) // 1024} KB)")


def main() -> None:
    data = json.loads(RESULTS.read_text(encoding="utf-8"))
    by_name = {f["name"]: f for f in data["frames"]}
    for name in FRAMES:
        render_frame(by_name[name], OUT_DIR / f"lattice_{Path(name).stem}.jpg")


if __name__ == "__main__":
    main()
