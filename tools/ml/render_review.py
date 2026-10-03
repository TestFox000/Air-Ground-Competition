"""渲染全部帧的标签可视化 + 生成离线 HTML 浏览页（免人工逐帧翻标注软件）。

产物（<out>/，纯静态，双击 index.html 即可，file:// 直接可用）：
    big/<frame>.jpg     1280 宽，画全部类框 + 英文类名
    thumbs/<frame>.jpg  400 宽缩略图
    index.html          网格浏览；可筛选「未决橙格/缺桥洞」；点击看大图

标签只画高层语义类（mine/cell/arrow/bridge/4橙格/3桶），类 12 用醒目红框
标 ORANGE?。帧状态：
    has_orange  含类 12（高空小字未分类）
    no_bridge   不含类 3（边缘帧，桥洞没补上）

用法：
    python tools/ml/render_review.py --images capture/aerial_field/images \\
        --labels capture/aerial_field/labels --out capture/aerial_field/review
退出码：0 成功；2 依赖/参数。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("需要 opencv / numpy", file=sys.stderr)
    raise SystemExit(2)


# BGR 显示色 + 英文短名
_STYLE = {
    0:  ((0, 255, 255),  "mine"),
    1:  ((255, 0, 255),  "cell"),
    2:  ((0, 230, 0),    "arrow"),
    3:  ((0, 230, 230),  "bridge"),
    4:  ((0, 160, 255),  "car_start"),
    5:  ((0, 100, 255),  "car_end"),
    6:  ((0, 220, 255),  "uav_start"),
    7:  ((255, 180, 0),  "uav_end"),
    8:  ((0, 0, 255),    "bucket_red"),
    9:  ((90, 255, 90),  "bucket_green"),
    10: ((255, 220, 0),  "bucket_cyan"),
    12: ((0, 0, 255),    "ORANGE?"),
}
_SEMANTIC = set(_STYLE)


def draw_frame(image, lines, width: int):
    """画全部语义类框，缩放到指定宽；类 12 加粗加底衬。"""
    h0, w0 = image.shape[:2]
    scale = width / w0
    vis = cv2.resize(image, (width, int(h0 * scale)))
    for line in lines:
        values = line.split()
        cls = int(values[0])
        if cls not in _SEMANTIC:
            continue
        color, label = _STYLE[cls]
        cx, cy, bw, bh_ = (float(values[1]) * width, float(values[2]) * width,
                          float(values[3]) * width, float(values[4]) * width)
        x1, y1 = int(cx - bw / 2), int(cy - bh_ / 2)
        x2, y2 = int(cx + bw / 2), int(cy + bh_ / 2)
        thickness = 5 if cls == 12 else 3
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, thickness)
        ty = max(18, y1 - 6)
        cv2.putText(vis, label, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 0), 4)
        cv2.putText(vis, label, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, color, 2)
    return vis


# ---------------------------------------------------------------- HTML

def index_html(frames_meta: list[dict]) -> str:
    data = json.dumps(frames_meta, ensure_ascii=False)
    return """<!doctype html>
<html lang="zh"><head><meta charset="utf-8">
<title>aerial field labels review</title>
<style>
 body{font-family:system-ui;margin:0;background:#1b1d22;color:#ddd}
 .bar{padding:10px 14px;background:#22252c;position:sticky;top:0;z-index:5;
      display:flex;gap:8px;align-items:center;flex-wrap:wrap}
 button{background:#333844;color:#ddd;border:1px solid #454b57;border-radius:6px;
        padding:6px 12px;cursor:pointer}
 button.on{background:#3d6fb5;border-color:#5a8fd0}
 .stat{color:#9aa3b2;font-size:13px;margin-left:6px}
 .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(230px,1fr));gap:8px;padding:12px}
 .cell{position:relative;border-radius:6px;overflow:hidden;border:2px solid #333844;cursor:pointer;background:#000}
 .cell img{width:100%;display:block}
 .cell .tag{position:absolute;left:4px;top:4px;font-size:11px;font-weight:700;
      background:rgba(0,0,0,.7);padding:2px 6px;border-radius:4px}
 .cell.warn{border-color:#e05252}
 .cell.nobr{border-color:#d8a13a}
 .ov{position:fixed;inset:0;background:rgba(0,0,0,.82);display:none;z-index:10;
     align-items:center;justify-content:center;padding:20px}
 .ov img{max-width:100%;max-height:100%;border-radius:8px}
 .ov x{position:absolute;right:24px;top:14px;font-size:34px;color:#fff;cursor:pointer}
</style></head><body>
<div class="bar">
 <b>标签总览</b>
 <button data-f="all" class="on">全部</button>
 <button data-f="has_orange">未决橙格</button>
 <button data-f="no_bridge">缺桥洞</button>
 <button data-f="ok">正常帧</button>
 <span class="stat" id="st"></span>
</div>
<div class="grid" id="g"></div>
<div class="ov" id="ov"><x id="cx">&#10005;</x><img id="ovimg"></div>
<script>
const FRAMES = __DATA__;
const g=document.getElementById('g'), st=document.getElementById('st');
let filter='all';
function status(f){
 if(f.has_orange)return 'warn';
 if(f.no_bridge)return 'nobr';
 return 'ok';
}
function render(){
 const list=FRAMES.filter(f=>filter==='all'||
   (filter==='ok'&&status(f)==='ok')||
   (filter==='has_orange'&&f.has_orange)||
   (filter==='no_bridge'&&f.no_bridge));
 g.innerHTML='';
 for(const f of list){
   const d=document.createElement('div');
   d.className='cell '+status(f);
   const lab={warn:'未决橙格 '+f.n_orange,nobr:'缺桥洞',ok:'正常'}[status(f)];
   d.innerHTML='<img loading="lazy" src="thumbs/'+f.file+'"><span class="tag">'+f.idx+' · '+lab+'</span>';
   d.onclick=()=>{document.getElementById('ovimg').src='big/'+f.file;
     document.getElementById('ov').style.display='flex';};
   g.appendChild(d);
 }
 const wo=FRAMES.filter(f=>f.has_orange).length;
 const nb=FRAMES.filter(f=>f.no_bridge).length;
 st.textContent='共 '+FRAMES.length+' 帧；未决橙格帧 '+wo+'；缺桥洞帧 '+nb+'；当前显示 '+list.length;
}
document.querySelectorAll('button[data-f]').forEach(b=>b.onclick=()=>{
 document.querySelectorAll('button[data-f]').forEach(x=>x.classList.remove('on'));
 b.classList.add('on');filter=b.dataset.f;render();
});
document.getElementById('cx').onclick=()=>document.getElementById('ov').style.display='none';
document.getElementById('ov').onclick=e=>{if(e.target.id==='ov')e.target.style.display='none';};
render();
</script></body></html>
""".replace("__DATA__", data)


# ---------------------------------------------------------------- 主流程

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True)
    parser.add_argument("--labels", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    images_dir, labels_dir, out = (Path(args.images), Path(args.labels),
                                  Path(args.out))
    big_dir, thumb_dir = out / "big", out / "thumbs"
    big_dir.mkdir(parents=True, exist_ok=True)
    thumb_dir.mkdir(parents=True, exist_ok=True)

    meta = []
    for idx, path in enumerate(sorted(images_dir.glob("*.jpg"))):
        image = cv2.imread(str(path))
        label_path = labels_dir / (path.stem + ".txt")
        lines = label_path.read_text(
            encoding="utf-8").splitlines() if label_path.exists() else []
        classes = [int(line.split()[0]) for line in lines]

        big = draw_frame(image, lines, 1280)
        cv2.imwrite(str(big_dir / path.name), big,
                    [cv2.IMWRITE_JPEG_QUALITY, 88])
        thumb = draw_frame(image, lines, 400)
        cv2.imwrite(str(thumb_dir / path.name), thumb,
                    [cv2.IMWRITE_JPEG_QUALITY, 80])

        n_orange = classes.count(12)
        meta.append({
            "idx": idx, "file": path.name,
            "has_orange": n_orange > 0, "n_orange": n_orange,
            "no_bridge": 3 not in classes,
        })

    (out / "index.html").write_text(index_html(meta), encoding="utf-8")
    print(f"渲染 {len(meta)} 帧 → {out.resolve()}")
    print(f"打开：{(out / 'index.html').resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
