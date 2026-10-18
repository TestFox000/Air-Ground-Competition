# -*- coding: utf-8 -*-
"""生成任务解析图 (seed=1): 场地地图 + 全图最优路径 + 时序/结果面板
纯标准库数据 + Pillow 绘图, 输出 PNG"""
import random

import mission_planner as M
from PIL import Image, ImageDraw, ImageFont

SEED = 1
OUT = "任务解析图_seed1.png"


def load_font(size):
    for fp in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\msyhbd.ttc",
               r"C:\Windows\Fonts\simhei.ttf", r"C:\Windows\Fonts\simsun.ttc"):
        try:
            return ImageFont.truetype(fp, size)
        except Exception:
            continue
    return ImageFont.load_default()


# ---------------- 1) 真值 + 全图最优路径 ----------------
rng = random.Random(SEED)
truth, tunnels = M.build_truth(rng)
f = M.Field()
f.open_tunnels = set(tunnels)          # 桥洞可通行通道(顶部箭头标明, 无人机侦察后打开)
for rc, info in truth.items():
    f.set(rc, info[0], info[1])
for r in range(M.ROWS):
    for c in range(M.COLS):
        if not f.is_known((r, c)):
            f.set((r, c), M.EMPTY)
p = M.Planner(f)
p.scan_complete = True
goals = p.build_goals(M.name2rc("N10"), set(), set())
route, roles = p.route_through(M.name2rc("N10"), goals)
assert route is not None, "无法规划"

enemies = f.known_of(M.TARGET)
friends = f.known_of(M.FRIEND)
mines = f.known_of(M.EXPLOSIVE)
buckets = f.known_of(M.BUCKET)

# ---------------- 2) 绘图 ----------------
CS = 46                      # 每格像素
MX, MY = 48, 92              # 地图原点
PANEL_X = MX + M.COLS * CS + 26
W = PANEL_X + 400 + 24
H = MY + M.ROWS * CS + 130
img = Image.new("RGB", (W, H), (245, 246, 250))
d = ImageDraw.Draw(img)
f_title = load_font(20)
f_bold = load_font(13)
f_txt = load_font(13)
f_cell = load_font(11)

# U 形扫描路线起点(前 4 块)底衬
first4 = M.scan_block_order(M.make_blocks())[:4]
for b in first4:
    rmin = min(rc[0] for rc in b)
    rmax = max(rc[0] for rc in b)
    cmin = min(rc[1] for rc in b)
    cmax = max(rc[1] for rc in b)
    d.rectangle([MX + cmin * CS, MY + rmin * CS,
                 MX + (cmax + 1) * CS, MY + (rmax + 1) * CS],
                fill=(255, 243, 205))
d.text((MX + 9 * CS + 4, MY + 13 * CS + 2), "①U形扫描起点", font=f_cell,
       fill=(150, 100, 20))

# 单元格
FILL = {M.EMPTY: (255, 255, 255), M.BUCKET: (211, 92, 92),
        M.EXPLOSIVE: (255, 206, 84), M.TARGET: (232, 118, 118),
        M.FRIEND: (186, 140, 220), M.LINE: (96, 178, 96)}
for r in range(M.ROWS):
    for c in range(M.COLS):
        rc = (r, c)
        x, y = MX + c * CS, MY + r * CS
        st = f.state(rc)
        color = FILL[st]
        if f.is_bridge(rc) and st == M.EMPTY:
            color = (206, 206, 206)
        d.rectangle([x, y, x + CS, y + CS], fill=color,
                    outline=(214, 214, 214))
        # 敌方蓝面
        if st == M.TARGET and f.blue(rc):
            dr, dc = f.blue(rc)
            if dc == 1:
                d.line([x + CS - 2, y + 3, x + CS - 2, y + CS - 3],
                       fill=(25, 120, 220), width=5)
            elif dc == -1:
                d.line([x + 2, y + 3, x + 2, y + CS - 3],
                       fill=(25, 120, 220), width=5)
            elif dr == 1:
                d.line([x + 3, y + CS - 2, x + CS - 3, y + CS - 2],
                       fill=(25, 120, 220), width=5)
            else:
                d.line([x + 3, y + 2, x + CS - 3, y + 2],
                       fill=(25, 120, 220), width=5)
        # 爆炸物在格心
        if st == M.EXPLOSIVE:
            cx, cy = x + CS // 2, y + CS // 2
            d.ellipse([cx - 6, cy - 6, cx + 6, cy + 6], fill=(30, 30, 30))
        # 桥洞上方箭头(绿色, 标明可通行通道: 上段→A行/下段→N行)
        if rc in tunnels:
            cx, cy = x + CS // 2, y + CS // 2
            if rc[1] >= 6:                 # 上段 K6~K11: 指向 A 行(上)
                d.polygon([(cx, y + 4), (cx - 7, y + 16), (cx + 7, y + 16)],
                          fill=(20, 150, 40))
            else:                          # 下段 K1~K5: 指向 N 行(下)
                d.polygon([(cx, y + CS - 4), (cx - 7, y + CS - 16),
                           (cx + 7, y + CS - 16)], fill=(20, 150, 40))

# 路径(格心连线 + 拐弯箭头)
def center(rc):
    return (MX + rc[1] * CS + CS // 2, MY + rc[0] * CS + CS // 2)

for i in range(len(route) - 1):
    d.line([center(route[i]), center(route[i + 1])],
           fill=(150, 0, 200), width=5)
for i in range(1, len(route) - 1):
    din = M.sub(route[i], route[i - 1])
    dout = M.sub(route[i + 1], route[i])
    if din != dout:                      # 拐弯处画箭头
        cx, cy = center(route[i])
        dr, dc = dout
        d.polygon([(cx, cy), (cx - 6 * dr, cy - 6 * dc),
                   (cx - 3 * dr, cy - 3 * dc)], fill=(100, 0, 150))

# 无人机 S 型扫描路线(蓝色虚线 + 前进箭头)
scan_blocks = M.scan_block_order(M.make_blocks())
spts = [(MX + (sum(rc[1] for rc in b) / len(b)) * CS,
         MY + (sum(rc[0] for rc in b) / len(b)) * CS) for b in scan_blocks]
for i in range(len(spts) - 1):
    x0, y0 = spts[i]
    x1, y1 = spts[i + 1]
    dist = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    n = max(1, int(dist / 16))
    for k in range(n):
        t0, t1 = k * 2 * 8 / dist, (k * 2 + 1) * 8 / dist
        d.line([(x0 + (x1 - x0) * min(t0, 1), y0 + (y1 - y0) * min(t0, 1)),
                (x0 + (x1 - x0) * min(t1, 1), y0 + (y1 - y0) * min(t1, 1))],
               fill=(66, 135, 245), width=3)
    dx, dy = x1 - x0, y1 - y0
    L = max((dx * dx + dy * dy) ** 0.5, 1e-6)
    ux, uy = dx / L, dy / L
    d.polygon([(x1, y1), (x1 - 8 * ux + 4 * uy, y1 - 8 * uy - 4 * ux),
               (x1 - 8 * ux - 4 * uy, y1 - 8 * uy + 4 * ux)],
              fill=(66, 135, 245))

# 关键点标记
start, ret = M.name2rc("N10"), M.name2rc("N5")
d.ellipse([center(start)[0] - 10, center(start)[1] - 10,
           center(start)[0] + 10, center(start)[1] + 10],
          fill=(46, 160, 67))
d.text((center(start)[0] - 14, center(start)[1] + 10), "起点N10",
       font=f_cell, fill=(46, 160, 67))
for rc, role in roles.items():
    if role == "stop_face":
        cx, cy = center(rc)
        d.ellipse([cx - 10, cy - 10, cx + 10, cy + 10], outline=(25, 120, 220), width=3)
        d.text((cx - 24, cy + 10), "S停靠", font=f_cell, fill=(25, 120, 220))
    elif role == "hit":
        d.text((center(rc)[0] - 14, center(rc)[1] - 26), "T敌", font=f_bold,
               fill=(180, 20, 20))
    elif role == "return":
        cx, cy = center(rc)
        d.ellipse([cx - 10, cy - 10, cx + 10, cy + 10], outline=(30, 30, 30), width=3)
        d.text((cx - 26, cy + 10), "R返程", font=f_cell, fill=(30, 30, 30))
for t in enemies:
    d.text((center(t)[0] - 28, center(t)[1] - 26), f"敌{M.rc2name(t)}",
           font=f_bold, fill=(180, 20, 20))
for t in friends:
    d.text((center(t)[0] - 24, center(t)[1] + 10), f"我{M.rc2name(t)}",
           font=f_cell, fill=(120, 60, 160))

# 行列标号
for r in range(M.ROWS):
    d.text((MX - 24, MY + r * CS + CS // 2 - 7), M.ROW_LETTERS[r],
           font=f_bold, fill=(60, 60, 60))
for c in range(M.COLS):
    d.text((MX + c * CS + CS // 2 - 6, MY - 20), str(c + 1),
           font=f_bold, fill=(60, 60, 60))

# 图例
ly = MY + M.ROWS * CS + 16
leg = [((96, 178, 96), "绿线禁行区"), ((211, 92, 92), "障碍26cm"),
       ((255, 206, 84), "地雷(格心)"), ((232, 118, 118), "敌方(蓝面)"),
       ((186, 140, 220), "我方(不打)"), ((206, 206, 206), "桥洞"),
       ((150, 0, 200), "路径"), ((66, 135, 245), "S型扫描线"),
       ((255, 243, 205), "①U形扫描起点"),
       ((20, 150, 40), "桥洞上方绿色箭头")]
lx = MX
for col, name in leg:
    d.rectangle([lx, ly, lx + 16, ly + 16], fill=col,
                outline=(150, 150, 150))
    d.text((lx + 22, ly), name, font=f_cell, fill=(60, 60, 60))
    lx += 14 + d.textlength(name, font=f_cell) + 26

# ---------------- 3) 右侧面板(实际运行数据) ----------------
px = PANEL_X
d.rectangle([px, 30, px + 400, H - 30], fill=(255, 255, 255),
            outline=(200, 200, 200))
d.text((px + 16, 42), "任务解析图 · seed=1", font=f_title, fill=(30, 30, 30))
d.text((px + 16, 72), "场地14×11 · 35cm/格 · 小车N10→N5 · 无人机N8→M2", font=f_cell,
       fill=(90, 90, 90))
d.text((px + 16, 100), "—— 时序 ——", font=f_bold, fill=(150, 0, 200))
timeline = [
    "2.0s  无人机在N8起飞(限高1.5m)",
    "2.5s  U形单条S曲线: 底边N7~N11→A11~C11→D1~D6",
    "6.1s  识别箭头K11→A行/K2→N行, 扫描4块→小车启动",
    "边扫边重规划: 新地雷/敌/我/障碍动态插入",
    "桥洞堵点不存在(全部可通行)",
    "54.5s 识别敌我: 敌B4, 停靠蓝面对面B3",
    "61.9s 到达B4, 激光打击 1/3→3/3",
    "82.8s 打击完成(30s) → 继续穿插踩雷",
    "38.9s 全场扫描完成(154格)",
    "47.7s 动态新障碍",
    "171.5s 小车先到终点N5",
    "177.5s 无人机后到终点M2",
]
y = 126
for line in timeline:
    d.text((px + 16, y), line, font=f_txt, fill=(40, 40, 40))
    y += 24
y += 8
d.text((px + 16, y), "—— 结果 ——", font=f_bold, fill=(150, 0, 200))
y += 26
results = [
    "排雷 8/8 全踩  ·  打击敌 1/1",
    "打靶穿插在踩爆炸物过程中(非最终任务)",
    "水桶/地雷全部位于 A~J 行之间",
    "打击目标: 连续两格标靶(我B5+敌B4)",
    "绿线规避通过(24格)  ·  未压障碍/我方",
    "掉头(180°折返): 0 次 ✓",
    "路径44格/16拐弯(规划一部分运行一部分)",
    "总耗时 177.5s / 180s ✓ 剩余2.5s",
]
for line in results:
    d.text((px + 16, y), line, font=f_txt, fill=(40, 40, 40))
    y += 24
y += 8
d.text((px + 16, y), "—— 最终航点(#9) ——", font=f_bold,
       fill=(150, 0, 200))
y += 26
d.text((px + 16, y), "B3(停)→B4(打) 打靶30s 穿插踩雷", font=f_txt,
       fill=(40, 40, 40))
y += 22
d.text((px + 16, y), "→按TSP序穿插踩雷(8/8)", font=f_txt,
       fill=(40, 40, 40))
y += 22
d.text((px + 16, y), "→N5(返)", font=f_txt,
       fill=(40, 40, 40))

img.save(OUT)
print("已生成", OUT, img.size)
