"""用裁判系统地图演示完整路径规划 + 可视化。"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "src" / "ag_integrate"))

from PIL import Image, ImageDraw, ImageFont
from ag_integrate.field_frame import CellRef, COLS, ROWS, LETTERS
from ag_integrate.world_model import WorldSnapshot, TargetRef
from ag_integrate.mission_router import plan_route, compress_route
from ag_integrate.car_adapter import adapt


def cell(name: str) -> CellRef:
    col = LETTERS.index(name[0])
    row = int(name[1:]) - 1
    return CellRef(col, row)


# === 裁判系统地图（按用户标注图重设）===
# 雷（黄）
MINES = ["I1", "A2", "G3", "B4", "H5", "E6", "C7", "G11"]
# 障碍（红"障"）——注意：桥洞列的墙不写死，由 passable 自动推导
OBSTACLES = ["C2", "F4", "D7", "H8", "A9", "G9", "B10", "J11"]
# 桥洞列：默认 L（与 docs/interface_contract.md 一致；现场拓扑待核实，
# 若实际场地桥洞在其他列，改 BRIDGE_COL 与 PASSABLE 即可）
BRIDGE_COL = "L"
# 桥洞可通行格（带箭头，每局随机）——router 自动把桥洞列其余格推为墙
PASSABLE = [f"{BRIDGE_COL}2", f"{BRIDGE_COL}11"]
# 移动靶（全场仅一个）：双面靶，蓝面=敌（打），红面=友（不打），
# 沿轨道在两格间匀速往返。轨道两端与蓝面朝向每局由裁判系统地图/现场标定。
TRACK = ("D8", "E8")   # 轨道两端格（水平轨道）
BLUE_FACE = "S"        # 蓝面朝南（车在南侧停靠可见蓝面）
START = "N11"  # 智能车起点（东半场）
GOAL = "N6"    # 智能车终点（东半场）

snapshot = WorldSnapshot(
    timestamp_ms=0,
    mines=tuple(cell(c) for c in MINES),
    passable=tuple(cell(c) for c in PASSABLE),
    obstacles=tuple(cell(c) for c in OBSTACLES),
    targets=(TargetRef(cell(TRACK[0]), True,
                       track=(cell(TRACK[0]), cell(TRACK[1])),
                       blue_face=BLUE_FACE),),
)

route = plan_route(
    snapshot, cell(START), cell(GOAL),
    start_heading="y",   # 车摆在 N11 朝北（向场地内）
    final_heading="y",   # 终点朝向拿 5 分
)
print(f"路径 ({len(route)} 格):")
print(" → ".join(route))

# === 统计：直线长度 / 转向次数 ===
cells = [cell(name) for name in route]
length = 0.0
turns = 0
turn_units_total = 0
prev_dir = None
for a, b in zip(cells, cells[1:]):
    d = (b.col - a.col, b.row - a.row)
    length += (d[0] * d[0] + d[1] * d[1]) ** 0.5
    if prev_dir is not None:
        dd = abs(d[0] - prev_dir[0]) + abs(d[1] - prev_dir[1])
        if dd == 1:
            turns += 1
            turn_units_total += 1
        elif dd == 2:
            turns += 1
            turn_units_total += 2
    prev_dir = d
print(f"直线长度 {length:.2f} 格 | 转向 {turns} 次（{turn_units_total}×45°）| 重复踩格 {len(route) - len(set(route))}")

# === 共线压缩：删掉同向直线段的中间格，命令更短、轨迹不变 ===
# KEEP 须含移动靶的停靠射击格（蓝面侧轨道邻格），防止停靠点被当中间格删掉
def _orthonames(name: str) -> list[str]:
    c = cell(name)
    out = []
    for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        if 0 <= c.col + dc < COLS and 0 <= c.row + dr < ROWS:
            out.append(CellRef(c.col + dc, c.row + dr).name)
    return out

_ta, _tb = cell(TRACK[0]), cell(TRACK[1])
if _ta.row == _tb.row:      # 水平轨道，蓝面 S(row+1)/N(row-1)
    _d = 1 if BLUE_FACE == "S" else -1
    SHOOT_KEEP = {CellRef(_ta.col, _ta.row + _d).name, CellRef(_tb.col, _tb.row + _d).name}
else:                        # 垂直轨道，蓝面 E(col+1)/W(col-1)
    _d = 1 if BLUE_FACE == "E" else -1
    SHOOT_KEEP = {CellRef(_ta.col + _d, _ta.row).name, CellRef(_tb.col + _d, _tb.row).name}
KEEP = set(MINES) | set(TRACK) | set(PASSABLE) | SHOOT_KEEP
compact = compress_route(route, KEEP)
print(f"\n压缩后 ({len(compact)} 点，原 {len(route)} 格):")
print(" → ".join(compact))

# === 生成车端 path_executor 命令（生产链路：禁火保险 + 每靶只停一次） ===
cmds = adapt(snapshot, compact, START, "y", "y")
shoot = list(cmds.shoot_cells)
print(f"停靠射击格: {shoot}（移动靶轨道 {TRACK[0]}↔{TRACK[1]}，蓝面朝 {'南' if BLUE_FACE == 'S' else '北'}）")
print(f"\nreset: {cmds.reset}")
print(f"route: {cmds.route}")

# === 验证：必经点全覆盖 + 不穿障碍 + 轨道不踩 + 射击停靠 ===
route_set = set(route)
missing = [c for c in MINES if c not in route_set]
violations = [c for c in route if c in OBSTACLES]
tread = [c for c in TRACK if c in route_set]
shoot_spots = sorted(SHOOT_KEEP & route_set)
print(f"\n排雷覆盖: {'全部覆盖 ✓' if not missing else f'缺失 {missing}'}")
print(f"障碍避让: {'未穿障 ✓' if not violations else f'穿障 {violations}'}")
print(f"轨道避让: {'未踩轨道 ✓' if not tread else f'踩轨道 {tread}'}")
print(f"射击停靠: {shoot_spots} ✓" if shoot_spots else "射击停靠: 缺失 ✗")
# 验证穿越桥洞列只走 PASSABLE 格
crossings = [(a, b) for a, b in zip(route, route[1:])
             if (a[0] == BRIDGE_COL or b[0] == BRIDGE_COL) and a[0] != b[0]]
gate_crossings = [c for pair in crossings for c in pair if c[0] == BRIDGE_COL and c not in PASSABLE]
print(f"桥洞穿越: {'仅走 ' + '/'.join(PASSABLE) + ' ✓' if not gate_crossings else f'违规穿越 {gate_crossings}'}")

# === 可视化 ===
CELL_PX = 60
PAD = 50
W = PAD * 2 + COLS * CELL_PX
H = PAD * 2 + ROWS * CELL_PX

img = Image.new("RGB", (W, H + 16), (30, 30, 30))
draw = ImageDraw.Draw(img)

try:
    font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 14)
    font_sm = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 10)
    font_tiny = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 9)
except Exception:
    font = ImageFont.load_default()
    font_sm = font
    font_tiny = font


def cell_xy(c: CellRef) -> tuple[int, int]:
    x = PAD + c.col * CELL_PX
    y = PAD + c.row * CELL_PX
    return x, y


def gradient_color(t: float) -> tuple[int, int, int]:
    """t in [0,1], 绿→黄→红 渐变，表示路径先后。"""
    if t < 0.5:
        r = int(255 * (t * 2))
        g = 255
        b = 0
    else:
        r = 255
        g = int(255 * (1 - (t - 0.5) * 2))
        b = 0
    return (r, g, b)


# 画网格
for r in range(ROWS + 1):
    y = PAD + r * CELL_PX
    draw.line([(PAD, y), (PAD + COLS * CELL_PX, y)], fill=(80, 80, 80), width=1)
for c in range(COLS + 1):
    x = PAD + c * CELL_PX
    draw.line([(x, PAD), (x, PAD + ROWS * CELL_PX)], fill=(80, 80, 80), width=1)

# 画列字母和行号
for c in range(COLS):
    draw.text((PAD + c * CELL_PX + CELL_PX // 2, PAD - 20), LETTERS[c], fill="white", font=font_sm, anchor="mm")
for r in range(ROWS):
    draw.text((PAD - 20, PAD + r * CELL_PX + CELL_PX // 2), str(r + 1), fill="white", font=font_sm, anchor="mm")

# 画障碍（红）
for name in OBSTACLES:
    c = cell(name)
    x, y = cell_xy(c)
    draw.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], fill=(180, 40, 40))
    draw.text((x + CELL_PX // 2, y + CELL_PX // 2), "障", fill="white", font=font, anchor="mm")

# 画桥洞列：passable 格为可通行（浅紫"洞"），其余为墙（深灰"墙"）
L_GATES = set(PASSABLE)
L_COL = LETTERS.index(BRIDGE_COL)
for r in range(ROWS):
    name = f"{BRIDGE_COL}{r + 1}"
    c = cell(name)
    x, y = cell_xy(c)
    if name in L_GATES:
        draw.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], fill=(80, 60, 120))
        draw.text((x + CELL_PX // 2, y + CELL_PX // 2), "洞", fill="white", font=font, anchor="mm")
    else:
        draw.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], fill=(60, 60, 60))
        draw.text((x + CELL_PX // 2, y + CELL_PX // 2), "墙", fill=(150, 150, 150), font=font_sm, anchor="mm")

# 画雷（黄）
for name in MINES:
    c = cell(name)
    x, y = cell_xy(c)
    draw.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], fill=(220, 200, 30))
    draw.text((x + CELL_PX // 2, y + CELL_PX // 2 - 6), "雷", fill="black", font=font, anchor="mm")

# 画桥洞通行箭头标记（桥洞列前一列、与门同行的"通"指向桥洞格）
APPROACH_COL = LETTERS[L_COL - 1]  # 桥洞列西侧相邻列（默认 L → K）
for gate in PASSABLE:
    name = f"{APPROACH_COL}{int(gate[1:])}"
    c = cell(name)
    x, y = cell_xy(c)
    draw.text((x + CELL_PX // 2, y + CELL_PX // 2), "→", fill=(40, 200, 80), font=font, anchor="mm")

# 画移动靶轨道（两格，靶沿轨道匀速往返）+ 蓝面朝向标注
for name in TRACK:
    c = cell(name)
    x, y = cell_xy(c)
    draw.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], fill=(95, 75, 140))
    draw.text((x + CELL_PX // 2, y + CELL_PX // 2 - 6), "轨", fill="white", font=font, anchor="mm")
# 轨道上方画往返双箭头
_ta, _tb = cell(TRACK[0]), cell(TRACK[1])
xa, ya = cell_xy(_ta); xb, yb = cell_xy(_tb)
mid_x = (xa + xb) // 2 + CELL_PX // 2
off = CELL_PX // 3 if _ta.row == _tb.row else 0
draw.text((mid_x, ya + CELL_PX // 2 - off), "⇄", fill=(255, 230, 130), font=font, anchor="mm")
# 蓝面侧文字标注（停靠射击发生在这一侧）
face_x = mid_x
face_y = ya + CELL_PX * (1.15 if BLUE_FACE == "S" else -0.3) if _ta.row == _tb.row else ya + CELL_PX // 2
draw.text((face_x, face_y), "蓝面→", fill=(120, 180, 255), font=font_sm, anchor="mm")

# 统计每个格子被经过的次数
from collections import Counter
visit_count = Counter(route)

# 画连续路径线（按顺序，颜色渐变）
route_cells = [cell(name) for name in route]
for i in range(len(route_cells) - 1):
    c1 = route_cells[i]
    c2 = route_cells[i + 1]
    x1, y1 = cell_xy(c1)
    x2, y2 = cell_xy(c2)
    color = gradient_color(i / max(len(route_cells) - 2, 1))
    draw.line(
        [(x1 + CELL_PX // 2, y1 + CELL_PX // 2), (x2 + CELL_PX // 2, y2 + CELL_PX // 2)],
        fill=color,
        width=4,
    )

# 画压缩后保留的关键点（大圆点）；被删的共线中间格画小灰点
compact_set = set(compact)
for i, name in enumerate(route):
    c = cell(name)
    x, y = cell_xy(c)
    cx, cy = x + CELL_PX // 2, y + CELL_PX // 2
    if name in compact_set:
        color = gradient_color(i / max(len(route) - 1, 1))
        r = 7
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=color, outline="white", width=1)
        cnt = visit_count[name]
        if cnt > 1:
            draw.text((cx + 10, cy - 10), f"×{cnt}", fill=(255, 255, 100), font=font_tiny, anchor="mm")
    else:
        r = 3
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(150, 150, 150))

# 画射击停靠格：金色准星 + 指向正对轨道端点的短箭头（车头正对靶心）
import math
for name in shoot:
    c = cell(name)
    x, y = cell_xy(c)
    cx, cy = x + CELL_PX // 2, y + CELL_PX // 2
    for t in [t for t in TRACK if name in _orthonames(t)]:
        tc = cell(t)
        tx, ty = cell_xy(tc)
        tcx, tcy = tx + CELL_PX // 2, ty + CELL_PX // 2
        ang = math.atan2(tcy - cy, tcx - cx)
        sx, sy = cx + 13 * math.cos(ang), cy + 13 * math.sin(ang)
        ex, ey = cx + 26 * math.cos(ang), cy + 26 * math.sin(ang)
        draw.line([(sx, sy), (ex, ey)], fill=(255, 200, 0), width=3)
    draw.ellipse([cx - 11, cy - 11, cx + 11, cy + 11], outline=(255, 200, 0), width=2)
    draw.line([(cx - 6, cy - 6), (cx + 6, cy + 6)], fill=(255, 200, 0), width=2)
    draw.line([(cx - 6, cy + 6), (cx + 6, cy - 6)], fill=(255, 200, 0), width=2)

# 画起点终点
sc = cell(START)
gc = cell(GOAL)
sx, sy = cell_xy(sc)
draw.rectangle([sx + 4, sy + 4, sx + CELL_PX - 4, sy + CELL_PX - 4], outline=(0, 255, 0), width=3)
draw.text((sx + CELL_PX // 2, sy + CELL_PX - 8), "起点", fill=(0, 255, 0), font=font_sm, anchor="mm")
gx, gy = cell_xy(gc)
draw.rectangle([gx + 4, gy + 4, gx + CELL_PX - 4, gy + CELL_PX - 4], outline=(255, 100, 0), width=3)
draw.text((gx + CELL_PX // 2, gy + CELL_PX - 8), "终点", fill=(255, 100, 0), font=font_sm, anchor="mm")

# 图例
legend_y = H - 20
draw.text((PAD, legend_y), "绿→黄→红 = 路径先后顺序", fill="white", font=font_sm)
draw.text((PAD + 250, legend_y), "×N = 该格经过次数", fill=(255, 255, 100), font=font_sm)
draw.text((PAD + 430, legend_y), f"{BRIDGE_COL}列: 墙=挡板/洞=桥洞通行格", fill=(200, 180, 255), font=font_sm)
legend_y2 = H + 2
draw.text((PAD, legend_y2), "轨=移动靶轨道(双面:蓝面朝准星侧,红面朝对侧) ⊕=停靠射击格(箭头指向正对端点)", fill=(255, 200, 100), font=font_sm)

out = Path(__file__).parent / "datasets" / "cell_labels" / "route_demo.png"
out.parent.mkdir(parents=True, exist_ok=True)
img.save(out)
print(f"\n可视化: {out}")
print(f"重复经过的格子: { {k: v for k, v in visit_count.items() if v > 1} }")
