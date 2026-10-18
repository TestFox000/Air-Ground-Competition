"""对比：无 L 列挡板 vs L 列挡板（仅 L2/L11 可通行），展示绕路代价。"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "src" / "ag_integrate"))

from PIL import Image, ImageDraw, ImageFont
from ag_integrate.field_frame import CellRef, COLS, ROWS, LETTERS
from ag_integrate.world_model import WorldSnapshot, TargetRef
from ag_integrate.mission_router import plan_route


def cell(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


MINES = ["I1", "A2", "G3", "B4", "H5", "E6", "C7", "G11"]
OBSTACLES = ["C2", "F4", "D7", "H8", "A9", "G9", "B10", "J11"]
PASSABLE = ["L2", "L11"]  # 随机通行格，router 自动推导 L 列墙
TARGETS_BLUE = ["D8", "E8"]
START = "N11"
GOAL = "N6"
L_GATES = set(PASSABLE)

snap_free = WorldSnapshot(
    timestamp_ms=0, mines=tuple(cell(c) for c in MINES), passable=(),
    obstacles=tuple(cell(c) for c in OBSTACLES),
    targets=tuple(TargetRef(cell(c), True) for c in TARGETS_BLUE),
)
snap_gate = WorldSnapshot(
    timestamp_ms=0, mines=tuple(cell(c) for c in MINES),
    passable=tuple(cell(c) for c in PASSABLE),
    obstacles=tuple(cell(c) for c in OBSTACLES),
    targets=tuple(TargetRef(cell(c), True) for c in TARGETS_BLUE),
)

route_free = plan_route(snap_free, cell(START), cell(GOAL), start_heading="y", final_heading="y")
route_gate = plan_route(snap_gate, cell(START), cell(GOAL), start_heading="y", final_heading="y")

print(f"无挡板:  {len(route_free)} 格")
print(f"L列挡板: {len(route_gate)} 格")

CELL_PX = 60
PAD = 50
GAP = 40
W = PAD * 2 + COLS * CELL_PX
H = PAD * 2 + ROWS * CELL_PX
CANVAS_W = W * 2 + GAP
CANVAS_H = H + 60

img = Image.new("RGB", (CANVAS_W, CANVAS_H), (25, 25, 25))
draw = ImageDraw.Draw(img)

try:
    font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 16)
    font_sm = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 11)
except Exception:
    font = ImageFont.load_default()
    font_sm = font


def cell_xy(c: CellRef, ox: int) -> tuple[int, int]:
    return ox + PAD + c.col * CELL_PX, PAD + c.row * CELL_PX


def draw_grid(ox: int):
    for r in range(ROWS + 1):
        y = PAD + r * CELL_PX
        draw.line([(ox + PAD, y), (ox + PAD + COLS * CELL_PX, y)], fill=(70, 70, 70), width=1)
    for c in range(COLS + 1):
        x = ox + PAD + c * CELL_PX
        draw.line([(x, PAD), (x, PAD + ROWS * CELL_PX)], fill=(70, 70, 70), width=1)
    for c in range(COLS):
        draw.text((ox + PAD + c * CELL_PX + CELL_PX // 2, PAD - 18), LETTERS[c], fill="white", font=font_sm, anchor="mm")
    for r in range(ROWS):
        draw.text((ox + PAD - 18, PAD + r * CELL_PX + CELL_PX // 2), str(r + 1), fill="white", font=font_sm, anchor="mm")


def draw_cells(ox: int, show_l_wall: bool):
    if show_l_wall:
        for r in range(ROWS):
            name = f"L{r + 1}"
            c = cell(name)
            x, y = cell_xy(c, ox)
            if name in L_GATES:
                draw.rectangle([x + 3, y + 3, x + CELL_PX - 3, y + CELL_PX - 3], fill=(80, 60, 120))
                draw.text((x + CELL_PX // 2, y + CELL_PX // 2), "洞", fill="white", font=font_sm, anchor="mm")
            else:
                draw.rectangle([x + 3, y + 3, x + CELL_PX - 3, y + CELL_PX - 3], fill=(60, 60, 60))
                draw.text((x + CELL_PX // 2, y + CELL_PX // 2), "墙", fill=(150, 150, 150), font=font_sm, anchor="mm")
    for name in OBSTACLES:
        c = cell(name)
        x, y = cell_xy(c, ox)
        draw.rectangle([x + 3, y + 3, x + CELL_PX - 3, y + CELL_PX - 3], fill=(160, 40, 40))
        draw.text((x + CELL_PX // 2, y + CELL_PX // 2), "障", fill="white", font=font_sm, anchor="mm")
    for name in MINES:
        c = cell(name)
        x, y = cell_xy(c, ox)
        draw.rectangle([x + 3, y + 3, x + CELL_PX - 3, y + CELL_PX - 3], fill=(210, 190, 30))
        draw.text((x + CELL_PX // 2, y + CELL_PX // 2 - 6), "雷", fill="black", font=font_sm, anchor="mm")
    for name in TARGETS_BLUE:
        c = cell(name)
        x, y = cell_xy(c, ox)
        draw.rectangle([x + 3, y + 3, x + CELL_PX - 3, y + CELL_PX - 3], fill=(150, 60, 200))
        draw.text((x + CELL_PX // 2, y + CELL_PX // 2 - 6), "靶", fill="white", font=font_sm, anchor="mm")
    sc = cell(START)
    x, y = cell_xy(sc, ox)
    draw.rectangle([x + 4, y + 4, x + CELL_PX - 4, y + CELL_PX - 4], outline=(0, 255, 0), width=3)
    gc = cell(GOAL)
    x, y = cell_xy(gc, ox)
    draw.rectangle([x + 4, y + 4, x + CELL_PX - 4, y + CELL_PX - 4], outline=(255, 120, 0), width=3)


def draw_route(ox: int, route: tuple[str, ...]):
    cells = [cell(n) for n in route]
    for i in range(len(cells) - 1):
        a, b = cells[i], cells[i + 1]
        x1, y1 = cell_xy(a, ox)
        x2, y2 = cell_xy(b, ox)
        crossed = (a.col < 11 <= b.col) or (b.col < 11 <= a.col)  # 跨越 L 列
        color = (255, 230, 0) if crossed else (120, 220, 120)
        draw.line([(x1 + CELL_PX // 2, y1 + CELL_PX // 2), (x2 + CELL_PX // 2, y2 + CELL_PX // 2)],
                  fill=color, width=4)
    for n in route:
        c = cell(n)
        x, y = cell_xy(c, ox)
        cx, cy = x + CELL_PX // 2, y + CELL_PX // 2
        draw.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill=(255, 255, 255), outline=(0, 0, 0), width=1)


def path_length(route: tuple[str, ...]) -> float:
    cells = [cell(n) for n in route]
    return sum(((b.col - a.col) ** 2 + (b.row - a.row) ** 2) ** 0.5 for a, b in zip(cells, cells[1:]))


ox0 = 0
draw.text((ox0 + PAD, 12), f"无 L 列挡板  {len(route_free)} 格 / {path_length(route_free):.1f} 格长",
          fill=(120, 220, 120), font=font)
draw_grid(ox0)
draw_cells(ox0, show_l_wall=False)
draw_route(ox0, route_free)

ox1 = W + GAP
draw.text((ox1 + PAD, 12), f"L 列挡板（仅 L2/L11 通行）  {len(route_gate)} 格 / {path_length(route_gate):.1f} 格长",
          fill=(255, 120, 120), font=font)
draw_grid(ox1)
draw_cells(ox1, show_l_wall=True)
draw_route(ox1, route_gate)

draw.text((PAD, H + 10), "黄线 = 跨越 L 列的那一步（穿桥洞）", fill="white", font=font_sm)
draw.text((ox1 + PAD, H + 10), f"绕路 +{len(route_gate) - len(route_free)} 格：必须经 L11 进、L2 出（或反之）",
          fill=(255, 200, 100), font=font_sm)

out = Path(__file__).parent / "datasets" / "cell_labels" / "route_compare.png"
out.parent.mkdir(parents=True, exist_ok=True)
img.save(out)
print(f"\n对比图: {out}")
