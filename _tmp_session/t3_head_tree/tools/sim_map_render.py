# -*- coding: utf-8 -*-
r"""sim_map_render.py —— 把「仿真跑过的路线」渲染成场地 PNG 图（横版）。

============================== 用户定版场地规则（2026-10） ==============================
① 场地 14 列 A~N × 11 行 1~11，每格 0.35 m（35×35 cm），整张图【横着放】(landscape)：
   长边（14 格）横向、用 A~N 命名；宽边（11 格）纵向、用 1~11 命名。不许竖着画。
② 代码坐标（geometry.block_to_meters）：
      x = (10.5 - (row-1)) * 0.35   行 1 → 3.675 … 行 11 → 0.175
      y = (13.5 - col_idx) * 0.35   A(col 0) → 4.725 … N(col 13) → 0.175
   屏幕变换（横版）：
      sx = 4.9 - y_code   → A 在【左】、N 在【右】
      sy = x_code         → 1 行在【上】、11 行在【下】
      θ_s = θ_code + 90°  → 代码 yaw=+90°（面朝 A 列）画出来是【朝左】
   自检锚点：N10 必须在图【右下角】，起点车头朝 A 列一侧 = 朝左。
③ 隔板：第 6/7 行之间、D~N 列【整条封死】，不允许触碰、不允许跨越；
   只有 A/B/C 三列（隔板西端）能上下穿行。画成连续粗黑线，【不在 K 列留缺口】。
④ K 列：整列是一座桥，每一格都是一个桥洞、彼此被侧壁隔开；
   横穿 K 列【不必强制直行】（用户 2026-10-11 口径：纯追踪 + 调好速度/前视即可平稳通过）。本局开放格 = map.bridges（**随图变**，如第三张图 K1/K11，青色），
   其余 K 格画成封闭桥室（浅灰）。穿越顺序：先 L→K→J（下半场向西），后 J→K→L（上半场向东）。
⑤ 起点 N10、终点 N5。射击停站（token 带 *）画品红星；折返 PIVOT（:PIVOT）画紫菱
   （真车底盘不支持倒车，折返一律 PIVOT 原地掉头）。
⑥ 小车体积：仿真与绘图必须考虑车体长宽。技术文档目前只有【轴距 0.157 m】
   （最小转弯半径 0.272 m），车长/车宽文档明写"需上实车测量"——
   下方 CAR_LEN_M / CAR_WID_M 是占位值，拿到实测尺寸后改这两个常量即可。

图例：灰格=障(水桶) · 红圈=雷(必须压过) · 橙块=一块靶板(F11+G11) · 青格=本局桥洞
      · 黑粗线=隔板 · 蓝线=参考路线 · 红线=仿真轨迹 · 紫菱=PIVOT · 品红星=射击停站
      · 绿点=起点 N10 · 金星=终点 N5 · 深灰车形=车身占位（待实测长宽）

用法（在工作区根）::

    $env:PYTHONPATH='D:\User\Desktop\dzrobot_v2\src\dzbrain'
    python tools\sim_map_render.py --csv artifacts\sim\check_auto\traj.csv ^
        --route _tmp_session\auto_route.lines.txt ^
        --out artifacts\sim\check_auto\route_map.png --title "自动规划 43 token 仿真"

    # bench 日志轨迹（[T] 行里的位置）
    python tools\sim_map_render.py --bench-log _tmp_session\bench_afterfix_240.txt ^
        --route _tmp_session\auto_route.lines.txt --out _tmp_session\bench_map.png

route 两种给法：一行一个 token 的文件（无 ,x/,y 后缀），或直接给 token 串
（`N10 N9 ... N5,x`，末点 ,x/,y 后缀会被剥掉）。
不指定 --route 时只画地图 + 轨迹。
只写 --out 一个 PNG 文件，不碰任何源码/配置。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import pathlib
import re
import sys

WS = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src" / "dzbrain"))          # 必须在 import dzbrain 之前（防 tools\dzbrain.py 遮蔽）

import yaml                                                  # noqa: E402
from dzbrain.core import geometry as geo                     # noqa: E402
from dzbrain.route.parser import parse_mission               # noqa: E402

# ---- 场地常量 ----
COLS = "ABCDEFGHIJKLMN"          # 14 列（横轴，A 左 → N 右）
ROWS = range(1, 12)              # 11 行（纵轴，1 上 → 11 下）
CELL = 0.35
FIELD_W = 14 * CELL              # 4.9 m（横）
FIELD_H = 11 * CELL              # 3.85 m（纵）

# ---- 车体尺寸（★占位值：技术文档只有轴距 0.157 m，车长/车宽需实车测量）----
CAR_LEN_M = 0.28                 # 车长（占位，待实测）
CAR_WID_M = 0.17                 # 车宽（占位，待实测）

# 画布边界（四周留标签余量）
SX_MIN, SX_MAX = -0.35, FIELD_W + 0.35
SY_MIN, SY_MAX = -0.40, FIELD_H + 0.55


def S(x_code: float, y_code: float) -> tuple[float, float]:
    """代码坐标 → 屏幕坐标（横版：A 左 N 右，1 上 11 下）。"""
    return FIELD_W - y_code, x_code


def cell_center(block: str) -> tuple[float, float]:
    """格名 → 屏幕格心坐标。"""
    x, y = geo.block_to_meters(block)
    return S(x, y)


def col_sx(letter: str) -> float:
    return 0.175 + 0.35 * (ord(letter) - 65)


def row_sy(row: int) -> float:
    return (11.5 - row) * CELL


def parse_tokens(text: str) -> list[str]:
    """token 串 → 列表；末点 ,x/,y 后缀剥掉（文件模式也一样，这里统一处理）。"""
    toks = [t for t in re.split(r"\s+", text.strip()) if t]
    if toks and re.fullmatch(r"[A-N]\d+,[xy]", toks[-1]):
        toks[-1] = toks[-1][:-2]
    return toks


def read_text_any(path: str) -> str:
    """按 BOM 自动选编码（PowerShell 重定向常写 UTF-16，Python 写 UTF-8）。"""
    with open(path, "rb") as fh:
        head = fh.read(2)
        rest = fh.read()
    if head == b"\xff\xfe":
        text = (head + rest).decode("utf-16")
    elif head == b"\xfe\xff":
        text = (head + rest).decode("utf-16-be")
    elif head.startswith(b"\xef\xbb"):
        text = rest.decode("utf-8-sig")
    else:
        text = (head + rest).decode("utf-8", errors="replace")
    return text


def load_route(src: str) -> list:
    """--route：文件（一行一个 token）或 token 串 → Route。"""
    p = pathlib.Path(src)
    if p.is_file():
        toks = parse_tokens(p.read_text(encoding="utf-8"))
    else:
        toks = parse_tokens(src)
    if not toks:
        raise ValueError(f"路线为空: {src}")
    return parse_mission(toks, name=os.path.basename(src) if p.is_file() else "inline")


def load_map_cells(map_path: str):
    with open(map_path, "r", encoding="utf-8") as fh:
        m = yaml.safe_load(fh)["map"]
    return m


def car_patch(ax, sx: float, sy: float, yaw_code: float):
    """车身矩形（占位长宽 CAR_LEN_M × CAR_WID_M），yaw 为代码系，屏幕角 = yaw + 90°。"""
    import matplotlib.pyplot as plt
    th = yaw_code + math.pi / 2
    hx, hy = math.cos(th), math.sin(th)      # 车头方向（屏幕系）
    rx, ry = -hy, hx                         # 车体右侧
    L, W = CAR_LEN_M, CAR_WID_M
    pts = [(sx + hx * L / 2 + rx * W / 2, sy + hy * L / 2 + ry * W / 2),
           (sx + hx * L / 2 - rx * W / 2, sy + hy * L / 2 - ry * W / 2),
           (sx - hx * L / 2 - rx * W / 2, sy - hy * L / 2 - ry * W / 2),
           (sx - hx * L / 2 + rx * W / 2, sy - hy * L / 2 + ry * W / 2)]
    return plt.Polygon(pts, closed=True, facecolor="0.25", edgecolor="0.05",
                       lw=0.8, alpha=0.45, zorder=9)


def render(map_path, route, csv_path, bench_log, out, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    fig, ax = plt.subplots(figsize=(12.8, 9.0), dpi=150)      # 横版画布
    ax.set_xlim(SX_MIN, SX_MAX)
    ax.set_ylim(SY_MIN, SY_MAX)
    ax.set_aspect("equal")
    ax.axis("off")

    m = load_map_cells(map_path)

    # ---- 网格（14 列横 × 11 行纵）----
    for c in range(15):
        ax.plot([c * CELL, c * CELL], [0, FIELD_H], color="0.85", lw=0.4)
    for r in range(12):
        ax.plot([0, FIELD_W], [r * CELL, r * CELL], color="0.85", lw=0.4)
    # 列标签 A~N（顶部），行标签 1~11（左侧）
    for c, L in enumerate(COLS):
        ax.text(c * CELL + CELL / 2, FIELD_H + 0.16, L, ha="center", va="center",
                fontsize=9, color="0.35", fontweight="bold")
    for r in ROWS:
        ax.text(-0.16, (11.5 - r) * CELL, str(r), ha="center", va="center",
                fontsize=9, color="0.35", fontweight="bold")

    # ---- K 列 = 桥（整列侧壁 + 每格隔开；开放格随图变，见 m['bridges']）----
    kx_w = col_sx("K") - CELL / 2           # K 列西缘
    kx_e = col_sx("K") + CELL / 2           # K 列东缘
    open_blocks = set(m["bridges"])
    for r in ROWS:
        blk = f"K{r}"
        y0, y1 = (11.5 - r) * CELL - CELL / 2, (11.5 - r) * CELL + CELL / 2
        if blk in open_blocks:
            ax.add_patch(plt.Rectangle((kx_w, y0), CELL, CELL,
                                       facecolor="cyan", alpha=0.45, edgecolor="none", zorder=2))
        else:
            ax.add_patch(plt.Rectangle((kx_w, y0), CELL, CELL,
                                       facecolor="0.88", alpha=0.7, edgecolor="none", zorder=2))
    ax.plot([kx_w, kx_w], [0, FIELD_H], color="0.25", lw=3.2)   # 桥西侧壁
    ax.plot([kx_e, kx_e], [0, FIELD_H], color="0.25", lw=3.2)   # 桥东侧壁
    for r in range(1, 12):
        ax.plot([kx_w, kx_e], [r * CELL, r * CELL], color="0.25", lw=1.2)  # 洞间隔板
    ax.text(col_sx("K"), FIELD_H + 0.16,
            f"K列桥(本局开 {'·'.join(m['bridges'])}, 直穿)", ha="center", va="center",
            fontsize=7.5, color="0.25")

    # ---- 隔板：第 6/7 行之间、D~N 整条封死（不可触碰/跨越；无 K 缺口）----
    bx0 = col_sx(m["barrier_col_from"]) - CELL / 2     # D 列西缘
    bx1 = FIELD_W                                      # 东侧场地边缘（N 列东缘）
    by = 1.75                                          # 行 6/7 分界线 (11.5-6)*0.35=1.925, (11.5-7)*0.35=1.575 → 1.75
    ax.plot([bx0, bx1], [by, by], color="black", lw=6, solid_capstyle="butt", zorder=4)
    ax.text((bx0 + bx1) / 2, by + 0.10, "隔板 D~N · 不可触碰/跨越", ha="center", va="bottom",
            fontsize=8.5, color="black")
    ax.text(bx0 - 0.10, by + 0.18, "<- 唯一上下通道 A/B/C", ha="right", va="bottom",
            fontsize=8, color="green")

    # ---- 障（水桶：不可通行）----
    for b in m["obstacles"]:
        sx, sy = cell_center(b)
        ax.add_patch(plt.Rectangle((sx - CELL / 2, sy - CELL / 2), CELL, CELL,
                                   facecolor="0.35", edgecolor="black", lw=0.6, zorder=3))

    # ---- 靶：F11+G11 = 【一块跨格靶板】（t8 定版：两格画成一整块橙色，不是两块独立靶）----
    if m["targets"]:
        cells = [cell_center(b) for b in m["targets"]]
        sx_min = min(p[0] for p in cells) - CELL / 2
        sx_max = max(p[0] for p in cells) + CELL / 2
        sy_min = min(p[1] for p in cells) - CELL / 2
        sy_max = max(p[1] for p in cells) + CELL / 2
        ax.add_patch(plt.Rectangle((sx_min, sy_min), sx_max - sx_min, sy_max - sy_min,
                                   facecolor="orange", alpha=0.75, edgecolor="darkorange",
                                   lw=1.6, zorder=3))
        ax.text((sx_min + sx_max) / 2, sy_min - 0.06, "一块靶板(移动电子靶)",
                ha="center", va="top", fontsize=8, color="darkorange", fontweight="bold")

    # ---- 雷（必须全部压过；比 PIVOT 菱标大，保证被压的雷也看得见红圈）----
    for b in m["mines"]:
        sx, sy = cell_center(b)
        ax.plot(sx, sy, marker="o", ms=10, mfc="red", mec="darkred", mew=1.8, ls="none", zorder=6)

    # ---- 起终点（★★ 一整个格子：35×35 cm，与场地格完全重合 —— 用户 2026-10-11 口径）----
    #   ⚠️ 以前画的是"绿点 / 金星"（点状标记），用户明确要求"起点终点都是一整个格子"，
    #      所以改成整格色块 + 格心上写标签（再大的轨迹压上去也认得出是哪一格）。
    for cell, face, edge, tag in ((m["start"], "#b2f2bb", "#2b8a3e", "起点"),
                                  (m["end"], "#ffe066", "#e8590c", "终点")):
        cx, cy = cell_center(cell)
        ax.add_patch(plt.Rectangle((cx - CELL / 2.0, cy - CELL / 2.0), CELL, CELL,
                                   facecolor=face, alpha=0.9, edgecolor=edge,
                                   lw=2.2, zorder=7))
        ax.annotate(f"{tag}\n{cell}", (cx, cy), ha="center", va="center",
                    fontsize=8.5, color="black", fontweight="bold", zorder=10,
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec=edge, alpha=0.75))

    # ---- 参考路线（蓝线 + 航点 + 停站/PIVOT 标记）----
    if route is not None:
        pts = [S(wp.x, wp.y) for wp in route.waypoints]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, color="royalblue", lw=2.2, alpha=0.85, zorder=3, label="参考路线")
        for wp in route.waypoints:
            sx, sy = S(wp.x, wp.y)
            # t8 定版：G10 同时是「射击停站 G10*」和「:PIVOT」→ 两个标记都要画
            if wp.pivot:
                ax.plot(sx, sy, marker="D", ms=7, mfc="darkviolet", mec="black",
                        mew=0.8, ls="none", zorder=8)
            if wp.shoot:
                ax.plot(sx, sy, marker="*", ms=19, mfc="magenta", mec="purple",
                        mew=1.2, ls="none", zorder=9)
        ax.plot(xs[1:-1], ys[1:-1], marker="o", ms=4.5, mfc="royalblue", mec="white",
                mew=0.5, ls="none", zorder=4)

    # ---- 仿真轨迹（红线） + 车身占位 ----
    traj = []
    car_poses = []                       # (sx, sy, yaw_code)
    if csv_path and os.path.isfile(csv_path):
        rows = read_text_any(csv_path).splitlines()
        reader = csv.DictReader(rows)
        for i, row in enumerate(reader):
            if i % 4 == 0:                       # 50Hz→12.5Hz 抽样，够平滑
                try:
                    tx, ty = float(row["truth_x"]), float(row["truth_y"])
                    sx, sy = S(tx, ty)
                    traj.append((sx, sy))
                    if i % 125 == 0:             # 每 2.5 s 画一个车身
                        car_poses.append((sx, sy, float(row["truth_yaw"])))
                except (KeyError, ValueError):
                    pass
    if bench_log and os.path.isfile(bench_log):
        pat = re.compile(r"\[T\].*?\((-?[\d.]+),(-?[\d.]+)\)")
        for line in read_text_any(bench_log).splitlines():
            mm = pat.search(line)
            if mm:
                sx, sy = S(float(mm.group(1)), float(mm.group(2)))
                traj.append((sx, sy))
    if traj:
        tx = [p[0] for p in traj]
        ty = [p[1] for p in traj]
        ax.plot(tx, ty, color="red", lw=1.8, alpha=0.9, zorder=5, label="仿真轨迹")
        # 起点车头朝向箭头（取轨迹首拍 yaw）
        if car_poses:
            sx0, sy0, yaw0 = car_poses[0]
            th = yaw0 + math.pi / 2
            ax.annotate("", xy=(sx0 + 0.45 * math.cos(th), sy0 + 0.45 * math.sin(th)),
                        xytext=(sx0, sy0),
                        arrowprops=dict(arrowstyle="-|>", color="green", lw=1.6))
    for sx, sy, yaw in car_poses:
        ax.add_patch(car_patch(ax, sx, sy, yaw))

    # ★ 2026-10-11：图例里的三处**旧图硬编码**改成从地图/路线派生
    #   （以前写死 "靶 = F11+G11 / 桥洞 K1/K7 / 射击停站 G10* / :PIVOT(G10/A9/F3)"，
    #    换到第三张图后这些字全是错的 —— 图上一眼看到的说明反而比不写更糟）。
    _tgt = "+".join(m["targets"]) or "?"
    _brg = "/".join(m["bridges"]) or "?"
    _piv = [wp.block for wp in route.waypoints if wp.pivot] if route is not None else []
    _sht = [wp.block for wp in route.waypoints if wp.shoot] if route is not None else []
    handles = [
        plt.Line2D([], [], color="royalblue", lw=2.2, label="参考路线"),
        plt.Line2D([], [], color="red", lw=1.8, label="仿真轨迹"),
        plt.Line2D([], [], marker="o", ms=7, mfc="red", mec="darkred", ls="none", label="雷(必须压过)"),
        plt.Rectangle((0, 0), 1, 1, facecolor="0.35", edgecolor="black", label="障(水桶)"),
        plt.Rectangle((0, 0), 1, 1, facecolor="orange", alpha=0.75, edgecolor="darkorange",
                      label=f"靶 = {_tgt}【一块靶板】"),
        plt.Rectangle((0, 0), 1, 1, facecolor="cyan", alpha=0.45, label=f"本局桥洞 {_brg}(直穿)"),
        plt.Line2D([], [], color="black", lw=6, label="隔板 D~N(不可触碰/跨越)"),
        plt.Line2D([], [], marker="D", ms=8, mfc="darkviolet", ls="none",
                   label=f":PIVOT 折返({'/'.join(_piv) if _piv else '无'})"),
        plt.Line2D([], [], marker="*", ms=16, mfc="magenta", ls="none",
                   label=f"射击停站 {'/'.join(_sht) if _sht else '无'}"),
        plt.Rectangle((0, 0), 1, 1, facecolor="#b2f2bb", edgecolor="#2b8a3e",
                      label=f"起点 {m['start']}(整格)"),
        plt.Rectangle((0, 0), 1, 1, facecolor="#ffe066", edgecolor="#e8590c",
                      label=f"终点 {m['end']}(整格)"),
        plt.Rectangle((0, 0), 1, 1, facecolor="0.25", alpha=0.45,
                      label=f"车身占位 {CAR_LEN_M:.2f}×{CAR_WID_M:.2f} m(待实测)"),
    ]
    # t8 定版：图例移到画布【右侧外部】，不再遮挡场地右上角（K 列桥 K1 与 N 列路线）
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.005, 1.0),
              fontsize=8.5, framealpha=0.95, borderpad=0.7)
    ax.set_title(title or "仿真路线图（横版场地 14列A~N × 11行1~11）", fontsize=14, pad=8)

    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"OK 已渲染 → {os.path.abspath(out)}（轨迹点 {len(traj)}，车身 {len(car_poses)}）")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="仿真路线图渲染（横版场地+路线+轨迹 → PNG）")
    ap.add_argument("--map", default=str(WS / "config" / "map.yaml"), help="map.yaml 路径")
    ap.add_argument("--route", default=None, help="路线：token 文件（一行一个）或 token 串")
    ap.add_argument("--csv", default=None, help="sim_cli 输出的 traj.csv（truth_x/truth_y/truth_yaw）")
    ap.add_argument("--bench-log", default=None, help="bench 日志（解析 [T] 行位置）")
    ap.add_argument("--out", required=True, help="输出 PNG 路径")
    ap.add_argument("--title", default=None)
    args = ap.parse_args(argv)

    route = load_route(args.route) if args.route else None
    render(args.map, route, args.csv, args.bench_log, args.out, args.title)
    return 0


if __name__ == "__main__":
    sys.exit(main())
