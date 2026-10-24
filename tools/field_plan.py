#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""field_plan.py —— 场地标注 → 车端规划 → 校验 → 下发串，一体化（2026-10-24）。

定位：路径规划分歧的仲裁工具 + 换图/换桥洞时的车端自主规划入口。
**全部规划在车端 dzbrain 规划器内完成**（不依赖无人机端 mission_router）。

用法（dzrobot_v2 仓库根）：

    # A. 现役地图（config/map.yaml）直接规划——桥洞组合自动扫描
    python tools/field_plan.py

    # B. 现场标注（覆盖式，不改 map.yaml）：
    python tools/field_plan.py \
        --mines A2 F4 H4 G5 E8 B10 D11 I11 \
        --obstacles B1 D1 F3 I6 H7 A8 C9 E9 \
        --targets D5 D6 \
        --bridges K7 K5            # 缺省 = 扫描全部 5×6 组合
        --start N10 --end N5

    # C. 输出下发串（A 通道 /nav/waypoint 直接可用）：
    python tools/field_plan.py --emit tokens

策略（2026-10-24 第六图实测定稿）：
    ① tracker-safe 引擎优先（规则最严：雷上直行/禁90°+/物理间隙）——能解即最优；
    ② tracker-safe 无解（如第六图对角雷带 × 雷上直行规则结构性冲突）自动降级
       **tour 引擎**（允许 180° 折返 token :PIVOT，与现役手写线同形态）；
    ③ 桥洞未指定时扫描全部组合（下半场 K7~K11 × 上半场 K1~K6），按 token 数
       （≈里程）排序给前 3 名；每名自动跑 route_check + validate 双门禁；
    ④ 全部失败时打印每组合的归因诊断（planner 内建）。

输出：
    - 控制台：每组合 [引擎/格数/门禁] 一行 + 最优路线 token 串
    - --png out.png：标注图（雷/障/靶/桥洞/路线，复用 map 元数据渲染）
    - --emit tokens|route：直接给可下发串（/rosin /nav/waypoint "多点 ..."）
"""
from __future__ import annotations

import argparse
import io
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
WS = HERE.parent
SRC = WS / "src" / "dzbrain"
for _e in (str(HERE), str(WS)):
    while _e in sys.path:
        sys.path.remove(_e)
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import yaml                                              # noqa: E402
from dzbrain.core.config import Config                   # noqa: E402
from dzbrain.route import planner as PL                  # noqa: E402
from dzbrain.route.validate import validate_route        # noqa: E402
from dzbrain.route.parser import parse_mission           # noqa: E402

LOWER_POOL = ("K7", "K8", "K9", "K10", "K11")
UPPER_POOL = ("K1", "K2", "K3", "K4", "K5", "K6")
DEFAULT_MAP = WS / "config" / "map.yaml"
DEFAULT_CFG = WS / "config" / "robot.yaml"


def build_map(args) -> dict:
    """命令行标注 → map_cfg dict（未给的项沿用现役 map.yaml）。"""
    base = yaml.safe_load(DEFAULT_MAP.read_text(encoding="utf-8"))
    m = dict(base.get("map") or {})
    if args.mines:
        m["mines"] = list(args.mines)
    if args.obstacles:
        m["obstacles"] = list(args.obstacles)
    if args.targets:
        m["targets"] = list(args.targets)
    if args.start:
        m["start"] = args.start
    if args.end:
        m["end"] = args.end
    return {"map": m}


def try_plan(map_cfg: dict, cfg: Config, lo: str, up: str
             ) -> tuple[str, list[str] | None, str]:
    """一组合两引擎：tracker-safe 优先，降级 tour。返回 (引擎, tokens, 说明)。"""
    try:
        res = PL.plan_route_tracker_safe(map_cfg, cfg, cross_lower=lo,
                                         cross_upper=up, max_seconds=10.0)
        return "tracker-safe", list(res.tokens), ""
    except PL.PlannerError as e_ts:
        reason = str(e_ts)
        try:
            res = PL.plan_route(map_cfg, cfg, cross_lower=lo, cross_upper=up,
                                allow_reverse=False, max_seconds=30.0)
            return "tour", list(res.tokens), f"tracker-safe 无解→tour（{reason.splitlines()[0][:60]}…）"
        except PL.PlannerError:
            return "none", None, reason


def gates(tokens: list[str], map_cfg: dict, cfg: Config, map_path: Path) -> tuple[bool, str]:
    """route_check + validate 双门禁。"""
    proc = subprocess.run(
        [sys.executable, str(WS / "tools" / "route_check.py"),
         "--route", " ".join(tokens), "--config", str(DEFAULT_CFG)],
        capture_output=True, text=True, encoding="utf-8", errors="replace",
        cwd=str(WS), timeout=300,
        env={"PYTHONIOENCODING": "utf-8", "PYTHONUTF8": "1",
             "ROBOT_MAP": str(map_path), **{}},
    )
    rc_ok = proc.returncode == 0
    issues = validate_route(parse_mission(tokens), map_cfg=map_cfg, cfg=cfg)
    v_ok = not any(i.level == "error" for i in issues)
    detail = ""
    if not v_ok:
        detail = "; ".join(f"{i.code}:{i.msg[:30]}" for i in issues if i.level == "error")
    elif not rc_ok:
        tail = (proc.stdout or "").strip().splitlines()
        detail = tail[-1][:60] if tail else "route_check exit!=0"
    return rc_ok and v_ok, detail


def main() -> int:
    ap = argparse.ArgumentParser(description="场地标注→车端规划→门禁→下发串")
    ap.add_argument("--mines", nargs="*", default=None)
    ap.add_argument("--obstacles", nargs="*", default=None)
    ap.add_argument("--targets", nargs="*", default=None)
    ap.add_argument("--bridges", nargs="*", default=None, metavar=("LOWER", "UPPER"),
                    help="下半场洞 上半场洞（如：K7 K5）；缺省扫描全部 30 组合")
    ap.add_argument("--start", default=None)
    ap.add_argument("--end", default=None)
    ap.add_argument("--map", default=None, help="替代基准 map.yaml（标注叠加在其上）")
    ap.add_argument("--top", type=int, default=3, help="展示前 N 名（按 token 数）")
    ap.add_argument("--png", default="", help="最优路线渲染 PNG 路径")
    ap.add_argument("--emit", choices=("tokens", "route"), default="",
                    help="输出可下发内容（tokens=空格串 / route=『多点 …』）")
    args = ap.parse_args()

    global DEFAULT_MAP
    if args.map:
        DEFAULT_MAP = Path(args.map)
    map_cfg = build_map(args)
    cfg = Config.load(str(DEFAULT_CFG))
    m = map_cfg["map"]

    combos: list[tuple[str, str]]
    if args.bridges and len(args.bridges) == 2:
        combos = [(args.bridges[0].upper(), args.bridges[1].upper())]
    else:
        combos = [(lo, up) for lo in LOWER_POOL for up in UPPER_POOL]

    # 标注摘要
    print("=" * 74)
    print("场地标注（车端规划）")
    print("=" * 74)
    print(f"  雷 {len(m.get('mines') or [])}: {' '.join(m.get('mines') or [])}")
    print(f"  障 {len(m.get('obstacles') or [])}: {' '.join(m.get('obstacles') or [])}")
    print(f"  靶: {' '.join(m.get('targets') or [])}   桥洞: {'扫描 30 组合' if not (args.bridges and len(args.bridges)==2) else ' '.join(combos[0])}")
    print(f"  起点 {m.get('start')} → 终点 {m.get('end')}")
    print()

    # 规划扫描
    results = []
    for lo, up in combos:
        engine, toks, note = try_plan(map_cfg, cfg, lo, up)
        if toks is None:
            print(f"  [{lo}/{up}] ✗ 两引擎无解")
            continue
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False,
                                         encoding="utf-8") as tf:
            yaml.safe_dump(map_cfg, tf, allow_unicode=True)
            tmp_map = Path(tf.name)
        ok, bad = gates(toks, map_cfg, cfg, tmp_map)
        try:
            tmp_map.unlink()
        except OSError:
            pass
        mark = "✅" if ok else "⚠️"
        print(f"  [{lo}/{up}] {mark} {engine:13s} {len(toks):3d} token"
              + (f"  {bad}" if bad else ""))
        if ok:
            results.append((len(toks), lo, up, engine, toks))

    if not results:
        print("\n❌ 全部组合无解/门禁不过——见上方归因；标注是否有误？")
        return 1

    results.sort()
    print()
    print(f"可下发路线前 {min(args.top, len(results))} 名（按里程）:")
    for n, (ntok, lo, up, eng, toks) in enumerate(results[:args.top], 1):
        print(f"  #{n} [{lo}/{up}] {eng} {ntok} token")
    best = results[0]
    toks = best[4]
    print()
    print(f"★ 最优: [{best[1]}/{best[2]}] {best[3]} {len(toks)} token")
    print("  " + " ".join(toks))
    if args.emit:
        print()
        print("下发串（/nav/waypoint）:")
        print("  " + (" ".join(toks) if args.emit == "tokens" else "多点 " + " ".join(toks)))
    if args.png:
        render_png(map_cfg, toks, args.png)
        print(f"  渲染: {args.png}")
    return 0


def render_png(map_cfg: dict, tokens: list[str], out: str) -> None:
    """标注图（雷/障/靶/桥/路线）——matplotlib 可选依赖。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as mp
    except ImportError:
        print("  (matplotlib 缺失，跳过渲染)")
        return
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    plt.rcParams["axes.unicode_minus"] = False
    m = map_cfg["map"]
    COLS = "ABCDEFGHIJKLMN"
    ROWS = 11
    fig, ax = plt.subplots(figsize=(14, 6.5), dpi=130)
    for i in range(15):
        ax.plot([i, i], [0, ROWS], color="#b8b8b8", lw=0.6)
    for j in range(ROWS + 1):
        ax.plot([0, 14], [j, j], color="#b8b8b8", lw=0.6)
    for c in range(14):
        for r in range(ROWS):
            ax.text(c + 0.06, ROWS - r - 0.16, f"{COLS[c]}{r+1}",
                    fontsize=5, color="#999")
    def cell_xy(name):
        return COLS.index(name[0]) + 0.5, ROWS - int(name[1:])
    for name in (m.get("obstacles") or []):
        x, y = cell_xy(name)
        ax.add_patch(mp.Rectangle((x-0.42, y-0.42), 0.84, 0.84,
                                  facecolor="#e74c3c", alpha=0.75))
    for name in (m.get("mines") or []):
        x, y = cell_xy(name)
        ax.add_patch(mp.Circle((x, y), 0.30, facecolor="#f1c40f", edgecolor="#b8860b"))
    for name in (m.get("targets") or []):
        x, y = cell_xy(name)
        ax.add_patch(mp.Rectangle((x-0.35, y-0.35), 0.7, 0.7,
                                  facecolor="#9b59b6", alpha=0.7))
    for name in (m.get("bridges") or []):
        x, y = cell_xy(name)
        ax.add_patch(mp.Rectangle((x-0.45, y-0.45), 0.9, 0.9,
                                  facecolor="#2ecc71", alpha=0.45, ls="--", lw=2))
    xs, ys = [], []
    for t in tokens:
        b = t.split(":")[0].split(",")[0].rstrip("*")
        if len(b) >= 2 and b[0] in COLS and b[1:].isdigit():
            x, y = cell_xy(b)
            xs.append(x); ys.append(y)
    ax.plot(xs, ys, color="#0984e3", lw=2.2, marker="o", ms=3.5, label="route")
    sx, sy = cell_xy(m.get("start", "N10"))
    ex, ey = cell_xy(m.get("end", "N5"))
    ax.scatter([sx, ex], [sy, ey], s=280, c=["#2980b9", "#8e44ad"], marker="*")
    ax.set_xlim(-0.4, 14.4); ax.set_ylim(-0.5, 11.5)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(f"field_plan · {len(tokens)} token · 桥 {'/'.join(m.get('bridges') or ['?'])}",
                 fontsize=12, fontweight="bold")
    fig.savefig(out, bbox_inches="tight", facecolor="white")


if __name__ == "__main__":
    sys.exit(main())
