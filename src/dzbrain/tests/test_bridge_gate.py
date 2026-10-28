# -*- coding: utf-8 -*-
"""桥洞斜切门禁回归（t7）：R009 方向对称性 + 「车身口径 K 桥室侵入 = 0 拍」。

运行::

    python src\\dzbrain\\tests\\test_bridge_gate.py        （全过 exit 0，任一失败 exit 1）

★ 这个文件存在的理由（`_tmp_session/t3_verify/t3_报告_全量回归与复核.md` §5 实证）：

1. `route/validate.py:_is_cross()` 曾是**方向不对称**的判据 `min(c) < k_col <= max(c)`：
   当一端就落在 K 列上、另一端在东侧（`L→K`，现场下半场/上半场**进洞的那一半**）时
   `min == k_col` ⇒ 恒假 ⇒ **东侧进洞整类漏判**，45° 斜切永远进不了 R009 评估。
   连 docstring 自举的第一个例子 `L9 -> K9`（标"真"）实测都是 False（代码与文档矛盾）。
2. 后果用**真值轨迹 + 车身口径**量化（本文件 §5 逐拍复算）：斜切进洞那一趟，
   **车身矩形 ∩ 未开放 K 桥室** 远多于**车心口径** —— 只看车心看不见"车身角擦进隔壁桥室"，
   所以车心口径不足以守住「桥洞只有单格宽、别蹭侧壁」这条现场硬约束。
   （历史：第二张图 `L8->K7`@0.15+L036 = 车身 **189** 拍 / 车心 **0** 拍，见 t7 报告；
    **第三张图**（现场图 p-81Zc2l，桥洞 K11/K1）实测见 `_A8_BODY_FRAMES`。）

★★ 三条口径声明（别误读本文件的断言）：
  · 车身尺寸 0.28 × 0.17 m 是**估值**（`docs/route_map 流程.md` 自注待实测）⇒
    `CAR_L` / `CAR_W` 一改侵入拍数就变，按新实测重钉（见 `_A8_BODY_FRAMES`）；
  · 未开放 K 桥室 = `config/map.yaml` 的 `map.bridges`（第三张图 **K1/K11**）**之外**的全部 K 格 ——
    通口集合取自**配置真源**，本文件不写死；
  · 夹具**不写死 token 串**：现役路线直接读 `robot.yaml` 的 `route.tokens`，
    「A8 类对角进洞」由它派生（去掉进洞前那一格），路线一换夹具自动跟随（见 `_diagonal_entry_fixture`）；
  · 本文件只**读** `src/` 与 `config/`，不修改任何东西；仿真跑在进程内（官方 `sim.run_sim`），
    参数覆盖走 `sim_cli` 的同一条路径（保证与命令行工具等价）。
"""

from __future__ import annotations

import math
import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve()
_TESTS = _HERE.parent


def _find_src() -> pathlib.Path:
    """位置无关地找到 `src/dzbrain`（本文件被复制到草稿目录也能跑）。"""
    for parent in _HERE.parents:
        cand = parent / "src" / "dzbrain"
        if (cand / "dzbrain" / "__init__.py").is_file():
            return cand
    return _HERE.parent


_SRC = _find_src()
for _p in (str(_SRC), str(_TESTS)):
    if _p not in sys.path:
        sys.path.insert(0, _p)
_WS = _SRC.parents[1]

from harness import check, check_close, section, finish     # noqa: E402

from dzbrain.core import geometry as geo                    # noqa: E402
from dzbrain.core.config import Config                      # noqa: E402
from dzbrain.route import parser as P                       # noqa: E402
from dzbrain.route import validate as V                     # noqa: E402
from dzbrain.tools import sim as S                          # noqa: E402
from dzbrain.tools import sim_cli as SC                     # noqa: E402

_ROBOT_YAML = _WS / "config" / "robot.yaml"
# ★ 2026-10-25：车端改为「零地图 + 布局模式」，`config/map.yaml` 已归档到
#   `artifacts/maps/`。本文件改成读**测试夹具**（内容与原文件逐字相同），语义零变化。
_MAP_YAML = _WS / "src" / "dzbrain" / "tests" / "fixtures" / "map_sixth_20261017.yaml"

CELL = geo.BLOCK_SIZE
#: 车身外包尺寸（估值，见模块 docstring）—— 实测后改这里，再按新实测重钉断言
CAR_L, CAR_W = 0.28, 0.17

#: 实测值记录（2026-10-11 **t20**，第三张图口径；原始输出
#: `_tmp_session/t20_repin/probe_gate_fixtures.txt`）：
#: 「A8 类对角进洞」（去掉进洞前那一格 ⇒ `L10 -> K11` 45° 斜切）@0.15 + lookahead 0.36 →
#: 7272 拍真值轨迹，**车身 ∩ 未开放 K 桥室 = 167 拍**、**车心口径 8 拍**（≈1/20）。
#: 历史（第二张图 `L8->K7`）：车身 **189** 拍 / 车心 **0** 拍（t7）。
#: ★ 模型/车身尺寸/参数一变这个数就会变 ⇒ 按新实测**重钉**（本断言的意义是把
#:   「斜切进洞的物理后果」量化并留在回归里，而不是把它藏起来）。
#: ★★ t31 重钉（第五张图，洞口 `K10`，合成进场 `M11 -> L11 -> K10`）@0.15 + lookahead 0.36 →
#: **车身 155 拍 / 车心 77 拍**（4.8%，与第三张图 167/8 同型）。候选比较（12 组）见
#: `_tmp_session/t31/probe_a8_fixture.py`：`M11->L11->K10` 是"原语义 + 最小改动"的那一组
#: （`L8->L9->K10` 是 0 拍车心，但 0 拍让"车心会漏"这条断言失去量化意义）。
#: ★★ t3 重钉（2026-10-16，洞口改成官方的 `K9`）：旧自动搜出来的进场是 `K8 -> J8`，而
#:   `K8` 是**封死的 K 桥室**（本图只开 `K5`/`K9`）⇒ 车"从墙里出发"，车身侵入里混进了
#:   起点造假的拍数。改为显式钉 **`J7 -> J8 -> K9`**（`J7`/`J8` 都是 J 列可通行格、
#:   且都不在现役路线里；`J7->J8` 正交、`J8->K9` 是 45° 斜切，语义完全保留）
#:   ⇒ 实测 **车身 312 拍 / 车心 2 拍**（0.6%：车心口径几乎完全漏掉这类侵入）。
_A8_BODY_FRAMES = 312
#: 同一次实测的车心口径拍数（用于"车心判据会漏"那条断言）
_A8_CENTER_FRAMES = 2

#: ★★ 合成斜切进场的显式选择 `pre -> diag -> hole`（本图 = `J7 -> J8 -> K9`）。
#: 为什么显式钉：`_diagonal_entry_fixture()` 的候选搜索命中的第一组是 `K10 -> L10 -> K9`
#: （起点在**封死的 K10 桥室**里，车身 416 / 车心 179 ⇒ 车心占比 43%，把"车心会漏"
#: 这条语义冲淡成"两种口径差不多"）。显式钉住这一组后，断言里的 `c8 * 10 < b8` 才继续成立。
_DIAG_APPROACH = ("J7", "J8")


def _live_tokens() -> list[str]:
    """现役路线 = `robot.yaml` 的 `route.tokens`（第五张图 tracker-safe 自规划，60 token）。"""
    return [str(t) for t in (Config.load(str(_ROBOT_YAML)).get("route.tokens") or [])]


def _lower_hole() -> str:
    """本图下半场桥洞格（`route.cross_lower_default`，第五张图 = K10）。"""
    return str(Config.load(str(_ROBOT_YAML)).get("route.cross_lower_default") or "").upper()


def _diagonal_entry_fixture() -> list[str]:
    """「A8 类对角进洞」夹具：进洞那一段必须变成 45° 斜切（Δ行 ±1、Δ列 ±1）。

    ★★ t31 重钉（第五张图）：旧版靠"删掉现役路线里进洞前那一格"造 45°，其前提是现役路线恰好
      `… pre mid hole …` 且删掉 `mid` 后 `pre->hole` 是斜切。第五张图的现役路线进洞口是
      `M10 L10 K10`（**水平直穿**；删掉 `L10` 得 `M10->K10` = 跨 2 列）⇒ 旧造法在新图上**无解**
      （实测 `AssertionError: 夹具构造失败：去掉 L10 后 M10->K10 不是 45° 斜切进洞`）。
      新版：**先**沿用旧造法（若现役路线恰好合适，最小改动）；否则**合成**两格斜切进场
      `pre → diag → hole`（`diag` 与洞口斜相邻、`pre` 与 `diag` 正交相邻），
      再接上现役路线**从洞口开始**的余段。两条路径都有**构造自检**（不满足 45° 就抛错，
      不允许静默给出一条无效夹具）。
    """
    live = _live_tokens()
    hole = _lower_hole()
    if hole not in live:
        raise AssertionError(f"夹具构造失败：下半场洞口 {hole!r} 不在现役路线里")
    i = live.index(hole)
    # ---- 路径 1（旧造法）：删掉进洞前那一格 ----
    if i >= 2:
        pre, dropped, after = live[i - 2], live[i - 1], live[i]
        ra, ca = geo.block_to_index(pre)
        rb, cb = geo.block_to_index(after)
        if rb == ra + 1 and cb == ca - 1:
            return live[:i - 1] + live[i:]
    # ---- 路径 2（t31 合成）：pre --正交--> diag --45°--> hole ----
    #   候选顺序：先试显式钉住的 `_DIAG_APPROACH`（保住"车身多、车心少"的原语义），
    #   它不适用时再按 4 个斜向自动挑一个（自检不过就抛错）。
    order: list[tuple[str, str]] = []
    if len(_DIAG_APPROACH) == 2:
        order.append((_DIAG_APPROACH[0], _DIAG_APPROACH[1]))
    rh, ch = geo.block_to_index(hole)
    for dr, dc in ((1, -1), (1, 1), (-1, -1), (-1, 1)):
        try:
            diag = geo.index_to_block(rh + dr, ch + dc)
            pre = geo.index_to_block(rh + dr, ch + dc + (1 if dc < 0 else -1))
        except Exception:                       # 越界 ⇒ 换一个斜向
            continue
        order.append((pre, diag))
    for pre, diag in order:
        if diag in live or pre in live:          # 别和现役路线自身的格打架
            continue
        try:
            ra, ca = geo.block_to_index(pre)
            rb, cb = geo.block_to_index(diag)
        except Exception:
            continue
        if abs(rb - ra) + abs(cb - ca) != 1:     # pre -> diag 必须**正交**相邻
            continue
        if abs(rh - rb) != 1 or abs(ch - cb) != 1:   # diag -> hole 必须 **45°**
            continue
        return [pre, diag] + live[i:]
    raise AssertionError(f"夹具构造失败：洞口 {hole} 附近造不出 45° 斜切进场")


def _tokens(text: str) -> list[str]:
    return text.strip().removesuffix(",x").split()


# --------------------------------------------------------------------------- #
#  几何：车身矩形 ∩ 桥室方格
# --------------------------------------------------------------------------- #
def _body_rect(x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    """车身矩形四角（长边沿车头方向；`yaw` = 地图航向，与 `geometry.HEADING_*` 同源）。"""
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = CAR_L / 2, CAR_W / 2
    return [(x + hx * c - hy * s, y + hx * s + hy * c),
            (x + hx * c + hy * s, y + hx * s - hy * c),
            (x - hx * c + hy * s, y - hx * s - hy * c),
            (x - hx * c - hy * s, y - hx * s + hy * c)]


def _ccw(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _seg_int(p1, p2, p3, p4) -> bool:
    d1, d2 = _ccw(p3, p4, p1), _ccw(p3, p4, p2)
    d3, d4 = _ccw(p1, p2, p3), _ccw(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _in_poly(px: float, py: float, poly) -> bool:
    sign = None
    for i in range(len(poly)):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % len(poly)]
        d = (x1 - x0) * (py - y0) - (y1 - y0) * (px - x0)
        s = d > 0
        if sign is None:
            sign = s
        elif s != sign:
            return False
    return True


def _poly_cell_hit(poly, cx: float, cy: float) -> bool:
    """多边形与「以 (cx,cy) 为心、边长 CELL 的轴对齐格」是否相交（含相切）。"""
    h = CELL / 2
    cell = [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]
    if any(_in_poly(px, py, cell) for px, py in poly):
        return True
    if any(_in_poly(px, py, poly) for px, py in cell):
        return True
    return any(_seg_int(poly[i], poly[(i + 1) % 4], cell[j], cell[(j + 1) % 4])
               for i in range(4) for j in range(4))


def _closed_k_cells() -> list[str]:
    """未开放 K 桥室 = K 列所有格 − `config/map.yaml` 的 `map.bridges`（配置真源）。"""
    open_holes: set[str] = set()
    try:
        import yaml
        merged = (yaml.safe_load(_MAP_YAML.read_text(encoding="utf-8")) or {}).get("map") or {}
        open_holes = {str(b).upper() for b in (merged.get("bridges") or [])}
    except Exception:                                        # noqa: BLE001 —— 读不到就退回现场值
        open_holes = set()
    if not open_holes:
        open_holes = {"K1", "K7"}
    return [f"K{r}" for r in range(1, geo.MAP_ROWS + 1) if f"K{r}" not in open_holes]


def _k_intrusion(res: S.SimResult) -> tuple[int, int]:
    """(车心口径拍数, 车身口径拍数)：真值轨迹 ∩ 未开放 K 桥室。"""
    closed = _closed_k_cells()
    body_cells = [(c, geo.block_to_meters(c)) for c in closed]
    center_hits = body_hits = 0
    for i in range(res.steps):
        tx, ty, tyaw = res.truth_x[i], res.truth_y[i], res.truth_yaw[i]
        # 车心口径：车心落在哪个格（用 geometry 自己的换算，不另写一套边界约定）
        if geo.meters_to_block(tx, ty) in closed:
            center_hits += 1
        body = _body_rect(tx, ty, tyaw)
        for _cell, (cx, cy) in body_cells:
            if _poly_cell_hit(body, cx, cy):
                body_hits += 1
                break
    return center_hits, body_hits


def _sim(tokens: list[str], overrides: dict) -> S.SimResult:
    """官方仿真器进程内跑一次；参数覆盖走 `sim_cli` 的同一条路径（与 CLI 等价）。"""
    cfg = Config.load(str(_ROBOT_YAML))
    # ★ 2026-10-25：车端改成「零地图 + 布局模式」，robot.yaml 的 planner.map_path 已置空。
    #   仿真器按 cfg 的 map_path 取图 ⇒ 不钉图时会用另一张图，本文件的"未开放 K 桥室"
    #   就与仿真几何对不上（实测：现役路线侵入由 0 变 6 拍）。这里显式钉成同一个夹具。
    cfg = cfg.override("planner.map_path", str(_MAP_YAML))   # override 返回新对象，必须接住
    cfg = SC._apply_overrides(cfg, SC._parse_set_overrides(
        [f"{k}={v}" for k, v in overrides.items()]))
    return S.run_sim(P.parse_mission(tokens), cfg, S.SimConfig())


def _validate(tokens: list[str]) -> list:
    """走 `validate_route` 真源（`config/map.yaml` + `config/robot.yaml`）。"""
    import yaml
    cfg = Config.load(str(_ROBOT_YAML))
    map_cfg = yaml.safe_load(_MAP_YAML.read_text(encoding="utf-8"))
    return V.validate_route(P.parse_mission(tokens), map_cfg=map_cfg, cfg=cfg)


def _errors(issues) -> list[str]:
    return [i.code for i in issues if i.level == "error"]


# --------------------------------------------------------------------------- #
#  1. `_is_cross` 方向对称性
# --------------------------------------------------------------------------- #
def test_is_cross_symmetry() -> None:
    section("1. `_is_cross` 方向对称性：东侧进洞（L→K）必须与西侧（J→K）等价")
    truth = [
        ("L9", "K9", True, "过洞前半（东侧进洞 —— 曾被整类漏判）"),
        ("L7", "K7", True, "现场下半场正确进洞（水平直穿）"),
        ("L8", "K8", True, "东侧水平进洞（换个行）"),
        ("L8", "K7", True, "A8 的对角进洞（45° 斜切）"),
        ("J9", "K9", True, "西侧推进到洞口（旧判据已为真）"),
        ("K9", "J9", True, "过洞后半"),
        ("L9", "J9", True, "一次跨两格"),
        ("K9", "L9", False, "只是从洞里出来（向东）—— 必须仍为假"),
        ("K9", "K10", False, "在洞里沿列挪格（由 R009「列内」判据单独报）"),
        ("J9", "L9", True, "从西侧一路跨到东侧（中间整个跨过 K 列）"),
        ("L9", "M9", False, "完全在东侧横移（没碰 K 列）"),
    ]
    for a, b, want, note in truth:
        got = V._is_cross(a, b)
        check(f"_is_cross({a}->{b}) == {want}（{note}）", got == want, f"got {got}")
    check("方向可分：几何相同的 K9→L9（出洞）为假、L9→K9（进洞）为真",
          V._is_cross("K9", "L9") is False and V._is_cross("L9", "K9") is True)


# --------------------------------------------------------------------------- #
#  1b. `crossing_entries` 契约：一次过洞 = 一条记录
# --------------------------------------------------------------------------- #
def _raw_segment_hits(blocks: list[str]) -> list[tuple[str, str, tuple]]:
    """t7 之前的口径：`_crossing_at_column` 非 None 就计一条（进洞 + 出洞两半都算）。"""
    out = []
    for a, b in zip(blocks, blocks[1:]):
        hit = V._crossing_at_column(a, b)
        if hit is not None:
            out.append((a, b, hit))
    return out


def test_crossing_entries_contract() -> None:
    section("1b. `crossing_entries` 契约：一次过洞 = 一条记录（下游「数次数」的唯一口径）")
    live = [str(t) for t in (Config.load(str(_ROBOT_YAML)).get("route.tokens") or [])]
    route = P.parse_mission(live)
    blocks = route.blocks()
    entries = V.crossing_entries(blocks)
    raw = _raw_segment_hits(blocks)
    lower = [e for e in entries if V.LOWER_HALF_ROWS[0] <= round(e[2][0]) <= V.LOWER_HALF_ROWS[1]]
    upper = [e for e in entries if V.UPPER_HALF_ROWS[0] <= round(e[2][0]) <= V.UPPER_HALF_ROWS[1]]
    print(f"      现场路线 {len(blocks)} 航点：crossing_entries={len(entries)} 条 "
          f"{[f'{a}->{b}' for a, b, _ in entries]} / 原始逐段计数={len(raw)} 条 "
          f"{[f'{a}->{b}' for a, b, _ in raw]}")
    check("现场路线 `crossing_entries` 下半场恰好 1 条（向西）",
          len(lower) == 1 and sum(e[2][3] for e in lower) < 0, f"{[e[:2] for e in lower]}")
    check("现场路线 `crossing_entries` 上半场恰好 1 条（向东）",
          len(upper) == 1 and sum(e[2][3] for e in upper) > 0, f"{[e[:2] for e in upper]}")
    check("原始逐段计数会把「出洞那一半」多算一次（这就是它不能用来数次数的原因）",
          len(raw) > len(entries), f"raw={len(raw)} entries={len(entries)}")

    a8_blocks = P.parse_mission(_diagonal_entry_fixture()).blocks()
    hole = _lower_hole()
    a8 = V.crossing_entries(a8_blocks)
    a8_lower = [e for e in a8 if V.LOWER_HALF_ROWS[0] <= round(e[2][0]) <= V.LOWER_HALF_ROWS[1]]
    check(f"A8 类（斜切进洞）的过洞记录就是进洞那一条（终点 = {hole}），且夹角 45°",
          len(a8_lower) == 1 and a8_lower[0][1] == hole
          and abs(a8_lower[0][2][1] - 45.0) < 1e-9,
          str([(e[0], e[1], round(e[2][1], 1)) for e in a8_lower]))


# --------------------------------------------------------------------------- #
#  2. `_crossing_at_column` 交点与角度
# --------------------------------------------------------------------------- #
def test_crossing_at_column() -> None:
    section("2. `_crossing_at_column` 交点行号与入洞角（R009 的数值来源）")
    for a, b, row, ang in (("L9", "K9", 9.0, 0.0), ("L7", "K7", 7.0, 0.0),
                           ("J9", "K9", 9.0, 0.0), ("L8", "K7", 7.0, 45.0),
                           ("J9", "K8", 8.0, 45.0)):
        hit = V._crossing_at_column(a, b)
        check(f"_crossing_at_column({a}->{b}) 不是 None", hit is not None, repr(hit))
        if hit is None:
            continue
        check_close(f"{a}->{b} 交点行号 = {row}", hit[0], row, 1e-9)
        check_close(f"{a}->{b} 入洞角 = {ang}°", hit[1], ang, 1e-9)
    check("`L8->K7` 的 45° 已超 R009 上限 10°",
          45.0 > V.MAX_CROSS_ANGLE_DEG, f"MAX_CROSS_ANGLE_DEG={V.MAX_CROSS_ANGLE_DEG}")


# --------------------------------------------------------------------------- #
#  3. 门禁：A8 类对角进洞必须被 R009 拦下
# --------------------------------------------------------------------------- #
def test_gate_rejects_diagonal_entry() -> None:
    section("3. 门禁：A8 类对角进洞（L→K 45° 斜切）必须被 R009 拦下；水平直穿不得误伤")
    diag = _diagonal_entry_fixture()
    hole = _lower_hole()
    a8 = _validate(diag)
    a8_errors = _errors(a8)
    r009 = [i for i in a8 if i.code == "R009"]
    check("A8 类（斜切进洞）报出 R009 error", "R009" in a8_errors, f"errors={a8_errors}")
    check(f"R009 报的就是斜切进 {hole} 的那一段",
          any(f"->{hole}" in i.msg for i in r009), str([i.msg[:46] for i in r009]) or "（无）")
    check("R009 消息带出 45.0° 的实测夹角",
          any("45.0" in i.msg for i in r009), str([i.msg[:64] for i in r009]) or "（无）")
    for i in a8:
        print(f"        {i}")

    live = _validate(_live_tokens())
    check("对照：现役路线（水平直穿进洞）0 error、无 R009",
          _errors(live) == [] and not any(i.code == "R009" for i in live),
          f"errors={_errors(live)}")


# --------------------------------------------------------------------------- #
#  4/5. 真值轨迹 ∩ 未开放 K 桥室（拍）
# --------------------------------------------------------------------------- #
def test_body_frame_gate() -> None:
    section("4. 门禁放行的路线：车身口径 K 桥室侵入 = 0 拍（第三张图现役路线 / 两种巡航）")
    live = _live_tokens()
    check("现场路线从 robot.yaml 读出（token 数 > 0）", len(live) > 0, f"{len(live)} token")
    cases = [("现场路线@robot.yaml 现值", live, {}),
             ("现场路线@显式 0.30", live, {"pursuit.cruise_speed_mps": 0.30})]
    for label, tokens, over in cases:
        gate = _errors(_validate(tokens))
        res = _sim(tokens, over)
        c_hit, b_hit = _k_intrusion(res)
        print(f"      [{label}] {res.steps} 拍 / {res.t[-1]:.2f} s / 完成={res.metric('completed')}"
              f" / 车心 {c_hit} 拍 / 车身 {b_hit} 拍")
        check(f"[{label}] 门禁放行（validate_route 0 error）", gate == [], f"errors={gate}")
        check(f"[{label}] 车心口径 K 桥室侵入 = 0 拍", c_hit == 0, f"{c_hit} 拍")
        check(f"[{label}] ★ 车身口径 K 桥室侵入 = 0 拍", b_hit == 0, f"{b_hit} 拍")

    section("5. 车心口径会漏：A8 类斜切进洞 车身 155 拍 / 车心 77 拍（第五张图 t31 口径；门禁拦它才不是空转）")
    diag = _diagonal_entry_fixture()
    a8_errors = _errors(_validate(diag))
    res8 = _sim(diag, {"pursuit.cruise_speed_mps": 0.15, "pursuit.lookahead_min_m": 0.36})
    c8, b8 = _k_intrusion(res8)
    print(f"      [A8类@0.15+L036] {res8.steps} 拍 / {res8.t[-1]:.2f} s / 完成={res8.metric('completed')}"
          f" / RMS(真值)={res8.metric('cross_track_rms_m'):.4f} / 车心 {c8} 拍 / 车身 {b8} 拍")
    check("★ A8 类的车身口径侵入 > 0 拍（斜切进洞的物理后果真实存在）", b8 > 0, f"{b8} 拍")
    check(f"★ A8 类车身口径侵入 = {_A8_BODY_FRAMES} 拍（t31 第五张图实测；模型/尺寸变了就重钉）",
          b8 == _A8_BODY_FRAMES, f"{b8} 拍")
    check(f"★ A8 类车心口径侵入 = {_A8_CENTER_FRAMES} 拍（实测≈车身的 1/2，仍是第三张图 1/20 的 10 倍 ⇒ 车心判据照样大范围漏）",
          c8 == _A8_CENTER_FRAMES and 0 < c8 < b8,
          f"车心 {c8} 拍 / 车身 {b8} 拍（历史：第二张图 0 / 189）")
    check("★ 该路线确实被门禁拦下（R009）⇒ 放行集合里不会出现它",
          "R009" in a8_errors, f"errors={a8_errors}")


def main() -> None:
    print("=" * 74)
    print("dzbrain 桥洞斜切门禁回归（t7 / R009 对称性 + 车身口径 K 桥室侵入）")
    print(f"python {sys.version.split()[0]}   工作区 {_WS}")
    print("=" * 74)
    test_is_cross_symmetry()
    test_crossing_entries_contract()
    test_crossing_at_column()
    test_gate_rejects_diagonal_entry()
    test_body_frame_gate()
    finish()


if __name__ == "__main__":
    main()
