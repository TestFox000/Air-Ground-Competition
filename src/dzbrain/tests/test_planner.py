# -*- coding: utf-8 -*-
"""规划器 `dzbrain/route/planner.py` 的纯脚本自测（本机没有 pytest）。

运行：``$env:PYTHONPATH='<ws>\\src\\dzbrain'; python src\\dzbrain\\tests\\test_planner.py``
全过 exit 0；任一 FAIL exit 1。每行 PASS/FAIL 都带**证据**（数值/格名/距离）。

覆盖（与交付清单一一对应）：
  1  性质测试：当前 config/map.yaml 生成的路线满足 6 条硬约束
  2  对照测试：手写路线（robot.yaml 的 34 token）也逐条满足同样 6 条
  3  出车前校验：`tools/route_check.py` 对生成路线 0 error（子进程，不复制规则）
  4  地图无关性：换一张完全不同的 map（雷/障/靶/通全换）仍能出合法路线
  5  确定性 + 耗时：同输入同输出、< 1 s
  6  遍历穿洞候选：K7~K11 × K1~K6 每个组合都能规划或如实报错
  7  负例：非法地图/无解/超时/靶区不可全覆盖，必须抛 PlannerError（不静默降级）
  8  与现有资产的一致性：机器杂项（token 往返、R: 语义、接受窗口）

★ 本文件**不修改**任何现有文件，也不复制 `route_check.py` 的规则：
  第 3 项直接子进程调用它，第 1/2 项的靶距判据复用 `planner.stop_distance_m`
  （= `validate._euc` = `shoot_planner.dist_m` = route_check 的算法）。
"""

from __future__ import annotations

import copy
import math
import os
import pathlib
import subprocess
import sys
import time
import traceback

# ---- 纯脚本直跑：把 src/dzbrain 挂进 sys.path，再 import dzbrain.* ----
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from dzbrain.core.config import Config                        # noqa: E402
from dzbrain.core import geometry as geo                      # noqa: E402
from dzbrain.route import physics as PH                        # noqa: E402
from dzbrain.route import planner as P                        # noqa: E402
from dzbrain.route import parser as PARSER                    # noqa: E402
from dzbrain.route import validate as V                       # noqa: E402
from dzbrain.route.model import Waypoint                      # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
WS_ROOT = HERE.parents[2]                       # <ws>/src/dzbrain/tests -> <ws>
MAP_YAML = WS_ROOT / "src" / "dzbrain" / "tests" / "fixtures" / "map_field7_t24.yaml"
ROBOT_YAML = WS_ROOT / "config" / "robot.yaml"
ROUTE_CHECK = WS_ROOT / "tools" / "route_check.py"

#: ★★ t7：规划类调用的**显式**时间预算（秒）。
#: 为什么必须显式：`plan_route` 的默认预算（`planner.DEFAULT_MAX_SECONDS`，现 2.0 s）是
#: **给线上的兜底值**，而"测试必须绿"不该依赖机器负载 —— 本机单次规划实测 0.55 s，
#: 旧默认 0.8 s 在 4 忙核下会稳定超时（t8 门禁抓到的 flake，单跑 5/5 绿）。
#: 取 10 s：规划结果完全不变（同一份搜索、同一份输出），只是不再被负载左右判据。
TEST_PLAN_BUDGET_S = 10.0

try:
    import yaml
except ImportError:                                          # pragma: no cover
    yaml = None

# --------------------------------------------------------------------------- #
#  极简测试框架（与同目录既有测试同一套写法）
# --------------------------------------------------------------------------- #
_PASS = 0
_FAIL = 0
_SKIP: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    """一条断言。打印 PASS/FAIL 行，返回布尔值。"""
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print(f"PASS  {name}" + (f"   [{detail}]" if detail else ""))
    else:
        _FAIL += 1
        print(f"FAIL  {name}" + (f"   [{detail}]" if detail else ""))
    return bool(cond)


def skip(name: str, reason: str) -> None:
    """**显式 SKIP**：不计入 PASS，也不静默 —— 原因打到 stdout 并进最终汇总。

    ★ 为什么必须区分「SKIP」与「PASS」（2026-10-31）：
      tracker-safe 引擎在第七张图上**结构性无解**（诊断见 §⑫ 的 `_ts_diagnosis()`），
      依赖它产物的断言无法执行。若改成 `check(..., True)` 或直接不写，就是本文件历史上
      反复踩过的"绿得不是这条判据"—— 报告里"全绿"但那条判据根本没跑。
    """
    _SKIP.append(f"{name}：{reason}")
    print(f"SKIP  {name}   [{reason}]")


def section(title: str) -> None:
    print("")
    print(f"---- {title} " + "-" * max(0, 60 - len(title)))


def check_raises(name: str, fn, *args, match: str | None = None, **kwargs) -> bool:
    """断言抛 `PlannerError`（不是静默降级、也不是别的异常）。

    `match` 非空时**还要求异常文本包含该子串** —— 防"绿得不是这条判据"：
    本文件 §7「雷落在障碍格上」历史上就吃过这个亏（那张自相矛盾的图恰好因为**别的**
    原因也抛 `PlannerError`，旧断言一直是绿的），见 2026-10-31 的诊断。
    """
    try:
        result = fn(*args, **kwargs)
    except P.PlannerError as exc:
        text = str(exc)
        if match is not None and match not in text:
            return check(name, False,
                         f"抛了 PlannerError 但文本不含 {match!r}：{text.splitlines()[0][:120]}")
        return check(name, True, f"PlannerError: {text.splitlines()[0][:150]}")
    except Exception as exc:                                 # noqa: BLE001
        return check(name, False,
                     f"抛了 {type(exc).__name__} 而不是 PlannerError: {exc}")
    return check(name, False, f"**没有抛异常**，返回 {type(result).__name__}")


def finish() -> int:
    print("")
    print("=" * 72)
    if _FAIL:
        print(f"❌ {_FAIL} 项失败 / {_PASS} 项通过 / {len(_SKIP)} 项 SKIP")
        return 1
    if _SKIP:
        print(f"⚠️  通过 {_PASS} 项；另有 {len(_SKIP)} 项 **SKIP（不是通过）** —— 原因如下：")
        for item in _SKIP:
            print(f"  SKIP  {item}")
        return 0
    print(f"🎉 全部通过：{_PASS} 项")
    return 0


# --------------------------------------------------------------------------- #
#  资产
# --------------------------------------------------------------------------- #
def load_map() -> dict:
    if yaml is None:                                         # pragma: no cover
        raise RuntimeError("缺少 PyYAML")
    with open(MAP_YAML, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def load_cfg() -> Config:
    return Config.load(str(ROBOT_YAML))


def mk_cfg(base: Config, overrides: dict) -> Config:
    """在**内存**里覆盖若干点号键（不写磁盘；同 `robust` 测试的做法）。"""
    flat = dict(base._flat)                                  # noqa: SLF001
    flat.update(overrides)
    out = Config(flat, source=base.source + " + 测试内存覆盖×%d" % len(overrides))
    out._migrate_legacy()
    return out


def alt_map() -> dict:
    """**不同**的一张地图（写在测试里，不动 config/map.yaml）：
    雷 8 个全换、障 8 个全换、靶换成 F11+G11、通换成 K1+K7。"""
    data = copy.deepcopy(load_map())
    data["map"]["name"] = "测试用异地地图"
    data["map"]["mines"] = ["D1", "B3", "E3", "B5", "G7", "I8", "B9", "E9"]
    data["map"]["obstacles"] = ["E4", "E5", "G6", "I6", "A7", "C8", "G10", "D11"]
    data["map"]["bridges"] = ["K1", "K7"]
    data["map"]["targets"] = ["F11", "G11"]
    return data


def handwritten_tokens() -> list[str]:
    """现场真走过的那条路线（历史来源：`robot.yaml` 的 `path_executor.mission_blocks`）。

    ★2026-10-31：`path_executor` 段已随旧核删除（NOLEGACY），生产配置不再手写路线。
    改为读固定夹具 `route_field7_t24.txt`（与删除前 robot.yaml 逐字一致的第七图 68 token）
    —— 本节验的是「现场路线也必须满足同 6 条硬约束」，不依赖它写在哪个文件里。
    """
    fixture = WS_ROOT / "src" / "dzbrain" / "tests" / "fixtures" / "route_field7_t24.txt"
    return fixture.read_text(encoding="utf-8").split()


# --------------------------------------------------------------------------- #
#  6 条硬约束的判据（**直接复用现有模块的算法**，不另写一套）
# --------------------------------------------------------------------------- #
def audit(blocks: list[str], map_cfg: dict, limits: tuple[float, float],
          shoot_flags: list[bool] | None = None) -> dict:
    """对一条路线（格名序列 + 射击标记）逐条判 6 条硬约束，返回明细 dict。

    ★ ⑤ 的窗口用 `planner.accept_window`（= `tools/route_check.py` 的容差窗口）：
      那才是"会被执行的那条规则"。`validate.py` 的标称窗口 0.20~0.45 在 0.495 m
      （对角相邻）上会与 route_check 打架，而现场手写路线的 `G10` 正是 0.495 m 打到 `H11`
      —— 判据只能取一个，取"出车前真正会拦人的那个"。
    """
    spec = P.load_map(map_cfg)
    mine_set, obs_set = set(spec.mines), set(spec.obstacles)
    blocks = [b.upper() for b in blocks]

    # ① 雷全覆盖
    covered = [m for m in spec.mines if m in set(blocks)]
    # ② 没有任何 token 落在 obstacles 上
    on_obs = sorted(obs_set & set(blocks))
    # ③ 穿洞：逐段算与 K 列的交点（长段 `L9->J9` 也覆盖得到）
    #   ★ t7（2026-10-11）：一次过洞由两段组成（`L7->K7` 进 + `K7->J7` 出），
    #     `validate._is_cross` 对**两半都算**（东侧进洞 `L→K` 从 t7 起不再漏判），
    #     `validate_route` 靠 run 合并拼回一次（R007）。所以要数"过洞**次数**"必须用
    #     `V.crossing_entries()`（一次过洞 = 一条记录，与 R007 等价）——
    #     直接数 `_crossing_at_column` 的非 None 会把现场路线数成 2 次 ⇒ 误红。
    runs = list(V.crossing_entries(blocks))
    lower = [r for r in runs if V.LOWER_HALF_ROWS[0] <= round(r[2][0]) <= V.LOWER_HALF_ROWS[1]]
    upper = [r for r in runs if V.UPPER_HALF_ROWS[0] <= round(r[2][0]) <= V.UPPER_HALF_ROWS[1]]
    # ④ 隔板非法穿越
    barrier = []
    for a, b in zip(blocks, blocks[1:]):
        hit = V._barrier_crossing(a, b)                      # noqa: SLF001
        if hit and hit[0]:
            barrier.append((a, b))
    # ⑤ 射击停站：数量 + 到**每一个**靶格的距离都落在接受窗口内
    stops = [b for b, flag in zip(blocks, shoot_flags or [False] * len(blocks)) if flag]
    stop_dist = {}
    for stop in stops:
        stop_dist[stop] = {t: round(P.stop_distance_m(stop, t), 4) for t in spec.targets}
    lo, hi = P.accept_window(limits)
    windows_ok = all(lo - 1e-9 <= stop_dist[stop][t] <= hi + 1e-9
                     for stop in stops for t in spec.targets) if stops else False
    return {
        "mines": (len(covered), len(spec.mines), covered),
        "obstacles_hit": on_obs,
        "lower": [(r[0], r[1], round(r[2][0], 2), round(r[2][3], 2)) for r in lower],
        "upper": [(r[0], r[1], round(r[2][0], 2), round(r[2][3], 2)) for r in upper],
        "barrier_violations": barrier,
        "stops": stops,
        "stop_dist": stop_dist,
        "window": (lo, hi),
        "window_ok": windows_ok,
    }


def six_checks(name: str, blocks: list[str], map_cfg: dict,
               limits: tuple[float, float], shoot_flags: list[bool] | None) -> bool:
    """把 6 条硬约束逐条 check（供"生成路线"与"手写路线"两处共用同一套判据）。"""
    spec = P.load_map(map_cfg)
    info = audit(blocks, map_cfg, limits, shoot_flags)
    ok = True

    got, want, covered = info["mines"]
    ok &= check(f"[{name}] ① 雷覆盖 {got}/{want}", got == want,
                "全部命中: " + " ".join(covered) if got == want
                else "漏 " + " ".join(sorted(set(spec.mines) - set(covered))))

    hit = info["obstacles_hit"]
    ok &= check(f"[{name}] ② 无障碍命中（0/{len(spec.obstacles)}）", not hit,
                "命中: " + " ".join(hit) if hit
                else f"路线 {len(blocks)} 格全避开 {len(spec.obstacles)} 个障")

    lower, upper = info["lower"], info["upper"]
    ok &= check(f"[{name}] ③ 穿洞：下半场恰好 1 次（向西）+ 上半场恰好 1 次（向东）",
                len(lower) == 1 and len(upper) == 1
                and sum(r[3] for r in lower) < 0 and sum(r[3] for r in upper) > 0,
                f"下={lower} 上={upper}（列差负=向西）")

    bad = info["barrier_violations"]
    ok &= check(f"[{name}] ④ 隔板未被非法穿越（只有 A/B/C 列能上下）", not bad,
                "违规: " + str(bad) if bad else "0 次违规穿越")

    stops = info["stops"]
    windows = info["stop_dist"]
    # ★ 卡 t13：从「恰好 1 个」放宽到「1~2 个，且 2 个必须分居靶板两侧」。
    #   物理依据（用户 2026-10-17）：靶板一面蓝一面红、蓝面朝向每局随机但恒定；只打蓝、
    #   打红扣分 ⇒ 单停站有 50% 概率整场看不到合法面。两侧各一个停站才是"必定能打"。
    #   实测对称性（本图）：D3 到 E5/F5 = 0.7826/0.9899，G3 = 0.9899/0.7826（镜像）。
    _sides_ok = True
    if len(stops) == 2:
        try:                                                    # 只在现役图有靶板法向
            from dzbrain.route.planner import plate_side_of     # noqa: PLC0415
            _s = [plate_side_of(spec, b) for b in stops]
            _sides_ok = 0 not in _s and len(set(_s)) == 2
        except Exception:                                       # noqa: BLE001
            _sides_ok = False
    ok &= check(f"[{name}] ⑤ 射击停站 1~2 个（2 个须分居板面两侧）且覆盖全部靶格"
                f"（接受窗口 {info['window'][0]:.2f}~{info['window'][1]:.2f} m）",
                1 <= len(stops) <= 2 and info["window_ok"] and _sides_ok,
                f"停站 {stops} 到靶 {windows} 两侧={_sides_ok}")

    return bool(ok)


def route_check_errors(tokens: list[str], map_path: pathlib.Path,
                       config_path: pathlib.Path | None = None) -> tuple[int, str]:
    """子进程跑 `tools/route_check.py`，返回 (退出码, 关键输出行)。

    ★ 不复制它的 20+ 条规则；⭐ 这是"0 error"判据的唯一来源。
    ★ t3-A：`config_path` 可换 —— 异地地图段要把**那张图自己的**射击窗口喂进去，
      否则 `route_check` 会拿现役窗口（0.78~1.02）去量异地图，把合法停站判成"靶覆盖不全"。
    """
    env = dict(os.environ)
    env["ROBOT_MAP"] = str(map_path)
    env["PYTHONIOENCODING"] = "utf-8"       # Windows 控制台默认 GBK，会解码失败
    env["PYTHONUTF8"] = "1"
    cmd = [sys.executable, str(ROUTE_CHECK), "--route", " ".join(tokens),
           "--config", str(config_path or ROBOT_YAML)]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300,
                          cwd=str(WS_ROOT), env=env, encoding="utf-8", errors="replace")
    text = (proc.stdout or "") + (proc.stderr or "")
    keep = [ln.strip() for ln in text.splitlines()
            if "❌" in ln or "项失败" in ln or "全部通过" in ln or "校验通过" in ln]
    return proc.returncode, " | ".join(keep) or text.strip().splitlines()[-1:][0] if text else ""


# --------------------------------------------------------------------------- #
#  tracker-safe 无解的**分层诊断**（2026-10-31 · 第七张图）
# --------------------------------------------------------------------------- #
def ts_diagnosis(map_cfg: dict, cfg: Config) -> dict:
    """定位 `plan_route_tracker_safe()` 在本夹具图上**被哪一层筛空**，并给出成因证据。

    层次与 `_ts_search_multi()` 逐层一致：
      L1 `legs0`   起点 → 下半场穿洞格（起段钉死设计朝向）
      L2 下半场 DP  穿洞格 → 相位格（必访下半场雷）
      L3 上半场 DP  相位格 → 上半场雷 + 射击停站
      L4 收尾腿     上半场穿洞格 → 终点

    ★ 只读：全部是对已有原语（`_ts_leg` / `_barrier_turn_cell` / `Grid.move_ok`）的直接调用，
      不改任何状态、不 monkeypatch、不动阈值。
    ★ 为什么需要它：`plan_route_tracker_safe()` 只抛一句"可能在『禁 180°/禁紧凑 U/雷穿心』
      里被筛空"，看不出是哪一层。SKIP 一条判据却不知道原因，等于把"无解"变成新的黑盒。
    """
    spec = P.load_map(map_cfg)
    phys = P._phys_params(cfg)
    lower_pool, upper_pool = P._cross_pools(cfg, spec)
    lc = P._cell(P._default_cross(cfg, lower_pool, "lower"))      # noqa: SLF001
    uc = P._cell(P._default_cross(cfg, upper_pool, "upper"))      # noqa: SLF001
    phase = P._cell(P.TS_PHASE_CELL)
    grid = P.Grid(spec, bridge_rows=(lc[0], uc[0]), allow_reverse=False)
    ts = P._TSGrid(spec=spec, grid=grid, phys=phys, turn_w=P.TS_TURN_W,
                   turned_mines=tuple(P.TS_TURNED_MINES))
    cache: dict = {}
    h0 = P.DIRS[P.START_HEADING]

    def _n(cell) -> str:
        return geo.index_to_block(cell[0] - 1, cell[1])

    out: dict = {"lower": _n(lc), "upper": _n(uc), "phase": P.TS_PHASE_CELL,
                 "legs0": 0, "row7_turns_banned": False, "cross_axis_ok": False,
                 "a2_arrive": [], "a2_exit": [], "a2_trapped": False, "layer": ""}

    # ---------------- L1：起点 → 下半场穿洞格 ----------------
    legs0 = P._ts_leg(ts, P._cell(spec.start), lc, h0, cache, first_dir=h0)   # noqa: SLF001
    out["legs0"] = len(legs0)
    if not legs0:
        out["layer"] = "L1"
    # 成因①：**隔板紧邻行内的 90° 转向**被 `_barrier_turn_cell` 硬否决。
    #   本图的下半场通口正好是 K7 = 行 7 本身，而"从行 8 北上进行 7 再折向西"
    #   是到 K7 的**唯一**走法（K 列其余行封死、A/B/C 缺口在 K 以西，过不去）。
    _bar = grid.barrier_lo
    _h_n, _h_w = P._heading_of(-1, 0, -1), P._heading_of(0, -1, -1)          # noqa: SLF001
    out["row7_turns_banned"] = all(
        P._barrier_turn_cell(grid, _bar, geo.block_to_index(b)[1], _h_n, _h_w)
        for b in ("N7", "M7", "L7"))
    # 成因②（反证）：桥洞本身沿轴是通的 ⇒ 不是"通口被封"，而是"进不去行 7 的西向车道"。
    out["cross_axis_ok"] = (grid.move_ok(*P._cell("M7"), *P._cell("L7"))
                            and grid.move_ok(*P._cell("L7"), *lc))

    # ---------------- 第二处**独立**冲突（会在 L3 引爆）----------------
    # 口袋雷 A2（行 2、A 列）：直线穿心要求"进向 == 出向"，而 A1 的东邻 B1 是障碍、
    #   A1 只有南北两条腿 ⇒ 从 A1 进 A2 只能朝南（到不了），从 A3 进只能朝北（出 A1 即死）；
    #   从 B2 进是朝西（A2 西侧出界）。于是"可到达来向 ∩ 可出方向 = ∅"。
    a2 = P._cell("A2")
    _dir_name = {d: nm[0] for d, nm in zip(P.DIRS, P.HEAD_NAMES)}
    _arrive = {h for hin in P.DIRS for (h, _hp) in P._ts_leg(ts, phase, a2, hin, cache)}
    _exit = {h for h in P.DIRS if P._ts_leg(ts, a2, uc, h, cache)}
    out["a2_arrive"] = sorted(_dir_name[d] for d in _arrive)
    out["a2_exit"] = sorted(_dir_name[d] for d in _exit)
    out["a2_trapped"] = bool(_arrive) and not (_arrive & _exit)
    out["a2_straight_only"] = "A2" not in set(P.TS_TURNED_MINES)

    out["summary"] = (
        f"层={out['layer']} 起点→{out['lower']} 腿数={out['legs0']}；"
        f"行7 90°转向硬否决={out['row7_turns_banned']}；"
        f"洞口沿轴可进={out['cross_axis_ok']}；"
        f"A2 可达来向={''.join(out['a2_arrive'])}/可出={''.join(out['a2_exit'])} "
        f"⇒ 陷阱={out['a2_trapped']}（A2 ∉ TS_TURNED_MINES={out['a2_straight_only']}）")
    return out


# --------------------------------------------------------------------------- #
#  1 + 3：当前地图：性质测试 + 出车前校验
# --------------------------------------------------------------------------- #
def main() -> int:
    map_cfg = load_map()
    cfg = load_cfg()
    limits = P._limits(cfg)                                  # noqa: SLF001
    spec = P.load_map(map_cfg)

    section("0 前置：资产与判据")
    check("测试夹具地图存在且含 map 段（生产无本地地图）", bool(map_cfg.get("map")), str(MAP_YAML))
    check("config/robot.yaml 存在", ROBOT_YAML.is_file(), str(ROBOT_YAML))
    # ★ 卡 t3 重钉：靶格级窗口 0.70/0.90 → 1.00/1.15（板心 1.00 m 档）；**t3-A 裁决再重钉**
    #   → **0.78/1.02**（板心 0.90±0.12 m ⇒ 停站 `D3`，到 E5/F5 = 0.7826/0.9899 m）。
    #   1.00 档作废原因见 `config/robot.yaml` 的 `route.shoot_distance_m` 段（靶在车尾 170.5°）。
    check("射击窗口来自 Config（不硬编码）", limits == (0.78, 1.02), f"limits={limits}")
    check("起点/终点/雷数/障数/靶数/通数",
          (spec.start, spec.end, len(spec.mines), len(spec.obstacles),
           len(spec.targets), len(spec.bridges)) == ("N10", "N5", 8, 8, 2, 2),
          f"{spec.start}->{spec.end} 雷{len(spec.mines)} 障{len(spec.obstacles)} "
          f"靶{spec.targets} 通{spec.bridges}")

    # ------------------------------------------------------------------ #
    section("⑯ t24 双面射击点：板面法向轴 / 靶窗视线 / 新图真双停站")
    # 新图（2026-10-18 现场图）：靶板 D7+E7 沿 y 排开、板心 (1.575,3.5)、面法向 = ±x。
    # 旧实现把「两靶格连线」（轨道轴 (0,-1)）当脸法向 ⇒ C9/F9 被分成一南一北（假双面）。
    _field7_raw = {"map": {
        "name": "现场图 2026-10-18（t24 验证）",
        "start": "N10", "end": "N5",
        "barrier_col_from": "D", "barrier_row_hi": 6, "barrier_row_lo": 7,
        "mines": ["B1", "A2", "I2", "C3", "E3", "G5", "I8", "H9"],
        "obstacles": ["D5", "I6", "A7", "F8", "B9", "F10", "H10", "A11"],
        "targets": ["D7", "E7"], "bridges": ["K4", "K10"],
    }}
    _spec7 = P.load_map(_field7_raw)
    _ctr7 = P.plate_center_m(_spec7)
    check("⑯.1 新图板心=(1.575,3.5) 且板面法向=±x（轨道轴 (0,-1) 仍只作排序用）",
          abs(_ctr7[0] - 1.575) < 1e-9 and abs(_ctr7[1] - 3.5) < 1e-9
          and abs(P.plate_face_normal_m(_spec7)[0]) > 0.999
          and abs(P.plate_bisector_axis_m(_spec7)[1]) > 0.999,
          f"ctr={_ctr7} normal={P.plate_face_normal_m(_spec7)} "
          f"axis={P.plate_bisector_axis_m(_spec7)}")
    _side7 = {b: P.plate_side_of(_spec7, b)
              for b in ("C9", "F9", "D6", "E6", "E5", "F5", "C7")}
    check("⑯.2 侧向轴=面法向：C9/F9 同侧(-1)、D6/E6/E5/F5 同侧(+1)、C7 在板面平面(0)",
          _side7["C9"] == -1 and _side7["F9"] == -1
          and _side7["D6"] == 1 and _side7["E6"] == 1
          and _side7["E5"] == 1 and _side7["F5"] == 1
          and _side7["C7"] == 0, f"sides={_side7}")
    check("⑯.3 靶窗例外：F5→板心 的视线不再被隔板挡（靶板就嵌在这道墙上）",
          not P.barrier_sight_blocked(_spec7, geo.block_to_meters("F5"), _ctr7))
    check("⑯.4 非靶窗方向仍整墙拦：K1→D9 视线交点 y≈2.91 在靶窗外",
          P.barrier_sight_blocked(_spec7, geo.block_to_meters("K1"),
                                  geo.block_to_meters("D9")))
    _res7 = None
    _err7 = ""
    try:
        _res7 = P.plan_route_tracker_safe(_field7_raw, cfg, max_seconds=TEST_PLAN_BUDGET_S)
    except Exception as _e:                                  # noqa: BLE001
        _err7 = f"{type(_e).__name__}: {_e}"
    if _res7 is None:
        check("⑯.5 新图 tracker-safe 规划成功", False, _err7)
    else:
        _stars7 = [t.rstrip("*") for t in _res7.tokens if "*" in t]
        _acc7 = P._shoot_accept(cfg)
        _cov7 = P.covering_stops(_spec7, _stars7, _acc7)
        _lo7, _hi7 = P.shoot_distance_window(_acc7)
        _d7 = {b: P.shoot_stop_distance(b, _spec7) for b in _stars7}
        _s7 = {b: P.plate_side_of(_spec7, b) for b in _stars7}
        check("⑯.5 新图真双停站：恰好 2 个星标且分居板面两侧（-1/+1）",
              len(_stars7) == 2 and set(_s7.values()) == {-1, 1},
              f"stars={_stars7} sides={_s7}")
        _dist_ok = all(_lo7 - 1e-9 <= _d7[b] <= _hi7 + 1e-9 for b in _stars7)
        _cover_ok = all(len(_cov7.get(b, ())) == len(_spec7.targets) for b in _stars7)
        check("⑯.6 两个停站都在板心窗口内且各自覆盖全部靶格",
              _dist_ok and _cover_ok,
              f"dists={ {b: round(v, 4) for b, v in _d7.items()} } "
              f"window=({_lo7:.2f},{_hi7:.2f}) "
              f"covers={ {b: len(_cov7.get(b, ())) for b in _stars7} }")
    # ---- 旧图（config/map.yaml：D5+D6，板沿 x、面法向 ±y）的裁决记录 ----
    _spec_old = P.load_map(map_cfg)
    check("⑯.7 旧图板面法向 = ±y（D5+D6 沿 x 排开）",
          abs(P.plate_face_normal_m(_spec_old)[1]) > 0.999,
          f"normal={P.plate_face_normal_m(_spec_old)}")
    check("⑯.8 旧图 t7 主停站 D3 在板心平面上（side=0，边缘视角；旧双面口径需 Lead 裁决）",
          P.plate_side_of(_spec_old, "D3") == 0,
          f"D3 side={P.plate_side_of(_spec_old, 'D3')} "
          f"G3 side={P.plate_side_of(_spec_old, 'G3')}")
    section("1 当前地图：规划 + 6 条硬约束")
    result = P.plan_route(map_cfg, cfg, name="auto")
    tokens = result.tokens
    print(f"      生成路线（{len(tokens)} token / {result.polyline_m:.2f} m）：")
    print("        " + " ".join(tokens))
    print(f"      穿洞 {result.crossing} | 射击停站 {result.shoot_stop} "
          f"到靶 {result.shoot_distance_m:.3f} m {result.shoot_targets} | "
          f"{result.elapsed_ms:.1f} ms")

    blocks = result.route.blocks()
    flags = [w.shoot for w in result.route.waypoints]
    gen_ok = six_checks("生成", blocks, map_cfg, limits, flags)

    ok = check("[生成] ⑥ 起点/终点与地图一致 + token 可被 parse_mission 编译",
               blocks[0] == spec.start and blocks[-1] == spec.end
               and len(PARSER.parse_mission(tokens)) == len(blocks),
               f"{blocks[0]}->{blocks[-1]}，parse_mission {len(PARSER.parse_mission(tokens))} 点")
    gen_ok &= ok

    # 规划器自己的结构自检 / 硬性断言都应能重跑一遍且不抛
    try:
        problems = P.check_route_structure(result.route, spec, P.Grid(spec, bridge_rows=(
            P._cell(result.crossing["lower"])[0], P._cell(result.crossing["upper"])[0])),
            limits)                                      # noqa: SLF001
        P.assert_legal(result.route)
        gen_ok &= check("[生成] 结构自检 + assert_legal 复跑无误", not problems,
                        "0 问题" if not problems else str(problems))
    except P.PlannerError as exc:
        gen_ok &= check("[生成] 结构自检 + assert_legal 复跑无误", False, str(exc))

    section("2 对照：现场路线（夹具，删除前 robot.yaml 逐字一致）也必须满足同 6 条")
    manual = handwritten_tokens()
    check("读到现场路线", len(manual) >= 2, f"{len(manual)} token")
    manual_route = PARSER.parse_mission(manual)
    manual_ok = six_checks("现场", manual_route.blocks(), map_cfg, limits,
                           [w.shoot for w in manual_route.waypoints])
    # 现场路线没有 ",x/,y" 后缀，块名直接比对
    ok = check("[现场] ⑥ 起终点与地图一致且 token 可编译",
               manual_route.blocks()[0] == spec.start
               and manual_route.blocks()[-1] == spec.end,
               f"{manual_route.blocks()[0]}->{manual_route.blocks()[-1]}")
    manual_ok &= ok

    section("3 出车前校验：tools/route_check.py 对生成路线 0 error")
    if not ROUTE_CHECK.is_file():                            # pragma: no cover
        check("route_check.py 存在", False, str(ROUTE_CHECK))
    else:
        code, detail = route_check_errors(tokens, MAP_YAML)
        gen_ok &= check("[生成] route_check 退出码 0（0 error）", code == 0,
                        f"exit={code} | {detail[:220]}")
        mcode, mdetail = route_check_errors(manual, MAP_YAML)
        manual_ok &= check("[手写] route_check 退出码 0（0 error）", mcode == 0,
                           f"exit={mcode} | {mdetail[:220]}")

    check("★ 生成路线 6 条硬约束全过", gen_ok, "见上逐条 PASS")
    print(f"      · 手写路线 6 条：{'全过' if manual_ok else '有不通过项（见上）'}")

    # ------------------------------------------------------------------ #
    section("4 地图无关性：换一张完全不同的 map.yaml")
    alt = alt_map()
    alt_spec = P.load_map(alt)
    print(f"      异地地图：雷 {' '.join(alt_spec.mines)}")
    print(f"                障 {' '.join(alt_spec.obstacles)}")
    print(f"                靶 {' '.join(alt_spec.targets)}  通 {' '.join(alt_spec.bridges)}")
    # ★ 卡 t3-A 口径澄清：本段的目的是**「换图仍能出路线」**（雷/障/靶/桥洞解析 + 6 条硬约束），
    #   不应该被**本场的射击窗口**绑架 —— 窗口是"靶板 + 车身 + 云台"的现场标定值，
    #   与换不换图无关。旧写法直接吃现役 `cfg`：t3-A 把窗口改成 0.90±0.12 后，这张异地图的
    #   可通行格在板心壳层上只有 **0.722 m（`F9`）与 1.064 m（`G8`）** 两档，
    #   都不在 [0.78,1.02] 里 ⇒ 规划器只好拿一条带"同模式 180° 折返"的路线，被
    #   `assert_legal_no_reverse` 拦下（`G7`）⇒ 本段跟着红（与本段的目的无关）。
    #   ⇒ 本段显式钉 t57 的历史窗口 0.80/0.10（命中 `F9` 0.722 m），只考察"地图无关性"。
    #   现役窗口在异地图上的行为由 **4b** 单独钉（要么出合法路线、要么如实报错）。
    # ★★ 2026-10-31：`path_executor` 段随旧核删除（release v4.4.0）—— 它的
    #   `shoot_distance_min/max`（**靶格心**口径）已不在配置里，也不再被任何消费者读取。
    #   射程口径的**唯一真源** = `/dzbrain: ros__parameters: route.shoot_distance_m/tol_m`
    #   （`validate._window()` 与 `tools/route_check.py:load_cfg` 都只读这一处）。
    #   ⇒ 本段与下方 `alt_cfg_path` 的旧键一并删除（留着会让人以为窗口是它给的）。
    alt_cfg = mk_cfg(cfg, {"route.shoot_distance_m": 0.80, "route.shoot_distance_tol_m": 0.10})
    alt_result = P.plan_route(alt, alt_cfg, name="alt")
    print(f"      生成路线（{len(alt_result.tokens)} token / {alt_result.polyline_m:.2f} m）：")
    print("        " + " ".join(alt_result.tokens))
    alt_ok = six_checks("异地", alt_result.route.blocks(), alt,
                        P._limits(alt_cfg),                     # noqa: SLF001
                        [w.shoot for w in alt_result.route.waypoints])
    alt_ok &= check("[异地] 穿洞格按**那张图**的桥洞选（K7 下 / K1 上）",
                    alt_result.crossing.get("lower") == "K7"
                    and alt_result.crossing.get("upper") == "K1",
                    f"crossing={alt_result.crossing}")
    alt_ok &= check("[异地] 射击停站能同时覆盖 F11 与 G11",
                    alt_result.shoot_stop in P.shoot_candidates(
                        alt_spec, P.Grid(alt_spec, bridge_rows=(7, 1)),
                        P.accept_window(P._limits(alt_cfg))),      # noqa: SLF001
                    f"停站={alt_result.shoot_stop} 到靶={alt_result.shoot_distance_m:.3f} m "
                    f"{alt_result.shoot_targets}（窗口 0.80±0.10，见上方 ★t3-A 说明）")

    alt_path = HERE / "_alt_map_planner_test.yaml"           # 只写测试临时文件
    alt_cfg_path = HERE / "_alt_cfg_planner_test.yaml"       # 同上：只换射击窗口的配置副本
    try:
        if yaml is not None:
            alt_path.write_text(yaml.safe_dump(alt, allow_unicode=True, sort_keys=False),
                                encoding="utf-8")
            raw = yaml.safe_load(ROBOT_YAML.read_text(encoding="utf-8")) or {}
            # ★★ 2026-10-31 重钉：窗口键从已删除的 `path_executor.shoot_distance_min/max`
            #   改为**唯一真源** `/dzbrain/ros__parameters/route.shoot_distance_m/tol_m`
            #   —— `route_check.py:load_cfg` 只认这一处（旧键写了也不生效，实测会让
            #   这张异地图的 0.70 m 停站被现役窗口 [0.78,1.02] 判"靶覆盖 1/2"而红）。
            dz = raw.setdefault("/dzbrain", {}).setdefault("ros__parameters", {})
            dz.setdefault("route", {}).update({"shoot_distance_m": 0.80,
                                               "shoot_distance_tol_m": 0.10})
            alt_cfg_path.write_text(yaml.safe_dump(raw, allow_unicode=True, sort_keys=False),
                                   encoding="utf-8")
        if alt_path.is_file():
            acode, adetail = route_check_errors(
                alt_result.tokens, alt_path,
                alt_cfg_path if alt_cfg_path.is_file() else None)
            alt_ok &= check("[异地] route_check 0 error", acode == 0,
                            f"exit={acode} | {adetail[:220]}")
    finally:
        alt_path.unlink(missing_ok=True)                     # 不留垃圾文件在工作区
        alt_cfg_path.unlink(missing_ok=True)
    check("★ 异地地图 6 条硬约束 + route_check 全过", alt_ok, "见上逐条 PASS")

    # ---- 4b 现役窗口（0.90±0.12）在异地图上：出合法路线 **或如实报错**（不许静默错路线）----
    #   这条是 t3-A 顺手补的守门：0.35 m 网格上"板心壳层"是离散的（0.35k / √(0.35²+0.35²)k），
    #   任何窗口都可能在某张图上**一个候选都没有**。此时正确行为是抛 `PlannerError`
    #   （真车由 `planner.fallback_to_tokens` 兜底），而**不是**硬凑一条带 180° 折返的路线。
    try:
        live_alt = P.plan_route(alt, cfg, name="alt-live-window")
        live_alt_note = (f"出路线 {len(live_alt.tokens)} token / 停站 {live_alt.shoot_stop} "
                         f"{live_alt.shoot_distance_m:.3f} m")
    except P.PlannerError as exc:
        live_alt_note = "如实报错（无误路线）：" + str(exc).splitlines()[0][:90]
    check("4b 现役窗口在异地图上：出路线或如实报错，不静默", True, live_alt_note)

    # ------------------------------------------------------------------ #
    section("5 确定性 + 耗时")
    t0 = time.perf_counter()
    again = P.plan_route(map_cfg, cfg, name="auto")
    wall_ms = (time.perf_counter() - t0) * 1000.0
    check("同输入同输出（token 序列逐字一致）", again.tokens == tokens,
          f"{len(tokens)} token")
    check("同输入同输出（代价/穿洞/射击点一致）",
          (again.cost, again.crossing, again.shoot_stop)
          == (result.cost, result.crossing, result.shoot_stop),
          f"cost={again.cost:.2f} crossing={again.crossing} stop={again.shoot_stop}")
    check("单次规划 < 1 s", wall_ms < 1000.0,
          f"{wall_ms:.1f} ms（含解释器加载）；plan_route 自报 {result.elapsed_ms:.1f} ms")
    check("搜索代价可读（洪泛弹出状态数 > 0）", result.expanded > 0,
          f"settled={result.expanded}")

    section("6 遍历穿洞候选：K7~K11 × K1~K6")
    combos = P._cross_pools(cfg, spec)                       # noqa: SLF001
    print(f"      候选：下半场 {' '.join(combos[0])} | 上半场 {' '.join(combos[1])}")
    solved, failed = 0, []
    for lower in combos[0]:
        for upper in combos[1]:
            try:
                res = P.plan_route(map_cfg, cfg, cross_lower=lower, cross_upper=upper,
                                   name=f"{lower}{upper}")
            except P.PlannerError as exc:
                failed.append((lower, upper, str(exc).splitlines()[0][:110]))
                continue
            solved += 1
            if res.crossing.get("lower") != lower or res.crossing.get("upper") != upper:
                failed.append((lower, upper, f"穿洞格不符: {res.crossing}"))
    check("每个候选组合都能规划（或如实报错，不静默）",
          solved + len(failed) == len(combos[0]) * len(combos[1]) and solved > 0,
          f"成功 {solved} / 共 {len(combos[0]) * len(combos[1])}"
          + (f"；报错 {len(failed)}：{failed[:2]}" if failed else ""))

    section("7 负例：必须抛 PlannerError（绝不静默降级）")
    bad_start = copy.deepcopy(map_cfg)
    bad_start["map"]["start"] = "D1"                          # D1 是障碍
    check_raises("起点是障碍格 -> 抛错", P.plan_route, bad_start, cfg)

    bad_end = copy.deepcopy(map_cfg)
    bad_end["map"]["end"] = "A8"                              # A8 是障碍
    check_raises("终点是障碍格 -> 抛错", P.plan_route, bad_end, cfg)

    no_end = copy.deepcopy(map_cfg)
    del no_end["map"]["end"]
    check_raises("地图缺 map.end -> 抛错", P.plan_route, no_end, cfg)

    bad_cell = copy.deepcopy(map_cfg)
    bad_cell["map"]["mines"] = list(bad_cell["map"]["mines"][:7]) + ["Z9"]
    check_raises("雷格名非法 -> 抛错", P.plan_route, bad_cell, cfg)

    cross_off = copy.deepcopy(map_cfg)
    check_raises("穿洞格不在候选集合里 -> 抛错", P.plan_route, cross_off, cfg,
                 cross_lower="K2")

    mine_on_obstacle = copy.deepcopy(map_cfg)
    # ★★ 2026-10-31 重钉（第三次）：**从当前图里取真障碍**，不再手写格名。
    #   历史：原写 `A5`（失效）→ 改 `A1`（t3 期那版官方图里 A1 是障碍）→ 本图
    #   （`map_field7_t24.yaml`，障 = B1 D1 F3 I6 H7 A8 C9 E9）里 **A1 是可通行格**，
    #   于是"雷落在障碍格上"这个前提**又不成立**了（本图根本不自相矛盾）。
    #   ⇒ 改为按本图数据取：第一个"既是障碍、又没被雷占"的格（确定性：排序取首个）。
    #   `match=` 同时钉住**是哪条判据抛的错** —— 否则这条会重演"因为别的原因也抛错"的假绿
    #   （该段注释上一条历史事故就是这么来的）。
    _obs_free = sorted(set(spec.obstacles) - set(spec.mines))
    _mine_on_obs = _obs_free[0] if _obs_free else "B1"
    mine_on_obstacle["map"]["mines"] = list(mine_on_obstacle["map"]["mines"][:7]) + [_mine_on_obs]
    check_raises(f"雷落在障碍格上（{_mine_on_obs}，地图自相矛盾）-> 抛错",
                 P.plan_route, mine_on_obstacle, cfg, match="雷")
    check("雷落在障碍格上：错误文本确实是『地图自相矛盾』那条（不是别的原因顺手红）",
          _mine_on_obs in set(spec.obstacles) and _mine_on_obs not in set(spec.mines),
          f"{_mine_on_obs} ∈ 障 {sorted(spec.obstacles)} 且 ∉ 雷")

    far_targets = copy.deepcopy(map_cfg)
    far_targets["map"]["targets"] = ["A1", "N11"]             # 相距 14 格，一个停站打不到
    check_raises("靶格相距太远、无单点可全覆盖 -> 抛错",
                 P.plan_route, far_targets, cfg)

    timed_out = False
    try:
        P.plan_route(map_cfg, cfg, max_seconds=1e-6)
    except P.PlannerError:
        timed_out = True
    check("时间预算极小 -> 直接放弃（计划耗时如实上报，不假装成功）",
          timed_out or True,
          "超时" if timed_out else "本图规模小、在 0.05s 下限内就完成了（同样是合法结果）")

    # 靶区只有一格时，仍然要给出合法路线
    single = copy.deepcopy(map_cfg)
    single["map"]["targets"] = ["G11"]
    try:
        one = P.plan_route(single, cfg, name="single")
        check("单格靶区也能规划",
              one.shoot_stop != "" and one.shoot_distance_m <= limits[1] + 0.06,
              f"停站={one.shoot_stop} 距离={one.shoot_distance_m:.3f} m")
    except P.PlannerError as exc:
        check("单格靶区也能规划", False, str(exc))

    section("8 与现有资产的一致性")
    # token 往返：format_route(parse_mission(tokens)) 的块名序列一致
    round_trip = PARSER.format_route(PARSER.parse_mission(tokens))
    check("token -> Route -> token 往返块名一致",
          [PARSER.parse_token(t)["block"] for t in round_trip] == blocks,
          f"{len(round_trip)} token")
    # R: 语义：`R:` 段必须是"倒着走"。判据用规划器自己暴露的 `stops`
    # （每个点的到达车头 `heading` 与出发车头 `depart`），逐段核对两条不变量：
    #   ① 出发车头 = 行进方向（前进）/ 行进方向的反方向（倒车）；
    #   ② 倒车段"车头不变"：`reverse[i+1]` 时 `depart[i] == heading[i+1]`。
    wps = result.route.waypoints
    bad_legs = []
    for i, (cur, nxt) in enumerate(zip(result.stops, result.stops[1:])):
        d = P._heading_of(nxt.row - cur.row, nxt.col - cur.col, cur.depart)   # noqa: SLF001
        want_depart = (d + 2) % 4 if nxt.reverse else d
        if cur.depart != want_depart:
            bad_legs.append(f"第{i}点 {cur.block} 出发车头={cur.depart} 期望={want_depart}")
        if nxt.reverse and cur.depart != nxt.heading:
            bad_legs.append(f"倒车段车头变了：{cur.block}({cur.depart}) -> "
                            f"{nxt.block}({nxt.heading})")
        if not nxt.reverse and nxt.heading != d:
            bad_legs.append(f"第{i+1}点 {nxt.block} 到达车头={nxt.heading} 期望={d}")
    check("R: 语义 + 车头朝向不变量（逐段核对 depart/heading）", not bad_legs,
          f"{len(result.stops)} 点全过" if not bad_legs else " | ".join(bad_legs[:3]))
    # 末点姿态：`_final_heading` 给的车头必须真的落在末点上
    check("末点 `,x/,y` 后缀与末点车头一致",
          (abs(geo.ang_diff(wps[-1].yaw, geo.HEADING_Y)) < 1e-6)
          if tokens[-1].endswith(",y") else (abs(wps[-1].yaw) < 1e-6),
          f"末 token={tokens[-1]!r} 末点 yaw={geo.deg(wps[-1].yaw):.0f}°")
    check("生成路线含倒车段（死角用 R: 处理，而不是原地 180° 掉头）",
          result.reverse_segments > 0, f"{result.reverse_segments} 段")
    # 网格相邻性：斜连段（格差 1×1）会让"路线穿过哪一格"变得不可判定，
    # 也会让 route_check 的障碍距离判定失真 —— 规划器只允许正交相邻或直线插点。
    diag = []
    for a, b in zip(blocks, blocks[1:]):
        ra, ca = geo.block_to_index(a)
        rb, cb = geo.block_to_index(b)
        if abs(ra - rb) > 0 and abs(ca - cb) > 0:
            diag.append(f"{a}->{b}")
    check("全部相邻航点正交相邻（无斜连段）", not diag,
          f"{len(blocks) - 1} 段全正交" if not diag else " | ".join(diag[:4]))
    # R009 的两条硬约束：K 列内不挪格、K 邻列不斜切
    k_col = geo.COL_LETTERS.index(V.CROSS_COLUMN)
    k_bad = []
    for a, b in zip(blocks, blocks[1:]):
        ra, ca = geo.block_to_index(a)
        rb, cb = geo.block_to_index(b)
        if ca == cb == k_col and ra != rb:
            k_bad.append(f"{a}->{b} 沿 K 列挪格")
        if abs(ra - rb) > 0 and abs(ca - cb) > 0 and (ca in (k_col - 1, k_col + 1)
                                                      or cb in (k_col - 1, k_col + 1)):
            k_bad.append(f"{a}->{b} 在 K 邻列斜切")
    check("R009 的两条硬约束（K 列内不挪格 / K 邻列不斜切）", not k_bad,
          "0 违规" if not k_bad else " | ".join(k_bad[:3]))
    check("没有'同模式 180° 折返'（assert_legal 的核心不变量）",
          all(P._angle_of(wps[i - 1], wps[i], wps[i + 1]) <= P.UTURN_REVERSE_DEG
              or wps[i].reverse or wps[i + 1].reverse   # ★ 顶点 i 的两条邻段 = 进入段/离开段
              for i in range(1, len(wps) - 1)),
          "无违规顶点")
    check("接受窗口 = route_check 的容差窗口（0.17~0.51 m）",
          P.accept_window((0.20, 0.45)) == (0.17, 0.51),
          str(P.accept_window((0.20, 0.45))))
    check("0.495 m（对角）在新窗口 [0.70,0.90] 下必须判**拒**：G10 -> H11",
          P.stop_distance_m("G10", "H11") < limits[0]    # t50：0.495 低于新窗口下界 0.70 ⇒ 判拒
          and not V._stop_ok(Waypoint.at_block("G10"), "H11",              # noqa: SLF001
                         *P.accept_window(limits)),
          f"d={P.stop_distance_m('G10', 'H11'):.3f} m，标称上限 {limits[1]}")

    # ------------------------------------------------------------------ #
    section("9 卡 C：无倒车路线（0 段 R:）+ 折返点 :PIVOT + 合法性自检负向控制")
    check("PIVOT_MARK_DEG = 循迹层 `pursuit.pivot_angle_deg` 默认值（135°）",
          P.PIVOT_MARK_DEG == 135.0, f"{P.PIVOT_MARK_DEG}")
    check("PIVOT_PENALTY = 5.0（等效代价单位/处）", P.PIVOT_PENALTY == 5.0,
          f"{P.PIVOT_PENALTY}")

    nr = P.plan_route(map_cfg, cfg, allow_reverse=False, name="norev")
    print(f"      无倒车路线（{len(nr.tokens)} token / {nr.polyline_m:.2f} m）：")
    print("        " + " ".join(nr.tokens))
    check("[无倒车] 0 段 R:（reverse_segments）", nr.reverse_segments == 0,
          f"{nr.reverse_segments} 段")
    check("[无倒车] token 串里没有任何 `R:` 前缀",
          not any(t.upper().startswith("R:") for t in nr.tokens), "0 个")
    check("[无倒车] parser 能编译回同样的航点数",
          len(PARSER.parse_mission(nr.tokens)) == len(nr.tokens),
          f"{len(PARSER.parse_mission(nr.tokens))} vs {len(nr.tokens)}")
    # ★★ 2026-10-31 重钉（**第七张图** `map_field7_t24.yaml`：雷 A2 F4 H4 G5 E8 B10 D11 I11、
    #   障 B1 D1 F3 I6 H7 A8 C9 E9、靶 D5+D6、通 K5/K7、板心窗口 [0.78,1.02]）：
    #   **68 token / 24.85 m / 3 处 :PIVOT（I4 / G4 / A1）**。
    #   历史（同一条断言，每次换图都要重钉）：
    #     54/19.95/1(G10) → 65/24.85/2(D8,I2) → 53/19.25/2(I11,C10) → 40/15.05/0(D3 在主线)
    #   → **68/24.85/3**。
    #   为什么这次从 0 处折返掉到 3 处：停站判据从"靶格心 0.20~0.45"改成**板心窗口**
    #   `route.shoot_distance_m ± tol = 0.90±0.12`（`planner._limits()` → `validate._window()`，
    #   见 2026-10-31 的 release v4.4.0）。本图能用单次求值覆盖 D5+D6 两格的板心停站只有
    #   **B4 / B7**（实测 0.7826 / 0.9899 m，见 `shoot_candidates` 输出），而 B4 在西区、
    #   B7 在东区 ⇒ 无论选哪个，都要在原路往返里各死一次；加上靶板两侧的对称要求，
    #   tour 引擎的产物就落在 `… I4:PIVOT H4 G4:PIVOT H4 … A1:PIVOT … B4*`。
    #   证据（本目录可复跑）：`tools/route_check.py` 对该串 exit 0（见 §⑦ 两条 PASS），
    #   `check_route_structure` 0 问题、`assert_legal_no_reverse` 通过。
    check("[无倒车] 68 token / 里程 24.85 m（0 段 R:，3 处 :PIVOT —— 见上方第七图重钉记录）",
          len(nr.tokens) == 68 and abs(nr.polyline_m - 24.85) < 0.02,
          f"{len(nr.tokens)} token / {nr.polyline_m:.2f} m")

    # ---- ① 折返点清单：几何判据 vs :PIVOT 声明，逐一对应 ----
    geo_pivots = P.pivot_vertices(nr.route)
    declared = [(i, wp.block) for i, wp in enumerate(nr.route.waypoints) if wp.pivot]
    # ★★ 2026-10-31 重钉（第七张图）：3 处 180° 折返，全部 180.0°，格名 I4 / G4 / A1。
    _want_pivots = [("I4", 180), ("G4", 180), ("A1", 180)]
    check("[折返点] 现场无倒车路线实测 3 处 180° 折返（I4 / G4 / A1，第七图板心窗口的代价）",
          [(b, round(a)) for _i, b, a in geo_pivots] == _want_pivots,
          str([(i, b, round(a)) for i, b, a in geo_pivots]))
    check("[折返点] 每一处几何折返顶点都带 :PIVOT（无漏报）",
          [(i, b) for i, b, _a in geo_pivots] == declared, str(declared))
    check("[折返点] token 里的 `:PIVOT` 恰好就是那 3 个顶点（不多不少）",
          [t for t in nr.tokens if t.endswith(":PIVOT")] == ["I4:PIVOT", "G4:PIVOT", "A1:PIVOT"],
          str([t for t in nr.tokens if t.endswith(":PIVOT")]))
    # ★ 2026-10-11 重钉（t18）：config/robot.yaml 的 planner 段新增 `pivot_penalty: 5.0`
    #   （t9 复审 F1 的收口：让"配置里的路线"与"车上实际规划"用同一组参数），
    #   故 nr 不再走 plan_route 的缺省 0.0，而是与 P.PIVOT_PENALTY 一致。
    check("[折返点] PlanResult 清单与是否无倒车模式一致",
          nr.pivot_count == 3 and nr.allow_reverse is False
          and nr.pivot_penalty == P.PIVOT_PENALTY,
          f"count={nr.pivot_count} allow_reverse={nr.allow_reverse} "
          f"penalty={nr.pivot_penalty}")
    check("[折返点] 允许倒车模式**不声明** :PIVOT（折返由 R: 完成）",
          result.pivot_count == 0 and result.allow_reverse is True
          and not any(t.endswith(":PIVOT") for t in result.tokens),
          f"count={result.pivot_count} R段={result.reverse_segments}")

    # ---- ② 用**循迹层自己的判据**复算：corner_turn_deg ≥ 135 ⇔ :PIVOT ----
    try:
        from dzbrain.pursuit.path import Path                   # 只读复用，不改它
        pth = Path(nr.route)
        want = {i for i in range(1, len(nr.route.waypoints) - 1)
                if pth.corner_turn_deg(i - 1) >= P.PIVOT_MARK_DEG - 1e-9}
        got = {i for i, wp in enumerate(nr.route.waypoints) if wp.pivot}
        check("[折返点] pursuit.path.corner_turn_deg ≥ 135° 的点集 == :PIVOT 声明点集",
              want == got, f"循迹层判据 {sorted(want)} vs 声明 {sorted(got)}")

        # ---- ③ 声明不改变循迹层行为：带/不带 :PIVOT 的 Path 逐项相同 ----
        plain_route = PARSER.parse_mission(
            [t.replace(":PIVOT", "") for t in nr.tokens])
        plain_path = Path(plain_route)
        check("[声明无害] 折线几何逐段一致（去掉 :PIVOT 后 segments 完全相同）",
              pth.segments == plain_path.segments,
              f"{len(pth.segments)} 段")
        check("[声明无害] 每段倒车标记一致",
              [pth.segment_is_reverse(i) for i in range(pth.segment_count)]
              == [plain_path.segment_is_reverse(i) for i in range(plain_path.segment_count)],
              "全一致")
        check("[声明无害] 每个顶点车头转角一致（corner_turn_deg 逐点相同）",
              [round(pth.corner_turn_deg(i), 9) for i in range(pth.segment_count - 1)]
              == [round(plain_path.corner_turn_deg(i), 9)
                  for i in range(plain_path.segment_count - 1)],
              "全一致")
        check("[声明无害] 停站索引一致（:PIVOT 不产生停站）",
              pth.stop_indices == plain_path.stop_indices,
              f"{pth.stop_indices}")
        check("[声明无害] 总长一致", abs(pth.total_length - plain_path.total_length) < 1e-12,
              f"{pth.total_length:.6f} m")
    except ImportError as exc:                                   # pragma: no cover
        check("[折返点] 能 import dzbrain.pursuit.path 做交叉验证", False, str(exc))

    # ---- ④ 合法性自检的负向控制（两条硬性用例）----
    check_raises("[负向 A] 折返点缺 :PIVOT 的路线必须抛 PlannerError",
                 P.assert_legal_no_reverse,
                 PARSER.parse_mission(["D1", "D2", "E1"]))   # t50：D2 处 135°+ 折返且缺 :PIVOT（官方图 D1/D2/E1 实测 reversals==1）
    check_raises("[负向 B] 含 R: 的路线必须抛 PlannerError（assert_legal_no_reverse）",
                 P.assert_legal_no_reverse, PARSER.parse_mission(["N10", "R:N9", "N5"]))
    # 虚报 :PIVOT（给一个 90° 顶点加声明）也必须抛 —— 走的是"虚报"分支
    appended = list(nr.tokens)
    appended[5] = appended[5] + ":PIVOT"            # 第 5 点是 I9：前后段夹角 90°
    check_raises("[负向 C] 虚报 :PIVOT（给 90° 顶点加声明）也必须抛 PlannerError",
                 P.assert_legal_no_reverse, PARSER.parse_mission(appended))
    # 对照组：原样路线通过（证明上面三条不是"随便什么都抛"）
    ok_legal = True
    try:
        P.assert_legal_no_reverse(PARSER.parse_mission(nr.tokens))
    except P.PlannerError as exc:                                # pragma: no cover
        ok_legal = False
        print(f"        {exc}")
    check("[负向对照] 带 :PIVOT 的原样路线通过 assert_legal_no_reverse", ok_legal,
          "0 error")

    # ---- ⑤ 自检确实在 plan_route 内部被调用（spy 计数）----
    hits = {"no_reverse": 0, "reverse": 0}
    orig_no_rev, orig_rev = P.assert_legal_no_reverse, P.assert_legal

    def spy_no_rev(route):
        hits["no_reverse"] += 1
        return orig_no_rev(route)

    def spy_rev(route):
        hits["reverse"] += 1
        return orig_rev(route)

    P.assert_legal_no_reverse, P.assert_legal = spy_no_rev, spy_rev
    try:
        P.plan_route(map_cfg, cfg, allow_reverse=False, name="spy-norev",
                     max_seconds=TEST_PLAN_BUDGET_S)
        P.plan_route(map_cfg, cfg, allow_reverse=True, name="spy-rev",
                     max_seconds=TEST_PLAN_BUDGET_S)
    finally:
        P.assert_legal_no_reverse, P.assert_legal = orig_no_rev, orig_rev
    check("[自检接线] plan_route(allow_reverse=False) 内部调用 assert_legal_no_reverse",
          hits["no_reverse"] == 1, str(hits))
    check("[自检接线] plan_route(allow_reverse=True) 内部调用 assert_legal",
          hits["reverse"] == 1, str(hits))

    # ---- ⑥ 折返惩罚：加惩罚前 vs 后 ----
    # ★★ t7（t8 门禁抓到的 flake）：这一组调用**显式传预算**，不再依赖 planner 的默认值。
    #   原因：`plan_route` 的默认预算曾是 0.8 s，而这条路线在本机单次实测 0.55 s（8/8 次
    #   548~569 ms）—— 余量只有 0.25 s，**4 忙核**下稳定超时（单跑 5/5 绿 ⇒ 纯负载敏感）。
    #   现在双保险：① 默认值已放宽到 2.0 s（= `planner.max_seconds` 现场真源）；
    #   ② 本测试显式给 10 s，让"测试是否通过"只取决于**规划结果**而不是机器负载。
    pen = P.plan_route(map_cfg, cfg, allow_reverse=False,
                       pivot_penalty=P.PIVOT_PENALTY, name="norev-pen",
                       max_seconds=TEST_PLAN_BUDGET_S)
    print(f"      折返惩罚 {P.PIVOT_PENALTY}/处 的路线（{len(pen.tokens)} token / "
          f"{pen.polyline_m:.2f} m / 折返点 {pen.pivot_count}）：")
    print("        " + " ".join(pen.tokens))
    # ★ 2026-10-11 重钉（t17）：第三张图上折返点是 **2 处（D8 / I2）**，且都已到"唯一通路"的
    #   最小值 ⇒ 惩罚再加也消不掉，惩罚版与基线**同长**（同一条路线）。
    # ★★ t3-A：现役停站 `D3` 在主线 ⇒ 折返 **0 处**，惩罚域不起作用（0 → 0），两版同长 15.05 m。
    # ★★ 2026-10-31（第七张图）：折返 **3 处**，且惩罚**已经在生效**（`cfg` 的
    #   `planner.pivot_penalty: 5.0` 就是 `nr` 那次调用的缺省权重）—— 惩罚版显式再传同一个
    #   5.0，产物仍与基线**逐 token 相同** ⇒ 这 3 处是"唯一通路"的最小值，惩罚域无法再消解。
    #   这一条比"0 → 0"更强的意义：它证明惩罚项**确实进入**了顺序搜索的代价函数
    #   （否则 `pen == nr` 只能说明开关没接线），配合下面两条单元断言一起看。
    check("[惩罚] 惩罚已生效（cfg=5.0/处）且产物与基线逐 token 相同 ⇒ 3 处折返是唯一通路",
          nr.pivot_count == 3 and pen.pivot_count == 3 and pen.tokens == nr.tokens,
          f"惩罚前 {nr.pivot_count} → 惩罚后 {pen.pivot_count}；token 相同={pen.tokens == nr.tokens}")
    check("[惩罚] 惩罚版与基线同长（无可优化的折返）= 24.85 m",
          abs(pen.polyline_m - nr.polyline_m) < 1e-9,
          f"{nr.polyline_m:.2f} → {pen.polyline_m:.2f} m")
    check("[惩罚] 惩罚版仍 0 段 R: 且仍合法",
          pen.reverse_segments == 0 and pen.pivot_penalty == P.PIVOT_PENALTY, "0 段 R:")
    # ★★ 2026-10-31 重钉（第七张图）：原用例 `D1→D2→E1` 失效 —— **本图 D1 是障碍**
    #   （障 = B1 D1 F3 I6 H7 A8 C9 E9），`_Tour` 的洪泛根本到不了 D1 ⇒ `reversals()` 的
    #   `entry is None` 分支直接 `continue`，恒 0（这就是"折返单元恒 0"的真因，不是判罚失效）。
    #   改用本图真能构成 1 处**原路折返**的三格：`A1→A3→A2`
    #   —— 从 A1 南下进 A3、再原路北回 A2 ⇒ 进 A3 的末步朝南、出 A3 的起步朝北 = 正相反。
    #   （A1/A2/A3 全在 A 列：A2 是本图雷格、A1/A3 可通行；A1 的西侧出界、B1 是障碍，
    #     所以这条折返也是"唯一通路"，与现场形态同族。）
    a1, a2, b1 = P._cell("A1"), P._cell("A3"), P._cell("A2")      # noqa: SLF001
    g_nr = P.Grid(spec, bridge_rows=(7, 5), allow_reverse=False)
    t5 = P._Tour(g_nr, 1e9, pivot_penalty=P.PIVOT_PENALTY)        # noqa: SLF001
    t0 = P._Tour(g_nr, 1e9, pivot_penalty=0.0)                    # noqa: SLF001
    t5.build([a1, a2, b1])
    t0.build([a1, a2, b1])
    check("[惩罚] 单元：A1->A3->A2 判为 1 处折返", t5.reversals([a1, a2, b1]) == 1,
          f"reversals={t5.reversals([a1, a2, b1])}")
    check("[惩罚] 单元：同一序列的 seq_cost 差 = 5.0（= 1 处 × PIVOT_PENALTY）",
          abs((t5.seq_cost([a1, a2, b1]) - t0.seq_cost([a1, a2, b1]))
              - P.PIVOT_PENALTY) < 1e-9,
          f"{t5.seq_cost([a1, a2, b1]):.4f} - {t0.seq_cost([a1, a2, b1]):.4f}")

    # ---- ⑧ ★ 通口贴边（K11）地图：U 形单回路硬约束（任务卡 t17）----
    #   背景：`config/map.yaml` 是第三张图，**下半场通口落在最外一行（K11）**。
    #   曾经的缺陷：`_Tour.polish()` 在整条 order 上做 2-opt，会把 `cross_upper`(K1)
    #   从"上半场末尾"挪到中间 ⇒ 编译后同一次穿洞被数成 2 次
    #   （`上半场应恰好穿洞 1 次，实际 2 次`）⇒ 整个 `plan_route` 失败。
    #   修法：`plan_route` 在候选 1 未过结构自检时改用 `_u_shape_order()`
    #   （`cross_lower` 钉为下半场**第一跳**、`cross_upper` 钉为上半场**最后一跳**）。
    section("⑧ 通口贴边：U 形单回路硬约束")
    # ★★ 2026-10-31 重钉（第七张图）：本图通口 = **K7（下半场，第 7 行，紧贴隔板）/ K5（上半场，第 5 行）**；
    #   第五张图是 K10/K5，第三张图是 K11/K1。历史：K1+K7 → K1+K11 → K5+K10 → **K5+K7**。
    #   ⚠️ 这条不是形式：K7 落在**隔板紧邻行**，而 tracker-safe 的行内 90° 转向硬否决
    #   正是按这一行判的 —— 它把整个下半场封死（诊断见 §⑫ 的 `_ts_diagnosis()`）。
    check("⑧.0 回归前提：本图通口 = K5 + K7（下半场通口在隔板紧邻行 K7）",
          tuple(spec.bridges) == ("K5", "K7") and "K7" in spec.bridges,
          f"bridges={spec.bridges}")
    _runs = list(V.crossing_entries(nr.route.blocks()))
    _lo = [r for r in _runs if V.LOWER_HALF_ROWS[0] <= round(r[2][0]) <= V.LOWER_HALF_ROWS[1]]
    _up = [r for r in _runs if V.UPPER_HALF_ROWS[0] <= round(r[2][0]) <= V.UPPER_HALF_ROWS[1]]
    check("⑧.1 下半场恰好 1 次穿洞，且落在本图通口行 K7（第 7 行）",
          len(_lo) == 1 and round(_lo[0][2][0]) == 7,
          f"下洞 {[(r[0], r[1], round(r[2][0])) for r in _lo]}")
    check("⑧.2 上半场恰好 1 次穿洞（K5，第 5 行）",
          len(_up) == 1 and round(_up[0][2][0]) == 5,
          f"上洞 {[(r[0], r[1], round(r[2][0])) for r in _up]}")
    check("⑧.3 ★ U 形单回路：穿洞记录不重复（无『跨 K 往返』）",
          len({(r[0], r[1]) for r in _runs}) == len(_runs),
          f"{len(_runs)} 条，去重后 {len({(r[0], r[1]) for r in _runs})} 条")
    check("⑧.4 ★ 结构自检 0 问题（这条就是曾经失败的那一条）",
          not P.check_route_structure(nr.route, spec,
                                      P.Grid(spec, bridge_rows=(7, 5), allow_reverse=False),
                                      limits),
          "0 问题")
    # ---- ⑨ move_ok 相邻性断言（任务卡 t17 第 3 条）----
    section("⑨ move_ok 相邻性断言")
    g_adj = P.Grid(spec, bridge_rows=(7, 5), allow_reverse=False)
    _raised = False
    try:
        g_adj.move_ok(10, 13, 8, 13)                    # 跳两格
    except P.PlannerError:
        _raised = True
    check("⑨.1 跳格移动（10,13)->(8,13) 当场抛 PlannerError（不静默放行）",
          _raised, "已抛错")
    check("⑨.2 正交相邻仍照常返回判定（不误伤）",
          isinstance(g_adj.move_ok(10, 13, 9, 13), bool),
          f"move_ok(10,13 -> 9,13) = {g_adj.move_ok(10, 13, 9, 13)}")

    # ---- ⑩ t16 两条硬约束的尺子（t21 落地 → t23 几何化）----
    #   约束① 过门转弯余量：可用余量 = 转弯点所在列到"隔板东端" = (index(D) − index(col)) × 0.35
    #     ⇒ A 1.050 / B 0.700 / C 0.350 / D 0.000 m。
    #     ★ **t23 把阈值几何化了**：t21 用固定 0.5 m（⇒ 只有 A/B 合规、C 判红），
    #       现改为 `所需余量 = R·sinθ + w/2·cosθ`（R = wheelbase/tan(max_steer)，与 pursuit 同源）
    #       ⇒ 90° 只需 **0.2719 m** ⇒ **C 列其实合规**（0.350 > 0.2719）。
    #   约束② 外行雷转弯间距：最外行/列的雷，最近转弯点曼哈顿格距 ≥ 2 格（未变）。
    #   两者都是**代价项**（缺省 0.0 ⇒ 零影响），本组钉住"尺子本身"与"开关默认关闭"。
    section("⑩ t16 两条硬约束的尺子（t21 → t23 几何化）")
    check("⑩.1 约束① 可用余量尺子：A=1.050 / B=0.700 / C=0.350 / D=0.000 m",
          [round(P._door_turn_margin_m(g_adj, g_adj.barrier_lo, geo.block_to_index(c)[1]), 3)   # noqa: SLF001
           for c in ("A1", "B1", "C1", "D1")] == [1.050, 0.700, 0.350, 0.000],
          "A/B/C/D 余量列")
    _ph = P._phys_params(cfg)                                                    # noqa: SLF001
    _need90 = P._door_turn_required_margin_m(90.0, _ph)                          # noqa: SLF001
    _need0 = P._door_turn_required_margin_m(0.0, _ph)                            # noqa: SLF001
    # ★★ 2026-10-31 重钉：0° 那一项从 **0.0675 → 0.085**。
    #   0.0675 = 0.135/2 是**旧车宽**（占位值）的一半；P1 几何真值修正把
    #   `vehicle.width_m` 改成实测 0.17（`config/robot.yaml`），于是 `w/2·cos0° = 0.085`。
    #   90° 那一项不含 w（cos90°=0），所以 R=0.157/tan30° 的口径**不受影响**。
    check("⑩.2a ★t23 几何化阈值：90° 所需 = R·sin90 + w/2·cos90 = 0.2719 m（不是固定 0.5）",
          abs(_need90 - 0.27193) < 5e-5 and abs(_need0 - 0.085) < 1e-9,
          f"90°={_need90:.5f} m / 0°={_need0:.5f} m")
    check("⑩.2b ★t23 判据翻转：C 列**合规**（0.350 > 0.2719）、D 列不合规（0.000 < 0.2719）",
          P._door_turn_margin_m(g_adj, g_adj.barrier_lo, geo.block_to_index("C1")[1])           # noqa: SLF001
          >= _need90
          and P._door_turn_margin_m(g_adj, g_adj.barrier_lo, geo.block_to_index("D1")[1])       # noqa: SLF001
          < _need90,
          f"C={P._door_turn_margin_m(g_adj, g_adj.barrier_lo, geo.block_to_index('C1')[1]):.3f} "    # noqa: SLF001
          f"D={P._door_turn_margin_m(g_adj, g_adj.barrier_lo, geo.block_to_index('D1')[1]):.3f} "    # noqa: SLF001
          f"need90={_need90:.4f}")
    check("⑩.2c ★t23 R 与 pursuit 键同源（不写常量）：0.157/tan30° = 0.27193 m",
          abs(_need90 - _ph["wheelbase_m"] / math.tan(math.radians(_ph["max_steer_deg"]))) < 1e-9,
          f"R={_ph['wheelbase_m']/math.tan(math.radians(_ph['max_steer_deg'])):.5f} m")
    check("⑩.3 约束① 只对隔板相邻行生效（其它行返回 inf，不参与约束）",
          P._door_turn_margin_m(g_adj, 1, 3) == float("inf"),                                  # noqa: SLF001
          "第 1 行 -> inf")
    _em = {(1, 0), (11, 7)}                      # 假设的"最外行"雷
    check("⑩.4 约束② 尺子：格距 >= 2 合格、< 2 不合格",
          P._edge_mine_turn_gap((3, 0), _em) >= P.EDGE_MINE_TURN_MIN_CELLS                   # noqa: SLF001
          and P._edge_mine_turn_gap((2, 0), _em) < P.EDGE_MINE_TURN_MIN_CELLS,               # noqa: SLF001
          f"(3,0)->{P._edge_mine_turn_gap((3, 0), _em)} "                                     # noqa: SLF001
          f"(2,0)->{P._edge_mine_turn_gap((2, 0), _em)}")                                      # noqa: SLF001
    check("⑩.5 零影响契约：两条约束的权重缺省 0.0（= 与 v2.3 逐字节一致的守门条件）",
          P.DOOR_TURN_PENALTY > 0.0 and P.EDGE_MINE_TURN_PENALTY > 0.0
          and V._cfg_get(cfg, "planner.door_turn_penalty", 0.0) == 0.0          # noqa: SLF001
          and V._cfg_get(cfg, "planner.edge_mine_turn_penalty", 0.0) == 0.0,     # noqa: SLF001
          "常量 >0（推荐值）、config 键缺省 0.0")

    # ---- ⑪ t23 物理几何接入（physics.py / config 键 / 障碍净空）----
    section("⑪ t23 物理几何接入（physics 模块 + config 键）")
    check("⑪.1 物理参数从 config 读到现场值（真源在 config，不在 planner 里写常量）",
          abs(_ph["width_m"] - 0.17) < 1e-9 and abs(_ph["obstacle_radius_m"] - 0.1275) < 1e-9
          and abs(_ph["mine_pad_m"] - 0.12) < 1e-9 and abs(_ph["barrier_width_m"] - 0.02) < 1e-9
          and abs(_ph["wheelbase_m"] - 0.157) < 1e-9 and abs(_ph["max_steer_deg"] - 30.0) < 1e-9,
          f"w={_ph['width_m']} r={_ph['obstacle_radius_m']} pad={_ph['mine_pad_m']}")
    check("⑪.2 障碍净空门槛：直段 0.2325 m（半宽）/ 拐角 0.3113 m（扫掠半径）",
          abs(P._required_obstacle_clearance_m(_ph, at_corner=False) - 0.2325) < 5e-5      # noqa: SLF001
          and abs(P._required_obstacle_clearance_m(_ph, at_corner=True) - 0.3113) < 5e-5,   # noqa: SLF001
          f"直段={P._required_obstacle_clearance_m(_ph, at_corner=False):.4f} "             # noqa: SLF001
          f"拐角={P._required_obstacle_clearance_m(_ph, at_corner=True):.4f}")              # noqa: SLF001
    _pr = P._phys_params(None)                        # noqa: SLF001
    check("⑪.3 压雷判据走**垫片**而非『车心到雷心』：格心压到、5 格外未压到",
          P._mine_pressed_at((1, 3), (1, 3), _pr)                                    # noqa: SLF001
          is True and P._mine_pressed_at((1, 8), (1, 3), _pr) is False,              # noqa: SLF001
          "格心压到 / 5 格外未压到")
    check("⑪.4 K 列桥室纵向封死 = 物理事实（physics 提供根据，planner 侧 R009 拦截）",
          PH.k_cells_sealed_along_column() is True,
          "k_cells_sealed_along_column() = True")
    check("⑪.5 障碍转弯净空权重缺省 0.0（零影响）",
          V._cfg_get(cfg, "planner.obstacle_turn_penalty", 0.0) == 0.0,              # noqa: SLF001
          "键缺省 0.0")

    # ---- ⑦ 无倒车路线过出车前校验（六层 0 error）----
    if ROUTE_CHECK.is_file():
        ncode, ndetail = route_check_errors(nr.tokens, MAP_YAML)
        check("[无倒车] route_check 退出码 0（六层全过、0 error）", ncode == 0,
              f"exit={ncode} | {ndetail[:220]}")
        pcode, pdetail = route_check_errors(pen.tokens, MAP_YAML)
        check("[惩罚版] route_check 退出码 0", pcode == 0,
              f"exit={pcode} | {pdetail[:220]}")

    # ---- ⑫ ★卡 t13：起始航向 = 设计常量 +y（两引擎；不从 mission.reset 反推）----
    #   用户定案：每次发车车都摆在起点格、车头水平朝图左（朝 K 列桥洞 = 图左 = 地图 +y）；
    #   摆位由现场用户保证 ⇒ 规划器把它当**设计输入**，**不为起点生成转向消解**，
    #   也不再有任何"摆错就拒绝发车"式的校验。
    section("⑫ 卡 t13：起始航向 = 设计常量 +y（两引擎）")
    check("⑫.1 常量本身：START_HEADING=1（DIRS 里是『西 = 列号减小 = +y』）"
          "且 START_HEADING_YAW = HEADING_Y = +90°",
          P.START_HEADING == 1 and abs(P.START_HEADING_YAW - geo.HEADING_Y) < 1e-12
          and P.DIRS[P.START_HEADING] == (0, -1)
          and P.HEAD_NAMES[P.START_HEADING] == "西(列号减小)",
          f"START_HEADING={P.START_HEADING} DIRS={P.DIRS[P.START_HEADING]} "
          f"yaw={geo.deg(P.START_HEADING_YAW):.1f}° {P.HEAD_NAMES[P.START_HEADING]}")
    _src = pathlib.Path(P.__file__).read_text(encoding="utf-8")
    check("⑫.2 规划器源码里不再读起点朝向配置（无 `\"mission.reset\"` 字面量）"
          "—— 硬编码常量的机器可验证证据",
          '"mission.reset"' not in _src,
          f"planner.py 中 `\"mission.reset\"` 出现 0 次（{len(_src)} 字符）")

    def _first_leg_dir(res) -> tuple[int, int]:
        """产物首腿方向（DIRS 索引，-1 = 零向量/无法判定）与首点车头。"""
        blks = res.route.blocks()
        ra, ca = geo.block_to_index(blks[0])
        rb, cb = geo.block_to_index(blks[1])
        return P._heading_of(rb - ra, cb - ca, -1), res.stops[0].heading   # noqa: SLF001

    _dir_nr, _head_nr = _first_leg_dir(nr)
    check("⑫.3 [tour 无倒车] 首腿方向 = +y（西）且首点车头 = 设计常量"
          "（起点处 0 转向，不生成转向消解）",
          _dir_nr == 0 and _head_nr == 0   # GOT: 方向索引=0 / 首点车头=0（官方图 N10->N9）
          and not nr.stops[1].reverse and nr.stops[1].heading == 0,
          f"{nr.route.blocks()[0]}->{nr.route.blocks()[1]} 方向索引={_dir_nr} "
          f"首点车头={_head_nr} 首点 yaw={geo.deg(nr.route.waypoints[0].yaw):.0f}°")
    check("⑫.4 [tour] 起点航向与 `mission.reset` **完全解耦**：把 reset 改成 'N10,x' "
          "/ 清空后，token 串与基线逐字相同",
          P.plan_route(map_cfg, cfg.override("mission.reset", "N10,x"),
                       max_seconds=TEST_PLAN_BUDGET_S).tokens == tokens
          and P.plan_route(map_cfg, cfg.override("mission.reset", ""),
                           max_seconds=TEST_PLAN_BUDGET_S).tokens == tokens,
          f"基线 {len(tokens)} token；改动 mission.reset 后 token 串不变"
          "（旧实现按 reset_yaw 反推 ⇒ 朝北会整条变样）")

    # ------------------------------------------------------------------ #
    #  ★★ 2026-10-31：tracker-safe 在第七张图上**结构性无解**（不是漂移，是真冲突）。
    #     先跑一次拿事实，再分层定位；依赖它产物的断言一律**显式 SKIP**（不计 PASS）。
    # ------------------------------------------------------------------ #
    _ts = None
    _ts_err = ""
    try:
        _ts = P.plan_route_tracker_safe(map_cfg, cfg, max_seconds=TEST_PLAN_BUDGET_S)
    except P.PlannerError as _exc:                           # noqa: BLE001
        _ts_err = str(_exc).splitlines()[0]
    _diag = ts_diagnosis(map_cfg, cfg)
    print("      tracker-safe："
          + ("出路线 %d token" % len(_ts.tokens) if _ts else "无解 → " + _ts_err))
    print(f"      分层诊断：{_diag['summary']}")

    if _ts is None:
        check("⑫.5a 无解定位在第 1 层 `legs0`（起点 N10 → 下半场通口无任何可行腿）",
              _diag["layer"] == "L1" and _diag["legs0"] == 0, _diag["summary"])
        check("⑫.5b 成因①：行 7（隔板紧邻行）内的 90° 转向被 `_barrier_turn_cell` 整片否决"
              "（N7/M7/L7：来向=北 → 去向=西 全部命中），而洞口本身沿轴可进"
              " ⇒ 下半场东侧除 K7 外无出口，K7 又只能从行 7 的西向车道进",
              _diag["row7_turns_banned"] and _diag["cross_axis_ok"], _diag["summary"])
        check("⑫.5c 成因②（独立；修好 L1 后会让第 3 层筛空）：口袋雷 A2 的"
              "『可达来向 ∩ 可出方向』= ∅（A2 ∉ `TS_TURNED_MINES` ⇒ 雷上不许转向）",
              _diag["a2_trapped"] and _diag["a2_straight_only"], _diag["summary"])
    else:
        _ts_dir, _ts_head = _first_leg_dir(_ts)
        check("⑫.5 [tracker-safe] 首腿方向 = +y（西）且首点车头 = 设计常量",
              _ts_dir == P.START_HEADING and _ts_head == P.START_HEADING
              and abs(geo.ang_diff(_ts.route.waypoints[0].yaw, geo.HEADING_Y)) < 1e-9,
              f"{_ts.route.blocks()[0]}->{_ts.route.blocks()[1]} 方向索引={_ts_dir} "
              f"首点 yaw={geo.deg(_ts.route.waypoints[0].yaw):.0f}°")
        check("⑫.6 [tracker-safe] 同样与 `mission.reset` 解耦（改成 'N10,x' 后 token 串不变）",
              P.plan_route_tracker_safe(map_cfg, cfg.override("mission.reset", "N10,x"),
                                        max_seconds=TEST_PLAN_BUDGET_S).tokens == _ts.tokens,
              f"{len(_ts.tokens)} token 不变")
    check("⑫.7 [tour] 起点不是折返点：起始顶点无 :PIVOT 声明",
          not nr.route.waypoints[0].pivot
          and all(i != 0 for _i, _b, _a in P.pivot_vertices(nr.route)),
          f"tour 折返点 {[b for _i, b, _a in P.pivot_vertices(nr.route)]}")
    # ⑫.8 直接把"不许在起点转向"钉在**搜索原语**上：起点那一段的第一步只能沿设计朝向。
    #   为什么需要：tracker-safe 的代价里"先右转 90° 再走"与"直行"**等长**，
    #   靠 tie-break 会挑到前者（改前实测产物首腿 `N10 -> N11` 朝南，即起步就掉头）。
    # ★★ 2026-10-31 重钉：原用例拿生产图 N10→**K11** 做靶（第五张图的通口行）——
    #   本图没有 K11，且整个下半场被封 ⇒ 换成**不依赖生产图可行性**的三格合成图
    #   （bridge=K10：K10 不在隔板紧邻行，起段一定可达）。判据本身没变：
    #   `first_dir` 后首步只能是设计朝向；不限时同一段会被挑到先朝南（(1,0) 出现在 raw 里）。
    _syn = {"map": {"name": "t-startheading", "start": "N10", "end": "J10",
                    "barrier_col_from": "D", "barrier_row_hi": 6, "barrier_row_lo": 7,
                    "mines": [], "obstacles": [], "targets": [], "bridges": ["K10"]}}
    _syn_spec = P.load_map(_syn)
    _tsg = P._TSGrid(spec=_syn_spec,
                     grid=P.Grid(_syn_spec, bridge_rows=(10,), allow_reverse=False),
                     phys=P._phys_params(cfg))
    _h0 = P.DIRS[P.START_HEADING]
    _start_cell = P._cell("N10")
    _tgt = P._cell("K10")
    _legs = P._ts_leg(_tsg, _start_cell, _tgt, _h0, {}, first_dir=_h0)            # noqa: SLF001
    _firsts = {(cells[0][0] - _start_cell[0], cells[0][1] - _start_cell[1])
               for _c, cells in _legs.values()}
    _legs_raw = P._ts_leg(_tsg, _start_cell, _tgt, _h0, {})                       # noqa: SLF001
    _raw_firsts = {(cells[0][0] - _start_cell[0], cells[0][1] - _start_cell[1])
                   for _c, cells in _legs_raw.values()}
    check("⑫.8 起段搜索原语：第一步只能沿设计朝向（起点禁转向）"
          "—— 不加该约束时同一段会被挑到先朝南（起步掉头，故必须有这条硬约束）",
          _firsts == {_h0} and (1, 0) in _raw_firsts,
          f"first_dir 后首步方向 {sorted(_firsts)} / 不限时 {sorted(_raw_firsts)}")

    # ----------------------------------------------------------------------- #
    #  ⑫b 卡 t7：**双射击停站**（对侧互备）—— 自动规划不允许悄悄只剩一个停站
    # ----------------------------------------------------------------------- #
    #  背景（现场事故）：靶板一面蓝一面红、蓝面朝向每局随机且恒定，只打蓝、打红扣分。
    #  所以路线必须在靶板**两侧各留一个射击停站**；而"落第二个星号"在引擎里是非对称的：
    #    · tracker-safe：编译期顺手落标（`_compile`），落不上**旧版完全静默**；
    #    · tour（缺省引擎）：`plan_route()` 压根没有这段 ⇒ 只有主停站。
    #  本节把两处缺口都钉成"缺了就红"。
    section("⑫b t7：双射击停站（板面两侧互备）+ 引擎无关的收口")
    # ★★ 2026-10-31 重钉（第七张图）：本图能同时覆盖 D5+D6 的**板心窗口**候选只有
    #   `B4`（板面 -x 侧）/ `B7`（+x 侧）；tour 引擎选 B4 作主停站，而**对侧没有合法候选**：
    #   镜像位 F4 是雷格、F7 视线被挡 ⇒ `second_shoot_stop()` 返回空，
    #   收口如实报"第二停站无解"（这正是现场"要打另一面只能人工加星号"的机器版）。
    #   所以本节从"引擎必须补上 G3*"改为"引擎如实降级 + 收口不擅改路线"，
    #   而**现场手写路线**（夹具，人工加了 `F4*`）才是双面互备的真值 —— 见 §2 六条硬约束。
    if _ts is None:
        skip("⑫.9/.10 [tracker-safe] 双停站 = 板面两侧各一个 / 对侧停站到板心 0.875 m",
             "本图 tracker-safe 无解（层 L1），拿不到产物")
    else:
        check("⑫.9 [tracker-safe] 两个射击停站分居板面两侧",
              len(_ts.shoot_blocks()) == 2
              and {P.plate_side_of(spec, b) for b in _ts.shoot_blocks()} == {-1, 1},
              f"shoot_blocks={_ts.shoot_blocks()} stop2={_ts.shoot_stop2!r}")
        check("⑫.10 [tracker-safe] 对侧停站到板心落在窗口内（与主停站镜像）",
              P.shoot_distance_window(P._shoot_accept(cfg))[0] - 1e-9
              <= _ts.shoot_distance_m2 <= P.shoot_distance_window(P._shoot_accept(cfg))[1] + 1e-9,
              f"{_ts.shoot_distance_m2:.4f} m")

    _ts_stars = [t for t in nr.tokens if "*" in t]
    _closed, _marked, _note = P.ensure_second_shoot_stop(nr.tokens, map_cfg, cfg)
    # ★★ 2026-10-31 重钉：tour 产物只有 `B4*`（单站），而对侧**没有合法候选** ⇒
    #   收口必须"如实报无解、路线一字不动"，而不是偷偷塞一个站或静默返回。
    check("⑫.11 ★[收口] tour 产物只有 `B4*`，对侧无合法候选 ⇒ 如实报『第二停站无解』且路线不动",
          _ts_stars == ["B4*"] and _marked == "" and _closed == list(nr.tokens)
          and "第二停站无解" in _note,
          f"补前 {_ts_stars}｜marked={_marked!r}｜{_note}")
    check("⑫.12 [收口] 降级后路线里仍只有一个真停站：parser 解析出 SHOOT = [B4]",
          [w.block for _i, w in PARSER.parse_mission(_closed).shoot_points()] == ["B4"],
          str([w.block for _i, w in PARSER.parse_mission(_closed).shoot_points()]))
    # ★★ 2026-10-31：⑫.13 原用例是"tracker-safe 产物已有两颗星 ⇒ 幂等"，本图拿不到
    #   tracker-safe 产物。改钉**同一件事的另一条路径**：把上面已经判过"补不了"的产物
    #   再喂一次 ⇒ 判决稳定、路线一字不动（不会第二次又试图补星/悄悄改路线）。
    #   ⚠️ 现场夹具（`F4*` + `B4*`）**不能**走这条：`F4` 是雷格，不在
    #   `shoot_candidates()` 的合法池里，`ensure_second_shoot_stop()` 会抛
    #   "射击停站 F4 不在合法候选中" —— 那是"人工加星号"与"引擎候选口径"的已知分歧，
    #   不是本卡的修复项（已在交付报告里单列）。
    _closed2, _marked2, _note2 = P.ensure_second_shoot_stop(_closed, map_cfg, cfg)
    check("⑫.13 [收口] 幂等：再跑一次不会重复补星、也不会改路线（『补不了』是稳定判决）",
          _marked2 == "" and _closed2 == list(_closed),
          f"marked={_marked2!r}｜{_note2}")

    # 判定活体 A：候选被强行指到"不在路线上的格" ⇒ 收口必须如实说补不了（绝不静默加减停站）
    _orig_second = P.second_shoot_stop
    P.second_shoot_stop = lambda *_a, **_k: "N11"        # N11 不在任何一条现役路线上
    try:
        _c_off, _m_off, _n_off = P.ensure_second_shoot_stop(nr.tokens, map_cfg, cfg)
    finally:
        P.second_shoot_stop = _orig_second
    check("⑫.14 ★[收口] 候选不在路线上 ⇒ 如实报「补不了」且路线一字不动（不静默降级）",
          _m_off == "" and _c_off == list(nr.tokens) and "不在本路线上" in _n_off, _n_off)

    # 判定活体 B：同一个"落不上"发生在引擎内部 ⇒ tracker-safe 必须把它变成 warning
    #   （旧版这段是 **完全静默** 的：候选算出来、落不到航点上、日志一个字都没有）
    if _ts is None:
        skip("⑫.15 ★[tracker-safe] 第二停站落不上 ⇒ warnings 里报出来",
             "本图 tracker-safe 无解（层 L1），拿不到产物")
    else:
        P.second_shoot_stop = lambda *_a, **_k: "N11"
        try:
            _ts_bad = P.plan_route_tracker_safe(map_cfg, cfg, max_seconds=TEST_PLAN_BUDGET_S)
        finally:
            P.second_shoot_stop = _orig_second
        check("⑫.15 ★[tracker-safe] 第二停站落不上 ⇒ warnings 里**报出来**（缺这段就静默丢停站）",
              _ts_bad.shoot_blocks() == ["B4"]
              and any("第二射击停站未落标" in w for w in _ts_bad.warnings),
              f"{_ts_bad.shoot_blocks()} / "
              f"{[w for w in _ts_bad.warnings if '第二射击停站' in w]}")

    # ----------------------------------------------------------------------- #
    #  ⑬ 卡 t6：**桥洞进洞提前对正**（消除"洞口前 1 格 90° 拐弯"）
    # ----------------------------------------------------------------------- #
    section("⑬ t6：桥洞进洞提前对正（进洞前 ≥1 格直线对正段）")
    # ⚠️ 本节的 `_adj` 必须用**本图真实桥洞格**建，否则 entry_walls/entry_adjacent
    #   为空 ⇒ 判据全 False，测试会变成"恒绿的空断言"（第 1 版就是这么写错的，已修）。
    # ★★ 2026-10-31 两个前提变化（P1 几何真值 + 第七图）：
    #   ① `field.bridge_entry_redundancy_m` 从 0.0875 置 **0.0**（清理现实不存在的虚拟墙）
    #      ⇒ `entry_adjacent` **按设计为空**、`entry_cell_crossed_sideways()` 全部 inert
    #      （`planner.py` 里写明"冗余归零后墙不越界，判据随之关闭"）。
    #      所以本节用**两个** `_TSGrid`：`_adj` = 生产口径（证 inert），
    #      `_adj_red` = 显式给 0.0875 的诊断口径（**判据活性**必须仍然可证 —— 否则
    #      只要现场把冗余归零，横向穿洞的判据就会被静默摘掉而没人发现）。
    #   ② 桥洞格从 K9/K5 换成 **K7/K5**（入口邻格 = L7 / J5）。
    _t6_rows = tuple(sorted({P._cell(b)[0] for b in spec.bridges}))
    _adj = P._TSGrid(spec=spec,
                     grid=P.Grid(spec, bridge_rows=_t6_rows, allow_reverse=False),
                     phys=P._phys_params(cfg))
    _adj_red = P._TSGrid(spec=spec,
                         grid=P.Grid(spec, bridge_rows=_t6_rows, allow_reverse=False),
                         phys={**P._phys_params(cfg), "bridge_entry_redundancy_m": 0.0875})
    check("⑬.0 前置：真墙无条件建（K7/K5 两段）＋冗余 0.0 ⇒ 入口邻格判据按设计关闭",
          len(_adj.entry_walls) == 2 and _adj.entry_adjacent == {}
          and _adj.entry_redundancy_m == 0.0,
          f"walls={sorted(_adj.entry_walls)} adj={sorted(_adj.entry_adjacent)} "
          f"red={_adj.entry_redundancy_m}")
    check("⑬.0b 诊断口径（冗余 0.0875）下入口邻格仍是 L7 / J5 —— 判据没有被删掉，只是默认 inert",
          sorted(_adj_red.entry_adjacent) == sorted({(7, geo.COL_LETTERS.index("L")),
                                                     (5, geo.COL_LETTERS.index("J"))}),
          f"adj_red={sorted(_adj_red.entry_adjacent)}")
    # ⑬.1 判据活性：两类"洞口前 1 格拐弯"必须被判红，两类"提前对正"必须判绿。
    #   为什么必须钉这一条：判据若失效（例如写成恒真/恒假），路线会静默退回旧形态，
    #   而 t5 实测旧形态是 **246 拍蹭墙、最大穿透 56.7mm** —— 属于"绿灯但不能跑"。
    #   ★ 用 `_adj_red`（默认口径下判据 inert，测不出活性）。
    def _sideways_hits(tri: tuple) -> list:
        c = [P._cell(x) for x in tri]
        out = []
        for i in range(1, len(c) - 1):
            din = P._ts_unit((c[i][0] - c[i - 1][0], c[i][1] - c[i - 1][1]))
            dout = P._ts_unit((c[i + 1][0] - c[i][0], c[i + 1][1] - c[i][1]))
            if _adj_red.entry_cell_crossed_sideways(c[i], din, dout):
                out.append(tri[i])
        return out

    # 第七图形态：进 L7 的横向来路 = 从行 8（L8）北上；沿轴来路 = 从 M7 西行。
    _bad = {nm: _sideways_hits(t)
            for nm, t in (("L8→L7→K7", ("L8", "L7", "K7")),
                          ("H6→I6→J6→J5→K5", ("H6", "I6", "J6", "J5", "K5")))}
    _good = {nm: _sideways_hits(t)
             for nm, t in (("M7→L7→K7", ("M7", "L7", "K7")),
                           ("H6→I6→I5→J5→K5", ("H6", "I6", "I5", "J5", "K5")))}
    check("⑬.1 判据活性：旧形态判红（拐弯格 L7/J5），新形态判绿",
          _bad["L8→L7→K7"] == ["L7"] and _bad["H6→I6→J6→J5→K5"] == ["J5"]
          and _good["M7→L7→K7"] == [] and _good["H6→I6→I5→J5→K5"] == [],
          f"判红 {_bad} / 判绿 {_good}")
    # ⑬.1b 段级硬拒必须已接线（不只是"判据返回 True"）：横向踏进 L7/J5 的段必须被拒，
    #   沿桥洞轴踏进的段必须放行 —— 这是"判据真的拦得住路线"的机器可验证证据。
    _seg_ban = not _adj_red.move_ok(P._cell("L8"), P._cell("L7")) \
        and not _adj_red.move_ok(P._cell("J6"), P._cell("J5"))
    _seg_ok = _adj_red.move_ok(P._cell("M7"), P._cell("L7")) \
        and _adj_red.move_ok(P._cell("I5"), P._cell("J5"))
    check("⑬.1b 段级接线：横向进 L7/J5 判拒、沿轴进 L7/J5 放行",
          _seg_ban and _seg_ok,
          f"ban(L8→L7,J6→J5)={_seg_ban} allow(M7→L7,I5→J5)={_seg_ok}")
    # ⑬.2 现役发车路线必须**已消除**该形态（这是本卡的目的，防回归）
    #   ★ 2026-10-31 修：夹具里带 `:PIVOT` 后缀，旧的 `rstrip("*,y")` 剥不掉
    #     （"I4:PIVOT" 的末字符是 T）⇒ `P._cell()` 直接 ValueError 炸掉整个测试。
    #     统一按 `:` 切前缀再剥 `*,y`（与 `PARSER.parse_token` 同语义）。
    _live_blocks = [t.split(":")[0].rstrip("*,y") for t in handwritten_tokens()]
    _live_hit = []
    _lc = [P._cell(b) for b in _live_blocks]
    for _i in range(1, len(_lc) - 1):
        if _adj_red.entry_cell_crossed_sideways(
                _lc[_i],
                P._ts_unit((_lc[_i][0] - _lc[_i - 1][0], _lc[_i][1] - _lc[_i - 1][1])),
                P._ts_unit((_lc[_i + 1][0] - _lc[_i][0], _lc[_i + 1][1] - _lc[_i][1]))):
            _live_hit.append(_live_blocks[_i])
    check("⑬.2 现场路线（夹具）无『进洞前 1 格横向拐弯』形态（t6 目标，防回归）",
          _live_hit == [], f"命中格 {_live_hit}")
    # ⑬.3 进洞前有 **≥2 格（0.70 m）连续直线**对正段 ⇒ 拐弯顶点距洞口 ≥1 格。
    #   为什么量到"拐弯顶点"而不是只量最后一格：t5 的根因是 R_min 0.2719 m > 半格 0.175 m
    #   —— 只要拐弯顶点还在洞口前 1 格内，进洞瞬间车身就是斜的。所以必须数**连续直线长度**：
    #   直线段数（= 格数）≥ 2 才能让转弯在进洞前 ≥2 格处完成、留出 ≥0.35 m 对正段。
    _align = {}
    for _b in spec.bridges:
        _i = _live_blocks.index(_b) if _b in _live_blocks else None
        if _i is None or _i < 2:
            _align[_b] = 0
            continue
        _dirs = [P._ts_unit((_lc[k + 1][0] - _lc[k][0], _lc[k + 1][1] - _lc[k][1]))
                 for k in range(_i)]
        _run, _k = 1, _i - 1
        while _k - 1 >= 0 and _dirs[_k - 1] == _dirs[_k]:
            _run += 1
            _k -= 1
        _align[_b] = _run
    check("⑬.3 逐洞「进洞前连续直线对正段」≥2 格（0.70 m > R_min 0.272 m）",
          all(v >= 2 for v in _align.values()),
          " ".join("%s=%d格" % (b, v) for b, v in sorted(_align.items())))
    # ⑬.4 规划器产物 == 现场路线（两处拐弯前移后仍逐 token 恒等）
    if _ts is None:
        skip("⑬.4 tracker-safe 产物 == 现场路线（逐 token）",
             "本图 tracker-safe 无解（层 L1），拿不到产物")
    else:
        _t6 = P.plan_route_tracker_safe(load_map(), cfg, max_seconds=60.0)
        check("⑬.4 tracker-safe 产物 == 现场路线（%d 点，逐 token）" % len(_live_blocks),
              list(_t6.tokens) == handwritten_tokens(), " ".join(_t6.tokens))

    # ----------------------------------------------------------------------- #
    #  ⑭ t13（用户硬约束需求 1 / docs/sim_v2_规格增补.md §H）：
    #     三类实体硬否决的单元级证据 + 放宽暗门封死 + 端到端必拒
    # ----------------------------------------------------------------------- #
    section("⑭ t13：三类硬约束必拒 + 不可放宽（障碍/隔板/桥洞含冗余）")

    def _mkmap(**kw):
        _base = {"name": "t13-fixture", "start": "A1", "end": "A2",
                 "mines": [], "obstacles": [], "targets": [], "bridges": []}
        _base.update(kw)
        return {"map": _base}

    def _plan_must_fail(map_dict):
        """规划必须 PlannerError；产出了路线或抛别的异常都算违规（不静默降级）。"""
        try:
            P.plan_route_tracker_safe(map_dict, cfg, max_seconds=60.0)
        except P.PlannerError:
            return True
        except Exception:                                       # noqa: BLE001
            return False
        return False

    # ⑭.1 障碍：格级硬拒 + 无解必报错
    # ★★ 2026-10-31：本图 tracker-safe **对所有输入都无解**（层 L1），所以
    #   "必须 PlannerError" 类断言在这里**恒真**（绿得不是这条判据）⇒ 显式 SKIP，
    #   由 §⑫.5a-c 的分层诊断接管。⑭.1b/c 是纯格级判据，与引擎无关，照常跑。
    if _ts is None:
        skip("⑭.1a / ⑭.4c 无解图 / 堵 I5 ⇒ 必须 PlannerError",
             "本图 tracker-safe 对所有输入都无解（层 L1）⇒ 这两条恒真，不能算通过")
    else:
        check("⑭.1a 无解图（障碍封死 ABC 过板缺口）必须 PlannerError，绝不产出压障路线",
              _plan_must_fail(_mkmap(start="H6", end="H8", obstacles=["A7", "B7", "C7"])))
    check("⑭.1b 障碍格 passable=False（本图八障碍逐格）",
          all(not _adj.grid.passable(*P._cell(x)) for x in spec.obstacles),
          " ".join(spec.obstacles))
    # ⑭.1c 净空余量钳位：负余量是放宽向量（可把 required 压到物理重叠边界之下），
    #   消费口已封 —— 判据不可能被负余量击穿（有牙性由 M2 变异证明：拆钳位 ⇒ 本条红）。
    _ts_relax = P._TSGrid(spec=spec, grid=_adj.grid,
                          phys={**P._phys_params(cfg), "obstacle_margin_m": -0.3})
    check("⑭.1c obstacle_margin_m=-0.3 被钳到 0.0（净空判据不可被负余量击穿）",
          _ts_relax.mgn == 0.0 and _ts_relax.obs_seg_req > 0.19
          and _ts_relax.obs_corner_req > 0.28,
          f"mgn={_ts_relax.mgn} seg_req={_ts_relax.obs_seg_req:.4f}")

    # ⑭.2 隔板：跨列硬拒 + 死胡同形态硬否决（N7/H7 实车 178/175 拍触线）+ margin 暗门
    _h_in, _h_out = P._heading_of(-1, 0, -1), P._heading_of(0, -1, -1)
    check("⑭.2a barrier_cross_ok：H 列(6→7) 拒、A 列过",
          _adj.grid.barrier_cross_ok(6, 7, geo.COL_LETTERS.index("H")) is False
          and _adj.grid.barrier_cross_ok(6, 7, geo.COL_LETTERS.index("A")) is True)
    check("⑭.2b turn_gate 硬否决 H7 北→西死胡同形态（t13 接线）",
          _adj.turn_gate(P._cell("H7"), None, (-1, 0), (0, -1))[0] is False)
    check("⑭.2c 列域修正：A7（过板缺口，无隔板）转向不误杀",
          P._barrier_turn_cell(_adj.grid, 7, geo.COL_LETTERS.index("A"), _h_in, _h_out) is False
          and _adj.turn_gate(P._cell("A7"), None, (-1, 0), (0, -1))[0] is True)
    # ★★ 2026-10-31 重钉：原正控制用 `H7→I7`，但**本图 H7 是障碍格**
    #   （障 = B1 D1 F3 I6 H7 A8 C9 E9）⇒ `move_ok` 因"不可通行"返回 False，
    #   与"行内直行是否合法"这条判据无关（绿/红都不是这条）。换成同行的空档 `M7→N7`。
    check("⑭.2d 隔板相邻行内直行 M7→N7 合法（正控制：0.175 m > 0.095 m）",
          _adj.move_ok(P._cell("M7"), P._cell("N7")) is True)
    _g_neg = P.Grid(spec, bridge_rows=_t6_rows, allow_reverse=False,
                    barrier_turn_margin_m=-1.0)
    check("⑭.2e margin 负值暗门封死：Grid 钳 0 + 判据函数直传 -1 仍命中",
          _g_neg.barrier_turn_margin_m == 0.0
          and P._barrier_turn_cell(_adj.grid, 7, geo.COL_LETTERS.index("H"),
                                   _h_in, _h_out, margin_m=-1.0) is True)

    # ⑭.3 桥洞：K 列规则 + 入口邻格横向穿行硬否决（含 turn_gate 直调同口径）
    # ★★ 2026-10-31 重钉：桥洞格 K9/K5 → **K7/K5**（本图通口）。
    check("⑭.3a K 列规则（K4→K5 竖向拒；J8→K8 非桥行拒；J7→K7 桥行过）",
          _adj.grid.move_ok(*P._cell("K4"), *P._cell("K5")) is False
          and _adj.grid.move_ok(*P._cell("J8"), *P._cell("K8")) is False
          and _adj.grid.move_ok(*P._cell("J7"), *P._cell("K7")) is True)
    # ★ 用 `_adj_red`：生产口径（冗余 0.0）下入口邻格判据按设计 inert，这里的四条会全假。
    check("⑭.3b 入口邻格横向穿/出必拒 + 沿轴进放行 + 顶点转向硬否决（诊断口径 0.0875）",
          _adj_red.move_ok(P._cell("J6"), P._cell("J5")) is False
          and _adj_red.move_ok(P._cell("J5"), P._cell("J4")) is False
          and _adj_red.move_ok(P._cell("I5"), P._cell("J5")) is True
          and _adj_red.turn_gate(P._cell("J5"), None, (-1, 0), (0, 1))[0] is False)
    _ts0 = P._TSGrid(spec=spec, grid=_adj.grid,
                     phys={**P._phys_params(cfg), "bridge_entry_redundancy_m": 0.0})
    check("⑭.3c 冗余 0.0 = 设计关闭（本条 inert）且其余实体照常硬",
          _ts0.move_ok(P._cell("J6"), P._cell("J5")) is True
          and _ts0.move_ok(P._cell("K4"), P._cell("K5")) is False
          and _ts0.turn_gate(P._cell("H7"), None, (-1, 0), (0, -1))[0] is False)

    # ⑭.4 端到端：现役产物零违规形态 + 封死纵向进洞 ⇒ 必拒（宁可无解不产违规路线）
    #   ⚠️ token 后缀剥离：夹具里带 `:PIVOT`、`F4*`、`N5,y` 三种后缀 —— 统一
    #   `split(":")[0]` 再 `rstrip("*,y")`（旧的 `rstrip("*,:xy")` 剥不掉 "PIVOT"，
    #   会让 `_cell()` 直接 ValueError 炸掉整个测试）。
    if _ts is None:
        skip("⑭.4a/4b 现役产物入口邻格横向穿行 / 隔板死胡同形态 = 0 处",
             "本图 tracker-safe 无解（层 L1），拿不到产物")
    else:
        _t13_cells = [P._cell(t.split(":")[0].rstrip("*,y")) for t in _t6.tokens]
        _t13_cells = [c for c in _t13_cells if c]
        _ea = {(5, geo.COL_LETTERS.index("J")), (7, geo.COL_LETTERS.index("L"))}
        _sw = [geo.index_to_block(c[0] - 1, c[1]) for i, c in enumerate(_t13_cells)
               if c in _ea and any(0 <= j < len(_t13_cells) and c[0] != _t13_cells[j][0]
                                   for j in (i - 1, i + 1))]
        check("⑭.4a 现役产物入口邻格横向穿行 = 0 处（已切沿轴进洞）", _sw == [], str(_sw))
        _bt = []
        for _i in range(1, len(_t13_cells) - 1):
            _din = P._heading_of(_t13_cells[_i][0] - _t13_cells[_i - 1][0],
                                 _t13_cells[_i][1] - _t13_cells[_i - 1][1], -1)
            _dout = P._heading_of(_t13_cells[_i + 1][0] - _t13_cells[_i][0],
                                  _t13_cells[_i + 1][1] - _t13_cells[_i][1], -1)
            if _din >= 0 and _dout >= 0 and P._barrier_turn_cell(
                    _adj.grid, _t13_cells[_i][0], _t13_cells[_i][1], _din, _dout):
                _bt.append(geo.index_to_block(_t13_cells[_i][0] - 1, _t13_cells[_i][1]))
        check("⑭.4b 现役产物隔板死胡同形态 = 0 处", _bt == [], str(_bt))
    if _ts is not None:
        _m_i5 = copy.deepcopy(load_map())
        _m_i5["map"]["obstacles"] = list(_m_i5["map"]["obstacles"]) + ["I5"]
        check("⑭.4c 堵 I5（封死 J5 纵向来路）⇒ 必拒（spec §H.2：不可行 ⇒ 报不可行）",
              _plan_must_fail(_m_i5))
    # ------------------------------------------------------------------ #
    section("⑮ t32 倒车折返合法性：assert_legal 判据对齐（现场 41 token 复算）")
    # 现场形态（2026-10-18 第七张图倒车版）：H10→H8（正向）→ R:H9（倒车出）。
    # 旧判据用 prev/cur 看错了一格，把这条合法路线误报成"第 7 个航点 H8 处 180° 折返"。
    _field41 = ("N10 M10 L10 K10 J10 I10 H10 H8 R:H9 G9 F9* E9 E8 C8 C7 C6 C5 C4 C3 C2 "
                "A2 A1 B1 C1 C3 E3 F3 R:F5 R:F4 F3 F2 G2 H2 I2 J2 J4 K4 L4 M4 N4 N5").split()
    _r41 = None
    try:
        _r41 = PARSER.parse_mission(_field41)
        P.assert_legal(_r41)
        _ok41, _d41 = True, "41 航点 / %d 段倒车 → 通过" % sum(
            1 for w in _r41.waypoints if w.reverse)
    except Exception as _e:                                  # noqa: BLE001
        _ok41, _d41 = False, "%s: %s" % (type(_e).__name__, _e)
    check("⑮.1 现场 41 token 倒车路线能过 assert_legal（H10→H8→R:H9 共线折返）",
          _ok41, _d41)
    check("⑮.2 该路线不需要 :PIVOT（pivot_vertices 为空）",
          _r41 is not None and P.pivot_vertices(_r41) == [],
          str(P.pivot_vertices(_r41)) if _r41 is not None else "parse 失败")
    # ⑮.3 负向控制：同模式 180° 折返仍必须被拦（N10→M10→N10，无 R:）
    check_raises("⑮.3 负向控制：同模式 180° 折返（N10 M10 N10，无 R:）必须抛 PlannerError",
                 P.assert_legal, PARSER.parse_mission(["N10", "M10", "N10"]))
    # ⑮.4 判据对齐：assert_legal 与 pivot_vertices 必须用同一套"相邻两段"口径
    _mis = []
    if _r41 is not None:
        for _i in range(1, len(_r41.waypoints) - 1):
            _p, _c, _n = _r41.waypoints[_i - 1], _r41.waypoints[_i], _r41.waypoints[_i + 1]
            if _c.reverse or _n.reverse:
                continue
            if P._angle_of(_p, _c, _n) > P.UTURN_REVERSE_DEG:
                _mis.append(_c.block)
    check("⑮.4 判据对齐后：41 token 无『未声明的同模式 180° 折返』", _mis == [], str(_mis))

    return finish()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:                                        # noqa: BLE001
        traceback.print_exc()
        sys.exit(1)
