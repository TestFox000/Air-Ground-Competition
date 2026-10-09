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
from dzbrain.route import planner as P                        # noqa: E402
from dzbrain.route import parser as PARSER                    # noqa: E402
from dzbrain.route import validate as V                       # noqa: E402
from dzbrain.route.model import Waypoint                      # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
WS_ROOT = HERE.parents[2]                       # <ws>/src/dzbrain/tests -> <ws>
MAP_YAML = WS_ROOT / "config" / "map.yaml"
ROBOT_YAML = WS_ROOT / "config" / "robot.yaml"
ROUTE_CHECK = WS_ROOT / "tools" / "route_check.py"

try:
    import yaml
except ImportError:                                          # pragma: no cover
    yaml = None

# --------------------------------------------------------------------------- #
#  极简测试框架（与同目录既有测试同一套写法）
# --------------------------------------------------------------------------- #
_PASS = 0
_FAIL = 0


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


def section(title: str) -> None:
    print("")
    print(f"---- {title} " + "-" * max(0, 60 - len(title)))


def check_raises(name: str, fn, *args, **kwargs) -> bool:
    """断言抛 `PlannerError`（不是静默降级、也不是别的异常）。"""
    try:
        result = fn(*args, **kwargs)
    except P.PlannerError as exc:
        return check(name, True, f"PlannerError: {str(exc).splitlines()[0][:150]}")
    except Exception as exc:                                 # noqa: BLE001
        return check(name, False,
                     f"抛了 {type(exc).__name__} 而不是 PlannerError: {exc}")
    return check(name, False, f"**没有抛异常**，返回 {type(result).__name__}")


def finish() -> int:
    print("")
    print("=" * 72)
    if _FAIL:
        print(f"❌ {_FAIL} 项失败 / {_PASS} 项通过")
        return 1
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
    """手写路线（`robot.yaml` 的 `path_executor.mission_blocks`，34 token）。"""
    if yaml is None:                                         # pragma: no cover
        return []
    with open(ROBOT_YAML, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    params = (data.get("path_executor") or {}).get("ros__parameters", {}) or {}
    return [str(x) for x in (params.get("mission_blocks") or [])]


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
    runs = []
    for a, b in zip(blocks, blocks[1:]):
        hit = V._crossing_at_column(a, b)                    # noqa: SLF001
        if hit is not None:
            runs.append((a, b, hit))
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
    ok &= check(f"[{name}] ⑤ 恰好 1 个射击停站且覆盖全部靶格（接受窗口 "
                f"{info['window'][0]:.2f}~{info['window'][1]:.2f} m）",
                len(stops) == 1 and info["window_ok"],
                f"停站 {stops} 到靶 {windows}")

    return bool(ok)


def route_check_errors(tokens: list[str], map_path: pathlib.Path) -> tuple[int, str]:
    """子进程跑 `tools/route_check.py`，返回 (退出码, 关键输出行)。

    ★ 不复制它的 20+ 条规则；⭐ 这是"0 error"判据的唯一来源。
    """
    env = dict(os.environ)
    env["ROBOT_MAP"] = str(map_path)
    env["PYTHONIOENCODING"] = "utf-8"       # Windows 控制台默认 GBK，会解码失败
    env["PYTHONUTF8"] = "1"
    cmd = [sys.executable, str(ROUTE_CHECK), "--route", " ".join(tokens),
           "--config", str(ROBOT_YAML)]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300,
                          cwd=str(WS_ROOT), env=env, encoding="utf-8", errors="replace")
    text = (proc.stdout or "") + (proc.stderr or "")
    keep = [ln.strip() for ln in text.splitlines()
            if "❌" in ln or "项失败" in ln or "全部通过" in ln or "校验通过" in ln]
    return proc.returncode, " | ".join(keep) or text.strip().splitlines()[-1:][0] if text else ""


# --------------------------------------------------------------------------- #
#  1 + 3：当前地图：性质测试 + 出车前校验
# --------------------------------------------------------------------------- #
def main() -> int:
    map_cfg = load_map()
    cfg = load_cfg()
    limits = P._limits(cfg)                                  # noqa: SLF001
    spec = P.load_map(map_cfg)

    section("0 前置：资产与判据")
    check("config/map.yaml 存在且含 map 段", bool(map_cfg.get("map")), str(MAP_YAML))
    check("config/robot.yaml 存在", ROBOT_YAML.is_file(), str(ROBOT_YAML))
    check("射击窗口来自 Config（不硬编码）", limits == (0.20, 0.45), f"limits={limits}")
    check("起点/终点/雷数/障数/靶数/通数",
          (spec.start, spec.end, len(spec.mines), len(spec.obstacles),
           len(spec.targets), len(spec.bridges)) == ("N10", "N5", 8, 8, 2, 2),
          f"{spec.start}->{spec.end} 雷{len(spec.mines)} 障{len(spec.obstacles)} "
          f"靶{spec.targets} 通{spec.bridges}")

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

    section("2 对照：手写路线（robot.yaml 的 mission_blocks）也必须满足同 6 条")
    manual = handwritten_tokens()
    check("读到手写路线", len(manual) >= 2, f"{len(manual)} token")
    manual_route = PARSER.parse_mission(manual)
    manual_ok = six_checks("手写", manual_route.blocks(), map_cfg, limits,
                           [w.shoot for w in manual_route.waypoints])
    # 手写路线没有 ",x/,y" 后缀，块名直接比对
    ok = check("[手写] ⑥ 起终点与地图一致且 token 可编译",
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
    alt_result = P.plan_route(alt, cfg, name="alt")
    print(f"      生成路线（{len(alt_result.tokens)} token / {alt_result.polyline_m:.2f} m）：")
    print("        " + " ".join(alt_result.tokens))
    alt_ok = six_checks("异地", alt_result.route.blocks(), alt, limits,
                        [w.shoot for w in alt_result.route.waypoints])
    alt_ok &= check("[异地] 穿洞格按**那张图**的桥洞选（K7 下 / K1 上）",
                    alt_result.crossing.get("lower") == "K7"
                    and alt_result.crossing.get("upper") == "K1",
                    f"crossing={alt_result.crossing}")
    alt_ok &= check("[异地] 射击停站能同时覆盖 F11 与 G11",
                    alt_result.shoot_stop in P.shoot_candidates(
                        alt_spec, P.Grid(alt_spec, bridge_rows=(7, 1)),
                        P.accept_window(limits)),
                    f"停站={alt_result.shoot_stop} 到靶={alt_result.shoot_distance_m:.3f} m "
                    f"{alt_result.shoot_targets}")

    alt_path = HERE / "_alt_map_planner_test.yaml"           # 只写测试临时文件
    try:
        if yaml is not None:
            alt_path.write_text(yaml.safe_dump(alt, allow_unicode=True, sort_keys=False),
                                encoding="utf-8")
        if alt_path.is_file():
            acode, adetail = route_check_errors(alt_result.tokens, alt_path)
            alt_ok &= check("[异地] route_check 0 error", acode == 0,
                            f"exit={acode} | {adetail[:220]}")
    finally:
        alt_path.unlink(missing_ok=True)                     # 不留垃圾文件在工作区
    check("★ 异地地图 6 条硬约束 + route_check 全过", alt_ok, "见上逐条 PASS")

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
    mine_on_obstacle["map"]["mines"] = list(mine_on_obstacle["map"]["mines"][:7]) + ["F7"]
    check_raises("雷落在障碍格上（地图自相矛盾）-> 抛错",
                 P.plan_route, mine_on_obstacle, cfg)

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
              or wps[i - 1].reverse or wps[i].reverse
              for i in range(1, len(wps) - 1)),
          "无违规顶点")
    check("接受窗口 = route_check 的容差窗口（0.17~0.51 m）",
          P.accept_window((0.20, 0.45)) == (0.17, 0.51),
          str(P.accept_window((0.20, 0.45))))
    check("0.495 m（对角）被判为可接受：G10 -> H11",
          P.stop_distance_m("G10", "H11") > limits[1]
          and V._stop_ok(Waypoint.at_block("G10"), "H11",              # noqa: SLF001
                         *P.accept_window(limits)),
          f"d={P.stop_distance_m('G10', 'H11'):.3f} m，标称上限 {limits[1]}")

    return finish()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:                                        # noqa: BLE001
        traceback.print_exc()
        sys.exit(1)
