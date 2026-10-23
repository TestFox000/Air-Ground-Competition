#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路线规划命令行入口   tools/plan_route.py
================================================================================
一句话：**给一张 `map.yaml`，自动算出可下发的路线** —— 不需要手写 token。

    # 用现场地图生成路线（人类可读报告）
    python tools/plan_route.py

    # 只输出路线串（可直接喂 /nav/waypoint 或填进 robot.yaml）
    python tools/plan_route.py --emit tokens

    # 换一张地图（用户的原始诉求："我只要把地图点位（map.yaml）替换掉"）
    python tools/plan_route.py --map config/site_b.yaml

    # 生成 + 立刻跑出车前校验（复用 tools/route_check.py，0 error 才算过）
    python tools/plan_route.py --check

    # 机器可读（含耗时/代价/穿洞格/射击点/折返点证据数值）
    python tools/plan_route.py --json

    # ★ 倒车开关：默认**无倒车**（真车底盘不支持倒车，180° 折返由循迹层 PIVOT 完成）
    python tools/plan_route.py --emit tokens          # 默认：0 段 R: + 折返点 :PIVOT
    python tools/plan_route.py --allow-reverse        # 旧行为：允许 R: 倒车段（卡 A 基线）

    # ★ 折返惩罚（顺序搜索里给"进去再原路退出来"记 5.0 等效代价/处）
    python tools/plan_route.py --pivot-penalty 5.0    # 现场图实测：折返点 2 → 0

    # 指定穿洞格（无人机情报：下半场 K10、上半场 K3），并遍历所有合法组合
    python tools/plan_route.py --cross-lower K10 --cross-upper K3
    python tools/plan_route.py --all-crossings
    python tools/plan_route.py --engine tracker-safe --from-cell M9 --already-covered C1,A3
                                 # t12：增量续算（G4）——从 M9 接着算、C1/A3 视为已压雷

--------------------------------------------------------------------------------
参数来源（真源唯一性，本工具不新造任何一份配置）
--------------------------------------------------------------------------------
* 地图要素（起/终/雷/障/靶/通）：`--map` 指定的 `map.yaml`
  （缺省 = `--config` 同目录的 `map.yaml`，再缺省 = 工作区 `config/map.yaml`）
* 约束（射击窗口、穿洞候选、起点朝向）：`--config` 指定的 `robot.yaml`
  经 `dzbrain.core.config.Config` 读取，键名与 `docs/P0接口约定.md` 一致
* **倒车/折返**：`--no-reverse`（默认）走 `plan_route(allow_reverse=False)` —— 0 段 `R:`、
  每个 180° 折返顶点带 `:PIVOT` 声明；`--allow-reverse` 是旧行为（卡 A 基线逐字节一致）
* 校验：`tools/route_check.py`（**子进程调用原文件**，不复制它的规则）
* 规划本体：`src/dzbrain/dzbrain/route/planner.py`

退出码：0 = 成功（`--check` 时还要求 route_check 0 error）
        1 = 规划失败（无解/结构矛盾/超时）或 route_check 有 error
        2 = 参数/文件问题
================================================================================
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
WS = HERE.parent
SRC = WS / "src" / "dzbrain"

# ★ 必须先把自己所在的 tools/ 从 sys.path[0] 摘掉：`python tools/plan_route.py` 会把
#   `tools/` 塞进搜索路径首位，而那里躺着一个**同名模块** `tools/dzbrain.py`
#   （一个 ROS 包装脚本）—— 于是 `import dzbrain.route.planner` 会变成
#   "No module named 'dzbrain.main'; 'dzbrain' is not a package"。本轮实测踩过。
for _entry in (str(HERE), str(WS)):
    while _entry in sys.path:
        sys.path.remove(_entry)
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from dzbrain.core.config import Config                       # noqa: E402
from dzbrain.route import planner as PL                      # noqa: E402
from dzbrain.route.validate import validate_route, summary   # noqa: E402

EXIT_OK, EXIT_FAIL, EXIT_USAGE = 0, 1, 2


# --------------------------------------------------------------------------- #
#  文件定位
# --------------------------------------------------------------------------- #
def default_config_path() -> pathlib.Path:
    """配置中心：车上 `~/dzrobot/config/robot.yaml`，开发机工作区 `config/robot.yaml`。"""
    car = pathlib.Path(os.path.expanduser("~/dzrobot/config/robot.yaml"))
    if car.is_file():
        return car
    return WS / "config" / "robot.yaml"


def find_map_path(cfg_path: pathlib.Path, explicit: str | None) -> pathlib.Path:
    """地图定位：`--map` > `$ROBOT_MAP` > 与配置同目录的 map.yaml > 工作区 config/map.yaml。

    与 `dzbrain.main.load_and_check` / `tools/route_check.py:_find_map_file` 同一套顺序
    —— 三个入口找不到同一个文件的话，就会出现"预检用 A 图、校验用 B 图"的鬼故事。
    """
    if explicit:
        return pathlib.Path(explicit)
    env = os.environ.get("ROBOT_MAP")
    if env:
        return pathlib.Path(env)
    near = cfg_path.parent / "map.yaml"
    if near.is_file():
        return near
    return WS / "config" / "map.yaml"


def load_yaml(path: pathlib.Path) -> dict:
    import yaml
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


# --------------------------------------------------------------------------- #
#  校验（复用 tools/route_check.py，不复制规则）
# --------------------------------------------------------------------------- #
def run_route_check(tokens: list[str], cfg_path: pathlib.Path,
                    map_path: pathlib.Path,
                    already_covered: tuple[str, ...] | None = None) -> tuple[int, str]:
    """调用 `tools/route_check.py --route "..."`。

    地图不在默认位置时把 `ROBOT_MAP` 传进子进程 —— 这是该工具自带的入口，
    比复制一份路径逻辑更可靠（换了 `--map` 之后校验的必须还是同一张图）。

    `already_covered`（t12 增量续算）：已压雷格 → `--already-covered` 透传给
    route_check，覆盖判据不要求这些雷重压；缺省不传、校验口径逐字节不变。
    """
    script = HERE / "route_check.py"
    if not script.is_file():
        return EXIT_USAGE, f"找不到 {script}"
    env = dict(os.environ)
    env["ROBOT_MAP"] = str(map_path)
    # ★ 必须钉住子进程的输出编码：Windows 控制台默认 GBK，route_check 打印的
    #   ✅/❌/⚠️ 在 utf-8 解码下会直接抛 UnicodeDecodeError（本轮踩过）。
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"
    cmd = [sys.executable, str(script), "--route", " ".join(tokens),
           "--config", str(cfg_path)]
    if already_covered:
        cmd += ["--already-covered", *already_covered]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300,
                              cwd=str(WS), env=env, encoding="utf-8", errors="replace")
    except subprocess.TimeoutExpired:
        return EXIT_FAIL, "route_check.py 超时（300s）"
    return proc.returncode, (proc.stdout or "") + (proc.stderr or "")


def grep_route_check(text: str) -> list[str]:
    """摘出 route_check 输出里的结论行（✅/❌/⚠️ 与总结），给 --emit tokens 用。"""
    keep = []
    for line in text.splitlines():
        body = line.strip()
        if not body:
            continue
        if any(mark in body for mark in ("✅", "❌", "⚠️", "🎉")):
            keep.append(body)
    return keep


# --------------------------------------------------------------------------- #
#  规划
# --------------------------------------------------------------------------- #
def plan_once(map_cfg: dict, cfg: Config, args, *, cross_lower: str | None,
              cross_upper: str | None, map_path: pathlib.Path) -> PL.PlanResult:
    """规划一次。`--engine` 选引擎（默认 `tour` = 历史行为，逐字节不变）。

    ★★ t7：`--engine tracker-safe` 走 `plan_route_tracker_safe()` —— 把"真车能跟住"
      当硬约束（禁 180° / 禁紧凑 U / 雷直线穿心），代价只由"实走长度 + 转角"决定。
      实测（现役图）：新引擎给出 **62 token / 21.35 m**，双档仿真与物理在环
      **逐项追平或优于手工 62 点路线**（对比表见 `_tmp_session/t7/报告.md`）。
    """
    if getattr(args, "engine", "tour") == "tracker-safe":
        turned = tuple(args.turned_mines.split(",")) if args.turned_mines else ()
        covered = (tuple(c.strip() for c in args.already_covered.split(",") if c.strip())
                   if getattr(args, "already_covered", None) else ())
        return PL.plan_route_tracker_safe(
            map_cfg, cfg, cross_lower=cross_lower, cross_upper=cross_upper,
            phase_cell=args.phase_cell, turn_w=args.t7_turn_w,
            turned_mines=turned,
            from_cell=getattr(args, "from_cell", None), already_covered=covered,
            max_seconds=args.max_seconds, name=map_path.stem or "route",
            source=str(map_path))
    return PL.plan_route(
        map_cfg, cfg, cross_lower=cross_lower, cross_upper=cross_upper,
        allow_reverse=bool(args.allow_reverse), pivot_penalty=args.pivot_penalty,
        name=map_path.stem or "route", source=str(map_path),
        max_seconds=args.max_seconds)


def crossing_combinations(map_cfg: dict, cfg: Config) -> list[tuple[str, str]]:
    """所有 (下半场, 上半场) 合法组合 —— 现场"每局桥洞随机"时用它逐个验证。"""
    spec = PL.load_map(map_cfg)
    lower, upper = PL._cross_pools(cfg, spec)                 # noqa: SLF001 同包入口
    return [(lo, up) for lo in lower for up in upper]


# --------------------------------------------------------------------------- #
#  CLI
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="路径规划器：换一张 map.yaml 就自动生成路线（可过 route_check）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(default_config_path()),
                    help="配置中心 robot.yaml（射击窗口/穿洞候选/起点朝向）")
    ap.add_argument("--map", default=None, help="地图 map.yaml（缺省按主流程顺序查找）")
    ap.add_argument("--emit", choices=("report", "tokens", "json"), default="report",
                    help="输出形态：report=人类可读 / tokens=只有路线串 / json=机器可读")
    ap.add_argument("--check", action="store_true",
                    help="生成后跑 tools/route_check.py（0 error 才算成功）")
    ap.add_argument("--validate", action="store_true",
                    help="生成后跑 dzbrain 自己的 validate_route（R001~R014）")
    ap.add_argument("--json", action="store_true", help="等价于 --emit json")
    ap.add_argument("--cross-lower", default=None, help="下半场穿洞格（如 K10）")
    ap.add_argument("--cross-upper", default=None, help="上半场穿洞格（如 K3）")
    ap.add_argument("--all-crossings", action="store_true",
                    help="遍历穿洞候选的**所有组合**逐个规划（桥洞随机时用）")
    ap.add_argument("--max-seconds", type=float, default=None,
                    help=f"单次规划时间预算（默认：tour 引擎 {PL.DEFAULT_MAX_SECONDS}s / "
                         f"tracker-safe 引擎 {PL.TS_MAX_SECONDS}s）")
    # ★ 倒车开关：默认**无倒车**（真车底盘不支持倒车）。两者互斥、语义相反，
    #   所以放同一组；`--no-reverse` 只是把默认值写明，方便脚本自解释。
    rev = ap.add_mutually_exclusive_group()
    rev.add_argument("--no-reverse", dest="allow_reverse", action="store_false",
                     default=False,
                     help="无倒车模式（**默认**）：0 段 R:，180° 折返顶点带 :PIVOT 声明")
    rev.add_argument("--allow-reverse", dest="allow_reverse", action="store_true",
                     default=False,
                     help="旧行为：允许 R: 倒车段处理死角（与卡 A 基线逐字节一致）")
    ap.add_argument("--pivot-penalty", type=float, default=None,
                    help=f"顺序搜索的折返惩罚（等效代价单位/处）。缺省=**跟随配置** "
                         f"planner.pivot_penalty（键缺失 → 0.0）；现场图用 {PL.PIVOT_PENALTY}")
    ap.add_argument("--mine-turn-penalty", type=float, default=None,
                    help=f"『雷格不得作拐点』的惩罚权重（等效代价单位）。缺省=**跟随配置** "
                         f"planner.mine_turn_penalty（键缺失 → {PL.MINE_TURN_PENALTY}）；"
                         f"0 = 关闭「雷心直线穿越」的排序偏好")
    ap.add_argument("--quiet", action="store_true", help="只输出结果，不打印过程信息")
    # ★★ t7：引擎选择（默认 tour = 历史行为，逐字节不变）
    ap.add_argument("--engine", choices=("tour", "tracker-safe"), default="tour",
                    help="规划引擎：tour=历史『顺序+朝向联合优化』（默认，行为不变）；"
                         "tracker-safe=t7 新增『真车可跟踪』引擎（禁 180°/禁紧凑 U/"
                         "雷直线穿心，实测追平手工 62 点）")
    ap.add_argument("--phase-cell", default=None,
                    help="tracker-safe 引擎的隔板过渡航点（默认跟随 PL.TS_PHASE_CELL=C6）")
    ap.add_argument("--t7-turn-w", type=float, default=None,
                    help="tracker-safe 引擎的转角权重（米/90°，默认 0.10）")
    ap.add_argument("--turned-mines", default=None,
                    help="tracker-safe 引擎里允许 90° 拐过的『口袋雷』（逗号分隔，默认 D8）")
    # ★★ t12：增量续算（G4）CLI 出口——只对 tracker-safe 引擎生效
    ap.add_argument("--from-cell", default=None,
                    help="增量续算起点格（如 M9）：从当前格接着算，不再要求车在地图起点"
                         "（仅 --engine tracker-safe）")
    ap.add_argument("--already-covered", default=None,
                    help="已压格集合，逗号分隔（如 C1,A3；只对雷生效，写进来的雷不再要求重压）"
                         "（仅 --engine tracker-safe）")
    args = ap.parse_args(argv)
    if (args.from_cell or args.already_covered) and args.engine != "tracker-safe":
        print("❌ --from-cell / --already-covered 只支持 --engine tracker-safe（增量续算 G4）",
              file=sys.stderr)
        return EXIT_USAGE
    if args.engine == "tracker-safe":
        if args.phase_cell is None:
            args.phase_cell = PL.TS_PHASE_CELL
        if args.t7_turn_w is None:
            args.t7_turn_w = PL.TS_TURN_W
        if args.turned_mines is None:
            args.turned_mines = ",".join(PL.TS_TURNED_MINES)
    if args.max_seconds is None:
        # ★ 两种引擎的默认预算不同（tracker-safe 的状态空间多一维、且要遍历射击点候选）
        args.max_seconds = (PL.TS_MAX_SECONDS if args.engine == "tracker-safe"
                            else PL.DEFAULT_MAX_SECONDS)

    cfg_path = pathlib.Path(args.config)
    if not cfg_path.is_file():
        print(f"❌ 配置中心不存在: {cfg_path}", file=sys.stderr)
        return EXIT_USAGE
    map_path = find_map_path(cfg_path, args.map)
    if not map_path.is_file():
        print(f"❌ 地图文件不存在: {map_path}", file=sys.stderr)
        return EXIT_USAGE

    try:
        cfg = Config.load(str(cfg_path))
        map_cfg = load_yaml(map_path)
    except Exception as exc:                                  # noqa: BLE001
        print(f"❌ 读取配置/地图失败: {type(exc).__name__}: {exc}", file=sys.stderr)
        return EXIT_USAGE

    def note(text: str) -> None:
        if not args.quiet and args.emit == "report":
            print(text)

    note("=" * 78)
    note("  路径规划器 · 换地图即出路线")
    note("=" * 78)
    note(f"  配置中心 : {cfg_path}")
    note(f"  地图文件 : {map_path}")
    spec_name = (map_cfg.get("map") or {}).get("name", "")
    note(f"  地图名称 : {spec_name}")
    note("")

    combos: list[tuple[str | None, str | None]]
    if args.all_crossings:
        try:
            combos = list(crossing_combinations(map_cfg, cfg))
        except PL.PlannerError as exc:
            print(f"❌ {exc}", file=sys.stderr)
            return EXIT_FAIL
    else:
        combos = [(args.cross_lower, args.cross_upper)]

    results: list[tuple[str, str, PL.PlanResult]] = []
    failures: list[tuple[str, str, str]] = []
    for cross_lower, cross_upper in combos:
        if cross_lower is None and cross_upper is None and not args.all_crossings:
            cross_lower = cross_upper = None
        try:
            res = plan_once(map_cfg, cfg, args, cross_lower=cross_lower,
                            cross_upper=cross_upper, map_path=map_path)
        except PL.PlannerError as exc:
            failures.append((str(cross_lower or "默认"), str(cross_upper or "默认"), str(exc)))
            continue
        results.append((res.crossing.get("lower", "?"), res.crossing.get("upper", "?"), res))

    if not results:
        print("❌ 规划失败：没有生成任何可行路线", file=sys.stderr)
        for lo, up, msg in failures:
            print(f"   · 下={lo} 上={up}: {msg}", file=sys.stderr)
        return EXIT_FAIL

    # ---------------- 输出 ----------------
    worst = EXIT_OK
    checked: dict[int, tuple[int, str]] = {}
    if args.check:
        # ★ t12：增量续算口径——已压雷透传给 route_check（不要求重压）
        covered: tuple[str, ...] = ()
        if getattr(args, "already_covered", None):
            covered = tuple(c.strip().upper()
                            for c in args.already_covered.split(",") if c.strip())
        for idx, (_, _, res) in enumerate(results):
            code, text = run_route_check(res.tokens, cfg_path, map_path,
                                         already_covered=covered or None)
            checked[id(res)] = (code, text)
            if code != 0:
                worst = EXIT_FAIL

    validates: dict[int, list] = {}
    if args.validate:
        from dzbrain.route.parser import parse_mission
        for res in (r for _, _, r in results):
            try:
                route = parse_mission(res.tokens)
                issues = validate_route(route, map_cfg=map_cfg, cfg=cfg)
            except Exception as exc:                          # noqa: BLE001
                issues = [f"{type(exc).__name__}: {exc}"]
            validates[id(res)] = issues
            if any(getattr(i, "level", "") == "error" for i in issues):
                worst = EXIT_FAIL

    if args.emit == "json" or args.json:
        payload = {
            "config": str(cfg_path), "map": str(map_path), "map_name": spec_name,
            "plans": [res.as_dict() for _, _, res in results],
            "failures": [{"lower": lo, "upper": up, "error": msg}
                         for lo, up, msg in failures],
        }
        if args.check:
            payload["route_check"] = [
                {"lower": res.crossing.get("lower"), "upper": res.crossing.get("upper"),
                 "exit": checked[id(res)][0], "lines": grep_route_check(checked[id(res)][1])}
                for _, _, res in results]
        if args.validate:
            payload["validate"] = [
                {"lower": res.crossing.get("lower"), "upper": res.crossing.get("upper"),
                 "issues": [str(i) for i in validates[id(res)]]}
                for _, _, res in results]
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return worst

    if args.emit == "tokens":
        for _, _, res in results:
            print(" ".join(res.tokens))
            if args.check:
                for line in grep_route_check(checked[id(res)][1]):
                    print("   " + line)
        return worst

    for lo, up, res in results:
        print(res.summary())
        if args.check:
            code, text = checked[id(res)]
            mark = "✅ 0 error" if code == 0 else f"❌ exit={code}"
            print(f"  route_check: {mark}")
            for line in grep_route_check(text):
                print("    " + line)
        if args.validate:
            issues = validates[id(res)]
            print("  validate_route: " + (summary(issues).replace("\n", "\n    ")
                                          if issues else "✅ 0 error / 0 warn"))
        print("")

    if failures:
        print("⚠️ 以下穿洞组合无解：")
        for lo, up, msg in failures:
            print(f"   · 下={lo} 上={up}: {msg}")

    if args.check and worst == EXIT_OK:
        print("✅ 规划 + 出车前校验全部通过（route_check 0 error）")
    return worst


if __name__ == "__main__":
    sys.exit(main())
