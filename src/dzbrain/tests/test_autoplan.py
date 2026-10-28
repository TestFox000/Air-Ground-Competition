# -*- coding: utf-8 -*-
"""任务卡 B 自测：MissionTask 自动规划（读 map.yaml 自算路线）+ planner 配置段 + --plan-only。

    python src\\dzbrain\\tests\\test_autoplan.py

**纯脚本、无 pytest**：全过 exit 0，任一失败 exit 1。

覆盖（逐条对应任务卡 B 的验收项，每条都给数值证据）：
  1. `plan_route_from_config()` 纯函数：真地图成功 / 坏图失败 / **宽捕获**（YAML 语法错）/
     文件不存在 / "缺 mines 的坏图其实能解"（诚实记录：它不是失败用例）
  2. 地图路径解析顺序：**绝对路径 → 相对配置文件目录 → 相对工作区**（三个候选**都存在**时
     证明"配置目录优先"；只有工作区存在时证明"相对工作区"）
  3. `auto_plan_enabled=true` → 阶段 DRIVE、横幅「路线来源：自动规划」、日志有**完整 token
     列表**与**规划耗时(ms)**，且**执行路线 == PlanResult.tokens**
  4. `auto_plan_enabled=false` → 横幅「路线来源：手动配置」、行为与基线等价（读 route.tokens、
     **完全不调规划器**，连 map_path 指向不存在的文件都无所谓）
  5. 回退分支：坏图 + `fallback_to_tokens=true` → warn（含异常类型+消息）+ 回退 route.tokens
     + **仍进入 DRIVE**
  6. 反向分支：坏图 + `fallback_to_tokens=false` → **停 IDLE**、error、不 publish
     `EV_MISSION_STARTED`、可重入多拍仍不出发（硬失败定格）
  7. `python -m dzbrain --plan-only` 入口：退出码 0、token 列表 + 耗时、**不起线程**、
     无 ROS（不 import rclpy）、不传 `--config` 也能解析到工作区 `config/robot.yaml`
  8. 边界：`planner.*` 缺键的默认值、非法 `max_seconds` 不崩、`--plan-only` 失败退出码 1
"""

from __future__ import annotations

import contextlib
import io
import os
import pathlib
import re
import sys
import tempfile
import threading

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

try:
    sys.stdout.reconfigure(errors="replace")          # type: ignore[union-attr]
except Exception:                                      # pragma: no cover
    pass

from harness import check, finish, section                        # noqa: E402

from dzbrain.core.blackboard import (P_DRONE, P_ROUTE, P_SAFETY, P_TELEMETRY,  # noqa: E402
                                     Blackboard)
from dzbrain.core.config import Config                            # noqa: E402
from dzbrain.core.events import (EV_BRIDGE_INFO, EV_MISSION_STARTED, EV_ROUTE_LOADED,  # noqa: E402
                                 EventBus)
from dzbrain.core.state import DroneState, SafetyState            # noqa: E402
from dzbrain.io.contract import MissionPhase                      # noqa: E402
from dzbrain.route import planner as PL                           # noqa: E402
from dzbrain.route.parser import parse_mission, parse_token       # noqa: E402
from dzbrain.tasks.mission import (ENGINE_TOUR, ENGINE_TRACKER_SAFE,      # noqa: E402
                                   ROUTE_SOURCE_AUTO, ROUTE_SOURCE_MANUAL,
                                   MissionTask, plan_route_from_config, resolve_map_path)

from fakes import FakeBridge, FakeClock                           # noqa: E402

# --------------------------------------------------------------------------- #
WS = pathlib.Path(__file__).resolve().parents[3]          # <ws>/src/dzbrain/tests/x.py
CONFIG = WS / "config" / "robot.yaml"
MAP = WS / "config" / "map.yaml"
# CAR-DUAL-20261024: 接口成功场景固定用合法双面图；生产配置只读。
GOOD_MAP = pathlib.Path(__file__).resolve().parent / "fixtures" / "map_carloop_20261024.yaml"
#: ★ TRACK-PLAN-B1-20261021 §10/§12 的夹具：第七图 t24（D7/E7 两格靶板、K4/K10 桥洞）。
#:   靶板是 1:1 圆面 + 35×70 跨格板，沿 0.70 m 轨道横移、不旋转 ⇒ 相邻两格都要留
#:   可打停站；夹具与 _tmp_session/t35/map7_t24.yaml 逐字一致。
MAP7_TEXT = """map:
  name: field7-t24
  start: N10
  end: N5
  barrier_col_from: D
  barrier_row_hi: 6
  barrier_row_lo: 7
  mines:
  - B1
  - A2
  - I2
  - C3
  - E3
  - G5
  - I8
  - H9
  obstacles:
  - D5
  - I6
  - A7
  - F8
  - B9
  - F10
  - H10
  - A11
  targets:
  - D7
  - E7
  bridges:
  - K4
  - K10"""

#: ★★ 自足夹具（卡 D 修复带出的**测试卫生**问题）：
#:   本套件原先读 `_scratch_auto_plan/mission/*.yaml` —— 那些文件在 `.gitignore` 里
#:   （`.gitignore:10  _scratch_*/`），**换一个干净检出/快照就全没了**。
#:   实测：`git archive HEAD` 解出来的干净树里跑本套件，6 项失败全部是
#:   `FileNotFoundError: 地图文件不存在: <tree>/_scratch_auto_plan/mission/broken_map.yaml`
#:   —— 代码没问题，是**测试依赖了不进版本库的临时文件**。
#:   现在所有夹具都在 `tempfile` 目录里**现场生成**，测试可脱离工作区独立运行。
#: ★ TRACK-PLAN-B1-20261021：本机文件沙箱拒绝向 tempfile.mkdtemp() 新建目录写文件
#:   （PermissionError，实测），故夹具固定落在**工作区外层的可写目录**（WS.parent 可写）；
#:   目录名固定 ⇒ 复跑可复用，且不再往仓库里写临时文件。
_FIX = WS.parent / "_tmp_session" / "test_autoplan_fixtures"
_FIX.mkdir(parents=True, exist_ok=True)
MAP7 = _FIX / "map7_t24.yaml"


def _sub_mines(text: str, repl: str) -> str:
    """把 map.yaml 里 `mines:` 那一行换成 `repl`（保留原缩进）。"""
    return re.sub(r"(?m)^([ \t]*)mines:.*$", lambda m: m.group(1) + repl, text)


def _drop_mines(text: str) -> str:
    """整行删掉 `mines:`（这就是"缺 mines 的坏图"，实测**能解**，只能当反例）。"""
    return re.sub(r"(?m)^[ \t]*mines:.*\n", "", text)


def _make_fixtures() -> None:
    map_text = GOOD_MAP.read_text(encoding="utf-8")
    # ① 坏图：mines 含非法格名 `Z9` → load_map 抛 PlannerError（**回退分支的正确夹具**）
    (_FIX / "broken_map.yaml").write_text(_sub_mines(map_text, 'mines:      ["H1", "Z9"]'),
                                          encoding="utf-8")
    # ② 反例：整行删掉 mines → 实测**能解出路线**（不是失败用例，见 §1.12）
    (_FIX / "missing_mines.yaml").write_text(_drop_mines(map_text), encoding="utf-8")
    # ③ YAML 语法非法 → ParserError（**不是** PlannerError，用来测宽捕获）
    (_FIX / "bad_yaml_map.yaml").write_text(
        'map:\n  name: "x"\n  mines:      ["H1", "G2"\n   obstacles:  ["D1",\n', encoding="utf-8")
    # ④ 顶层不是映射 → ValueError
    (_FIX / "not_a_mapping_map.yaml").write_text("- a\n- b\n", encoding="utf-8")
    # ⑤ 门禁必红配置：真 robot.yaml + 一条过时的 2 点手写路线（幂等注入到 `    route:` 后）
    lines = CONFIG.read_text(encoding="utf-8").splitlines(keepends=True)
    at = next(i for i, ln in enumerate(lines) if ln.rstrip("\r\n") == "    route:")
    lines.insert(at + 1, '      tokens: ["N10", "N5"]          # ★ 本套件现场注入的过时路线\n')
    #   ★ 2026-10-25：本轮配置改成 auto_plan_enabled=true（布局模式）。本夹具测的是
    #   "手写 route.tokens 过时 => 门禁 R012 必红"，属**文件模式/手写路线**语义，
    #   必须钉回 false；否则自动规划绕过 route.tokens，R012 自然不再触发。
    _gf = re.sub(r"(?m)^      auto_plan_enabled:.*$",
                 "      auto_plan_enabled: false",
                 _success_map_config("".join(lines)))
    (_FIX / "gate_fail.yaml").write_text(_gf, encoding="utf-8")
    # ⑦ ★ TRACK-PLAN-B1：第七图 t24 夹具（§10/§12 换图重钉的靶心）
    MAP7.write_text(MAP7_TEXT, encoding="utf-8")
    # ⑥ ★卡 t21：`engine: tour` 的配置副本 —— 把"读配置"落到夹具里。
    #   为什么需要：§1.12/1.13（缺 mines 的图能解）与 §9（`--map` 透传）测的是 **tour 引擎**的
    #   既有语义，而 `config/robot.yaml` 的 `planner.engine` 自第七趟（用户指令）起是
    #   `tracker-safe` —— 它对这两张夹具图会"无解 / 被双门禁拦下"（那是 t11 引入的正确行为）。
    #   夹具只改 engine 与 map_path，其余逐字照抄真配置。
    _lines = CONFIG.read_text(encoding="utf-8").splitlines(keepends=True)
    (_FIX / "engine_tour.yaml").write_text(
        _success_map_config("".join("      engine: tour\n"
                if (ln.startswith("      ") and ln.lstrip().startswith("engine:")) else ln
                for ln in _lines)), encoding="utf-8")


def _success_map_config(text: str) -> str:
    """只在临时配置副本钉住成功图，不改变生产 config/。"""
    return re.sub(r"(?m)^      map_path:.*$",
                  lambda _m: f'      map_path: "{GOOD_MAP.as_posix()}"', text)


_make_fixtures()
BROKEN = _FIX / "broken_map.yaml"                 # mines 含非法格名 → PlannerError
BAD_YAML = _FIX / "bad_yaml_map.yaml"             # YAML 语法错 → ParserError（非 PlannerError）
MISSING_MINES = _FIX / "missing_mines.yaml"       # "缺 mines" 的坏图（其实能解）
GATE_FAIL = _FIX / "gate_fail.yaml"               # 门禁必红（route.tokens 只有 2 个点）
CFG_TOUR = _FIX / "engine_tour.yaml"              # ★卡 t21：engine=tour 的真配置副本
NO_SUCH_MAP = _FIX / "no_such_map.yaml"           # 不存在的路径（不需要真的建）


def real_cfg(**kv) -> Config:
    """真的 `config/robot.yaml`（`--config` 那条路径）+ 就地覆盖测试用键。

    ★ 用**真配置**而不是 `Config.from_dict`：planner 段有没有写进 ros__parameters 树、
    `source` 指向哪里（决定"相对配置文件目录"这条解析规则），都只有真配置能验证。
    """
    cfg = Config.load(str(CONFIG))
    cfg.override("planner.map_path", str(GOOD_MAP))
    cfg.override("mission.auto_start", True)
    cfg.override("mission.auto_start_delay_s", 0.0)       # 测试不需要等 10s
    for k, v in kv.items():
        cfg.override("planner." + k, v)
    return cfg


class Rig:
    """一辆"车"：FakeBridge + Blackboard + MissionTask + 事件记录（与 test_mission.py 同构）。"""

    WATCHED = (EV_MISSION_STARTED, EV_ROUTE_LOADED)

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.logs: list[str] = []
        self.task = MissionTask(self.bridge, self.bb, self.bus, cfg,
                                log=self.logs.append, clock=self.clock.now)
        self.sub = self.bus.subscribe("test", list(self.WATCHED), maxsize=512)
        self.events: list = []
        self.bb.set(P_SAFETY, SafetyState(ok=True), "test")
        self.task.setup()

    def tick(self, dt: float = 0.1) -> None:
        self.task.tick()
        self.events.extend(self.sub.poll(512))
        self.clock.advance(dt)

    def ticks(self, n: int, dt: float = 0.1) -> None:
        for _ in range(n):
            self.tick(dt)

    # ---- 观察 ----
    def text(self) -> str:
        return "\n".join(self.logs)

    def count(self, ev_type: str) -> int:
        return sum(1 for e in self.events if e.type == ev_type)

    def state(self) -> dict:
        return self.task.state()

    def logged_tokens(self) -> list[str]:
        """从日志里抠出「自动规划 token 列表（N）：...」那一行（验收项要求的"完整 token 列表"）。"""
        for line in self.logs:
            if line.startswith("自动规划 token 列表（"):
                return line.split("：", 1)[1].split()
        return []

    def logged_elapsed_ms(self) -> float | None:
        for line in self.logs:
            m = re.search(r"耗时 ([0-9.]+) ms", line)
            if m:
                return float(m.group(1))
        return None


def blocks_of(tokens) -> list[str]:
    return [parse_token(t)["block"] for t in tokens]


# =========================================================================== #
section("1. plan_route_from_config()：成功 / 坏图 / 宽捕获 / 文件不存在 / 缺 mines 的真相")
# =========================================================================== #
cfg = real_cfg(auto_plan_enabled=True, map_path=str(GOOD_MAP),
               allow_reverse=False, max_seconds=2.0, fallback_to_tokens=True)
out = plan_route_from_config(cfg, log=lambda s: None)
check("1.1 真地图 + 无倒车模式 → ok=True", out.ok, f"error={out.error!r}")
check("1.2 token 数 ≥ 2 且无空串", len(out.tokens) >= 2 and all(out.tokens),
      f"{len(out.tokens)} token")
check("1.3 ★ 无倒车模式：0 个 `R:` 段", not any(t.startswith("R:") for t in out.tokens),
      f"R段={sum(1 for t in out.tokens if t.startswith('R:'))}")
check("1.4 规划耗时 > 0 ms 且被记进 PlanOutcome", out.elapsed_ms > 0.0,
      f"{out.elapsed_ms:.1f} ms")
check("1.5 地图解析成了绝对路径", pathlib.Path(out.map_path).is_absolute()
      and pathlib.Path(out.map_path).is_file(), out.map_path)
check("1.6 每个 token 都能被 parse_token 解析（不是只在规划器内部合法）",
      all(parse_token(t)["block"] for t in out.tokens), f"首批 {out.tokens[:4]}")

out_bad = plan_route_from_config(cfg.override("planner.map_path", str(BROKEN)),
                                 log=lambda s: None)
check("1.7 坏图（mines 含非法格名 Z9）→ ok=False", not out_bad.ok, f"ok={out_bad.ok}")
check("1.8 ★ error 里带**异常类型**与**原始消息**",
      out_bad.error.startswith("PlannerError:") and "Z9" in out_bad.error, out_bad.error)
check("1.9 失败时 tokens 为空、耗时 0", out_bad.tokens == [] and out_bad.elapsed_ms == 0.0,
      f"tokens={out_bad.tokens} ms={out_bad.elapsed_ms}")

out_yaml = plan_route_from_config(cfg.override("planner.map_path", str(BAD_YAML)),
                                  log=lambda s: None)
check("1.10 ★ 宽捕获：YAML 语法错（**不是** PlannerError）也变成可回退的失败",
      (not out_yaml.ok) and ("Error" in out_yaml.error), out_yaml.error)

out_nofile = plan_route_from_config(cfg.override("planner.map_path", str(_FIX / "nope.yaml")),
                                    log=lambda s: None)
check("1.11 地图文件不存在 → FileNotFoundError（含实际尝试的路径）",
      out_nofile.error.startswith("FileNotFoundError:") and "nope.yaml" in out_nofile.error,
      out_nofile.error)

out_mm = plan_route_from_config(cfg.override("planner.map_path", str(MISSING_MINES)),
                                log=lambda s: None)
# ★★ 卡 t21（读配置口径）：这对断言说的是 **tour 引擎**的既有容忍度（缺 mines 的图它照样解得出）。
#   现役 `config/robot.yaml` 的 `planner.engine` 自第七趟（用户指令）起是 `tracker-safe`，
#   它对缺 mines 的图会**如实**被双门禁拦下（route_check「雷覆盖 1/8」）—— 那是 t11 引入的
#   正确行为，不是"缺 mines 能解"被推翻。所以这里把引擎**显式钉成 tour**（就地覆盖，不碰配置），
#   并把两种引擎下的行为都记进证据串，判据不再依赖配置现值。
out_mm_tour = plan_route_from_config(
    cfg.override("planner.map_path", str(MISSING_MINES)).override("planner.engine", ENGINE_TOUR),
    log=lambda s: None)
check("1.12 ★ 诚实记录：'缺 mines 的坏图'对 **tour** 引擎**不是一个失败用例**（它真能解出路线；"
      "现役 tracker-safe 则按双门禁如实拦下）",
      out_mm_tour.ok and (out_mm.ok or (out_mm.gate_ok is False)
                          or "门禁" in (out_mm.error or "")),
      f"tour ok={out_mm_tour.ok}({len(out_mm_tour.tokens)} token) / "
      f"配置引擎 ok={out_mm.ok} err={(out_mm.error or '')[:70]!r}")
check("1.13 所以回退分支的坏图夹具必须是 broken_map.yaml（mines 非法），不是缺 mines 的副本",
      (not out_bad.ok) and out_mm_tour.ok,
      f"坏图失败={not out_bad.ok} 缺mines(tour)成功={out_mm_tour.ok}")

logs_bad: list[str] = []
plan_route_from_config(cfg.override("planner.map_path", str(BROKEN)), log=logs_bad.append)
check("1.14 失败时日志回调收到「规划失败」+ 异常类型（车上的 warn 就靠它）",
      any("规划失败" in s and "PlannerError" in s for s in logs_bad),
      (logs_bad[0][:70] + "…") if logs_bad else "回调没收到任何日志")

#: ★ 本节之后一律用**同一次规划**的 token 数做基准，不写死 43 ——
#  规划器由 `planner-pivot` 并行演进（可能改 token 数），写死会让本卡在别人一提交就红。
N_EXPECT = len(out.tokens)


# =========================================================================== #
section("2. 地图路径解析顺序：绝对路径 → 相对配置文件目录 → 相对工作区")
# =========================================================================== #
abs_out = resolve_map_path(None, str(MAP))
check("2.1 绝对路径原样返回（不拼接）", abs_out == MAP, str(abs_out))

# 两个候选**都存在**的对照：配置目录里的那份必须赢过工作区里的那份。
# ★ 用 `config/robot.yaml` 当相对路径：工作区那份天然存在（仓库文件），配置目录那份放临时目录里
#   （2026-10-25：原用 config/map.yaml，该文件已归档 → 换成同样存在的 robot.yaml，判据不变），
#   于是**两个候选同时为真**，且测试不再往工作区里写任何文件（原先会留 `_scratch_auto_plan` 垃圾）。
probe_dir = _FIX / "_order_probe"
(probe_dir / "config").mkdir(parents=True, exist_ok=True)
rel = "config/robot.yaml"
ws_side = WS / "config" / "robot.yaml"
cfg_side = probe_dir / "config" / "robot.yaml"
cfg_side.write_text('map:\n  name: "CONFIGDIR"\n', encoding="utf-8")
cfg_probe = Config.from_dict({"x": 1}, source=str(probe_dir / "cfg.yaml"))
picked = resolve_map_path(cfg_probe, rel)
check("2.2 ★ 两个候选都存在时：**相对配置文件目录**优先于相对工作区",
      picked == cfg_side and picked != ws_side, f"picked={picked}")
check("2.3 内容确实是配置目录那份（marker=CONFIGDIR）",
      "CONFIGDIR" in picked.read_text(encoding="utf-8"), picked.read_text(encoding="utf-8").strip())

cfg_side.unlink()                                   # 只删配置目录那份，工作区那份是仓库文件
picked_ws = resolve_map_path(cfg_probe, rel)
check("2.4 配置目录那份不存在时 → 落到**相对工作区**（工作区那份确实存在）",
      picked_ws == ws_side and picked_ws.is_file() and "CONFIGDIR" not in
      picked_ws.read_text(encoding="utf-8"), f"picked={picked_ws}")

picked_none = resolve_map_path(Config.from_dict({}, source=str(probe_dir / "cfg.yaml")),
                               "no_such_dir/no_such_map.yaml")
check("2.5 都不存在 → 返回工作区候选（错误信息里能看到实际尝试的路径）",
      picked_none == WS / "no_such_dir/no_such_map.yaml" and not picked_none.is_file(),
      str(picked_none))
check("2.6 `Config.from_dict`（无真实文件）只走工作区候选，不炸",
      resolve_map_path(Config.from_dict({}), "config/map.yaml") == MAP,
      str(resolve_map_path(Config.from_dict({}), "config/map.yaml")))


# =========================================================================== #
section("3. auto_plan_enabled=true：启动即自己算路（验收项 2）")
# =========================================================================== #
rig = Rig(real_cfg(auto_plan_enabled=True, map_path=str(GOOD_MAP),
                   allow_reverse=False, max_seconds=2.0, fallback_to_tokens=True))
rig.ticks(3)
st = rig.state()
check("3.1 阶段 = DRIVE", rig.task.phase == MissionPhase.DRIVE, rig.task.phase)
check("3.2 ★ 横幅出现「路线来源：自动规划」",
      f"路线来源：{ROUTE_SOURCE_AUTO}" in rig.text(), "见日志")
check("3.3 遥测 route_source = 自动规划", st["route_source"] == ROUTE_SOURCE_AUTO,
      str(st["route_source"]))
check("3.4 恰好 publish 一次 EV_MISSION_STARTED", rig.count(EV_MISSION_STARTED) == 1,
      str(rig.count(EV_MISSION_STARTED)))
check("3.5 publish 了 EV_ROUTE_LOADED", rig.count(EV_ROUTE_LOADED) == 1,
      str(rig.count(EV_ROUTE_LOADED)))

toks = rig.logged_tokens()
check("3.6 ★ 日志里有**完整 token 列表**（无省略号、与独立规划同长）",
      len(toks) == N_EXPECT and "…" not in " ".join(toks), f"{len(toks)} token（基准 {N_EXPECT}）")
check("3.7 日志里的 token 列表 = 执行路线的块名序列（**执行路线 == PlanResult.tokens**）",
      blocks_of(toks) == list(rig.task.route.blocks()),
      f"log={blocks_of(toks)[:5]} route={list(rig.task.route.blocks())[:5]}")
check("3.8 执行路线长度 = token 数", len(rig.task.route) == len(toks),
      f"{len(rig.task.route)} vs {len(toks)}")
check("3.9 ★ 无倒车：执行路线里 0 个 R: 段（真车底盘不支持倒车）",
      not any(t.startswith("R:") for t in toks), f"R段={sum(1 for t in toks if t.startswith('R:'))}")
check("3.10 ★ 日志里有**规划耗时 (ms)**", (rig.logged_elapsed_ms() or 0.0) > 0.0,
      f"{rig.logged_elapsed_ms()} ms")
check("3.11 遥测 plan.elapsed_ms 与日志同源", float(st["plan"]["elapsed_ms"]) > 0.0,
      str(st["plan"]["elapsed_ms"]))
check("3.12 遥测记下了实际用的地图绝对路径与 allow_reverse=False",
      pathlib.Path(st["plan"]["map_path"]).is_file() and st["plan"]["allow_reverse"] is False,
      f"{st['plan']['map_path']} reverse={st['plan']['allow_reverse']}")
check("3.13 与 `plan_route_from_config` 独立重算的结果一致（确定性）",
      [parse_token(t)["block"] for t in out.tokens] == blocks_of(toks),
      f"独立={len(out.tokens)} / 任务={len(toks)}")
check("3.14 横幅标注「无倒车」",
      "无倒车" in rig.text(), "见 '路线来源：自动规划（map=... 耗时 ... 无倒车）'")
check("3.15 路线载入的 source 标明了自动规划",
      any("路线载入[自动规划(" in s for s in rig.logs), "见 路线载入[...]")


# =========================================================================== #
section("4. auto_plan_enabled=false：完全走旧逻辑（验收项 3）")
# =========================================================================== #
cfg_off = real_cfg(auto_plan_enabled=False, map_path=str(MAP),
                   allow_reverse=False, max_seconds=2.0, fallback_to_tokens=True)
rig_off = Rig(cfg_off)
rig_off.ticks(3)
base = parse_mission(cfg_off.get("route.tokens"))
st_off = rig_off.state()
check("4.1 阶段 = DRIVE", rig_off.task.phase == MissionPhase.DRIVE, rig_off.task.phase)
check("4.2 ★ 横幅出现「路线来源：手动配置」",
      f"路线来源：{ROUTE_SOURCE_MANUAL}" in rig_off.text(), "见日志")
check("4.3 遥测 route_source = 手动配置", st_off["route_source"] == ROUTE_SOURCE_MANUAL,
      str(st_off["route_source"]))
check("4.4 ★ 与基线逐字节等价：执行路线 == parse_mission(route.tokens)",
      list(rig_off.task.route.blocks()) == list(base.blocks())
      and len(rig_off.task.route) == len(base), f"{len(rig_off.task.route)} vs {len(base)}")
check("4.5 折线长度与基线一致（1e-9）",
      abs(rig_off.task.route.path_length() - base.path_length()) < 1e-9,
      f"{rig_off.task.route.path_length():.6f} vs {base.path_length():.6f}")
check("4.6 ★ 完全不调规划器：遥测 plan 为空 + 日志无「自动规划成功」",
      st_off["plan"] == {} and "自动规划成功" not in rig_off.text(), str(st_off["plan"]))
check("4.7 基线日志里也没有出现 token 列表行（没有规划就没有它）",
      rig_off.logged_tokens() == [], str(rig_off.logged_tokens()))

# 极端证明："手动模式根本不看 map_path" —— 指向不存在的文件也必须照常出发
rig_off2 = Rig(real_cfg(auto_plan_enabled=False, map_path=str(NO_SUCH_MAP)))
rig_off2.ticks(3)
check("4.8 ★ map_path 指向不存在的文件也照常出发（证明手动模式没碰规划器）",
      rig_off2.task.phase == MissionPhase.DRIVE
      and list(rig_off2.task.route.blocks()) == list(base.blocks()),
      f"{rig_off2.task.phase} / {len(rig_off2.task.route)} 点")


# =========================================================================== #
section("5. 回退分支：坏图 + fallback_to_tokens=true → warn 后仍进入 DRIVE（验收项 5）")
# =========================================================================== #
fb_cfg = real_cfg(auto_plan_enabled=True, map_path=str(BROKEN),
                  allow_reverse=False, max_seconds=2.0, fallback_to_tokens=True)
rig_fb = Rig(fb_cfg)
rig_fb.ticks(3)
txt = rig_fb.text()
st_fb = rig_fb.state()
check("5.1 ★ 仍进入 DRIVE（不因为地图坏了就不出车）",
      rig_fb.task.phase == MissionPhase.DRIVE, rig_fb.task.phase)
check("5.2 ★ warn 日志（⚠️）含异常类型 PlannerError 与原始消息 Z9",
      "⚠️" in txt and "PlannerError" in txt and "Z9" in txt, "见日志")
check("5.3 warn 里点名了 fallback_to_tokens=true 与「回退」动作",
      "fallback_to_tokens=true" in txt and "回退 route.tokens" in txt, "见日志")
check("5.4 ★ 回退后的执行路线 == route.tokens（54 点）",
      list(rig_fb.task.route.blocks()) == list(base.blocks()), f"{len(rig_fb.task.route)} 点")
check("5.5 路线来源改标「手动配置」（实际用的就是手动路线）+ 横幅同源",
      st_fb["route_source"] == ROUTE_SOURCE_MANUAL
      and f"路线来源：{ROUTE_SOURCE_MANUAL}" in txt, str(st_fb["route_source"]))
check("5.6 遥测记下失败证据：plan.ok=False + error 非空 + 失败的是 broken_map.yaml",
      st_fb["plan"]["ok"] is False and bool(st_fb["plan"]["error"])
      and "broken_map.yaml" in st_fb["plan"]["map_path"],
      f"ok={st_fb['plan']['ok']} err={st_fb['plan']['error'][:40]!r}")
check("5.7 仍然 publish 了 EV_MISSION_STARTED（回退也要出车）",
      rig_fb.count(EV_MISSION_STARTED) == 1, str(rig_fb.count(EV_MISSION_STARTED)))
check("5.8 载入 source 标明是回退来的",
      any("route.tokens/mission_blocks(自动规划失败回退)" in s for s in rig_fb.logs), "见日志")

# 文件不存在也算失败 → 同样回退
rig_nf = Rig(real_cfg(auto_plan_enabled=True, map_path=str(NO_SUCH_MAP),
                      allow_reverse=False, max_seconds=2.0, fallback_to_tokens=True))
rig_nf.ticks(3)
check("5.9 地图文件不存在 → 同样回退并进入 DRIVE",
      rig_nf.task.phase == MissionPhase.DRIVE
      and "FileNotFoundError" in rig_nf.text(), rig_nf.task.phase)


# =========================================================================== #
section("6. 反向分支：坏图 + fallback_to_tokens=false → 停 IDLE 不出发（验收项 6）")
# =========================================================================== #
nf_cfg = real_cfg(auto_plan_enabled=True, map_path=str(BROKEN),
                  allow_reverse=False, max_seconds=2.0, fallback_to_tokens=False)
rig_nf2 = Rig(nf_cfg)
rig_nf2.ticks(60)                                   # 6 s 仿真时间：足够触发 + 足够"重试窗口"
txt_nf = rig_nf2.text()
st_nf = rig_nf2.state()
check("6.1 ★ 阶段保持 IDLE（60 拍后仍不出发）",
      rig_nf2.task.phase == MissionPhase.IDLE, rig_nf2.task.phase)
check("6.2 ★ 没有 publish EV_MISSION_STARTED", rig_nf2.count(EV_MISSION_STARTED) == 0,
      str(rig_nf2.count(EV_MISSION_STARTED)))
check("6.3 也没有 EV_ROUTE_LOADED（路线根本没换）", rig_nf2.count(EV_ROUTE_LOADED) == 0,
      str(rig_nf2.count(EV_ROUTE_LOADED)))
check("6.4 ★ error 日志（❌）含异常类型 + 消息 + '停在 IDLE'",
      "❌" in txt_nf and "PlannerError" in txt_nf and "停在 IDLE，不出发" in txt_nf, "见日志")
check("6.5 黑盒 P_ROUTE 没被写入（其余线程不会被误导去循迹）",
      rig_nf2.bb.get(P_ROUTE) is None, str(rig_nf2.bb.get(P_ROUTE)))
check("6.6 遥测里的阶段仍是 IDLE（其余线程据此不敢循迹）",
      (rig_nf2.bb.get(P_TELEMETRY) or {}).get("mission_state", {}).get("phase")
      == MissionPhase.IDLE,
      str((rig_nf2.bb.get(P_TELEMETRY) or {}).get("mission_state", {}).get("phase")))
check("6.7 ★ 硬失败定格：error 只打一次（不是每拍刷屏重试）",
      txt_nf.count("❌ 自动规划失败") == 1, str(txt_nf.count("❌ 自动规划失败")))
check("6.8 遥测 plan.ok=False 且 fallback_to_tokens=False 被记下",
      st_nf["plan"]["ok"] is False and st_nf["plan"]["fallback_to_tokens"] is False,
      f"ok={st_nf['plan']['ok']} fb={st_nf['plan']['fallback_to_tokens']}")
check("6.9 也没有「路线来源：手动配置」的任务横幅（压根没开会）",
      "【任务开始】" not in txt_nf, "见日志")

# 定格只属于"那个任务实例"：换一个任务（手动模式，不碰规划器）照常出发
rig_rec = Rig(real_cfg(auto_plan_enabled=False))
rig_rec.ticks(2)
check("6.10 硬失败定格不污染后续：另起一个任务（手动模式）照常出发",
      rig_rec.task.phase == MissionPhase.DRIVE
      and list(rig_rec.task.route.blocks()) == list(base.blocks()),
      f"{rig_rec.task.phase} / {len(rig_rec.task.route)} 点")
check("6.11 P_TELEMETRY 里的 mission_state.plan 为失败证据（遥测可见，不藏在日志里）",
      (rig_nf2.bb.get(P_TELEMETRY) or {}).get("mission_state", {}).get("plan", {}).get("ok") is False,
      str((rig_nf2.bb.get(P_TELEMETRY) or {}).get("mission_state", {}).get("plan", {}).get("ok")))


# =========================================================================== #
section("7. --plan-only 入口（验收项 7）")
# =========================================================================== #
import dzbrain.main as M                                          # noqa: E402

check("7.1 新建了 __main__.py（`python -m dzbrain` 可达）",
      (WS / "src" / "dzbrain" / "dzbrain" / "__main__.py").is_file(), "__main__.py")
check("7.2 无 --config 也能解析到工作区 config/robot.yaml",
      pathlib.Path(M.default_config_path()).resolve() == CONFIG.resolve(),
      M.default_config_path())

cwd0 = os.getcwd()
n_before = threading.active_count()
mods0 = "rclpy" in sys.modules
try:
    os.chdir(WS.parent)                                # ★ 换到任何目录，靠 __file__ 定位
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = M.main(["--plan-only", "--map", str(GOOD_MAP)])
    out_txt = buf.getvalue()
finally:
    os.chdir(cwd0)
check("7.3 退出码 = 0", rc == 0, str(rc))
_tok_lines = [l for l in out_txt.splitlines() if l.startswith("token 列表（")]
check("7.4 打印了**完整 token 列表**（与独立规划同长）",
      len(_tok_lines) == 1 and len(_tok_lines[0].split("：", 1)[1].split()) == N_EXPECT,
      _tok_lines[0][:60] if _tok_lines else "没有这行")
check("7.5 ★ 打印了规划耗时 ms",
      re.search(r"规划耗时：[0-9.]+ ms", out_txt) is not None,
      str(re.findall(r"规划耗时：[0-9.]+ ms", out_txt)))
check("7.6 标明未起线程/未发指令/未连 ROS",
      "未起线程、未发指令、未连 ROS" in out_txt, "见输出")
check("7.7 ★ 不起线程", threading.active_count() == n_before,
      f"{n_before} → {threading.active_count()}")
check("7.8 ★ 无 ROS：全程没有 import rclpy", ("rclpy" in sys.modules) is mods0,
      f"before={mods0} after={'rclpy' in sys.modules}")
check("7.9 显式地图解析到合法双面夹具（cwd 无关）",
      str(GOOD_MAP) in out_txt, str(GOOD_MAP))
check("7.10 模式行 = 无倒车（planner.allow_reverse=false）",
      "无倒车（planner.allow_reverse=False）" in out_txt, "见输出")

# 坏图 → --plan-only 退出码 1（非 0 才有人看日志）
banner_bad = io.StringIO()
with contextlib.redirect_stdout(banner_bad):
    rc_bad = M.run_plan_only(real_cfg(auto_plan_enabled=True, map_path=str(BROKEN)))
check("7.11 坏图 → --plan-only 退出码 1（不假装成功）", rc_bad == 1, str(rc_bad))
check("7.12 坏图输出里含异常类型与消息",
      "PlannerError" in banner_bad.getvalue() and "Z9" in banner_bad.getvalue(), "见输出")

# ★ 门禁豁免：`route.tokens` 过时（手写路线已经不合法）时，`--plan-only` 仍必须能算出新路线
check("7.13 门禁必红的测试配置存在（route.tokens 被换成 2 个点）", GATE_FAIL.is_file(), str(GATE_FAIL))
buf_gf = io.StringIO()
with contextlib.redirect_stdout(buf_gf):
    rc_gf = M.main(["--plan-only", "--config", str(GATE_FAIL)])
gf_txt = buf_gf.getvalue()
check("7.14 ★ 门禁不过时 --plan-only 仍继续规划（旧版这一步被 `if errors: return 2` 挡成死代码）",
      rc_gf == 0, f"exit={rc_gf}")
check("7.15 但把门禁错误如实打印出来（不隐瞒）",
      "启动门禁未通过" in gf_txt and "[R012]" in gf_txt,
      [l for l in gf_txt.splitlines() if "[R012]" in l][:1])
buf_dr = io.StringIO()
with contextlib.redirect_stdout(buf_dr):
    rc_dr = M.main(["--dry-run", "--config", str(GATE_FAIL)])
check("7.16 ★ --dry-run 的门禁语义**未变**（同一份配置仍是 exit 2）", rc_dr == 2, str(rc_dr))
buf_bn = io.StringIO()
with contextlib.redirect_stdout(buf_bn):
    rc_bn = M.main(["--bench", "--seconds", "0.5", "--config", str(GATE_FAIL)])
check("7.17 ★ --bench 的门禁语义**未变**（坏路线不放行，exit 2）", rc_bn == 2, str(rc_bn))


# =========================================================================== #
section("8. 边界：缺键默认值 / 非法 max_seconds / 配置段真的在 ros__parameters 树下")
# =========================================================================== #
check("8.1 planner 段在真配置里、且是点号键（ros__parameters 空前缀）",
      "planner.auto_plan_enabled" in list(Config.load(str(CONFIG)).keys())
      and "planner.map_path" in list(Config.load(str(CONFIG)).keys()),
      "planner.* 可见")
raw_yaml = CONFIG.read_text(encoding="utf-8")
lines = raw_yaml.splitlines()
idx = [i for i, l in enumerate(lines) if l.startswith("    planner:")]
check("8.2 `planner:` 与 `route:`/`mission:` 同级同缩进（4 空格）",
      len(idx) == 1 and lines[idx[0]].startswith("    planner:")
      and any(l.startswith("    route:") for l in lines)
      and any(l.startswith("    mission:") for l in lines), f"行号 {idx[0] + 1}")
# ★ 2026-10-11 重钉（t20）：第三次换图后 `planner:` 段是 **7 个键**（新增 `mine_turn_penalty`），
#   且 `auto_plan_enabled` 被**关闭**（robot.yaml 该行注释：n11 图规划器有 t15 已知缺陷，
#   本次跑车用 `mission_blocks` 手工 v3 62 token；t16 修复落地后再改回 true）。
#   段范围改为"到下一个 4 空格顶层键为止"，不再用写死的 7 行窗口（配置里还有多行注释）。
def _is_top_key(line: str) -> bool:
    return (line.startswith("    ") and not line.startswith("     ")
            and not line.lstrip().startswith("#") and ":" in line)


_end = next((j for j in range(idx[0] + 1, len(lines)) if _is_top_key(lines[j])), len(lines))
seg = lines[idx[0]:_end]
# ★ 卡 t11：注释行（`      # …`）**不是键** —— 段内加注释是本卡的正常做法（`engine` 的依据
#   必须写在现场看得见的地方），所以过滤条件显式排除 `#` 开头，再加进键值断言。
_key_lines = [l for l in seg[1:]
              if l.startswith("      ") and not l.startswith("       ")
              and not l.lstrip().startswith("#") and ":" in l]
# ★ 2026-10-11 重钉（t22）：`planner:` 段再增一键 = **8 个键**
#   （t20 是 7 个；t22 物理几何卡新增 `obstacle_margin_m: 0.02`）。
# ★ 卡 t11 重钉：再增三键 = **11 个键** —— `engine`、`tracker_max_seconds`、`footprint_gate`。
# ★★ 卡 t21（读配置口径）：`want` 不再硬编码 `engine: tour` / `tracker_max_seconds: 5.0`
#   ——那两条是 t11 时代的默认值，第七趟（用户指令）现场已把 `engine` 改成 `tracker-safe`、
#   t19 又把预算按车端实测改成 `10.0`。现在**逐条从 `config/robot.yaml` 读**：
#   本断言要证明的是"这 11 个键都在 `planner:` 段里、值与配置一致"，不是"值必须是某个旧字面量"。
def _kv(name: str, value, *, quote: bool = False) -> str:
    """把配置值渲染成 `robot.yaml` 里那一行的写法（bool → true/false，str 原样）。"""
    text = ("true" if value else "false") if isinstance(value, bool) else str(value)
    return f'{name}: "{text}"' if quote else f"{name}: {text}"


_cfg_now = Config.load(str(CONFIG))
want = [_kv("auto_plan_enabled", _cfg_now.get("planner.auto_plan_enabled")),
        _kv("map_path", _cfg_now.get("planner.map_path"), quote=True),
        _kv("fallback_to_tokens", _cfg_now.get("planner.fallback_to_tokens")),
        _kv("allow_reverse", _cfg_now.get("planner.allow_reverse")),
        _kv("max_seconds", _cfg_now.get("planner.max_seconds")),
        _kv("pivot_penalty", _cfg_now.get("planner.pivot_penalty")),
        _kv("mine_turn_penalty", _cfg_now.get("planner.mine_turn_penalty")),
        _kv("obstacle_margin_m", _cfg_now.get("planner.obstacle_margin_m")),
        _kv("engine", _cfg_now.get("planner.engine")),
        _kv("tracker_max_seconds", _cfg_now.get("planner.tracker_max_seconds")),
        _kv("footprint_gate", _cfg_now.get("planner.footprint_gate"))]
check("8.3 ★ planner 段的实际 11 个键值 = **配置里读到的值**（含 engine / tracker_max_seconds）",
      all(any(w in l for l in seg) for w in want), "; ".join(want))
check("8.4 11 个键都在 planner 段内部（恰好 6 空格缩进，注释行不计）",
      len(_key_lines) == 11,
      f"{len(_key_lines)} 行：{[l.strip().split(':')[0] for l in _key_lines]}")

cfg_blank = Config.from_dict({"mission": {"auto_start": True, "auto_start_delay_s": 0.0},
                              "route": {"tokens": list(cfg_off.get("route.tokens"))}})
check("8.5 `planner.*` 全缺时的默认值：auto_plan 默认 **关**、fallback 默认 **开**、预算 2.0",
      bool(cfg_blank.get("planner.auto_plan_enabled", False)) is False
      and bool(cfg_blank.get("planner.fallback_to_tokens", True)) is True
      and float(cfg_blank.get("planner.max_seconds", 2.0)) == 2.0, "见 mission.py:287-296")
rig_blank = Rig(cfg_blank)
rig_blank.ticks(2)
check("8.6 缺 planner 段 → 按手动模式出发（向后兼容旧配置）",
      rig_blank.task.phase == MissionPhase.DRIVE
      and rig_blank.state()["route_source"] == ROUTE_SOURCE_MANUAL,
      f"{rig_blank.task.phase} / {rig_blank.state()['route_source']}")

cfg_junk = real_cfg(auto_plan_enabled=True, map_path=str(GOOD_MAP),
                    allow_reverse=False, max_seconds="not-a-number", fallback_to_tokens=True)
out_junk = plan_route_from_config(cfg_junk, log=lambda s: None)
check("8.7 非法 max_seconds 不崩（回落到默认预算 2.0s）", out_junk.ok
      and len(out_junk.tokens) == len(out.tokens), f"ok={out_junk.ok} err={out_junk.error!r}")

not_map = _FIX / "not_a_mapping_map.yaml"
not_map.write_text("- a\n- b\n", encoding="utf-8")          # 顶层是 list，不是地图映射
out_nm = plan_route_from_config(cfg_junk.override("planner.map_path", str(not_map)),
                                log=lambda s: None)
check("8.8 顶层不是映射的 YAML → 也算失败（ValueError），绝不拿空路线去发车",
      (not out_nm.ok) and out_nm.error.startswith("ValueError:") and out_nm.tokens == [],
      out_nm.error)


# =========================================================================== #
section("9. 卡 D finding D2/D3：--plan-only 的 --map 透传 + 门禁报文只打一次且标来源")
# =========================================================================== #
import tempfile                                                # noqa: E402


def _tokens_of(text: str) -> str:
    for line in text.splitlines():
        if line.startswith("token 列表（"):
            return line.split("：", 1)[1].strip()
    return ""


# 9.1 造一张**会改路线**的变体图：把一颗雷挪到远角的空格（config/ 只读，写在临时目录）
import yaml as _yaml                                           # noqa: E402

_raw = _yaml.safe_load(GOOD_MAP.read_text(encoding="utf-8"))
#: ★ TRACK-PLAN-B1：§10/§12 换到第七图 t24 夹具后，门禁/校验必须用同一张图的 dict。
_raw7 = _yaml.safe_load(MAP7.read_text(encoding="utf-8"))
_mines = list((_raw.get("map") or _raw).get("mines") or [])
_obst = set((_raw.get("map") or _raw).get("obstacles") or [])
_taken = set(_mines) | _obst | {"N5", "N10"}
_far = next(c for c in ("A11", "A1", "N7", "A3", "N1") if c not in _taken)
# ★ 沙箱不允许向 TemporaryDirectory 新建目录写文件（同 _FIX 的原因）⇒ 固定落在 _FIX 下
_tmpdir = _FIX / "cards"
_tmpdir.mkdir(parents=True, exist_ok=True)
_variant = _tmpdir / "map_variant.yaml"
_variant_raw = _yaml.safe_load(GOOD_MAP.read_text(encoding="utf-8"))
_variant_raw["map"]["mines"][0] = _far
_variant.write_text(_yaml.safe_dump(_variant_raw, allow_unicode=True), encoding="utf-8")
check("9.1 变体图已生成（把第一颗雷挪到远角空格，config/ 未改动）",
      _variant.is_file() and _far not in _mines, f"{_mines[0]}→{_far} @ {_variant}")

# ★★ 卡 t21（读配置口径）：本节的判据是 **D2「`--map` 透传」** —— 与"用哪个引擎、门禁过不过"无关。
#   变体图（把一颗雷挪到远角 A11/A1/…）在 **tracker-safe** 下是"**无解**"（禁 180°/禁紧凑 U/
#   雷穿心把它筛空），会让 `--plan-only` 退 1、把"地图透传"这条判据污染成"引擎能不能解这张图"。
#   故本节走 `CFG_TOUR`：显式 tour + 合法双面夹具；其余取真配置。
#   判据回到 D2 本身，同时验证未传 --map 时用配置里的 GOOD_MAP。
_buf_def = io.StringIO()
with contextlib.redirect_stdout(_buf_def):
    _rc_def = M.main(["--plan-only", "--config", str(CFG_TOUR)])
_default_tokens = _tokens_of(_buf_def.getvalue())

_buf_var = io.StringIO()
with contextlib.redirect_stdout(_buf_var):
    _rc_var = M.main(["--plan-only", "--config", str(CFG_TOUR), "--map", str(_variant)])
_var_txt = _buf_var.getvalue()
_var_tokens = _tokens_of(_var_txt)

check("9.2 两次 --plan-only 都 exit 0（engine=tour 夹具：本节只判 `--map` 透传，不判引擎能否解该图）",
      _rc_def == 0 and _rc_var == 0, f"默认={_rc_def} 变体={_rc_var}")
check("9.3 ★D2：`--map <变体图>` 真的决定规划用的地图（打印的是变体图）",
      str(_variant) in _var_txt and str(GOOD_MAP) in _buf_def.getvalue(),
      [l for l in _var_txt.splitlines() if l.startswith("地图:")][:1])
check("9.4 ★D2：变体图的 token 与默认图**不同**（旧代码：静默忽略 --map、逐字节相同）",
      # ★ 卡 t3：删掉"token 数也必须不同"这条附加条件 —— 50 点新路线下两张图的 tour 产物
      #   **恰好都是 49 token**（点数相同、内容不同），旧判据会误红。
      #   D2 要证明的是"`--map` 没被静默忽略"，token 串逐字不同 + 9.3 的"打印的是变体图"
      #   两条合起来就是充分证据。
      #   ⚠️ 也**不能**改成"变体路线必须经过被挪走的雷 A11"：tour 引擎的历史语义本来就是
      #   "缺 mines 的图也能解"（见 `planner.engine` 注释：tour 不跑门禁、行为逐字不变），
      #   实测它确实把 A11 漏掉了 —— 那是 tour 引擎的既有口径，不是 `--map` 没生效。
      _var_tokens != _default_tokens,
      f"默认 {len(_default_tokens.split())} token / 变体 {len(_var_tokens.split())} token；"
      f"挪走的雷 {_far}（tour 引擎会漏压它，属既有语义）；token 串"
      f"{'不同' if _var_tokens != _default_tokens else '相同'}")
check("9.5 D2 第二入口：`run_plan_only(cfg, map_path=…)` 也按该图规划",
      M.run_plan_only(real_cfg(auto_plan_enabled=True, engine=ENGINE_TOUR),
                      map_path=str(_variant), log=lambda s: None) == 0, "函数直调 exit 0")
_out_variant = plan_route_from_config(
    real_cfg(auto_plan_enabled=True, engine=ENGINE_TOUR), map_path=str(_variant),
    log=lambda s: None)
check("9.6 D2：`plan_route_from_config(map_path=…)` 的结果与 CLI 一致（同一段代码）",
      _out_variant.ok and " ".join(_out_variant.tokens) == _var_tokens,
      f"ok={_out_variant.ok} / {len(_out_variant.tokens)} token")

# 9.7 D3：门禁报文只打一次，且标注"只针对 route.tokens"
check("9.7 前置：门禁必红配置存在", GATE_FAIL.is_file(), str(GATE_FAIL))
_buf_g = io.StringIO()
with contextlib.redirect_stdout(_buf_g):
    _rc_g = M.main(["--plan-only", "--config", str(GATE_FAIL)])
_g = _buf_g.getvalue()
check("9.8 ★D3：门禁报文不再重复打印（出现次数 == 1）",
      _g.count("启动门禁未通过") == 1, f"出现 {_g.count('启动门禁未通过')} 次")
check("9.9 ★D3：报文标注了来源（只针对 route.tokens、与规划结果无关）",
      "只针对 route.tokens" in _g and "与下面的规划结果无关" in _g,
      [l for l in _g.splitlines() if "只针对" in l][:1])
check("9.10 D3：规划结果照旧产出（门禁报文不吞掉规划输出）",
      _rc_g == 0 and "静态规划结果" in _g and "token 列表（" in _g, f"exit={_rc_g}")

# 9.11 `announce=False` 时 load_and_check 自己不打印（dedup 的实现依据）
_buf_lc = io.StringIO()
with contextlib.redirect_stdout(_buf_lc):
    _cfg_lc, _map_lc, _errs_lc = M.load_and_check(str(GATE_FAIL), None, announce=False)
check("9.11 ★`load_and_check(announce=False)` 自己一个字都不打，但错误照样返回",
      _buf_lc.getvalue().strip() == "" and len(_errs_lc) >= 5,
      f"打印={_buf_lc.getvalue()!r} 错误数={len(_errs_lc)}")
_buf_lc2 = io.StringIO()
with contextlib.redirect_stdout(_buf_lc2):
    M.load_and_check(str(GATE_FAIL), None, announce=True)
check("9.12 对照：`announce=True`（bench/live/--dry-run 走这条）仍会打印门禁报文",
      "启动门禁未通过（拒绝启动）" in _buf_lc2.getvalue(), "见输出")
# 固定夹具目录不删（下一轮可直接复用；本机沙箱也无法删除仓外新建目录后的父链）

# =========================================================================== #
section("10. 卡 t11：`planner.engine` 开关（默认 tour）+ tracker-safe 双门禁 / 失败回退")
# =========================================================================== #
#  设计要点：
#   · **默认零影响**用"engine 键缺省（YAML null）vs 显式 tour"逐字一致来钉（10.8）。
#     ★ 卡 t21 修正：原先拿"第 1 节那次规划"当缺省值，而第 1 节走的是**配置现值**
#     （第七趟用户指令起 `planner.engine: tracker-safe`）⇒ 该条必然红；现在两侧都显式钉住。
#   · 新引擎的两个产物断言（无 :PIVOT / 不比 tour 长）都来自 t7 报告 §3 的硬指标；
#     门禁断言对"本机有没有 tools/route_check.py"做**条件化**处理：有则必须 rc=0，
#     没有（车端形态）则必须留痕说明退化为内置 validate（绝不假装跑过）。
#   · 失败回退用**注入假门禁**（monkeypatch `MT.check_planned_route`）确定性钉死，
#     不依赖"某张图恰好算不出路线"这种脆断言。
import dzbrain.tasks.mission as MT                                 # noqa: E402

check("10.1 ★ `planner.engine` 缺省 → tour（默认行为不变）",
      MT.resolve_plan_engine(Config.from_dict({})) == MT.ENGINE_TOUR,
      MT.resolve_plan_engine(Config.from_dict({})))
# ★★ 卡 t21（读配置口径）：真配置里的引擎**读出来**是什么就是什么（第七趟起是 tracker-safe），
#   不再硬编码 tour；断言改成"配置值与 `resolve_plan_engine` 的结论一致，且是已知引擎"。
check("10.2 真配置 robot.yaml → **配置里写明的引擎**（逐条读配置，且必须是已知引擎）",
      MT.resolve_plan_engine(real_cfg()) == str(real_cfg().get("planner.engine")).strip().lower()
      and MT.resolve_plan_engine(real_cfg()) in MT.PLAN_ENGINES,
      f"{real_cfg().get('planner.engine')!r} → {MT.resolve_plan_engine(real_cfg())}")
_eng_alias = [MT.resolve_plan_engine(real_cfg(engine=v))
              for v in ("tracker-safe", "tracker_safe", "Tracker-Safe", "t7")]
check("10.3 引擎名容错：tracker-safe / tracker_safe / Tracker-Safe / t7 → 同一个新引擎",
      set(_eng_alias) == {MT.ENGINE_TRACKER_SAFE}, str(_eng_alias))
_logs_eng: list[str] = []
_bad_eng = MT.resolve_plan_engine(real_cfg(engine="trour"), log=_logs_eng.append)
check("10.4 ★ 键名写错 → 回退 tour + 告警（不因一个配置字符串停摆）",
      _bad_eng == MT.ENGINE_TOUR and any("不是已知引擎" in s for s in _logs_eng),
      f"{_bad_eng} / {(_logs_eng[0][:40] if _logs_eng else '无日志')}")
check("10.5 YAML 写 null（键在但值为空）→ tour",
      MT.resolve_plan_engine(real_cfg(engine=None)) == MT.ENGINE_TOUR, "engine=None")
# ★★ 卡 t21（读配置口径）：两个预算都**从配置读**（tour=`max_seconds`、tracker-safe=
#   `tracker_max_seconds`）—— t19 已把后者按车端 ARM 实测重定为 10.0，硬编码 5.0 必然过期。
check("10.6 预算按引擎分开：tour=planner.max_seconds / tracker-safe=tracker_max_seconds（读配置）",
      abs(MT.plan_budget_seconds(real_cfg(), MT.ENGINE_TOUR)
          - float(_cfg_now.get("planner.max_seconds"))) < 1e-9
      and abs(MT.plan_budget_seconds(real_cfg(), MT.ENGINE_TRACKER_SAFE)
              - float(_cfg_now.get("planner.tracker_max_seconds"))) < 1e-9,
      f"{MT.plan_budget_seconds(real_cfg(), 'tour')} / "
      f"{MT.plan_budget_seconds(real_cfg(), 'tracker-safe')} "
      f"(config: {_cfg_now.get('planner.max_seconds')} / "
      f"{_cfg_now.get('planner.tracker_max_seconds')})")
check("10.7 新引擎预算可现场覆盖（planner.tracker_max_seconds=7.5）",
      abs(MT.plan_budget_seconds(real_cfg(tracker_max_seconds=7.5),
                                 MT.ENGINE_TRACKER_SAFE) - 7.5) < 1e-9, "7.5s")

# ---- 10.B 默认零影响：显式 tour == `engine` 键缺省（YAML 写 null）----
#   ★ 卡 t21：原判据拿"第 1 节那次规划"当缺省值，而第 1 节走的是 **配置现值**（第七趟起
#     = tracker-safe）⇒ t11 之后这条必然红。现在两侧都显式钉住：一次 `engine: null`
#     （缺省语义）、一次 `engine: tour`，逐字一致才叫"缺省 = tour、零影响"。
_out_tour = plan_route_from_config(
    real_cfg(auto_plan_enabled=True, map_path=str(MAP7), engine=MT.ENGINE_TOUR),
    log=lambda s: None)
_out_tour_null = plan_route_from_config(
    real_cfg(auto_plan_enabled=True, map_path=str(MAP7), engine=None),
    log=lambda s: None)
check("10.8 ★ engine 键缺省（YAML null）与显式 tour 逐字一致：token 列表完全相同",
      _out_tour.ok and _out_tour_null.ok and _out_tour_null.tokens == _out_tour.tokens,
      f"null={len(_out_tour_null.tokens)} vs tour={len(_out_tour.tokens)} token")
check("10.9 ★ tour 不跑门禁（gate_ok 恒 True，detail 写明'不跑'，历史行为逐字节不变）",
      _out_tour.gate_ok and _out_tour.engine == MT.ENGINE_TOUR
      and "不跑门禁" in _out_tour.gate_detail, _out_tour.gate_detail)

# ---- 10.C tracker-safe：车上入口真的走新引擎，且产物过了双门禁 ----
_cfg_ts = real_cfg(auto_plan_enabled=True, map_path=str(MAP7), engine=MT.ENGINE_TRACKER_SAFE,
                   fallback_to_tokens=True)
_logs_ts: list[str] = []
_out_ts = plan_route_from_config(_cfg_ts, log=_logs_ts.append)
check("10.10 ★ engine=tracker-safe → 规划成功且走的是新引擎",
      _out_ts.ok and _out_ts.engine == MT.ENGINE_TRACKER_SAFE,
      f"ok={_out_ts.ok} err={_out_ts.error[:60]!r}")
check("10.11 ★ 新引擎产物无 `:PIVOT`（仿真/真车都不建模原地折返，t7 的硬规则）",
      len(_out_ts.tokens) >= 2 and not any("PIVOT" in t for t in _out_ts.tokens),
      f"{sum(1 for t in _out_ts.tokens if 'PIVOT' in t)} 处")
check("10.12 ★ 新引擎路线长度 = 22.05 m（第七图 t24 夹具 tracker-safe 实测，t35 §3 同值）",
      0.0 < _out_ts.route_len_m <= 23.0 and abs(_out_ts.route_len_m - 22.05) < 0.03,
      f"{_out_ts.route_len_m:.2f} m vs tour {_out_tour.route_len_m:.2f} m"
      f"（64 token / 22.05 m 是 t35 复核在 map7_t24 上实测的 tracker-safe 基线；"
      f"换图重钉依据见 REPORT『旧期望→新期望』表）")
check("10.13 有射击停站点（`*` 后缀；route_check 的靶覆盖靠它）",
      any(t.endswith("*") for t in _out_ts.tokens),
      ",".join(t for t in _out_ts.tokens if t.endswith("*")))
_rc_script = MT.route_check_script(_cfg_ts)
_rc_ok_text = "route_check rc=0" if _rc_script else "route_check 不可用"
check("10.14 ★ 门禁通过：route_check 0 error + validate 0 error/0 warn",
      _out_ts.gate_ok and "validate 0 error, 0 warn" in _out_ts.gate_detail
      and _rc_ok_text in _out_ts.gate_detail,
      f"script={_rc_script} / {_out_ts.gate_detail}")
check("10.15 门禁结论进日志流（现场一眼看得见用哪个引擎、门禁过没过）",
      any("规划门禁[tracker-safe]" in s for s in _logs_ts), str(_logs_ts[:1])[:90])

# ---- 10.D 门禁函数本身：好路线过 / 坏路线拦 / 模块级状态不串味 ----
_g_ok = MT.check_planned_route(_out_ts.tokens, _raw7, _cfg_ts)
check("10.16 ★ 门禁（含 route_check）：新引擎路线 ok；有脚本时 route_check 可用且 rc=0",
      _g_ok.ok and (not _rc_script or (_g_ok.route_check_available
                                       and _g_ok.route_check_rc == 0)),
      _g_ok.summary)
_g_skip = MT.check_planned_route(_out_ts.tokens, _raw7, _cfg_ts, route_check_module=False)
check("10.17 ★ route_check 不可用/显式跳过时退化：只以 validate 为门禁 + notes 留痕（车端不部署 tools/）",
      _g_skip.ok and not _g_skip.route_check_available
      and any(("不可用" in n) or ("跳过" in n) for n in _g_skip.notes), _g_skip.summary)
_broken_route = ["N10", "N5"]
_g_bad = MT.check_planned_route(_broken_route, _raw7, _cfg_ts)
check("10.18 ★ 坏路线（跳过全部雷/靶、0 停站）→ 门禁 ❌，两道门都红",
      (not _g_bad.ok) and bool(_g_bad.validate_errors)
      and (not _rc_script or _g_bad.route_check_rc not in (None, 0)),
      _g_bad.summary)
_g_again = MT.check_planned_route(_out_ts.tokens, _raw7, _cfg_ts)
check("10.19 ★ 门禁不串味：坏路线之后同进程再判好路线仍 ok（FAIL/WARN 每次调用复位）",
      _g_again.ok and _g_again.route_check_errors == [] and _g_again.validate_errors == [],
      _g_again.summary)

# ---- 10.E 车上入口端到端：门禁通过 → 执行的真的是新引擎路线 ----
rig_ts = Rig(real_cfg(auto_plan_enabled=True, map_path=str(MAP7),
                      engine=MT.ENGINE_TRACKER_SAFE, fallback_to_tokens=True))
rig_ts.ticks(3)
st_ts = rig_ts.state()
check("10.20 ★ 阶段 DRIVE + 遥测标了引擎 tracker-safe 与门禁通过",
      rig_ts.task.phase == MissionPhase.DRIVE
      and st_ts["plan"].get("engine") == MT.ENGINE_TRACKER_SAFE
      and st_ts["plan"].get("gate_ok") is True,
      f"{rig_ts.task.phase} / engine={st_ts['plan'].get('engine')} / "
      f"gate={st_ts['plan'].get('gate_ok')}")
check("10.21 ★ 执行路线 == 新引擎产物（不是回退的手工 62 点）",
      list(rig_ts.task.route.blocks()) == blocks_of(_out_ts.tokens)
      and len(rig_ts.task.route) == len(_out_ts.tokens),
      f"{len(rig_ts.task.route)} 点 vs 规划 {len(_out_ts.tokens)} 点")
check("10.22 任务横幅里看得见引擎与门禁结论",
      "引擎 tracker-safe" in rig_ts.text() and "门禁✅通过" in rig_ts.text(), "见横幅")
check("10.23 遥测 plan.max_seconds 用的是新引擎预算（读配置 `tracker_max_seconds`，且 > tour 的）",
      abs(float(st_ts["plan"].get("max_seconds") or 0.0)
          - float(_cfg_now.get("planner.tracker_max_seconds"))) < 1e-9
      and float(st_ts["plan"].get("max_seconds") or 0.0)
      > MT.plan_budget_seconds(real_cfg(), MT.ENGINE_TOUR),
      f"遥测={st_ts['plan'].get('max_seconds')} / config="
      f"{_cfg_now.get('planner.tracker_max_seconds')} / "
      f"tour={MT.plan_budget_seconds(real_cfg(), MT.ENGINE_TOUR)}")

# ---- 10.F 门禁未过 → 按 fallback_to_tokens 回退 / 停 IDLE（注入假门禁，确定性钉死）----
_gate_real = MT.check_planned_route


def _gate_injected(*_a, **_k):
    """假门禁：恒红（模拟"新引擎产物在某张图上没过门禁"）。"""
    return MT.GateResult(ok=False, route_check_available=True, route_check_rc=1,
                         route_check_errors=["（测试注入）雷覆盖 6/8"],
                         validate_warns=["（测试注入）R011 段长"])


MT.check_planned_route = _gate_injected
try:
    rig_gf = Rig(real_cfg(auto_plan_enabled=True, map_path=str(MAP7),
                          engine=MT.ENGINE_TRACKER_SAFE, fallback_to_tokens=True))
    rig_gf.ticks(3)
    txt_gf, st_gf = rig_gf.text(), rig_gf.state()
    check("10.24 ★ 门禁未过 + fallback=true → 仍进入 DRIVE（回退也能出车）",
          rig_gf.task.phase == MissionPhase.DRIVE, rig_gf.task.phase)
    check("10.25 ★ 回退后的执行路线 == 现役 route.tokens（手工 62 点，逐字节一致）",
          list(rig_gf.task.route.blocks()) == list(base.blocks()),
          f"{len(rig_gf.task.route)} 点 vs 现役 {len(base)} 点")
    check("10.26 ★ 打印了原因：门禁报文 + 注入证据 + 回退动作，三样都在",
          "未过门禁" in txt_gf and "（测试注入）雷覆盖 6/8" in txt_gf
          and "回退 route.tokens" in txt_gf, "见日志")
    check("10.27 遥测记下失败证据：plan.ok=False / engine=tracker-safe / gate_ok=False",
          st_gf["plan"]["ok"] is False
          and st_gf["plan"]["engine"] == MT.ENGINE_TRACKER_SAFE
          and st_gf["plan"]["gate_ok"] is False
          and "未过门禁" in str(st_gf["plan"]["error"]),
          f"ok={st_gf['plan']['ok']} gate={st_gf['plan']['gate_ok']} "
          f"err={st_gf['plan']['error'][:48]!r}")

    rig_gf2 = Rig(real_cfg(auto_plan_enabled=True, map_path=str(MAP7),
                           engine=MT.ENGINE_TRACKER_SAFE, fallback_to_tokens=False))
    rig_gf2.ticks(60)
    txt_gf2 = rig_gf2.text()
    check("10.28 ★ 门禁未过 + fallback=false → 停 IDLE、不 publish EV_MISSION_STARTED",
          rig_gf2.task.phase == MissionPhase.IDLE
          and rig_gf2.count(EV_MISSION_STARTED) == 0
          and "❌" in txt_gf2 and "停在 IDLE，不出发" in txt_gf2,
          f"{rig_gf2.task.phase} / started={rig_gf2.count(EV_MISSION_STARTED)}")
    check("10.29 硬失败定格：错误日志只打一次（不是每拍刷屏）",
          txt_gf2.count("❌ 自动规划失败") == 1, str(txt_gf2.count("❌ 自动规划失败")))
finally:
    MT.check_planned_route = _gate_real
check("10.30 假门禁已还原（后续用例/其它套件不受污染）",
      MT.check_planned_route is _gate_real, str(MT.check_planned_route))

# ---- 10.G 参考点偏移（现场卷尺实测 +7.5 cm）与足迹诊断 ----
#  真源：`config/robot.yaml` 的 `/dzactuator: ros__parameters: vehicle: ref_offset_m`
#  （与 length_m/width_m 同段）。这里钉三件事：① 读得到真源（含段前缀回退）；
#  ② 诊断能报出 t5/t11 实测的挡板风险；③ 严重级开关在 warn/error 间真的起作用。
_vg = MT.vehicle_geometry(Config.load(str(CONFIG)))
check("10.31 ★ 车辆几何读得到真源：长度/宽度/偏移 = 0.28 / 0.17 / +0.075",
      abs(_vg["length_m"] - 0.28) < 1e-9 and abs(_vg["width_m"] - 0.17) < 1e-9
      and abs(_vg["offset_m"] - 0.075) < 1e-9, str(_vg))
check("10.32 ★ 段前缀回退真的在起作用（`dzactuator.vehicle.*` 是 YAML 里的键路径）",
      "dzactuator.vehicle.ref_offset_m" in list(Config.load(str(CONFIG)).keys()),
      "见 robot.yaml /dzactuator: ros__parameters: vehicle:")
_fp_cfg = real_cfg(auto_plan_enabled=True, map_path=str(GOOD_MAP))
_fp = MT.footprint_diagnostic(out.tokens, _raw, _fp_cfg)
check("10.33 ★ 足迹诊断用实测偏移跑得动（折线口径 400+ 采样）",
      _fp.samples > 400 and abs(_fp.offset_m - 0.075) < 1e-9,
      f"samples={_fp.samples} offset={_fp.offset_m}")
check("10.34 ★ t11 实测风险可见：+7.5 cm 下报挡板侵入（折线保守上界，非硬门禁）",
      (not _fp.ok) and any("挡板" in r for r in _fp.risks)
      and _fp.min_barrier_m < 9e8, _fp.summary[:110])
check("10.35 偏移改成 0 → 同一路线挡板不再报侵入（证明风险确实来自该偏移）",
      MT.footprint_diagnostic(out.tokens, _raw,
                              Config.load(str(CONFIG)).override("dzactuator.vehicle.ref_offset_m",
                                                                 0.0)).ok,
      "offset=0 时 ok")
check("10.36 门禁严重级默认 warn：足迹报侵入也不拦现役 tour（历史行为不变）",
      str(Config.load(str(CONFIG)).get("planner.footprint_gate")) == "warn"
      and _out_tour.ok and _out_tour.footprint_ok is False,
      f"gate={Config.load(str(CONFIG)).get('planner.footprint_gate')} "
      f"tour_ok={_out_tour.ok} footprint_ok={_out_tour.footprint_ok}")
_g_warn = MT.check_planned_route(_out_ts.tokens, _raw7, _cfg_ts)
_g_err = MT.check_planned_route(_out_ts.tokens, _raw,
                                _cfg_ts.override("planner.footprint_gate", "error"))
check("10.37 ★ 严重级开关有效：warn → ok=True；error → ok=False（同一门禁、同一路线）",
      _g_warn.ok and (not _g_err.ok)
      and any("足迹诊断" in n for n in _g_err.notes),
      f"warn.ok={_g_warn.ok} error.ok={_g_err.ok}")
check("10.38 遥测把足迹诊断原样带出来（现场不翻日志也能看到）",
      "足迹诊断" in str(_out_tour.footprint) and _out_tour.footprint_ok is False,
      _out_tour.footprint[:80])


# =========================================================================== #
#  11. ★卡 t6：无人机桥洞情报 → 规划器 cross_lower/cross_upper（自动规划路径）
# =========================================================================== #
section("11. 无人机桥洞情报 → 自动规划透传 cross_lower/upper（卡 t6）")
#  夹具：合法双面图副本，只把 `bridges:` 放宽成 K4/K5/K9/K10；
#  其余要素照抄 GOOD_MAP，避免接口测试依赖生产图能否完成双面任务。
_T6_MAP = _FIX / "map_bridge_intel.yaml"
_T6_MAP.write_text(
    re.sub(r"(?m)^([ \t]*)bridges:.*$",
           lambda m: m.group(1) + 'bridges:    ["K4", "K5", "K9", "K10"]',
           GOOD_MAP.read_text(encoding="utf-8")), encoding="utf-8")


def intel_cfg(**kv) -> Config:
    """自动规划 + 桥洞候选放宽到 {K9,K10}/{K4,K5} 的配置（其余逐字来自现役 robot.yaml）。"""
    cfg = real_cfg(auto_plan_enabled=True, map_path=str(_T6_MAP),
                   engine=MT.ENGINE_TOUR, fallback_to_tokens=True)
    cfg.override("route.cross_lower_candidates", ["K9", "K10"])
    cfg.override("route.cross_upper_candidates", ["K4", "K5"])
    cfg.override("route.cross_lower_default", "K10")
    cfg.override("route.cross_upper_default", "K5")
    cfg.override("mission.bridge_info_wait_s", 0.0)      # 本节的等待语义由 test_mission 覆盖
    cfg.override("drone.info_stale_s", 30.0)
    for key, val in kv.items():
        cfg.override(key, val)
    return cfg


def push_drone(rig: Rig, *, half=None, block=None, online: bool = True, age: float = 0.0,
               publish: bool = True) -> DroneState:
    """忠实模拟 `DroneTask._on_bridge_passable`（drone.py:283-287）：**写槽 + 发事件**。

    ★ 卡 t13：情报真源 = `EV_BRIDGE_INFO` 事件流（每条自带 half）；`P_DRONE` 只有**一个**
      桥洞槽且被覆盖式赋值，所以"先 lower 后 upper"之后槽里只剩 upper。夹具必须两件事都做，
      否则测的是"接线前 Mission 根本不读情报"的老盘面。
    """
    now = rig.clock.now()
    ts = (now - age) if block else 0.0
    state = DroneState(online=bool(online),
                       last_heartbeat=(now if online else 0.0),
                       bridge_half=half, bridge_block=block,
                       bridge_confidence=0.9, bridge_ts=ts)
    rig.bb.set(P_DRONE, state, "test")
    if publish and half in ("lower", "upper") and block:
        rig.bus.publish(EV_BRIDGE_INFO, source="drone", half=half, block=block,
                        confidence=0.9, ts=ts)
    return state


def kcells(rig: Rig) -> list[str]:
    return [b for b in rig.task.route.blocks() if b.startswith("K")]


# ---- 11.A 纯函数：cross_lower/upper 真的进了规划器（缺省 = 与接线前一致）----
_out_intel = plan_route_from_config(intel_cfg(), log=lambda s: None, cross_lower="K9")
check("11.1 ★ `cross_lower='K9'` 透传到底：规划产物 crossing.lower=K9、upper 保持默认 K5",
      _out_intel.ok and _out_intel.cross_lower == "K9" and _out_intel.cross_upper == "K5"
      and any(t.startswith("K9") for t in _out_intel.tokens),
      f"ok={_out_intel.ok} cross={_out_intel.cross_lower}/{_out_intel.cross_upper}")
_out_plain = plan_route_from_config(intel_cfg(), log=lambda s: None)
check("11.2 ★ 一个参数都不传（= 接线前）→ crossing 走 route.cross_*_default（K10/K5）",
      _out_plain.ok and _out_plain.cross_lower == "K10" and _out_plain.cross_upper == "K5",
      f"cross={_out_plain.cross_lower}/{_out_plain.cross_upper}")
check("11.3 换洞真的改了路线（不是只在遥测里改字）：token 列表不同",
      _out_intel.tokens != _out_plain.tokens,
      f"{len(_out_intel.tokens)} vs {len(_out_plain.tokens)} token")
_out_up = plan_route_from_config(intel_cfg(), log=lambda s: None, cross_upper="K4")
check("11.4 ★ 两半场独立：只给 upper=K4 → upper=K4、lower 仍是默认 K10",
      _out_up.ok and _out_up.cross_upper == "K4" and _out_up.cross_lower == "K10",
      f"cross={_out_up.cross_lower}/{_out_up.cross_upper}")
_outside_cfg = real_cfg(map_path=str(GOOD_MAP), engine=MT.ENGINE_TOUR)
_outside_lower, _outside_upper = PL._cross_pools(_outside_cfg, PL.load_map(_raw))
check("11.5a 前置：GOOD_MAP 下半场只有 K10，K9 确实在候选池外",
      _outside_lower == ["K10"] and "K9" not in _outside_lower,
      f"lower={_outside_lower} upper={_outside_upper}")
_out_bad = plan_route_from_config(
    _outside_cfg,
    log=lambda s: None, cross_lower="K9")
check("11.5 ★ t31 权威门放宽：现役候选只有 K10，但 K9 在**下半场行段（7~11 行）内** "
      "⇒ 规划器采纳（无人机新鲜情报可纠正过期地图）",
      _out_bad.ok and _out_bad.cross_lower == "K9" and _out_bad.cross_upper == "K4",
      f"ok={_out_bad.ok} cross={_out_bad.cross_lower}/{_out_bad.cross_upper} "
      f"err={_out_bad.error[:80]}")
_out_off = plan_route_from_config(
    Config.load(str(CONFIG)).override("planner.engine", MT.ENGINE_TOUR),
    map_path=str(GOOD_MAP),                # 2026-10-25 零地图后必须显式给图
    log=lambda s: None, cross_lower="K3")
check("11.5b ★ t31 **仍拒**：下半场给上半场行段的 K3 → PlannerError（跨半场借用不放宽）",
      (not _out_off.ok) and _out_off.error.startswith("PlannerError:")
      and "行段" in _out_off.error, _out_off.error[:90])
_out_z = plan_route_from_config(
    Config.load(str(CONFIG)).override("planner.engine", MT.ENGINE_TOUR),
    map_path=str(GOOD_MAP),                # 2026-10-25 零地图后必须显式给图
    log=lambda s: None, cross_lower="Z9")
check("11.5c ★ t31 **仍拒**：格名非法 Z9 → PlannerError（不是 ValueError 上抛、不炸调用方）",
      (not _out_z.ok) and _out_z.error.startswith("PlannerError:") and "Z9" in _out_z.error,
      _out_z.error[:90])

# ---- 11.B MissionTask：情报注入 → 执行路线用 K9 ----
rig = Rig(intel_cfg())
push_drone(rig, half="lower", block="K9")
rig.ticks(3)
st = rig.state()
check("11.6 ★ 自动规划 + 情报 → 阶段 DRIVE 且执行路线穿 K9（上半场仍默认 K5）",
      rig.task.phase == MissionPhase.DRIVE and kcells(rig) == ["K9", "K5"],
      f"{rig.task.phase} / {kcells(rig)}")
check("11.7 ★ 遥测同时记下『请求的格』与『规划产物实际用的格』（现场对账两栏都在）",
      st["plan"]["cross_requested"] == {"lower": "K9", "upper": ""}
      and st["plan"]["cross_lower"] == "K9" and st["plan"]["cross_upper"] == "K5",
      f"req={st['plan']['cross_requested']} got={st['plan']['cross_lower']}/{st['plan']['cross_upper']}")
check("11.8 遥测 `bridge.used` = 规划产物实际用格 {lower:K9, upper:K5}",
      st["bridge"]["used"] == {"lower": "K9", "upper": "K5"}, str(st["bridge"]["used"]))
check("11.9 日志两处可复盘：透传行 + 自动规划成功行都点了穿洞格",
      any("穿洞格（无人机情报）：下半场 K9" in s for s in rig.logs)
      and any("穿洞 下=K9 上=K5" in s for s in rig.logs),
      str([s for s in rig.logs if "穿洞" in s][:2]))
check("11.10 门禁行仍按引擎口径（tour 不跑门禁）",
      str(st["plan"]["engine"]) == MT.ENGINE_TOUR and st["plan"]["gate_ok"] is True,
      f"engine={st['plan']['engine']} gate={st['plan']['gate_ok']}")

# ---- 11.B2 ★卡 t13（D3）：单槽被 upper 覆盖后，lower 仍走事件流生效（自动规划路径）----
rig_pair = Rig(intel_cfg())
push_drone(rig_pair, half="lower", block="K9")     # 无人机先报下半场
st_pair_last = push_drone(rig_pair, half="upper", block="K4")   # 再报上半场 → 槽被覆盖
rig_pair.ticks(3)
check("11.10b ★★ D3：槽里只剩 upper（框架层 bridge_of('lower') 恒 None），"
      "自动规划仍同时用 K9 与 K4",
      str(getattr(st_pair_last, "bridge_half", "")) == "upper"
      and st_pair_last.bridge_of("lower", now=rig_pair.clock.now(), stale_s=30.0) is None
      and kcells(rig_pair) == ["K9", "K4"],
      f"槽 half={getattr(st_pair_last, 'bridge_half', None)!r} k={kcells(rig_pair)} "
      f"seen={list(rig_pair.state()['bridge']['seen'])}")
check("11.10c 遥测 `bridge` 暴露事件通道证据（订阅在听 + 已消费条数 + 溢出计数）",
      rig_pair.state()["bridge"]["subscribed"] is True
      and rig_pair.state()["bridge"]["events"] >= 2
      and rig_pair.state()["bridge"]["events_dropped"] == 0,
      str({k: rig_pair.state()["bridge"][k]
           for k in ("subscribed", "events", "events_dropped")}))

# ---- 11.C 无情报 → 与接线前逐 token 一致 ----
rig_off = Rig(intel_cfg())
rig_off.ticks(3)
check("11.11 ★ 无 P_DRONE（无无人机）→ 执行路线与『不传 cross 参数』的规划产物逐 token 一致",
      [b for b in rig_off.task.route.blocks()]
      == [parse_token(t)["block"] for t in _out_plain.tokens],
      f"{len(rig_off.task.route)} 点 vs 基线 {len(_out_plain.tokens)} 点")
check("11.12 无情报时遥测 source=no_link、used 走默认格",
      rig_off.state()["bridge"]["lower"]["source"] == "no_link"
      and rig_off.state()["plan"]["cross_requested"] == {"lower": "", "upper": ""},
      str(rig_off.state()["bridge"]["lower"]))

# ---- 11.D 池外情报：t31 新语义 = 行段内采纳；真非法的仍稳（fallback=false 也不停在 IDLE） ----
cfg_strict = intel_cfg(fallback_to_tokens=False)
cfg_strict.override("route.cross_lower_candidates", ["K10"])     # 候选只有 K10（可过期）
rig_bad = Rig(cfg_strict)
push_drone(rig_bad, half="lower", block="K9")     # t31：K9 在下半场行段内 ⇒ 采纳
rig_bad.ticks(3)
st_bad = rig_bad.state()
check("11.13 ★★ t31 权威门放宽：池外的 K9（下半场行段内）+ fallback=false → "
      "**采纳并照常 DRIVE**（换洞生效，不是原地回默认）",
      rig_bad.task.phase == MissionPhase.DRIVE and rig_bad.count(EV_MISSION_STARTED) == 1
      and kcells(rig_bad) == ["K9", "K5"],
      f"phase={rig_bad.task.phase} started={rig_bad.count(EV_MISSION_STARTED)} k={kcells(rig_bad)}")
check("11.14 t31：池外采纳的证据齐全：source=drone + 原因写明『池外…纠正过期地图』+ 实际走 K9",
      st_bad["bridge"]["lower"]["source"] == "drone"
      and "池外" in st_bad["bridge"]["lower"]["reason"]
      and "纠正过期地图" in st_bad["bridge"]["lower"]["reason"]
      and st_bad["plan"]["cross_lower"] == "K9" and kcells(rig_bad) == ["K9", "K5"],
      f"{st_bad['bridge']['lower']['reason']} / k={kcells(rig_bad)}")
check("11.15 采纳只记日志、不上抛（任务无异常，日志写明口径）",
      rig_bad.task.health().errors == 0 and any("池外" in s for s in rig_bad.logs),
      f"errors={rig_bad.task.health().errors}")
rig_rej = Rig(cfg_strict)                          # 同一个严格配置（fallback=false）
push_drone(rig_rej, half="lower", block="A1")      # 真非法：不在 K 列
rig_rej.ticks(3)
check("11.16 ★ t31：真非法情报（A1 非 K 列）+ fallback=false → 拒收 + 仍 DRIVE"
      "（不炸规划、不卡发车），穿洞回落默认 K10",
      rig_rej.task.phase == MissionPhase.DRIVE and rig_rej.count(EV_MISSION_STARTED) == 1
      and rig_rej.state()["bridge"]["lower"]["source"] == "rejected"
      and kcells(rig_rej) == ["K10", "K5"], f"k={kcells(rig_rej)}")

# ---- 11.E 过期/离线情报：自动规划也不换洞 ----
rig_exp = Rig(intel_cfg())
push_drone(rig_exp, half="lower", block="K9", age=99.0)
rig_exp.ticks(3)
check("11.16 过期情报（99s）→ 自动规划用默认洞 K10（且与基线 token 一致）",
      kcells(rig_exp) == ["K10", "K5"]
      and [b for b in rig_exp.task.route.blocks()]
      == [parse_token(t)["block"] for t in _out_plain.tokens],
      f"k={kcells(rig_exp)} source={rig_exp.state()['bridge']['lower']['source']}")
rig_offl = Rig(intel_cfg())
push_drone(rig_offl, half="lower", block="K9", online=False)
rig_offl.ticks(3)
check("11.17 无人机离线 → 自动规划用默认洞 K10",
      kcells(rig_offl) == ["K10", "K5"]
      and rig_offl.state()["bridge"]["lower"]["source"] == "offline",
      f"k={kcells(rig_offl)}")

# =========================================================================== #
section("12. ★t7 自动规划必须保留所有 `*` 停站（双射击停站收口）")
# =========================================================================== #
#  靶板一面蓝一面红、蓝面朝向**每局随机且恒定**，只打蓝、打红扣分 ⇒ 自动规划给出
#  CAR-DUAL-20261024 起 tour 自己落双星；这里同时验证原生产物、幂等收口、
#  真实单星输入的兼容补标，以及 Mission 遥测与 token 同源。

def _stars(toks) -> list[str]:
    """路线 token 里的射击停站（`*`）串。"""
    return [t for t in toks if parse_token(t)["shoot"]]


check("12.1 ★ tracker-safe（参考引擎）：第七图 t24 的 C9* 与 C5* **同时存在**",
      _stars(_out_ts.tokens) == ["C9*", "C5*"], str(_stars(_out_ts.tokens)))
check("12.2 PlanOutcome 的射击停站 = ('C9','C5')，且是引擎自带（second_stop_added 为空）",
      _out_ts.shoot_stops == ("C9", "C5") and _out_ts.second_stop_added == "",
      f"{_out_ts.shoot_stops} / added={_out_ts.second_stop_added!r}")
check("12.3 ★ tour（缺省引擎）自动规划**也**必须有 C5*（对侧第二停站不能被静默丢掉）",
      _stars(_out_tour.tokens) == ["C9*", "C5*"],
      f"{_stars(_out_tour.tokens)} / {len(_out_tour.tokens)} token、{_out_tour.route_len_m:.2f} m")
_raw_tour = PL.plan_route(_raw7, real_cfg(auto_plan_enabled=True),
                          allow_reverse=False, max_seconds=10.0)
check("12.4 tour 原生双站经收口后完全幂等（token 与 metadata 同源）",
      _stars(_raw_tour.tokens) == ["C9*", "C5*"]
      and _out_tour.second_stop_added == ""
      and _out_tour.tokens == _raw_tour.tokens
      and _raw_tour.shoot_stop2 == "C5",
      f"裸引擎 {_stars(_raw_tour.tokens)} → 收口后 {_stars(_out_tour.tokens)}｜"
      f"added={_out_tour.second_stop_added!r}｜两边都是 {len(_raw_tour.tokens)} token")
_single_tokens = [t.replace("*", "", 1) if parse_token(t)["block"] == "C5" else t
                  for t in _raw_tour.tokens]
_closed_tokens, _added, _note = PL.ensure_second_shoot_stop(
    _single_tokens, _raw7, real_cfg())
check("12.4b 真单星输入只补合法对侧星、保留全部路线几何",
      _stars(_single_tokens) == ["C9*"] and _closed_tokens == _raw_tour.tokens
      and _added == "C5", _note)
_ensure_real = PL.ensure_second_shoot_stop
PL.ensure_second_shoot_stop = lambda toks, *_a, **_k: (list(toks), "", "（测试：收口被关掉）")
try:
    _out_tour_off = plan_route_from_config(
        real_cfg(auto_plan_enabled=True, map_path=str(MAP7), engine=ENGINE_TOUR),
        log=lambda s: None)
finally:
    PL.ensure_second_shoot_stop = _ensure_real
check("12.5 关闭补标仍保留 tour 原生双站（不依赖 wrapper 补星）",
      _out_tour_off.tokens == _raw_tour.tokens and _out_tour_off.second_stop_added == "",
      str(_stars(_out_tour_off.tokens)))
check("12.6 收口已还原（后续用例/其它套件不受污染）",
      PL.ensure_second_shoot_stop is _ensure_real, str(PL.ensure_second_shoot_stop))

# ---- 12.B 任务级：tour 自动规划上车后真的装了两个射击停站 ----
rig_t7 = Rig(real_cfg(auto_plan_enabled=True, map_path=str(MAP7),
                      engine=ENGINE_TOUR, fallback_to_tokens=True))
rig_t7.ticks(3)
st_t7 = rig_t7.state()
check("12.7 ★ 上车执行（tour 自动规划）：两个射击停站都真的装在执行路线里",
      rig_t7.task.phase == MissionPhase.DRIVE
      and [w.block for w in rig_t7.task.route.waypoints if w.shoot] == ["C9", "C5"],
      str([w.block for w in rig_t7.task.route.waypoints if w.shoot]))
check("12.8 遥测 plan.shoot_stops / second_stop_added 与产物同源",
      st_t7["plan"]["shoot_stops"] == ["C9", "C5"]
      and st_t7["plan"]["second_stop_added"] == "",
      f"{st_t7['plan']['shoot_stops']} / added={st_t7['plan']['second_stop_added']!r}")

# ---- 12.C 停站台账：SHOOT 与终点必须分开（终点不是"第二个打靶停站"）----
check("12.9 ★ 停站台账区分 SHOOT 与终点：射击 2 / 终点 1，逐点 kind",
      st_ts["stops"]["counts"] == {"shoot": 2, "scan": 0, "end": 1}
      and [p["block"] for p in st_ts["stops"]["points"]] == ["C9", "C5", "N5"]
      and [p["kind"] for p in st_ts["stops"]["points"]] == ["SHOOT", "SHOOT", "END"],
      str(st_ts["stops"]))
_ledger_line = next((ln for ln in rig_ts.logs if ln.startswith("停站点 ")), "")
check("12.10 横幅台账一行看清类型：'停站点 3 个（射击 2 / 终点 1）' + 逐点标注",
      _ledger_line.startswith("停站点 3 个（射击 2 / 终点 1）")
      and "22:C9(射击)" in _ledger_line and "38:C5(射击)" in _ledger_line
      and "63:N5(终点)" in _ledger_line, _ledger_line)
# 反向控制：把当前路线的一个真射击点退回普通航点 ⇒ 台账必须如实变"射击 1 / 终点 1"
#   为什么需要：防止台账把"射击 2"写死（那样 12.9 会变成永远绿的假证据）。
_one = list(cfg_off.get("route.tokens") or [])
_first_shoot = next(i for i, t in enumerate(_one) if parse_token(t)["shoot"])
_one[_first_shoot] = _one[_first_shoot].replace("*", "", 1)
_cfg_one = real_cfg(auto_plan_enabled=False)
_cfg_one.override("route.tokens", _one)
rig_one = Rig(_cfg_one)
rig_one.ticks(3)
check("12.11 反向控制：单停站路线 ⇒ 台账如实变'停站点 2 个（射击 1 / 终点 1）'",
      rig_one.state()["stops"]["counts"] == {"shoot": 1, "scan": 0, "end": 1}
      and "停站点 2 个（射击 1 / 终点 1）" in rig_one.text(),
      str(rig_one.state()["stops"]["counts"]) + " / "
      + next((ln for ln in rig_one.logs if ln.startswith("停站点 ")), ""))

section("13. 第六图无合法对侧：明确拒绝与既有回退行为")
# 固定 2026-10-17 第六图内容，避免生产 map.yaml 后续换图悄悄消掉负例。
# F4 是雷；合法全靶候选 B4/B7 同侧，不能拿 F4*/B4* 冒充合法双面产物。
_map6 = _FIX / "map6_no_opposite.yaml"
_map6.write_text('''map:
  name: field6-no-opposite-regression
  start: N10
  end: N5
  barrier_col_from: D
  barrier_row_hi: 6
  barrier_row_lo: 7
  mines: [A2, F4, H4, G5, E8, B10, D11, I11]
  obstacles: [B1, D1, F3, I6, H7, A8, C9, E9]
  targets: [D5, D6]
  bridges: [K5, K7]
''', encoding="utf-8")
_raw6 = _yaml.safe_load(_map6.read_text(encoding="utf-8"))
_spec6 = PL.load_map(_raw6)
_pool6 = PL.shoot_candidates(_spec6, PL.Grid(_spec6), PL._shoot_accept(real_cfg()))
_cover6 = PL.covering_stops(_spec6, _pool6, PL._shoot_accept(real_cfg()))
_full6 = [b for b in _pool6 if len(_cover6[b]) == len(_spec6.targets)]
check("13.1 第六图负例真实：F4为雷，完整覆盖候选B4/B7均同侧",
      "F4" in _spec6.mines and "F4" not in _pool6 and set(_full6) == {"B4", "B7"}
      and len({PL.plate_side_of(_spec6, b) for b in _full6}) == 1, str(_full6))
# ★RECT-PLAN-FIX-20261026：原断言（before）
#     not _no_pair.ok and not _no_pair.tokens and "双射击停站" in _no_pair.error
#   期望：单站 -> 整趟失败（ok=False / 0 token）。
#   现断言（after）：**意图不变**（单站不得静默成功），机制换成"显式降级"：
#     ok=True + 恰好 1 个停站 B4 + 运营日志显式告警 + 引擎侧 degraded 三元组齐全；
#     同时钉死"2 个停站仍走全量校验"（同侧/重复/非法候选一条都不放松）。
_logs6: list[str] = []
_no_pair = plan_route_from_config(real_cfg(map_path=str(_map6), engine=ENGINE_TOUR,
                                          max_seconds=10.0), log=_logs6.append)
check("13.2 ★ 单站不再判死：规划成功且只有 1 个停站 B4",
      _no_pair.ok and list(_no_pair.shoot_stops) == ["B4"],
      f"ok={_no_pair.ok} stops={_no_pair.shoot_stops} err={_no_pair.error[:80]}")
check("13.2a 单站不许静默：运营日志必须有『只有 1 个射击停站』告警",
      any("只有 1 个射击停站" in ln for ln in _logs6),
      " / ".join(ln for ln in _logs6 if "射击停站" in ln))
_res6 = PL.plan_route(_raw6, real_cfg(), allow_reverse=False, objective="legacy",
                      max_seconds=10.0)
check("13.2b 引擎侧 degraded=True + reason 命中三分支之一 + 进 warnings",
      _res6.degraded and bool(_res6.degraded_reason)
      and any(w.startswith("单站降级") for w in _res6.warnings)
      and any(k in _res6.degraded_reason
              for k in ("没有可入池的合法候选", "本路线未兑现", "落在板心平面")),
      _res6.degraded_reason)
_acc6 = PL._shoot_accept(real_cfg())
_2same_side = PL.check_shoot_stops(["B4", "B7"], _spec6, _acc6)
_2dup = PL.check_shoot_stops(["B4", "B4"], _spec6, _acc6)
_2illegal = PL.check_shoot_stops(["F4", "B4"], _spec6, _acc6)
check("13.2c ★ 2 个停站仍走全量校验：同侧 / 重复 / 非法候选一条都不放松",
      any("分居板面两侧" in s for s in _2same_side)
      and any("重复" in s for s in _2dup)
      and any("不在合法候选中" in s for s in _2illegal),
      f"{_2same_side} | {_2dup} | {_2illegal}")
# ★RECT-PLAN-FIX-20261026：13.3/13.4 的**意图**是「规划失败时回退 / 停 IDLE 的机制不变」。
#   旧机制 = 单站规划失败（A 方案后单站不再失败）⇒ 机制换成**注入 PlannerError**；
#   **断言体一个字没改**（回退手写路线 / 停 IDLE / 不 publish EV_MISSION_STARTED）。
def _boom_plan(*_a, **_k):
    raise PL.PlannerError("（13.3/13.4 注入）规划器失败：本用例只验回退/IDLE 机制")


_orig_plan_route = PL.plan_route
try:
    PL.plan_route = _boom_plan
    _fallback6 = Rig(real_cfg(auto_plan_enabled=True, map_path=str(_map6), engine=ENGINE_TOUR,
                             max_seconds=10.0, fallback_to_tokens=True))
    _fallback6.ticks(3)
finally:
    PL.plan_route = _orig_plan_route
check("13.3 允许回退时保留既有手写路线且遥测记录规划失败",
      _fallback6.task.phase == MissionPhase.DRIVE
      and _fallback6.state()["route_source"] == ROUTE_SOURCE_MANUAL
      and not _fallback6.state()["plan"]["ok"]
      and list(_fallback6.task.route.blocks()) == blocks_of(cfg_off.get("route.tokens")),
      _fallback6.state()["plan"]["error"])
# 13.3b：新口径本身 —— 单站不再失败 ⇒ 不回退手写路线，走自动规划产物（带 B4 单站）
_fallback6b = Rig(real_cfg(auto_plan_enabled=True, map_path=str(_map6), engine=ENGINE_TOUR,
                           max_seconds=10.0, fallback_to_tokens=True))
_fallback6b.ticks(3)
_st6b = _fallback6b.state()
check("13.3b ★ 单站不再触发回退：route_source=自动规划、停站只有 B4",
      _fallback6b.task.phase == MissionPhase.DRIVE
      and _st6b["route_source"] == ROUTE_SOURCE_AUTO
      and list(_st6b["plan"]["shoot_stops"]) == ["B4"],
      str((_st6b["route_source"], _st6b["plan"].get("shoot_stops"))))
try:
    PL.plan_route = _boom_plan
    _stop6 = Rig(real_cfg(auto_plan_enabled=True, map_path=str(_map6), engine=ENGINE_TOUR,
                         max_seconds=10.0, fallback_to_tokens=False))
    _stop6.ticks(3)
finally:
    PL.plan_route = _orig_plan_route
check("13.4 禁止回退时保持IDLE、没有EV_MISSION_STARTED",
      _stop6.task.phase == MissionPhase.IDLE and _stop6.count(EV_MISSION_STARTED) == 0,
      _stop6.task.phase)
check("13.4b 注入已还原（后续用例不受污染）",
      PL.plan_route is _orig_plan_route, str(PL.plan_route))

finish()
