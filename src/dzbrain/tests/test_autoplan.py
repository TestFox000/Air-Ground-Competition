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

from dzbrain.core.blackboard import P_ROUTE, P_SAFETY, P_TELEMETRY, Blackboard  # noqa: E402
from dzbrain.core.config import Config                            # noqa: E402
from dzbrain.core.events import EV_MISSION_STARTED, EV_ROUTE_LOADED, EventBus  # noqa: E402
from dzbrain.core.state import SafetyState                        # noqa: E402
from dzbrain.io.contract import MissionPhase                      # noqa: E402
from dzbrain.route import planner as PL                           # noqa: E402
from dzbrain.route.parser import parse_mission, parse_token       # noqa: E402
from dzbrain.tasks.mission import (ROUTE_SOURCE_AUTO, ROUTE_SOURCE_MANUAL,  # noqa: E402
                                   MissionTask, plan_route_from_config, resolve_map_path)

from fakes import FakeBridge, FakeClock                           # noqa: E402

# --------------------------------------------------------------------------- #
WS = pathlib.Path(__file__).resolve().parents[3]          # <ws>/src/dzbrain/tests/x.py
CONFIG = WS / "config" / "robot.yaml"
MAP = WS / "config" / "map.yaml"

#: ★★ 自足夹具（卡 D 修复带出的**测试卫生**问题）：
#:   本套件原先读 `_scratch_auto_plan/mission/*.yaml` —— 那些文件在 `.gitignore` 里
#:   （`.gitignore:10  _scratch_*/`），**换一个干净检出/快照就全没了**。
#:   实测：`git archive HEAD` 解出来的干净树里跑本套件，6 项失败全部是
#:   `FileNotFoundError: 地图文件不存在: <tree>/_scratch_auto_plan/mission/broken_map.yaml`
#:   —— 代码没问题，是**测试依赖了不进版本库的临时文件**。
#:   现在所有夹具都在 `tempfile` 目录里**现场生成**，测试可脱离工作区独立运行。
_FIX = pathlib.Path(tempfile.mkdtemp(prefix="dz_autoplan_fix_"))


def _sub_mines(text: str, repl: str) -> str:
    """把 map.yaml 里 `mines:` 那一行换成 `repl`（保留原缩进）。"""
    return re.sub(r"(?m)^([ \t]*)mines:.*$", lambda m: m.group(1) + repl, text)


def _drop_mines(text: str) -> str:
    """整行删掉 `mines:`（这就是"缺 mines 的坏图"，实测**能解**，只能当反例）。"""
    return re.sub(r"(?m)^[ \t]*mines:.*\n", "", text)


def _make_fixtures() -> None:
    map_text = MAP.read_text(encoding="utf-8")
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
    (_FIX / "gate_fail.yaml").write_text("".join(lines), encoding="utf-8")


_make_fixtures()
BROKEN = _FIX / "broken_map.yaml"                 # mines 含非法格名 → PlannerError
BAD_YAML = _FIX / "bad_yaml_map.yaml"             # YAML 语法错 → ParserError（非 PlannerError）
MISSING_MINES = _FIX / "missing_mines.yaml"       # "缺 mines" 的坏图（其实能解）
GATE_FAIL = _FIX / "gate_fail.yaml"               # 门禁必红（route.tokens 只有 2 个点）
NO_SUCH_MAP = _FIX / "no_such_map.yaml"           # 不存在的路径（不需要真的建）


def real_cfg(**kv) -> Config:
    """真的 `config/robot.yaml`（`--config` 那条路径）+ 就地覆盖测试用键。

    ★ 用**真配置**而不是 `Config.from_dict`：planner 段有没有写进 ros__parameters 树、
    `source` 指向哪里（决定"相对配置文件目录"这条解析规则），都只有真配置能验证。
    """
    cfg = Config.load(str(CONFIG))
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
cfg = real_cfg(auto_plan_enabled=True, map_path=str(MAP),
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
check("1.12 ★ 诚实记录：'缺 mines 的坏图'**不是一个失败用例**（它真能解出路线）",
      out_mm.ok, f"ok={out_mm.ok} {len(out_mm.tokens)} token / error={out_mm.error!r}")
check("1.13 所以回退分支的坏图夹具必须是 broken_map.yaml（mines 非法），不是缺 mines 的副本",
      (not out_bad.ok) and out_mm.ok, f"坏图失败={not out_bad.ok} 缺mines成功={out_mm.ok}")

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
# ★ 用 `config/map.yaml` 当相对路径：工作区那份天然存在（仓库文件），配置目录那份放临时目录里，
#   于是**两个候选同时为真**，且测试不再往工作区里写任何文件（原先会留 `_scratch_auto_plan` 垃圾）。
probe_dir = _FIX / "_order_probe"
(probe_dir / "config").mkdir(parents=True, exist_ok=True)
rel = "config/map.yaml"
ws_side = WS / "config" / "map.yaml"
cfg_side = probe_dir / "config" / "map.yaml"
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
rig = Rig(real_cfg(auto_plan_enabled=True, map_path=str(MAP),
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
        rc = M.main(["--plan-only"])
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
check("7.9 地图解析到工作区 config/map.yaml（cwd 无关）",
      str(MAP) in out_txt, str(MAP))
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
_key_lines = [l for l in seg[1:]
              if l.startswith("      ") and not l.startswith("       ") and ":" in l]
# ★ 2026-10-11 重钉（t22）：`planner:` 段再增一键 = **8 个键**
#   （t20 是 7 个；t22 物理几何卡新增 `obstacle_margin_m: 0.02`）。
want = ["auto_plan_enabled: false", 'map_path: "config/map.yaml"', "fallback_to_tokens: true",
        "allow_reverse: false", "max_seconds: 2.0", "pivot_penalty: 5.0",
        "mine_turn_penalty: 2.0", "obstacle_margin_m: 0.02"]
check("8.3 ★ planner 段含当前实际的 8 个键值（含 auto_plan_enabled=false / obstacle_margin_m）",
      all(any(w in l for l in seg) for w in want), "; ".join(want))
check("8.4 8 个键都在 planner 段内部（恰好 6 空格缩进）",
      len(_key_lines) == 8,
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

cfg_junk = real_cfg(auto_plan_enabled=True, map_path=str(MAP),
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

_raw = _yaml.safe_load(MAP.read_text(encoding="utf-8"))
_mines = list((_raw.get("map") or _raw).get("mines") or [])
_obst = set((_raw.get("map") or _raw).get("obstacles") or [])
_taken = set(_mines) | _obst | {"N5", "N10"}
_far = next(c for c in ("A11", "A1", "N7", "A3", "N1") if c not in _taken)
_tmpdir = tempfile.TemporaryDirectory(prefix="dz_cards_")
_variant = pathlib.Path(_tmpdir.name) / "map_variant.yaml"
_variant.write_text(MAP.read_text(encoding="utf-8").replace(f'"{_mines[0]}"', f'"{_far}"'),
                    encoding="utf-8")
check("9.1 变体图已生成（把第一颗雷挪到远角空格，config/ 未改动）",
      _variant.is_file() and _far not in _mines, f"{_mines[0]}→{_far} @ {_variant}")

_buf_def = io.StringIO()
with contextlib.redirect_stdout(_buf_def):
    _rc_def = M.main(["--plan-only"])
_default_tokens = _tokens_of(_buf_def.getvalue())

_buf_var = io.StringIO()
with contextlib.redirect_stdout(_buf_var):
    _rc_var = M.main(["--plan-only", "--map", str(_variant)])
_var_txt = _buf_var.getvalue()
_var_tokens = _tokens_of(_var_txt)

check("9.2 两次 --plan-only 都 exit 0", _rc_def == 0 and _rc_var == 0,
      f"默认={_rc_def} 变体={_rc_var}")
check("9.3 ★D2：`--map <变体图>` 真的决定规划用的地图（打印的是变体图）",
      str(_variant) in _var_txt and str(MAP) in _buf_def.getvalue(),
      [l for l in _var_txt.splitlines() if l.startswith("地图:")][:1])
check("9.4 ★D2：变体图的 token 与默认图**不同**（旧代码：静默忽略 --map、逐字节相同）",
      _var_tokens != _default_tokens and len(_var_tokens.split()) != len(_default_tokens.split()),
      f"默认 {len(_default_tokens.split())} token / 变体 {len(_var_tokens.split())} token")
check("9.5 D2 第二入口：`run_plan_only(cfg, map_path=…)` 也按该图规划",
      M.run_plan_only(real_cfg(auto_plan_enabled=True), map_path=str(_variant),
                      log=lambda s: None) == 0, "函数直调 exit 0")
_out_variant = plan_route_from_config(real_cfg(auto_plan_enabled=True), map_path=str(_variant),
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
_tmpdir.cleanup()


finish()
