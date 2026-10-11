# -*- coding: utf-8 -*-
"""MissionTask（10 Hz）—— 任务调度：出发 → 航点推进 → 停站三段 → 终点横幅。

对应 `docs/P1接口约定.md` §3.3；停站三段式与超时兜底复刻旧实现
`path_executor` 的打靶四阶段状态机（survey §4.3，旧 2034–2193），改动两处：

1. **三段而不是四段**：旧版 `SETTLE → ACQUIRE → STABLE → FIRE` 里的 `STABLE`
   （等瞄稳 40 px 连续 5 帧）在新版归 `AimTask`（云台自己判定"锁定"），
   Mission 只在 `ACQUIRE` 里等 `aim_status == TRACK` 且视觉有目标 → 直接进 `FIRE`。
2. **每段独立超时**：绝不允许任何一段把主循环卡住（旧版 `SHOOT_WAIT_TIMEOUT` 是死变量）。

铁律：**宁可漏打，不可卡死**。所以每一段都有"超时即放行"分支，且放行前一定
`publish_fire_gate(False)` 收尾（旧版 2181–2183 的同一条纪律）。

其他约定与出处：

- 出发：`mission.auto_start` + `mission.auto_start_delay_s` → `parse_reset('N10,y')` 复位 belief
  → `parse_mission(route.tokens)` → 写 `P_ROUTE` → `EV_ROUTE_LOADED` → 阶段 `DRIVE`（§3.3）。
- 开机跨帧一致性：`mission.scan_mode == 'continuous'` 时开局发 `publish_scan_request(2)`
  （**持续**扫描）。readme【十】第 31 条：绝不能在到点时发 `1` 把它降级成单趟。
- 终点：最后一个航点到达 → `FINISH`（**锁存**）、`publish_brake()`、`publish_scan_request(0)`、
  `EV_MISSION_FINISHED`、结束横幅（用时/发数/漂移）。
- ★ PIVOT 原地掉头顶点：Tracker 完成/放弃 PIVOT 时经 `PIVOT_DONE_KEY` 通知本任务，
  该顶点直接判到达（`pivot_dist_m` > `arrival_m`，正常判据恒假，bench 实测 E11/A6/B6
  被拐角绕圈兜底强制跳过）。停站点恰是掉头顶点时照常进入三段式停站。
- 外部命令（`/nav/waypoint`）：`reset:BLOCK,HEAD` / `load:文件` / `单点 BLOCK,HEAD` / `多点 ...`
  全部支持；**未知命令只告警不崩**（旧版 `waypoint_callback` 的教训）。

★ 时间来源一律走构造注入的 `clock`（默认真实 `time.monotonic`），三段超时全部基于它，
  测试用 `FakeClock` 快进即可 —— 任务内**不得**直接调 `time.time()`。

★ 自动规划（任务卡 B，2026-10-08）：`planner.auto_plan_enabled: true` 时**启动即自己算路** ——
  读 `planner.map_path` 的地图 → `route.planner.plan_route(..., allow_reverse=...)` →
  成功就用 `PlanResult.tokens` 当执行路线（日志打**完整 token 列表 + 规划耗时(ms)**，
  横幅标「路线来源：自动规划」）。失败语义按 `planner.fallback_to_tokens`：
  `true` → warn（含异常类型+消息）后回退 `route.tokens`，**仍进入 DRIVE**；
  `false` → error + **停在 IDLE 不出发**（绝不 publish `EV_MISSION_STARTED`）。
  规划入口 `plan_route_from_config()` 与 `python -m dzbrain --plan-only` **共用同一段代码**
  —— 否则"静态规划通过"不等于"车上规划通过"，那种绿灯毫无意义。
  ⚠️ `planner.*` 目前**未登记进 `core/config.py` 的 SCHEMA**（不在本卡允许改动范围内）：
  `Config.validate()` 会给这 5 个键各记一条 "未知配置段" warn（不是 error，门禁仍过），
  且键名拼错不会被 SCHEMA 兜住 —— 建议收口时补进 SCHEMA 并把 `planner.` 加进
  `known_prefixes`。

⚠️ SCHEMA 缺键（本次未改 `core/config.py`，故用带默认值的 `cfg.get`）：
  `mission.shots_per_stop` / `mission.settle_time_s` / `mission.settle_eps_m` /
  `mission.settle_speed_mps` / `mission.lock_timeout_s` / `mission.shot_timeout_s` /
  `mission.scan_stop_timeout_s`。建议 P1 收口时补进 SCHEMA，否则 `Config.validate()`
  看不到它们（键拼错不会被"未知配置段"告警兜住）。
"""

from __future__ import annotations

import json
import math
import pathlib
from dataclasses import dataclass, field
from typing import Any, Callable

from ..core import geometry as geo
from ..core.blackboard import (P_MISSION_PHASE, P_POSE, P_ROUTE, P_ROUTE_INDEX, P_ROUTE_S,
                               P_SAFETY, P_STOP_POINT, P_TELEMETRY, Blackboard)
from ..core.config import Config
from ..core.events import (EV_MISSION_FINISHED, EV_MISSION_STARTED, EV_ARRIVED_STOP_POINT,
                           EV_ROUTE_LOADED, EV_STOP_POINT_DONE, EV_WAYPOINT_REACHED, EventBus)
from ..core.logging import Throttle, banner
from ..core.state import SafetyState, StopKind, StopPhase, StopPointState
from ..core.task import TaskThread
from ..io.contract import AimStatus, MissionPhase
from ..pursuit.path import Path
from ..route.model import Route, Waypoint
from ..route.parser import parse_mission, parse_reset, parse_token
from .tracker import PIVOT_DONE_KEY, TRACKER_CMD_KEY, PivotPhase

__all__ = ["MissionTask", "MISSION_STATE_KEY", "plan_route_from_config", "resolve_map_path",
           "PlanOutcome", "AutoPlanBlocked", "ROUTE_SOURCE_AUTO", "ROUTE_SOURCE_MANUAL"]

#: 遥测键：Mission 的当前上下文（停站点/航点/用时）
MISSION_STATE_KEY = "mission_state"

# --------------------------------------------------------------------------- #
#  ★ 自动规划（任务卡 B）：路线来源标注 + 规划入口
# --------------------------------------------------------------------------- #
#: 横幅/日志里的"路线来源"字样（验收项要求原文出现这两个词）
ROUTE_SOURCE_AUTO = "自动规划"
ROUTE_SOURCE_MANUAL = "手动配置"
#: 外部命令（/nav/waypoint）载入的路线
ROUTE_SOURCE_COMMAND = "外部命令"

#: 默认地图路径（相对配置文件目录 / 相对工作区，二者按序解析）
DEFAULT_MAP_PATH = "config/map.yaml"
_DEFAULT_PLAN_SECONDS = 2.0


class AutoPlanBlocked(RuntimeError):
    """`planner.fallback_to_tokens=false` 且规划失败 —— 停在 IDLE，**绝不**带病出车。"""


@dataclass
class PlanOutcome:
    """一次自动规划的结果（**不抛异常**，失败信息全在 `error` 里，便于日志与测试断言）。"""

    ok: bool = False
    tokens: list[str] = field(default_factory=list)
    elapsed_ms: float = 0.0
    error: str = ""                      # 失败时 "<异常类型>: <消息>"
    map_path: str = ""                   # 实际用的地图文件（已解析成绝对路径）
    allow_reverse: bool = False
    route_len_m: float = 0.0
    reverse_segments: int = 0
    cost: float = 0.0

    @property
    def source(self) -> str:
        return ROUTE_SOURCE_AUTO if self.ok else ""

    @property
    def mode(self) -> str:
        return "允许倒车" if self.allow_reverse else "无倒车"


def workspace_root() -> pathlib.Path:
    """工作区根目录（`<ws>/src/dzbrain/dzbrain/tasks/mission.py` → `parents[4]`）。

    ⚠️ 这里是 **parents[4]** 不是 [3]：`tasks/` 比 `main.py` 深一层
    （`main.py` 在 `<ws>/src/dzbrain/dzbrain/`，其 `parents[3]` 才是 `<ws>`）。
    取错一位会让"相对工作区"的地图候选变成 `<ws>/src/config/map.yaml` —— 实测
    `python -m dzbrain --plan-only` 报 `FileNotFoundError: 地图文件不存在:
    D:\\...\\dzrobot_v2\\src\\config\\map.yaml`，正好把这个 off-by-one 暴露出来。
    """
    return pathlib.Path(__file__).resolve().parents[4]


def config_dir(cfg: Any) -> pathlib.Path | None:
    """配置文件所在目录（`Config.from_dict` 造的配置没有真实文件 → None）。"""
    src = str(getattr(cfg, "source", "") or "")
    if not src or src in ("<memory>", "<dict>"):
        return None
    return pathlib.Path(src).parent


def resolve_map_path(cfg: Any = None, spec: str | None = None) -> pathlib.Path:
    """地图路径解析顺序（任务卡 B 规格）：**绝对路径 → 相对配置文件目录 → 相对工作区**。

    三个候选按序 `is_file()`；都不存在时返回**工作区候选**（调用方据此报"地图文件不存在"，
    错误信息里能看到实际尝试的路径）。本函数不抛异常、也不读文件内容。
    """
    text = spec if spec is not None else (cfg.get("planner.map_path") if cfg is not None else None)
    text = str(text or DEFAULT_MAP_PATH)
    raw = pathlib.Path(text).expanduser()
    if raw.is_absolute():
        return raw
    cands: list[pathlib.Path] = []
    cdir = config_dir(cfg) if cfg is not None else None
    if cdir is not None:
        cands.append(cdir / raw)
    cands.append(workspace_root() / raw)
    for cand in cands:
        if cand.is_file():
            return cand
    return cands[-1]


def _read_map_yaml(path: pathlib.Path) -> dict:
    """读地图 YAML（失败即抛，由 `plan_route_from_config` 统一转成 `PlanOutcome.error`）。"""
    import yaml                                   # 车上/本地都有 PyYAML（core/config.py 同款）

    if not path.is_file():
        raise FileNotFoundError(f"地图文件不存在: {path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"地图文件不是 YAML 映射（顶层应为 dict）: {path}")
    return raw


def plan_route_from_config(cfg: Any, *, map_path: str | None = None,
                           log: Callable[[str], None] | None = None) -> PlanOutcome:
    """按 `planner.*` 配置规划一条路线 —— **永不抛异常**（失败 → `ok=False` + `error`）。

    调用契约（与任务卡 A 冻结的规划器 API 一致，本文件**只消费不修改** `route/`）：
        `plan_route(map_cfg, cfg, allow_reverse=bool, max_seconds=float, source=str) -> PlanResult`
        `PlanResult.tokens: list[str]` / `.elapsed_ms: float` —— 失败抛 `PlannerError`。

    ★ 宽捕获 `Exception`：任何规划器内部错误都必须变成"可回退的失败"，
      绝不允许把 MissionTask 一起炸掉（宁可漏规划，不可卡死）。
    """
    want_reverse = bool(cfg.get("planner.allow_reverse", False))
    try:
        budget = float(cfg.get("planner.max_seconds", _DEFAULT_PLAN_SECONDS)
                       or _DEFAULT_PLAN_SECONDS)
    except (TypeError, ValueError):
        budget = _DEFAULT_PLAN_SECONDS
    try:
        pivot_penalty = float(cfg.get("planner.pivot_penalty", 0.0) or 0.0)
    except (TypeError, ValueError):
        pivot_penalty = 0.0
    path = resolve_map_path(cfg, map_path if map_path is not None
                            else cfg.get("planner.map_path"))
    map_text = str(path)
    try:
        raw = _read_map_yaml(path)
        from ..route import planner as PL          # ★ 惰性导入：无规划器时也能 import 本模块
        res = PL.plan_route(raw, cfg, allow_reverse=want_reverse,
                            pivot_penalty=pivot_penalty,
                            max_seconds=budget, source=map_text)
        tokens = [str(t) for t in (res.tokens or [])]
        if not tokens:
            raise ValueError("规划器返回了空 token 列表")
        return PlanOutcome(
            ok=True, tokens=tokens, elapsed_ms=float(getattr(res, "elapsed_ms", 0.0) or 0.0),
            error="", map_path=map_text, allow_reverse=want_reverse,
            route_len_m=float(getattr(res, "polyline_m", 0.0) or 0.0),
            reverse_segments=int(getattr(res, "reverse_segments", 0) or 0),
            cost=float(getattr(res, "cost", 0.0) or 0.0),
        )
    except Exception as exc:                       # noqa: BLE001  ★ 规划失败一律回退，不崩
        msg = f"{type(exc).__name__}: {exc}"
        if log is not None:
            log(f"规划失败（将按 fallback_to_tokens 处理）：map={map_text} "
                f"预算 {budget:g}s {msg}")
        return PlanOutcome(ok=False, tokens=[], elapsed_ms=0.0, error=msg,
                           map_path=map_text, allow_reverse=want_reverse)


#: 只有这些阶段允许"发车/推进"（★ 白名单：未知阶段一律不动）
_DRIVING_PHASES: tuple[str, ...] = (MissionPhase.DRIVE, MissionPhase.APPROACH)

#: 终局阶段：不再自动推进，也不再被外部命令复位
_TERMINAL_PHASES: tuple[str, ...] = (MissionPhase.FINISH, MissionPhase.SAFE_STOP)

_EPS = 1e-9


@dataclass
class _StopContext:
    """一个停站点的**可变**上下文（只在 Mission 线程内使用，不跨线程共享）。"""

    index: int = -1
    block: str = ""
    kind: str = StopKind.SHOOT
    phase: str = StopPhase.SETTLE
    started_at: float = 0.0
    phase_started_at: float = 0.0
    settle_armed_at: float | None = None
    last_x: float | None = None
    last_y: float | None = None
    shots_at_arrival: int = 0
    shots_needed: int = 3
    detail: str = ""

    @property
    def active(self) -> bool:
        return self.index >= 0 and self.phase != StopPhase.DONE


class MissionTask(TaskThread):
    """任务调度（10 Hz）。"""

    def __init__(self, bridge, bb: Blackboard, bus: EventBus, cfg: Config, *,
                 log: Callable[[str], None] = print, group=None,
                 clock: Callable[[], float] | None = None) -> None:
        super().__init__("MissionTask", 10.0, bus=bus, blackboard=bb, logger=log)
        self.bridge = bridge
        self.cfg = cfg
        self.group = group
        if clock is None:
            import time as _time
            clock = _time.monotonic
        self._clock = clock

        # ---- 配置（一次性读出；缺键全部走显式默认值，见模块 docstring 的 ⚠️） ----
        self._auto_start = bool(cfg.get("mission.auto_start"))
        self._auto_start_delay_s = float(cfg.get("mission.auto_start_delay_s") or 0.0)
        self._reset_spec = str(cfg.get("mission.reset") or "")
        self._scan_mode = str(cfg.get("mission.scan_mode") or "").strip().lower()
        self._shots_per_stop = int(cfg.get("mission.shots_per_stop", 3) or 3)
        self._settle_time_s = float(cfg.get("mission.settle_time_s", 0.8))
        self._settle_eps_m = float(cfg.get("mission.settle_eps_m", 0.02))
        self._settle_speed_mps = float(cfg.get("mission.settle_speed_mps", 0.02))
        self._lock_timeout_s = float(cfg.get("mission.lock_timeout_s", 18.0))
        self._shot_timeout_s = float(cfg.get("mission.shot_timeout_s", 30.0))
        self._scan_stop_timeout_s = float(cfg.get("mission.scan_stop_timeout_s", 8.0))
        self._settle_grace_s = max(0.4 * self._shot_timeout_s, 1.0)     # ★ 旧版 0.4×SHOOT_TIMEOUT
        self._arrival_m = float(cfg.get("pursuit.arrival_m"))
        self._stop_radius_m = float(cfg.get("pursuit.stop_radius_m"))
        self._approach_m = float(cfg.get("pursuit.approach_m"))
        self._rate_limit_s = float(cfg.get("logging.rate_limit_s") or 1.0)
        # ---- 拐角绕圈兜底（仿真实测噪声 0.01m/拍会让车原地转圈、末点误差 2.96m）----
        self._stall_timeout_s = float(cfg.get("mission.stall_timeout_s", 12.0))
        self._stall_yaw_deg = float(cfg.get("mission.stall_yaw_deg", 360.0))
        # ---- ★自动规划（任务卡 B）：全部走 `planner.*`，缺键用显式默认 ----
        self._auto_plan_enabled = bool(cfg.get("planner.auto_plan_enabled", False))
        self._planner_map_path = str(cfg.get("planner.map_path") or DEFAULT_MAP_PATH)
        self._planner_fallback = bool(cfg.get("planner.fallback_to_tokens", True))
        self._planner_allow_reverse = bool(cfg.get("planner.allow_reverse", False))
        try:
            self._planner_max_seconds = float(cfg.get("planner.max_seconds",
                                                      _DEFAULT_PLAN_SECONDS)
                                              or _DEFAULT_PLAN_SECONDS)
        except (TypeError, ValueError):
            self._planner_max_seconds = _DEFAULT_PLAN_SECONDS

        # ---- 任务态 ----
        self._created_at = self._clock()
        self._started_at: float | None = None
        self._phase = MissionPhase.IDLE
        self._route: Route | None = None
        self._path: Path | None = None
        self._route_name = ""
        #: ★路线来源（`ROUTE_SOURCE_AUTO` / `ROUTE_SOURCE_MANUAL` / `ROUTE_SOURCE_COMMAND`），
        #: 横幅与遥测的"路线来源"行用它；`planner.auto_plan_enabled=false` 时开局即手动配置。
        self._route_source = ROUTE_SOURCE_MANUAL if not self._auto_plan_enabled else ""
        #: 最近一次自动规划的证据（map/耗时/错误），写进 `MISSION_STATE_KEY` 便于遥测与测试断言
        self._plan_info: dict[str, Any] = {}
        #: ★硬失败定格：`fallback_to_tokens=false` 且规划失败 → 停在 IDLE 且**不再每拍重试**
        self._autoplan_blocked = False
        self._waypoint_count = 0
        self._passed: list[bool] = []
        self._arrived = 0
        self._next_wp = 1
        self._handled_stops: set[int] = set()
        # ⚠️ 名字必须避开 `self._stop`：`threading.Thread` 内部有同名方法，
        #    被实例属性遮蔽后 `stop()/join()` 会在 `self._stop()` 处炸
        #    （`TypeError: 'NoneType' object is not callable`）—— 已实测踩过。
        self._stopctx: _StopContext | None = None
        self._last_stop_state = StopPointState()
        self._fire_gate_state: bool = False
        self._route_s: float | None = None
        self._dist_to_next_stop: float | None = None
        self._finish_latched = False
        self._finish_done = False
        self._finish_info: dict[str, Any] = {}
        self._scans: list[int] = []
        self._cmd_stats: dict[str, int] = {"total": 0, "rejected": 0}
        self._log_throttle = Throttle(self._rate_limit_s)

        # 停站点（含末点）的弧长前缀与目标索引，路线载入时算一次
        self._stop_s: dict[int, float] = {}
        self._stop_blocks: list[str] = []
        self._last_s = 0.0
        # ---- 拐角绕圈兜底的状态 ----
        self._stall_since: float | None = None       # 弧长开始停滞的时刻（None = 没在停滞）
        self._stall_last_s: float = 0.0              # 停滞开始时的弧长
        self._stall_last_yaw: float | None = None    # 停滞期间累计航向（绕圈判定）
        #: ★到点判定的"是否贴在路径上"：`_current_s` 现算投影时记下的横向偏差（米）
        self._last_lat_m: float | None = None
        self._stall_yaw_sum: float = 0.0
        self._forced_skips: int = 0                  # 强制跳过航点的次数（结束横幅要汇总）
        # ---- ★PIVOT 完成通知（Tracker 经 `PIVOT_DONE_KEY` 写，见 tracker.py 注释）----
        #: 经 PIVOT 通知判到达的航点集合：停站点若由此判到达，仍要进三段式停站
        #: （`_next_stop_candidate` 据此放行），不能像普通"已过"那样直接跳过。
        self._passed_by_pivot: set[int] = set()
        #: 通知判到达的"车仍在顶点附近"窗口：真完成时车距顶点 ≈ pivot_dist_m（掉头蠕转
        #: 还会带出少量位移）；跨轮次/竞态残留的通知里车在起点，二者差好几段，被窗口挡住。
        self._pivot_accept_m = max(float(cfg.get("pursuit.pivot_dist_m")) * 3.0,
                                   self._arrival_m * 6.0)

        # ---- ★逐点到达台账（卡 D finding D1：绝不静默丢点）----
        #: 每个待推进航点的**历史最近距离**（米）。几何复核用：s 一步跨过航点时，
        #: 只看"当拍"的距离会误判 —— 车常在几拍前就从格心旁穿过（此时 s 还没到），
        #: 等到 s 跳过去那一拍，车已经走远了。历史最近距离能如实回答"它到底来没来过"。
        self._min_dist: list[float] = []
        #: 索引 → 判达依据（"s+近路径" / "距格心" / "PIVOT 通知" / "最近接近 X m"）
        self._arrived_by: dict[int, str] = {}
        #: 索引 → 显式跳过依据（停滞兜底 / 收尾清账）。**凡未到达必在此或 `_arrived_by` 里**
        self._skipped_at: dict[int, str] = {}
        #: 索引 → "越窗待定"依据（finding D1）。**只是候选，不置位**：投影跳变的同一拍车
        #: 可能马上又靠近这个点 —— 立即判死会把本来能登记到达的航点提前毙掉
        #: （bench 实测：雷 I8 在基线里是「到点 8/45」，立即跳过版把它变成跳点 ⇒ 雷 7/8）。
        self._pending_skip: dict[int, str] = {}
        #: s 已越过、但车从未接近 ⇒ 显式跳过（与停滞兜底分开计数，便于分别审计）
        self._cross_skips: int = 0
        #: "最近接近"复核半径 = 半格（0.175 m）。**刻意不取更宽**：格间距就是 0.35 m，
        #: 半径超过半格会把"从邻居格心穿过"也算成到过。正常切角（离线心 ~0.29 m）由
        #: `near`（横向偏差判据）负责登记，这里只认"真从格心附近穿过"。
        self._close_m = max(self._arrival_m, 0.5 * geo.BLOCK_SIZE)
        #: 判"越窗跳过"的**超窗余量**：s 越过航点不足这个距离时先挂起（可能正在靠近），
        #: 越过这么多还没接近过 ⇒ 显式记账。取值 = 4×arrival_m 与 1 格取大（≈0.48 m）：
        #: 既不会被投影跳变的瞬间误判成"跳点"，也保证一个航点不会无限期悬着。
        self._skip_over_m = max(4.0 * self._arrival_m, geo.BLOCK_SIZE)

    # ------------------------------------------------------------------ #
    #  只读接口（测试/遥测）
    # ------------------------------------------------------------------ #
    @property
    def phase(self) -> str:
        return self._phase

    @property
    def route(self) -> Route | None:
        return self._route

    @property
    def stop_context(self) -> _StopContext | None:
        return self._stopctx

    @property
    def handled_stops(self) -> set[int]:
        return set(self._handled_stops)

    def state(self) -> dict[str, Any]:
        """最近一拍的任务上下文（与 `P_TELEMETRY[MISSION_STATE_KEY]` 一致）。"""
        return {
            "phase": self._phase, "route_len": self._waypoint_count,
            "next_wp": self._next_wp, "arrived": self._arrived,
            "handled_stops": sorted(self._handled_stops),
            "stop_index": self._stopctx.index if self._stopctx else -1,
            "stop_block": self._stopctx.block if self._stopctx else "",
            "stop_phase": self._stopctx.phase if self._stopctx else StopPhase.DONE,
            "stop_detail": self._stopctx.detail if self._stopctx else "",
            "dist_to_next_stop": self._dist_to_next_stop,
            "finish": dict(self._finish_info),
            "commands": dict(self._cmd_stats),
            # ★路线来源与自动规划证据（横幅同源；遥测/测试据此断言「自动规划」/「手动配置」）
            "route_source": self._route_source,
            "plan": dict(self._plan_info),
            # ★逐点到达台账（卡 D finding D1）：`unexplained` 恒为 0 才叫"无静默丢点"
            "accounting": self.accounting(),
        }

    def accounting(self) -> dict[str, Any]:
        """逐点到达台账（遥测/测试/结束横幅共用同一份口径）。

        口径（与任务卡 D 的验收一致，**不放宽**）：
          · 分母 = `len(route) - 1`（起点不计）；
          · `arrived` = 登记到达数（含"最近接近"几何复核、含 PIVOT 通知判达）；
          · `skipped` = 显式跳过数（停滞兜底 + s 越窗），每一条都有事件与日志；
          · `unexplained` = 既没登记到达、也没显式跳过的航点 —— **必须为空**。
        """
        n = self._waypoint_count
        unexplained = [i for i in range(1, n)
                       if not self._passed[i] and i not in self._skipped_at
                       and i not in self._pending_skip] if self._passed else []
        by: dict[str, int] = {}
        for reason in self._arrived_by.values():
            key = reason.split(" ")[0]
            by[key] = by.get(key, 0) + 1
        return {
            "denominator": max(0, n - 1),
            "arrived": self._arrived,
            "arrived_by": by,
            "stall_skips": self._forced_skips,
            "cross_skips": self._cross_skips,
            "skipped_total": len(self._skipped_at),
            "skipped": {str(k): v for k, v in sorted(self._skipped_at.items())},
            # ★"越窗待定"（已记账、未定案）：收尾时由 `_close_ledger()` 转成正式跳过，
            #   期间若车又靠近，上面的判据会把它登记为**到达**（雷 I8 就是这种）
            "pending_skips": {str(k): v for k, v in sorted(self._pending_skip.items())},
            "unexplained": unexplained,
            "close_m": round(self._close_m, 4),
            "skip_over_m": round(self._skip_over_m, 4),
        }

    def closest_approaches(self) -> dict[int, float]:
        """每个航点的**历史最近距离**（米，-1 表示还没测到）。

        只读接口，供"车体到底有没有盖住格心"的复核（t8 的雷 A9 / t10 的雷覆盖对账）——
        本方法**不改变**任何到达/跳过判定，只是把 Mission 已经算出来的距离摊开给人看。
        """
        return {i: (round(v, 4) if v != math.inf else -1.0)
                for i, v in enumerate(self._min_dist)}

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        self._write_phase(MissionPhase.IDLE)
        self._log(f"就绪：auto_start={self._auto_start} 延时={self._auto_start_delay_s:g}s "
                  f"复位={self._reset_spec!r} 扫描模式={self._scan_mode or '未设置'}")

    def teardown(self) -> None:
        try:
            self.bridge.publish_fire_gate(False)     # ★ 收尾关闸：绝不留一个开着的放行门
        except Exception:                            # noqa: BLE001
            pass
        try:
            self.bridge.publish_brake()
        except Exception:                            # noqa: BLE001
            pass

    # ------------------------------------------------------------------ #
    #  单拍
    # ------------------------------------------------------------------ #
    def tick(self) -> None:
        now = self._clock()
        self._drain_commands()

        safety: SafetyState | None = self.bb.get(P_SAFETY)
        if safety is not None and not safety.ok:
            self._enter_safe_stop()
            self._publish_state(now)
            return
        if self._phase == MissionPhase.SAFE_STOP:
            self._recover_from_safe_stop()

        self._maybe_auto_start()
        if self._phase == MissionPhase.FINISH:
            pass                                    # ★ 锁存：FINISH 后不再推进（也不重复发事件）
        elif self._phase in _DRIVING_PHASES:
            self._drive()
        elif self._phase == MissionPhase.STOP:
            self._run_stop_point()
        self._check_finish()
        self._publish_state(now)

    # ------------------------------------------------------------------ #
    #  出发
    # ------------------------------------------------------------------ #
    def _maybe_auto_start(self) -> None:
        if not self._auto_start or self._phase != MissionPhase.IDLE:
            return
        if (self._clock() - self._created_at) < self._auto_start_delay_s:
            return
        if self._route is not None:                 # 外部命令已经载入过路线
            return
        if self._autoplan_blocked:                  # ★硬失败已定格（error 日志出过一次）→ 不再重试
            return
        try:
            self._reset_belief(self._reset_spec)
            self._load_startup_route()              # ★ 自动规划 / 读 route.tokens（见函数 docstring）
        except AutoPlanBlocked:
            self._autoplan_blocked = True           # ★ 停在 IDLE：不 publish EV_MISSION_STARTED
            return
        except Exception as exc:                    # noqa: BLE001  配置错就停着，绝不带病发车
            if self._log_throttle.allow("auto_start_failed", self._clock()):
                self._log(f"自动出发失败（保持 IDLE）：{type(exc).__name__}: {exc}")
            return
        self._log(f"自动出发：延时 {self._auto_start_delay_s:g}s 到，复位 {self._reset_spec!r}")
        self._start_mission()

    def _load_startup_route(self) -> None:
        """启动路线来源：`planner.auto_plan_enabled` 决定"自动规划"还是"读 route.tokens"。

        ★ 失败语义（任务卡 B 规格，两条分支都有独立测试）：
          - `planner.fallback_to_tokens: true` → warn（含异常类型+消息）后回退 `route.tokens`，
            **仍进入 DRIVE**（宁可路线不是最优，也要出车）；
          - `false` → error 日志 + 抛 `AutoPlanBlocked` → `_maybe_auto_start` 捕获后**停在 IDLE**，
            绝不 publish `EV_MISSION_STARTED`。
        """
        if not self._auto_plan_enabled:
            self._route_source = ROUTE_SOURCE_MANUAL
            tokens = self.cfg.get("route.tokens") or self.cfg.get("mission_blocks")
            self._load_tokens(tokens, source="route.tokens(或 mission_blocks)")
            return

        out = plan_route_from_config(self.cfg, map_path=self._planner_map_path, log=self._log)
        self._plan_info = {
            "ok": out.ok, "map_path": out.map_path, "elapsed_ms": out.elapsed_ms,
            "error": out.error, "allow_reverse": out.allow_reverse,
            "max_seconds": self._planner_max_seconds, "token_count": len(out.tokens),
            "fallback_to_tokens": self._planner_fallback,
        }
        if out.ok:
            self._route_source = ROUTE_SOURCE_AUTO
            # ★ 验收项：日志必须能看到【完整 token 列表】与【规划耗时(ms)】
            self._log(f"自动规划成功：map={out.map_path} {out.mode} "
                      f"耗时 {out.elapsed_ms:.1f} ms / {len(out.tokens)} token / "
                      f"{out.route_len_m:.2f} m / 倒车段 {out.reverse_segments} / "
                      f"A* 代价 {out.cost:.2f}")
            self._log(f"自动规划 token 列表（{len(out.tokens)}）：" + " ".join(out.tokens))
            self._load_tokens(out.tokens, source=f"{ROUTE_SOURCE_AUTO}({out.map_path})")
            return

        detail = (f"自动规划失败：map={out.map_path} {out.mode} "
                  f"预算 {self._planner_max_seconds:g}s → {out.error}")
        if self._planner_fallback:
            self._log(f"⚠️ {detail}；fallback_to_tokens=true → 回退 route.tokens/mission_blocks 并继续出发")
            self._route_source = ROUTE_SOURCE_MANUAL
            tokens = self.cfg.get("route.tokens") or self.cfg.get("mission_blocks")
            self._load_tokens(tokens, source="route.tokens/mission_blocks(自动规划失败回退)")
            return
        self._log(f"❌ {detail}；fallback_to_tokens=false → 停在 IDLE，不出发")
        raise AutoPlanBlocked(out.error)

    def _start_mission(self) -> None:
        """路线就绪 → 阶段 DRIVE + 事件 + 持续扫描（见模块 docstring）。"""
        self.bus.publish(EV_MISSION_STARTED, source=self.name,
                         route=len(self._route) if self._route else 0)
        self._write_phase(MissionPhase.DRIVE)
        if self._scan_mode == "continuous":
            self._request_scan(2)                   # ★ 2 = 持续，绝不用 1（会被降级成单趟）
        else:
            self._request_scan(1 if self._scan_mode == "waypoint" else 0)
        self._banner_start()

    def _banner_start(self) -> None:
        route = self._route
        if route is None:
            return
        # ★ 路线来源行：验收项要求横幅里能读到「路线来源：自动规划 / 手动配置」，
        #   自动规划时顺带把地图与规划耗时一起打出来（与上面的"自动规划成功"日志同源）。
        src_line = f"路线来源：{self._route_source or ROUTE_SOURCE_MANUAL}"
        if self._plan_info.get("ok"):
            src_line += (f"（map={self._plan_info.get('map_path')} "
                         f"耗时 {float(self._plan_info.get('elapsed_ms') or 0.0):.1f} ms "
                         f"{'允许倒车' if self._plan_info.get('allow_reverse') else '无倒车'}）")
        banner([
            f"【任务开始】路线 {len(route)} 点 / {route.path_length():.2f} m / 穿洞 {route.crossing}",
            f"停站点 {len(self._stop_s)} 个："
            + " ".join(f"{i}:{b}" for i, b in sorted(self._stop_s.items())),
            f"扫描模式 {self._scan_mode or '未设置'}（continuous → scan_request=2）",
            src_line,
        ], log=self._log)

    def _reset_belief(self, spec: str) -> None:
        """`reset:BLOCK,HEAD` / `mission.reset` → 把 belief 设为该格心 + 朝向。"""
        if not spec or not str(spec).strip():
            raise ValueError("复位串为空")
        block, yaw = parse_reset(spec)
        x, y = geo.block_to_meters(block)
        pose = geo.Pose(x, y, yaw)
        self.bb.update(self.name, **{P_POSE: pose})
        self.bb.update(self.name, **{P_ROUTE_INDEX: -1, P_ROUTE_S: 0.0})
        self._last_s = 0.0
        self._route_s = 0.0
        self._log(f"复位 belief：{block} ({x:.3f}, {y:.3f}) yaw={geo.deg(yaw):.1f}°")

    # ------------------------------------------------------------------ #
    #  路线载入
    # ------------------------------------------------------------------ #
    def _load_tokens(self, tokens, *, source: str = "route.tokens") -> int:
        if not tokens:
            raise ValueError(f"{source} 为空，无法载入路线")
        route = parse_mission(tokens, shoot_planner=self._shoot_planner, name=source)
        n = len(route)
        if n < 2:
            raise ValueError(f"路线只有 {n} 个航点（Path 至少需要 2 点）")
        self._install_route(route, source=source)
        self._started_at = self._clock()            # 计时起点 = 路线就绪（终点横幅用它算用时）
        self.bus.publish(EV_ROUTE_LOADED, source=self.name, route=n, blocks=list(route.blocks()))
        self._log(f"路线载入[{source}]：{route.describe()}")
        return n

    def _load_single_point(self, spec: str) -> int:
        """单点命令 `BLOCK,HEAD`（可带 `*` / `:SCAN` / `R:`）。

        ★ 单点**不经过 `parse_mission`**：只有 1 个航点，`Path` 会拒绝编译，
          而且 `parse_mission` 的"朝向后缀只允许出现在最后一个 token"对单点是恒真的，
          直接在末尾航点上落 `yaw` 更简单、也更贴合旧版 `send_waypoint.py` 的语义。
        """
        text = spec.strip()
        if not text:
            raise ValueError("单点命令缺少格名")
        heading = None
        if len(text) >= 2 and text[-2] == ",":
            heading = text[-1].lower()
            if heading not in ("x", "y"):
                raise ValueError(f"朝向后缀只能是 ,x 或 ,y：{spec!r}")
            text = text[:-2].strip()
        info = parse_token(text)
        block = info["block"]
        x, y = geo.block_to_meters(block)
        yaw = geo.heading_token_to_yaw(heading) if heading else geo.HEADING_X
        wp = Waypoint(block=block, x=x, y=y, yaw=yaw, reverse=info["reverse"],
                      shoot=info["shoot"], scan=info["scan"], token=info["token"])
        route = Route((wp,), name="单点命令")
        self._install_route(route, source="单点命令")
        self._started_at = self._clock()
        self.bus.publish(EV_ROUTE_LOADED, source=self.name, route=1, blocks=[block])
        self._log(f"单点命令：{info['token']}（单点不编译 Path；Tracker 侧投影不可用，"
                  f"只用于「停在该点」类调试）")
        return 1

    def _install_route(self, route: Route, *, source: str = "") -> None:
        self._route = route
        self._route_name = source or route.name
        # ★ 单点路线（`单点 BLOCK,HEAD`）只有 1 个航点，而 `Path` 要求 ≥2 点，
        #   所以这里 `_path = None`；推进/终点/停站判定全部对 `_path is None` 做防御。
        self._path = Path(route) if len(route) >= 2 else None
        self._waypoint_count = len(route)
        self._passed = [False] * len(route)
        self._arrived = 0
        self._next_wp = 1
        self._handled_stops = set()
        self._passed_by_pivot = set()                # ★ 新轮次：旧通知的"已到达"作废
        # ★★ 逐点到达台账换轮次：新路线必须从零开始，否则上一轮的"最近接近"会把
        #    新路线里**同名不同索引**的航点误判成到过（路线自折叠时格名会重复）。
        self._min_dist = [math.inf] * len(route)
        self._arrived_by = {}
        self._skipped_at = {}
        self._pending_skip = {}
        self._cross_skips = 0
        self._stopctx = None
        self._last_stop_state = StopPointState()
        self._route_s = 0.0
        self._last_s = 0.0
        self._dist_to_next_stop = None
        self._finish_info = {}
        self._stop_s = {}
        if self._path is not None:
            self._stop_s = {i: self._path.s_at_waypoint(i) for i, _ in route.stop_points()}
            if (len(route) - 1) not in self._stop_s:     # 末点一定是一个"停站点"（终点）
                self._stop_s[len(route) - 1] = self._path.total_length
        self._stop_blocks = [route[i].block for i in sorted(self._stop_s)]
        self.bb.update(self.name, **{P_ROUTE: route})
        self.bb.update(self.name, **{P_ROUTE_INDEX: 0, P_ROUTE_S: 0.0})
        # ★ 清掉上一轮残留的 PIVOT 通知：旧通知对新路线毫无意义，
        #   留着会（在最坏的同几何重载场景）把新轮次航点开局误判成"已到达"。
        self.bb.update(self.name, **{PIVOT_DONE_KEY: None})

    # ------------------------------------------------------------------ #
    #  `SHOOT@<靶标>` 的候选链计算
    # ------------------------------------------------------------------ #
    def _shoot_planner(self, target: str) -> list[str]:
        """靶格 → 候选射击停站点（4 邻格中落在射击窗口内的）。

        语法：`SHOOT@G11` 是**指令不是坐标**（`route/parser.py`），由本回调展开成候选链。
        取窗口与候选保留数沿用现场键 `path_executor.shoot_distance_target` /
        `shoot_distance_min` / `shoot_distance_max`（缺省 0.35 / 0.20 / 0.45，与 map.yaml 一致）。
        """
        tx, ty = geo.block_to_meters(target)
        win_lo = float(self.cfg.get("path_executor.shoot_distance_min", 0.20))
        win_hi = float(self.cfg.get("path_executor.shoot_distance_max", 0.45))
        self.cfg.get("path_executor.shoot_distance_target", 0.35)
        keep = int(self.cfg.get("path_executor.shoot_candidates_keep", 3) or 3)

        row_idx, col_idx = geo.block_to_index(target)
        cands: list[tuple[float, str]] = []
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            r, c = row_idx + dr, col_idx + dc
            if not (0 <= r < geo.MAP_ROWS and 0 <= c < geo.MAP_COLS):
                continue
            block = geo.index_to_block(r, c)
            bx, by = geo.block_to_meters(block)
            dist = math.hypot(bx - tx, by - ty)
            if win_lo - _EPS <= dist <= win_hi + _EPS:
                cands.append((dist, block))
        cands.sort(key=lambda item: (item[0], item[1]))
        out = [block for _, block in cands[:keep]]
        return out or [target]                       # 兜底：靶标自身一格（旧实现的降级行为）

    # ------------------------------------------------------------------ #
    #  航点推进
    # ------------------------------------------------------------------ #
    def _current_s(self) -> float:
        """本拍沿程弧长。

        ★ 读 `P_ROUTE_S` 之前**必须验一致性**：该键可能是上一次任务残留的旧值
          （例如刚 `reset`、或本轮 Localization/Tracker 还没写过），
          拿它当"车已经走到哪"会让"到点/到站/终点"判定全部失真。
          验法是把它与"用当前位姿现算的弧长"比对：差得离谱就丢掉、用现算值，
          并把现算值写回（与 Localization 写的是**同一投影语义**，不会互相打架）。
        """
        path = self._path
        if path is None:                            # 单点路线：没有弧长概念
            return self._last_s
        pose = self.bb.get(P_POSE)
        cached = self.bb.get(P_ROUTE_S)
        s: float | None = None
        if pose is not None:
            proj = path.project(pose.x, pose.y, s_hint=self._last_s)
            self._last_lat_m = float(proj.dist)      # ★ 横向偏差（"是否贴在路径上"的判据）
            if isinstance(cached, (int, float)) and abs(float(cached) - proj.s) <= self._arrival_m:
                s = float(cached)                   # 与位姿一致 → 沿用（Localization 的权威值）
            else:
                s = float(proj.s)
                self.bb.update(self.name, **{P_ROUTE_INDEX: int(proj.seg_index), P_ROUTE_S: s})
        elif isinstance(cached, (int, float)):
            s = float(cached)
        if s is None:
            return self._last_s
        self._last_s = s
        self._route_s = s
        return s

    def _advance_waypoints(self) -> None:
        route, path = self._route, self._path
        if route is None or path is None:
            return
        pose = self.bb.get(P_POSE)
        s = self._current_s()
        notice = self._pivot_notice()                # 本拍读一次，循环里复用
        for i, wp in enumerate(route.waypoints):
            if self._passed[i]:
                continue
            if i == 0:
                self._passed[0] = True           # 起点不报"到达"
                continue
            # ★ 弧长判据必须与"车确实在附近"**合取**，不能单用：
            #   投影链会跳变/冻结（诊断见 docs/诊断_投影与顶点.md），实测 s 从 3.50 冲到 4.20 时，
            #   #11 F11 / #12 E11 / #13 F11 三个**从未走到**的点被默默判成"已到达" ——
            #   这既不诚实（没有 forced_skip 记录），又会让后续航点计数整体错位。
            #
            # ★★ 但"在附近"的门槛不能取 2×arrival_m(0.24 m)：纯追踪**正常切角**时车离格心
            #   会超过它 —— bench 实测 L10→L9→K9 的直角，车路过 L9 时离线心 **0.288 m**
            #   ⇒ `near` 恒假 ⇒ L9 **永不置位** ⇒ 停滞兜底 12 s 后把它当"不可达"补跳。
            #   一次跑下来从 #3 一路补跳到 #27（12 个），车光在等 12 s×12 就把时间耗光了。
            #
            # ★★★ 真正的护栏是**"刚越过"窗口**，不是距离：s 跳变会一次性跨过好几个航点，
            #   那些点的 `over` 远大于 arrival_m，窗口直接把它们挡住；而正常逐拍推进时
            #   每个点都在窗口内被逮住（10 Hz × 0.15 m/s = 15 mm/拍 ≪ 120 mm）。
            #   距离门槛据此放宽到 4×arrival_m(≈0.48 m)：放行正常切角，仍挡住"车根本不在
            #   这条路附近，是投影跑到别处去了"的情形。
            s_wp = path.s_at_waypoint(i)
            dist = None if pose is None else wp.distance_to(pose.x, pose.y)
            if dist is not None and dist < self._min_dist[i]:
                # ★ 历史最近距离（几何复核用）：车常在"s 还没到"的几拍前从格心旁穿过，
                #   等 s 跳过去那一拍它已经走远 —— 只看当拍距离就会把这种**真的到过**
                #   判成"没到"。本行是 finding D1 里"静默丢点"的第一道解药。
                self._min_dist[i] = float(dist)
            over = s + self._arrival_m - s_wp        # >0 = 已越过该航点
            s_reached = over >= 0.0
            # ★★ "车在附近"的判据改成**贴近路径**（横向偏差 `_last_lat_m`），不是贴近格心：
            #   纯追踪的**正常切角**会离线心很远却仍在路径上 —— bench 实测 L10→L9→K9 的
            #   直角，车路过 L9 时离线心 **0.288 m**、但横向偏差只有 ~0.09 m。旧判据
            #   `dist ≤ 2×arrival_m(0.24 m)` 在这种正常切角下恒假 ⇒ L9 **永不置位** ⇒
            #   停滞兜底 12 s 后把它当"不可达"补跳，一次跑从 #3 补跳到 #27（12 个），
            #   240 s 全耗在等待上（到点 30 → 16、绕圈 3 → 12、跑不完全程）。
            #   而"投影跑到别处造成的假到点"里车离**路径**也远（§11 用例：横偏 0.35 m），
            #   两条判据由此区分开，互不误伤。
            lat = self._last_lat_m
            near = ((lat is not None and lat <= 0.30)
                    or dist is None or dist <= 2.0 * self._arrival_m)
            why = ""
            if (s_reached and near) or (dist is not None and dist <= self._arrival_m):
                why = "s+近路径" if s_reached else "距格心"
            elif pose is not None and self._pivot_notice_accepts(notice, i, s_wp, s, dist):
                # ★ PIVOT 完成通知判到达：原地掉头后车原路返回，上面的两条判据恒假
                #   （pivot_dist_m > arrival_m），见 `PIVOT_DONE_KEY` 注释与
                #   `_pivot_notice_accepts` 的护栏说明。bench 实测 E11/A6/B6 三处
                #   此前就是在这里判不到、12 s 后被拐角绕圈兜底强制跳过。
                why = "PIVOT 通知"
                self._passed_by_pivot.add(i)
            elif s_reached and self._min_dist[i] <= self._close_m:
                # ★★ 几何复核（finding D1 的第二道解药）：s 跳变把"刚越过窗口"跳过去了，
                #    但**历史最近距离**证明车确实从格心附近穿过 ⇒ 如实登记为到达，
                #    而不是既不记到达、也不记跳过的静默丢弃。
                why = f"最近接近 {self._min_dist[i]:.3f}m"
            if why:
                revoked = self._pending_skip.pop(i, None)   # ★ 待定被"后来居上"的到达撤销
                self._passed[i] = True
                self._arrived += 1
                self._arrived_by[i] = why
                self._next_wp = i + 1
                self.bus.publish(EV_WAYPOINT_REACHED, source=self.name, index=i,
                                 block=wp.block, s=round(path.s_at_waypoint(i), 3),
                                 reason=why)
                self._log(f"到点 {i}/{len(route) - 1} {wp.block}"
                          f"（s={path.s_at_waypoint(i):.2f} m，依据 {why}）"
                          + ("（撤销先前的越窗待定）" if revoked else ""))
                continue
            if s_reached and over >= self._skip_over_m:
                # ★★ 显式记账（finding D1 的第三道解药，**绝不静默丢点**）：
                #    s 已越过这么多，而车**从头到尾**都没进过 `_close_m`(半格) ⇒ 它确实
                #    还没走到。这里只把它登记成"越窗**待定**"，**不置位 `_passed`** ——
                #    投影跳变的同一拍车可能马上又靠回来，那时上面的判据会正常登记到达
                #    （bench 实测雷 I8：基线里是「到点 8/45」，一旦立即判死就变成跳点、
                #    雷覆盖从 8/8 掉到 7/8）。真正定案放在 `_close_ledger()`（任务收尾），
                #    所以"无静默丢点"在收尾时仍然成立，而正常到达的机会一点没少。
                detail = (f"s 已越过 {over:.2f} m（> {self._skip_over_m:.2f} m），"
                          f"但最近只到 {self._min_dist[i]:.3f} m"
                          f"（> 复核半径 {self._close_m:.3f} m，横向偏差 {lat}）")
                if i not in self._pending_skip:
                    self._pending_skip[i] = detail
                    self._log(f"⚠️ 越窗待定（显式记账，不静默丢点）：航点 {wp.block}（#{i}）{detail}")

    def _detect_stall_and_skip(self) -> None:
        """拐角绕圈兜底：弧长停滞 / 原地绕圈 → 强制跳过不可达航点。

        ★ 为什么需要（仿真实测，见 docs/C线_仿真器报告.md §4）：
          当定位噪声或里程比例误差让"车的投影弧长 s"无法越过下一个航点时，
          `_advance_waypoints` 的 `s + arrival_m >= s_at_waypoint(i)` 永远不成立，
          车就会在拐角原地转圈（实测噪声 0.01m/拍 → 46.8s 卡死、末点误差 2.96m）。
          按 readme【八】27 "宁可漏打，不可卡死"，这里主动把卡住的点判为不可达并跳过，
          让任务继续，绝不原地空转。

        判据（任一成立即触发）：
          ① s 在 `mission.stall_timeout_s` 内前进不足 `arrival_m`；
          ② 停滞期间车累计转满 `mission.stall_yaw_deg`（原地绕圈特征）。
        ★ 例外：Tracker 正在执行 PIVOT 有界机动时**整体不计停滞**（见 `_pivot_in_progress`）——
          180° 折返的 PIVOT 实测要 ~10.8s、累计转 ~180°，与上面两条判据**在数值上正面撞车**：
          PIVOT 被修成"真的会转"之后，看门狗把每一次正常折返都当成绕圈（实闭环 bench
          实测 F11/A6/D6 三处误跳点）。PIVOT 自带 14s 硬超时且必然自终止，无卡死风险。
        触发后：把"下一个未到达航点"强制标记为已过（跳过），发事件并告警；
                若该点是停站点（打靶），跳过意味着放弃这一击 —— 宁可漏打，不可卡死。
        """
        route, path = self._route, self._path
        if route is None or path is None:
            self._reset_stall()
            return

        # ---- ★ 有界机动（PIVOT）期间不计停滞：时间窗整体重置（不是忽略本次判定）----
        if self._pivot_in_progress():
            self._reset_stall()
            return

        s = self._last_s
        pose = self.bb.get(P_POSE)
        yaw = pose.yaw if pose is not None else None

        # ---- 维护停滞窗口 ----
        progressed = self._stall_last_s is None or (s - self._stall_last_s) >= self._arrival_m
        if progressed:
            self._reset_stall()
        if self._stall_since is None:
            self._stall_since = self._clock()
            self._stall_last_s = s
            self._stall_last_yaw = yaw
            self._stall_yaw_sum = 0.0
        elif yaw is not None and self._stall_last_yaw is not None:
            self._stall_yaw_sum += abs(geo.ang_diff(yaw, self._stall_last_yaw))
            self._stall_last_yaw = yaw

        # ---- 判定 ----
        stalled_time = self._clock() - self._stall_since
        yaw_circled = self._stall_yaw_sum >= math.radians(self._stall_yaw_deg)
        if stalled_time < self._stall_timeout_s and not yaw_circled:
            return

        # ---- 找下一个未到达航点（跳过它）----
        next_idx = next((i for i in range(1, len(route)) if not self._passed[i]), None)
        if next_idx is None:
            self._reset_stall()
            return
        wp = route[next_idx]
        self._passed[next_idx] = True
        self._forced_skips += 1
        self._next_wp = next_idx + 1
        # ★ 进台账（finding D1）：兜底跳过与"越窗跳过"共用一本账，收尾时逐点对账。
        self._skipped_at[next_idx] = (f"停滞兜底 {stalled_time:.1f}s / 累计转 "
                                      f"{math.degrees(self._stall_yaw_sum):.0f}°")
        self._log(f"⚠️ 拐角绕圈兜底：航点 {wp.block}（#{next_idx}）停滞 "
                  f"{stalled_time:.1f}s / 累计转 {math.degrees(self._stall_yaw_sum):.0f}° 仍不可达，"
                  f"强制跳过（宁可漏点，不可卡死）")
        self.bus.publish(EV_WAYPOINT_REACHED, source=self.name, index=next_idx,
                         block=wp.block, forced_skip=True, reason="stall")
        self._reset_stall()

    def _pivot_in_progress(self) -> bool:
        """Tracker 是否正在执行 PIVOT 机动（读它的遥测快照，不直接依赖其内部状态）。

        ★ 为什么停滞判定必须让位：180° 折返的 PIVOT 实测要 **~10.8s**、车累计转 **~180°**，
          而绕圈判据是"`stall_timeout_s`(=12s) 内 s 不前进" + "累计转满 `stall_yaw_deg`"
          —— **正常折返与"原地绕圈"在数值上完全撞车**。PIVOT 被修成真的会转之后，
          看门狗就会把每一次正常折返都判成绕圈跳点（实闭环 bench 实测 F11/A6/D6 三处，
          跳点=漏压雷/漏打靶）。PIVOT 自带 14s 硬超时并且必然自终止，没有卡死风险，
          所以有界机动期间整个停滞窗口重置（而不是只跳过这一次判定）。
        """
        snap = self.bb.get(P_TELEMETRY)
        if not isinstance(snap, dict):
            return False
        cmd = snap.get(TRACKER_CMD_KEY)
        if not isinstance(cmd, dict):
            return False
        return str(cmd.get("pivot_phase", PivotPhase.IDLE)) != PivotPhase.IDLE

    # ------------------------------------------------------------------ #
    #  ★PIVOT 完成通知（Tracker 写、本任务读）
    # ------------------------------------------------------------------ #
    def _pivot_notice(self) -> tuple | None:
        """读 PIVOT 完成/放弃通知：`(顶点航点索引, 顶点弧长, 结局)`；无/非法 → None。"""
        value = self.bb.get(PIVOT_DONE_KEY)
        if isinstance(value, tuple) and len(value) == 3:
            return value
        return None

    def _pivot_notice_accepts(self, notice, idx: int, s_wp: float, s: float,
                              dist: float | None) -> bool:
        """PIVOT 通知是否可把航点 `idx` 判为到达。

        护栏（防跨轮次残留/竞态误判，bench 实测缺一不可）：
          ① 航点索引与顶点弧长**逐位**对上（每条路线各顶点弧长唯一）；
          ② 车当前仍在顶点附近：`s_wp - s` 与几何距离都不得超过
             `_pivot_accept_m`（≈3×pivot_dist_m）——真完成时二者 ≈ pivot_dist_m，
             而残留通知里车在起点，差好几段长。
        """
        if notice is None or notice[0] != idx:
            return False
        if abs(float(notice[1]) - s_wp) > 1e-6:
            return False
        win = self._pivot_accept_m
        if (s_wp - s) > win:
            return False
        if dist is not None and dist > win:
            return False
        return True

    def _reset_stall(self) -> None:
        self._stall_since = None
        self._stall_last_s = 0.0
        self._stall_last_yaw = None
        self._stall_yaw_sum = 0.0

    def _next_stop_candidate(self) -> int | None:
        """下一个需要停车的停站点索引；**末点不算停站点**。

        ★ 末点虽然也在 `_stop_s` 里（它是"终点减速/终点判定"的锚），但它由 `FINISH`
          处理，绝不能进三段式停站流程 —— 否则终点会被当成一个打靶点停在那里。
        """
        last = self._waypoint_count - 1
        for idx in sorted(self._stop_s):
            if idx == last:
                continue
            if idx in self._handled_stops:
                continue
            # ★ 经 PIVOT 通知判到达的停站点**不**跳过：它还没走三段式停站
            #   （`_should_trigger_stop` 里同样的通知会让它立刻触发进站）。
            if idx < len(self._passed) and self._passed[idx] and idx not in self._passed_by_pivot:
                continue
            return idx
        return None

    def _dist_to_stop(self, idx: int, pose) -> float | None:
        if pose is None or self._route is None or idx >= len(self._route):
            return None
        wp = self._route[idx]
        return math.hypot(wp.x - pose.x, wp.y - pose.y)

    # ------------------------------------------------------------------ #
    #  驱动
    # ------------------------------------------------------------------ #
    def _drive(self) -> None:
        self._advance_waypoints()
        self._detect_stall_and_skip()     # ★ 拐角绕圈兜底：宁可漏点，不可卡死
        idx = self._next_stop_candidate()
        if idx is None:
            self._dist_to_next_stop = None
            self._write_phase(MissionPhase.DRIVE)
            return
        pose = self.bb.get(P_POSE)
        dist = self._dist_to_stop(idx, pose)
        self._dist_to_next_stop = dist
        s = self._current_s()
        if self._should_trigger_stop(idx, s, dist):
            self._enter_stop_point(idx, dist)
            return
        approach_m = self._approach_m
        self._write_phase(MissionPhase.APPROACH if (dist is not None and dist <= approach_m)
                          else MissionPhase.DRIVE)

    def _should_trigger_stop(self, idx: int, s: float, dist: float | None) -> bool:
        """到点判据：沿程已过停站点弧长，或几何距离进 `stop_radius_m`（旧版 0.17 m）。

        ★ 弧长与半径**取或**：弧长判据在"路线自交/投影跳段"时可能提前触发，
          半径判据在"车贴着点但投影没跟上"时兜底，两者互补（旧版只有半径 + 开过头补触发）。
        ★ PIVOT 通知也算：停站点恰是掉头顶点时，正常两条判据同样够不着
          （pivot_dist_m > arrival_m），但车已在顶点跟前完成掉头 → 照常进三段式。
        """
        path = self._path
        if path is not None and s + self._stop_radius_m >= self._stop_s.get(idx, math.inf):
            return True
        if dist is not None and dist <= max(self._stop_radius_m, self._arrival_m):
            return True
        if self._pivot_notice_accepts(self._pivot_notice(), idx,
                                      self._stop_s.get(idx, math.inf), s, dist):
            return True
        return False

    # ------------------------------------------------------------------ #
    #  停站点三段式（每段独立超时，绝不阻塞）
    # ------------------------------------------------------------------ #
    def _enter_stop_point(self, idx: int, dist: float | None) -> None:
        route = self._route
        if route is None:
            return
        wp = route[idx]
        now = self._clock()
        kind = StopKind.SCAN if (wp.scan and not wp.shoot) else StopKind.SHOOT
        shots = int(self.bridge.shoot_count())
        self._stopctx = _StopContext(
            index=idx, block=wp.block, kind=kind, phase=StopPhase.SETTLE,
            started_at=now, phase_started_at=now,
            shots_at_arrival=shots,
            shots_needed=(self._shots_per_stop if kind == StopKind.SHOOT else 0),
            detail=f"到站（距离 {dist:.3f} m，累计发数 {shots}）" if dist is not None
                   else f"到站（累计发数 {shots}）",
        )
        self._write_phase(MissionPhase.STOP)
        self._publish_stop_state(now)
        self.bus.publish(EV_ARRIVED_STOP_POINT, source=self.name, index=idx, block=wp.block,
                         kind=kind, shots=shots)
        self._log(f"★ 停站点 {idx} {wp.block}（{kind}）{self._stopctx.detail}"
                  f"；三段超时 SETTLE>{self._settle_grace_s:.1f}s "
                  f"ACQUIRE>{self._lock_timeout_s:.1f}s FIRE>{self._shot_timeout_s:.1f}s")

    def _run_stop_point(self) -> None:
        stop = self._stopctx
        if stop is None:                            # 阶段与上下文不一致 → 安全回落
            self._write_phase(MissionPhase.DRIVE)
            return
        now = self._clock()
        if stop.kind == StopKind.SCAN:
            self._run_scan_stop(stop, now)
        elif stop.phase == StopPhase.SETTLE:
            self._phase_settle(stop, now)
        elif stop.phase == StopPhase.ACQUIRE:
            self._phase_acquire(stop, now)
        elif stop.phase == StopPhase.FIRE:
            self._phase_fire(stop, now)
        else:
            self._abandon_stop(stop, f"未知阶段 {stop.phase}")
        self._publish_stop_state(now)

    # ---- 段 1：SETTLE ----
    def _phase_settle(self, stop: _StopContext, now: float) -> None:
        pose = self.bb.get(P_POSE)
        moved = None
        if pose is not None and stop.last_x is not None and stop.last_y is not None:
            moved = math.hypot(pose.x - stop.last_x, pose.y - stop.last_y)
        if pose is not None:
            stop.last_x, stop.last_y = pose.x, pose.y

        speed = self._last_speed()
        # 停稳 = 位置连续不变 **或** 速度近零（两者任一成立就武装，避免过站慢速时白等）
        if (moved is not None and moved <= self._settle_eps_m) or \
                (speed is not None and abs(speed) <= self._settle_speed_mps):
            if stop.settle_armed_at is None:
                stop.settle_armed_at = now
                stop.detail = f"停稳计时开始（Δ={moved if moved is not None else float('nan'):.4f} m，"
                stop.detail += f"v={speed if speed is not None else float('nan'):.3f} m/s）"

        if stop.settle_armed_at is not None:
            if (now - stop.settle_armed_at) >= self._settle_time_s:
                self._goto_phase(stop, StopPhase.ACQUIRE, now,
                                 f"停稳 {(now - stop.settle_armed_at):.2f}s"
                                 f" ≥ {self._settle_time_s:.2f}s")
                return
        if (now - stop.phase_started_at) >= self._settle_grace_s:
            self._goto_phase(stop, StopPhase.ACQUIRE, now,
                             f"停稳超时 {(now - stop.phase_started_at):.2f}s"
                             f" ≥ {self._settle_grace_s:.2f}s → 降级放行")

    # ---- 段 2：ACQUIRE ----
    def _phase_acquire(self, stop: _StopContext, now: float) -> None:
        if stop.kind == StopKind.SCAN:
            self._goto_phase(stop, StopPhase.FIRE, now, "扫描停站：跳过锁定")
            return
        status = int(self.bridge.aim_status())
        obs = self.bridge.vision(0.5)
        has_target = bool(obs is not None and getattr(obs, "has_target", False))
        if status == AimStatus.TRACK and has_target:
            self._goto_phase(stop, StopPhase.FIRE, now, "云台 TRACK 且有目标")
            return
        if (now - stop.phase_started_at) >= self._lock_timeout_s:
            self._abandon_stop(stop, f"锁定超时 {(now - stop.phase_started_at):.2f}s"
                                     f" ≥ {self._lock_timeout_s:.2f}s"
                                     f"（aim={AimStatus.name(status)} 目标={has_target}）")

    # ---- 段 3：FIRE ----
    def _phase_fire(self, stop: _StopContext, now: float) -> None:
        if stop.kind == StopKind.SCAN:
            if (now - stop.phase_started_at) >= self._scan_stop_timeout_s:
                self._complete_stop(stop, f"扫描停站完成（{self._scan_stop_timeout_s:.1f}s）")
            return
        self._set_fire_gate(True)                   # ★ 放行门：进本段即打开（幂等）
        delta = int(self.bridge.shoot_count()) - stop.shots_at_arrival
        if delta >= stop.shots_needed:
            self._complete_stop(stop, f"发数增量 {delta} ≥ {stop.shots_needed} → 命中窗口结束")
            return
        if (now - stop.phase_started_at) >= self._shot_timeout_s:
            self._set_fire_gate(False)
            self._abandon_stop(stop, f"开火超时 {(now - stop.phase_started_at):.2f}s"
                                     f" ≥ {self._shot_timeout_s:.2f}s（增量 {delta}"
                                     f"/{stop.shots_needed}）")

    # ---- 扫描停站（`F2:SCAN_STOP`）：不停顿打靶，只保证到位停稳 ----
    def _run_scan_stop(self, stop: _StopContext, now: float) -> None:
        if stop.phase == StopPhase.SETTLE:
            self._phase_settle(stop, now)
            return
        if stop.phase in (StopPhase.ACQUIRE, StopPhase.FIRE):
            if stop.phase == StopPhase.ACQUIRE:
                stop.phase = StopPhase.FIRE
                stop.phase_started_at = now
            if (now - stop.phase_started_at) >= self._scan_stop_timeout_s:
                self._complete_stop(stop, f"扫描停站完成（{self._scan_stop_timeout_s:.1f}s）")
            return
        self._abandon_stop(stop, f"未知阶段 {stop.phase}")

    # ---- 三段收尾 ----
    def _goto_phase(self, stop: _StopContext, phase: str, now: float, detail: str) -> None:
        self._set_fire_gate(False)                  # ★ 每段结束都关闸（旧版 2181–2183 的纪律）
        stop.phase = phase
        stop.phase_started_at = now
        stop.settle_armed_at = None
        stop.detail = detail
        self._log(f"停站点 {stop.index} {stop.block}：{detail} → {phase}")

    def _complete_stop(self, stop: _StopContext, detail: str) -> None:
        self._set_fire_gate(False)
        stop.detail = detail
        self._log(f"停站点 {stop.index} {stop.block} 完成：{detail}")
        self._close_stop_point(stop)

    def _abandon_stop(self, stop: _StopContext, detail: str) -> None:
        self._set_fire_gate(False)
        stop.detail = detail
        self._log(f"停站点 {stop.index} {stop.block} 放弃：{detail}（放行下一个候选）")
        self._close_stop_point(stop)

    def _close_stop_point(self, stop: _StopContext) -> None:
        stop.phase = StopPhase.DONE
        self._handled_stops.add(stop.index)
        self._last_stop_state = self._build_stop_state(stop)
        self.bus.publish(EV_STOP_POINT_DONE, source=self.name, index=stop.index,
                         block=stop.block, kind=stop.kind, detail=stop.detail)
        self._stopctx = None
        self._write_phase(MissionPhase.DRIVE)

    def _publish_stop_state(self, now: float) -> None:
        stop = self._stopctx
        state = self._build_stop_state(stop) if stop is not None else self._last_stop_state
        self.bb.set(P_STOP_POINT, state, source=self.name)

    def _build_stop_state(self, stop: _StopContext) -> StopPointState:
        return StopPointState(
            index=stop.index, block=stop.block, kind=stop.kind, phase=stop.phase,
            started_at=stop.started_at, phase_started_at=stop.phase_started_at,
            shots_at_arrival=stop.shots_at_arrival, shots_needed=stop.shots_needed,
            detail=stop.detail,
        )

    def _last_speed(self) -> float | None:
        """读 Tracker 最近一拍的速度（只读遥测键，不跨任务调用）。"""
        snap = self.bb.get(P_TELEMETRY)
        if not isinstance(snap, dict):
            return None
        cmd = snap.get("tracker_cmd")
        if not isinstance(cmd, dict):
            return None
        value = cmd.get("speed")
        return float(value) if isinstance(value, (int, float)) else None

    # ------------------------------------------------------------------ #
    #  放行门 / 扫描 / 阶段 / 遥测
    # ------------------------------------------------------------------ #
    def _set_fire_gate(self, allow: bool) -> None:
        """放行门是**锁存式**的：只在变化时发（避免 10 Hz 刷屏，同时保证状态可追）。"""
        allow = bool(allow)
        if allow == self._fire_gate_state:
            return
        self.bridge.publish_fire_gate(allow)
        self._fire_gate_state = allow

    def _request_scan(self, mode: int) -> None:
        mode = int(mode)
        if mode == 1 and 2 in self._scans:
            # ★ readme【十】31：持续扫描期间绝不能再发 1（会被降级成单趟）
            if self._log_throttle.allow("scan_downgrade", self._clock()):
                self._log("拒绝 scan_request=1：当前是持续扫描（2），单趟会把它降级")
            return
        self.bridge.publish_scan_request(mode)
        self._scans.append(mode)

    def _write_phase(self, phase: str) -> None:
        if phase == self._phase:
            return
        old, self._phase = self._phase, phase
        self.bb.set(P_MISSION_PHASE, phase, source=self.name)
        if self._log_throttle.allow(f"phase:{old}->{phase}", self._clock()):
            self._log(f"阶段 {old} → {phase}")

    def _enter_safe_stop(self) -> None:
        reasons = self.bb.get(P_SAFETY)
        if self._phase != MissionPhase.SAFE_STOP:
            self._set_fire_gate(False)
            self._log(f"安全接管：{getattr(reasons, 'reasons', ())} → SAFE_STOP")
        self._write_phase(MissionPhase.SAFE_STOP)

    def _recover_from_safe_stop(self) -> None:
        """安全恢复后回到 DRIVE（停站点上下文若还在就继续那一段，不重来）。"""
        if self._stopctx is not None and self._stopctx.active:
            self._write_phase(MissionPhase.STOP)
        else:
            self._write_phase(MissionPhase.DRIVE)
        self._log("安全恢复 → 继续任务")

    def _publish_state(self, now: float) -> None:
        snap = self.bb.get(P_TELEMETRY)
        data = dict(snap) if isinstance(snap, dict) else {}
        data[MISSION_STATE_KEY] = self.state()
        self.bb.set(P_TELEMETRY, data, source=self.name)
        self._publish_stop_state(now)

    # ------------------------------------------------------------------ #
    #  终点（锁存）
    # ------------------------------------------------------------------ #
    def _check_finish(self) -> None:
        route, path = self._route, self._path
        if route is None or path is None or self._finish_latched:
            return
        if self._phase in (MissionPhase.IDLE, MissionPhase.SAFE_STOP):
            return
        last = len(route) - 1
        self._advance_waypoints()                   # 先补最后一次推进（末点也要报到达）
        s = self._current_s()
        if not (self._passed[last] or s + self._arrival_m >= path.s_at_waypoint(last)):
            return

        self._finish_latched = True
        self._close_ledger()                        # ★ 收尾：把"还悬着"的航点显式记账
        elapsed = (self._clock() - self._started_at) if self._started_at is not None else 0.0
        drift = 0.0
        fix = self.bb.get("fix")
        if fix is not None:
            drift = float(getattr(fix, "max_drift_m", 0.0) or 0.0)
        shots = int(self.bridge.shoot_count())
        hits = int(self.bridge.take_hits())
        self._finish_info = {"elapsed_s": round(elapsed, 2), "shots": shots, "hits": hits,
                             "drift_m": round(drift, 3), "waypoints": len(route)}
        self._write_phase(MissionPhase.FINISH)
        self.bridge.publish_brake()
        self.bridge.publish_scan_request(0)         # 0 = 停扫
        self._scans.append(0)
        if not self._finish_done:
            self._finish_done = True
            self.bus.publish(EV_MISSION_FINISHED, source=self.name, **self._finish_info)
            acc = self.accounting()
            denom = acc["denominator"]
            rate = (acc["arrived"] / denom * 100.0) if denom else 0.0
            banner([
                "【任务结束】" + (f"路线 {len(route)} 点 / {route.path_length():.2f} m"),
                f"用时 {elapsed:.1f} s（{self._fmt_duration(elapsed)}）",
                f"发数 {shots} / 命中 {hits} / 漂移 {drift:.3f} m",
                f"停站点 {len(self._stop_s)} 个（完成 {len(self._handled_stops)}）",
                # ★★ 逐点到达台账（finding D1 的验收口径）：登记到达 / 显式跳过 / 无主航点
                f"逐点到点率 {acc['arrived']}/{denom} = {rate:.2f}%"
                f"（依据 {acc['arrived_by']}）",
                f"显式跳过 {acc['skipped_total']} 个"
                f"（停滞兜底 {acc['stall_skips']} + s 越窗 {acc['cross_skips']}）"
                + (f"：{list(acc['skipped'])}" if acc["skipped"] else ""),
            ], log=self._log)
            unexplained = acc["unexplained"]
            if unexplained:
                # ★ 不变量被破坏才会走到这里：说明还有航点既没登记到达、也没显式记账。
                #   修复前（t4 finding D1）这里会列出 7 个"静默丢点"；修复后应恒为空。
                banner([f"❌ 存在 {len(unexplained)} 个**无主航点**（既非到达、也非显式跳过）："
                        f"{' '.join(f'{route.waypoints[i].block}#{i}' for i in unexplained)}"],
                       char="!", log=self._log)
            else:
                self._log("✅ 逐点台账闭合：全部航点要么登记到达、要么显式跳过（无静默丢点）")
            # ★ 车体覆盖复核（只报不改）：t8 提出"雷 A9 心在 PIVOT 顶点差 12.7 cm、
            #   车体没盖住"。Mission 只知道"车离航点最近到过多少"，把这份数据摊开，
            #   供 t8/t10 用 0.127 m 之类的阈值逐雷对账 —— 本行**不影响**任何判定。
            uncovered = [(i, v) for i, v in self.closest_approaches().items()
                         if i >= 1 and v >= 0.0 and v > self._arrival_m]
            if uncovered:
                self._log(f"航点最近接近复核（min_dist > 到达半径 {self._arrival_m:.3f} m "
                          f"⇒ 车体未必盖住格心）："
                          + " ".join(f"#{i}{route.waypoints[i].block}={v:.3f}m"
                                     for i, v in uncovered))

    def _close_ledger(self) -> None:
        """收尾清账：任务结束时若还有航点"悬着"（s 越过但没到判跳过的余量），显式记账。

        finding D1 的"绝不静默丢点"在**任何退出路径**上都要成立 —— 不能只在
        "s 越窗"那一条分支上成立。收尾时把这些点一次性记为显式跳过并告警，
        使 `accounting()['unexplained']` 恒为空。
        """
        route = self._route
        if route is None or self._path is None:
            return
        last = len(route) - 1
        s = self._last_s
        # ★ 含末点：`_check_finish` 允许"弧长达标"就判完成（`s + arrival >= s_at_waypoint(last)`），
        #   那种情况下 `_passed[last]` 仍是 False —— 若把他排除在外，台账就永远闭不上
        #   （`unexplained` 里会永远吊着末点）。这里把他一并清账，并如实写明"车没接近"。
        for i in range(1, len(route)):
            if self._passed[i] or i in self._skipped_at:
                continue
            wp = route[i]
            over = s + self._arrival_m - self._path.s_at_waypoint(i)
            self._passed[i] = True
            self._cross_skips += 1
            tail = "（末点：由「弧长达标」判完成，车未接近）" if i == last else ""
            # ① 之前记过"越窗待定"的，用它自己的依据定案（含它当时看到的 min_dist/横向偏差）；
            # ② 其余（从没触发过越窗、也没到达的）用收尾清账的依据。
            detail = self._pending_skip.get(i) or (
                f"任务收尾清账（s 越过 {over:+.2f} m，最近只到 "
                f"{self._min_dist[i]:.3f} m > {self._close_m:.3f} m）")
            self._skipped_at[i] = detail
            self._pending_skip.pop(i, None)
            reason = "s_over_step" if "s 已越过" in detail else "close_ledger"
            self._log(f"⚠️ 逐点定案（显式记账）：航点 {wp.block}（#{i}）{detail}{tail}")
            self.bus.publish(EV_WAYPOINT_REACHED, source=self.name, index=i,
                             block=wp.block, forced_skip=True, reason=reason)

    @staticmethod
    def _fmt_duration(seconds: float) -> str:
        seconds = max(0.0, float(seconds))
        return f"{int(seconds // 60)}分{seconds % 60:.1f}秒"

    # ------------------------------------------------------------------ #
    #  外部命令（/nav/waypoint）：未知命令只告警不崩
    # ------------------------------------------------------------------ #
    def _drain_commands(self) -> None:
        try:
            commands = self.bridge.take_waypoint_commands(8)
        except Exception as exc:                    # noqa: BLE001  读命令失败不该杀任务
            if self._log_throttle.allow("cmd_read_failed", self._clock()):
                self._log(f"读取外部命令失败（忽略本拍）：{type(exc).__name__}: {exc}")
            return
        for raw in commands or []:
            self._cmd_stats["total"] += 1
            try:
                self._handle_command(raw)
            except Exception as exc:                # noqa: BLE001  ★ 一条坏命令不许崩
                self._cmd_stats["rejected"] += 1
                self._log(f"⚠️ 外部命令无效（已忽略）{raw!r}：{type(exc).__name__}: {exc}")

    def _handle_command(self, raw: Any) -> None:
        text = str(raw or "").strip()
        if not text:
            raise ValueError("空命令")
        low = text.lower()
        # ★ 外部命令载入的路线一律标"外部命令"来源（横幅会覆盖"自动规划/手动配置"），
        #   否则横幅会把 /nav/waypoint 下发的路线说成"启动时手动配置的 route.tokens"。
        self._route_source = ROUTE_SOURCE_COMMAND
        self._plan_info = {}

        if low.startswith("reset:"):
            self._reset_belief(text[len("reset:"):].strip())
            self._clear_route("reset 命令")
            self._write_phase(MissionPhase.IDLE)    # 复位后回到 IDLE，等下一单点/多点/自动出发
            return

        if low.startswith("load:"):
            spec = text[len("load:"):].strip()
            if not spec:
                raise ValueError("load: 后面缺少文件名")
            self._load_tokens(spec, source=f"load:{spec}")
            self._start_mission()
            return

        if low.startswith("shoot@"):
            raise ValueError("SHOOT@ 只能作为路线 token 出现，不能单独下发给 /nav/waypoint")

        # `单点 BLOCK,HEAD` → 单点路线（显式前缀，便于日志区分）
        for prefix in ("单点", "single", "goto"):
            if low == prefix or low.startswith(prefix + " ") or low.startswith(prefix + ":"):
                rest = text[len(prefix):].lstrip(" :").strip()
                self._load_single_point(rest)
                self._start_mission()
                return

        # `多点 ...` → 多航点路线（前缀只为可读性，内容与裸 token 列表一致）
        for prefix in ("多点", "multi", "route"):
            if low == prefix or low.startswith(prefix + " ") or low.startswith(prefix + ":"):
                rest = text[len(prefix):].lstrip(" :").strip()
                if not rest:
                    raise ValueError(f"{prefix} 后面缺少航点")
                self._load_tokens(self._normalize_command_tokens(rest), source="多点命令")
                self._start_mission()
                return

        tokens = self._normalize_command_tokens(text)
        if len(tokens) == 1:
            self._load_single_point(tokens[0])      # 裸 `BLOCK,HEAD` 也当单点
            self._start_mission()
            return
        self._load_tokens(tokens, source="waypoint命令")   # 裸多航点
        self._start_mission()

    @staticmethod
    def _normalize_command_tokens(text: str) -> list[str]:
        """`BLOCK,HEAD` / `A B C,y`（空格/分号/竖线分隔）→ token 列表。

        旧版 `waypoint_callback` 兼容 `BLOCK,HEAD`（逗号分隔）与空格分隔的串。
        朝向后缀 `,x|,y` 只允许挂在**最后一个** token 上（`parse_mission` 的硬约束），
        所以这里把误写在中间的裸 `x`/`y` 挪到最后，而不是直接报错。
        """
        body = text.replace(";", " ").replace("|", " ").strip()
        pieces = [p.strip() for p in body.replace(",", " ").split() if p.strip()]
        if not pieces:
            raise ValueError("命令里没有可解析的 token")
        tokens = pieces
        heads = {"x", "y"}
        moved: list[str] = []
        for tok in tokens:
            low = tok.lower()
            if low in heads:
                moved.append(low)
        if moved:
            tokens = [t for t in tokens if t.lower() not in heads]
            if not tokens:
                raise ValueError("只有朝向 token，缺少格名")
            tokens = tokens[:-1] + [tokens[-1] + "," + moved[-1]]
        return tokens

    def _clear_route(self, why: str) -> None:
        self._route = None
        self._path = None
        self._waypoint_count = 0
        self._passed = []
        self._passed_by_pivot = set()
        self._handled_stops = set()
        self._stopctx = None
        self._last_stop_state = StopPointState()
        self._stop_s = {}
        self._stop_blocks = []
        self._dist_to_next_stop = None
        self._finish_latched = False
        self._finish_done = False
        self._finish_info = {}
        self._last_s = 0.0
        self._route_s = 0.0
        self._set_fire_gate(False)
        self.bb.set(P_ROUTE, None, source=self.name)
        self.bb.update(self.name, **{P_ROUTE_INDEX: -1, P_ROUTE_S: 0.0,
                                     PIVOT_DONE_KEY: None})
        self._log(f"清空路线（{why}）")
