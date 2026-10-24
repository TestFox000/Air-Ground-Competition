# -*- coding: utf-8 -*-
"""车侧采纳门与任务流**镜像**（t6 BridgeIntel 口径，函数名锚定出处）。

镜像而非 import 的理由：``dzbrain.tasks.mission`` 属车侧主线（本任务只读、不 import
其运行时状态机）；本模块按 ``mission.py`` 的采纳顺序**复述**判定。出处一律用
**函数名锚定**（不写行号：车侧在途改动会让 file:line 全部漂移——2026-10-24 审计
实测 8 处行号出处全部失效，漂移 +46 ~ +325 行）。

## 镜像的判定（``tasks/mission.py``，函数名锚定）

- 采纳顺序 ``mission.py::_intel_of``：
  disabled（drone.enabled=false）→ no_link（无 P_DRONE）→ offline（无心跳，即使情报
  仍在过期窗口内也回落）→ missing（该半场从未收到）→ stale（age > ``drone.info_stale_s``）
  → rejected（``mission.py::_intel_cell_ok`` 校验失败）→ drone（新鲜情报采纳）。
- ``mission.py::_intel_cell_ok``：**现车已放宽**为 ①在 K 列 ②行号落在本半场行段
  （lower 7~11 / upper 1~6）两道；旧第三道"③在 ``mission.py::_bridge_pool``
  （候选 ∩ 地图桥洞）内"已删除（无人机情报可纠正过期地图；``route/planner.py``
  的 R008 同口径放宽：候选集合只用来挑缺省洞）。本模块：``authority_gate=True``
  = **现车语义**（只做①②）；``authority_gate=False`` 保留③ = t6 前旧口径**对照**
  （B2/B7 臂的"情报被拒"叙事即旧口径）。
- 发车窗口 ``mission.py::_should_wait_for_intel``：现车按 F1 裁决已收回判据
  （无链路 / 离线 → **不等待**）；本镜像保留"窗口照跑 2.0 s"的 t6 前口径
  （差异在 README 模型边界 4 如实标注）。
- 路线装载 ``mission.py::_load_startup_route``：情报新鲜的半场把格名透传给
  规划器（自动规划），否则走手动路线三连换洞（``mission.py::rewrite_crossing_tokens``
  形状校验 + ``mission.py::_swapped_tokens_ok`` 的 validate_route 0 error 门禁，
  本模块同款两道）；自动规划失败 → ``fallback_to_tokens`` 回退 route.tokens。
- ★ 执行中改道：**现车已实装**桥前等待 + 迟到情报改道一次（``mission.py`` 桥前等待段、
  ``mission.py::_bridge_wait_cap``）——本模块 ``wait_at_bridge`` 与其同型：穿洞入口格
  （approach = 穿洞三连前格，见 ``approach_cell``；现役 68-token 线下 L7 / 上 J5）前等
  "两半场情报新鲜"，等齐后重规划，上限 ``mission.wait_cap_s``；超限按"已得情报"出发。

## 对照与改进变体（台账显式标注）

- ``authority_gate``：True = 现车（采纳门①②）；False = t6 前池锁死口径（采纳门①②③）。
- ``wait_at_bridge``：与现车桥前等待同型；旧注记"现车代码没有"已按 2026-10-24
  对账更正。
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field

import yaml

from dzbrain.core.config import Config
from dzbrain.route.parser import parse_mission
from dzbrain.route.planner import plan_route_tracker_safe
from dzbrain.route.validate import validate_route
from dzbrain.tools.sim import parse_map_yaml

from .geo import block_to_index
from .link import DeliveredMessage
from .scenarios import Scenario

CROSS_LOWER_ROWS = (7, 11)
CROSS_UPPER_ROWS = (1, 6)
CROSS_COLUMN = "K"


def _pure_block(token: str) -> str | None:
    """token → 纯格名（剥 ``R:`` 前缀 / ``*`` 停站 / ``,y`` 朝向 / ``:PIVOT``）；非法 → None。"""
    text = str(token).strip().upper()
    base = text[2:] if text.startswith("R:") else text
    base = base.split(":")[0].strip("*").rstrip(",y").strip(",")
    if not base:
        return None
    try:
        block_to_index(base)
    except ValueError:
        return None
    return base


def approach_cell(tokens: list[str], half: str) -> str | None:
    """穿洞**入口格**（approach）= 本半场穿洞三连的前一格（桥前等待/侦察截止的决策点）。

    现役 68-token 线：下洞三连 ``M7 L7 K7 J7`` → lower 入口 = ``L7``；
    上洞三连 ``I5 J5 K5 L5 M5`` → upper 入口 = ``J5``。旧版硬编码 ``L10`` 在换线后
    会静默失效（2026-10-24 审计：B3 改道/等待整段死代码的根因），故一律现算。
    路线里找不到本半场穿洞格 → 返回 ``None``（调用方必须显式告警，不许静默跳过）。
    """
    band = CROSS_LOWER_ROWS if half == "lower" else CROSS_UPPER_ROWS
    toks = [str(t) for t in tokens]
    for idx, tok in enumerate(toks):
        base = _pure_block(tok)
        if base is None or not base.startswith(CROSS_COLUMN) or idx == 0:
            continue
        row = block_to_index(base)[0] + 1
        if not (band[0] <= row <= band[1]):
            continue
        return _pure_block(toks[idx - 1])
    return None


@dataclass
class BridgeRecord:
    half: str
    cell: str
    ts: float              # 车侧**收到时刻**（drone.py::_on_bridge_passable：载荷 ts 车侧不用）
    confidence: float = 0.0


@dataclass
class IntelVerdict:
    """一个半场某一时刻的采纳结论（镜像 ``BridgeIntel``，mission.py:233-266）。"""

    half: str
    cell: str              # 本次采用的穿洞格（恒非空 = 回落默认值）
    default: str
    source: str            # drone/default/offline/missing/stale/rejected/disabled/no_link
    raw: str
    reason: str
    age_s: float | None = None

    @property
    def from_drone(self) -> bool:
        return self.source == "drone"

    def describe(self) -> str:
        if self.from_drone:
            age = "（ts 差未知）" if self.age_s is None else f"（新鲜度 {self.age_s:.2f}s）"
            return f"{self.half} → {self.cell} ★无人机情报{age}；默认 {self.default}"
        return f"{self.half} → {self.cell}（{self.source}：{self.reason}）"


@dataclass
class GateEvent:
    t: float
    half: str
    source: str
    cell: str
    default: str
    reason: str


@dataclass
class RouteVersion:
    """一版执行路线（token 列表）及其来历。"""

    tokens: list[str]
    kind: str              # baseline / startup_adopted / bridge_replan
    crosses: dict[str, str]  # 本版实际采用的穿洞格
    note: str = ""


@dataclass
class MissionFlowResult:
    versions: list[RouteVersion] = field(default_factory=list)
    events: list[GateEvent] = field(default_factory=list)     # 采纳门事件日志
    wait_lower_s: float = 0.0
    wait_upper_s: float = 0.0
    commit_t: float = 0.0
    intel_complete_t: float | None = None
    reroute_count: int = 0
    adopted_changes: int = 0
    notes: list[str] = field(default_factory=list)


class CarIntelGate:
    """车侧采纳门镜像（无状态机；每拍可由 ``intel_snapshot`` 取结论）。"""

    def __init__(self, scenario: Scenario):
        self.sc = scenario
        self.records: dict[str, BridgeRecord] = {}
        self.online = False
        self.last_heartbeat = -1e9
        self.received: dict[str, list[float]] = {"heartbeat": [], "bridge_passable": [],
                                                 "target_zone": [], "field_verify": []}
        self.bad_messages = 0
        self.missing_fields = 0
        self.events: list[GateEvent] = []
        self._prev: dict[str, IntelVerdict] = {}
        self.adopted_changes = 0

    def feed(self, t: float, payload: dict) -> None:
        """一条已投递报文（镜像 ``DroneTask._handle_raw``，drone.py:204-289）。"""
        if not isinstance(payload, dict):
            self.bad_messages += 1
            return
        mtype = str(payload.get("type") or "").strip()
        if mtype == "heartbeat":
            self.online = True
            self.last_heartbeat = t
        elif mtype == "bridge_passable":
            half = str(payload.get("half") or "").strip().lower()
            block = str(payload.get("block") or "").strip().upper()
            if half not in ("lower", "upper") or not block:
                self.missing_fields += 1
                return
            conf = float(payload.get("confidence") or 0.0)
            self.records[half] = BridgeRecord(half, block, t, conf)
        self.received.setdefault(mtype, []).append(t)

    def intel_snapshot(self, t: float) -> dict[str, IntelVerdict]:
        """两半场结论（镜像 mission.py:1490-1524 + 1391-1410）。"""
        out: dict[str, IntelVerdict] = {}
        for half in ("lower", "upper"):
            default = self.sc.defaults.get(half, "")
            if not self.sc.mission.drone_enabled:
                v = IntelVerdict(half, default, default, "disabled", "",
                                 "drone.enabled=false → 不读情报")
            elif self.last_heartbeat < 0:
                v = IntelVerdict(half, default, default, "no_link", "",
                                 "未收到任何无人机报文（无链路）→ 用默认格")
            elif t - self.last_heartbeat > self.sc.mission.heartbeat_timeout_s:
                v = IntelVerdict(half, default, default, "offline", "",
                                 f"无人机离线（心跳超 {self.sc.mission.heartbeat_timeout_s:g}s）"
                                 f"→ 用默认格", t - self.last_heartbeat)
            elif half not in self.records:
                v = IntelVerdict(half, default, default, "missing", "",
                                 "该半场从未收到 bridge_passable → 用默认格")
            else:
                rec = self.records[half]
                age = t - rec.ts
                if age > self.sc.mission.info_stale_s:
                    v = IntelVerdict(half, default, default, "stale", rec.cell,
                                     f"情报已过期 {age:.2f}s > {self.sc.mission.info_stale_s:g}s"
                                     f" → 用默认格", age)
                else:
                    ok, why = self._cell_ok(half, rec.cell)
                    if not ok:
                        v = IntelVerdict(half, default, default, "rejected", rec.cell,
                                         f"情报 {rec.cell} 不可用（{why}）→ 用默认格", age)
                    else:
                        v = IntelVerdict(half, rec.cell, default, "drone", rec.cell,
                                         f"新鲜情报（{age:.2f}s < 过期阈值 "
                                         f"{self.sc.mission.info_stale_s:g}s）", age)
            out[half] = v
        for half, v in out.items():
            prev = self._prev.get(half)
            if prev is None or (prev.source, prev.cell) != (v.source, v.cell):
                self.events.append(GateEvent(t, v.half, v.source, v.cell, v.default, v.reason))
                self.adopted_changes += 1
                self._prev[half] = v
        return out

    def _cell_ok(self, half: str, cell: str) -> tuple[bool, str]:
        """镜像 mission.py:1391-1410（authority 模式跳过 ③候选池）。"""
        text = str(cell or "").strip().upper()
        if not text:
            return False, "格名为空"
        try:
            row_idx, col_idx = block_to_index(text)
        except ValueError as exc:
            return False, f"格名非法（{exc}）"
        if text[0] != CROSS_COLUMN:
            return False, f"不在 {CROSS_COLUMN} 列（桥洞列）上"
        band = CROSS_LOWER_ROWS if half == "lower" else CROSS_UPPER_ROWS
        row = row_idx + 1
        if not (band[0] <= row <= band[1]):
            return False, f"第 {row} 行不属于{'下' if half == 'lower' else '上'}半场（{band[0]}~{band[1]} 行）"
        if not self.sc.authority_gate:
            pool = self.sc.pool(half)
            if text not in pool:
                return False, f"不在本半场候选集合（{' '.join(pool)}）里"
        return True, ""


class MissionFlow:
    """任务流镜像：发车窗口 → 路线装载（规划器/回退）→ （可选）桥前等待重规划。"""

    def __init__(self, scenario: Scenario, cfg: Config, baseline_tokens: list[str]):
        self.sc = scenario
        self.cfg = cfg
        self.baseline = baseline_tokens
        self.gate = CarIntelGate(scenario)
        self.result = MissionFlowResult()
        #: 真值桥洞行（用于物理场 FieldSpec——由 runner 构造）
        self._planner_calls = 0

    # ------------------------------------------------------------------ #
    #  发车窗口（镜像 _should_wait_for_intel / _load_startup_route）
    # ------------------------------------------------------------------ #
    def commit_decision(self, delivered: list[DeliveredMessage]) -> tuple[float, dict[str, IntelVerdict]]:
        """按时间序喂报文（严格喂到决策时刻为止），返回 (发车时刻, 发车时两半场结论)。

        - 有链路（收到过任何报文）且两半场未齐 → 等到齐或窗口耗尽（2.0 s）；
        - 无链路（无任何报文）→ 窗口照跑，窗口耗尽即发车（镜像 mission.py:1557-1572：
          DroneTask 在跑但 online=False 也照常等待）。
        """
        wait_s = self.sc.mission.bridge_info_wait_s
        complete_t: float | None = None
        for msg in delivered:
            if msg.recv_t > wait_s + 1e-9:
                break
            self.gate.feed(msg.recv_t, msg.payload)
            snap = self.gate.intel_snapshot(msg.recv_t)
            if complete_t is None and all(v.from_drone for v in snap.values()):
                complete_t = msg.recv_t
                break                       # 两半场齐了 → 立即发车（镜像 1572）
        commit = complete_t if complete_t is not None else wait_s
        self.result.commit_t = commit
        self.result.intel_complete_t = complete_t
        snap = self.gate.intel_snapshot(commit)
        return commit, snap

    # ------------------------------------------------------------------ #
    #  路线装载
    # ------------------------------------------------------------------ #
    def load_route(self, snap: dict[str, IntelVerdict]) -> RouteVersion:
        """镜像 ``_load_startup_route``（mission.py:1782-1859）。"""
        cross = {half: (snap[half].cell if snap[half].from_drone else None)
                 for half in ("lower", "upper")}
        if self.sc.mission.auto_plan_enabled:
            tokens, crosses, note = self._plan(cross)
            kind = "baseline" if crosses == {"lower": self.sc.defaults["lower"],
                                             "upper": self.sc.defaults["upper"]} \
                else "startup_adopted"
        else:
            tokens, crosses, note = self._manual_rewrite(list(self.baseline), cross)
            kind = "startup_adopted" if crosses else "baseline"
        return RouteVersion(tokens=tokens, kind=kind, crosses=crosses, note=note)

    def _repo_map_cfg(self) -> dict | None:
        """只读仓库 ``config/map.yaml`` 的内存副本；靶格 patch 跟情景真值走（t28③）。

        :return: ``{'map': {...}}`` dict；文件不可读 → None（调用方保守回退）。
        """
        import os as _os
        _repo = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                                               "..", "..", "..", "dzrobot_v2"))
        try:
            with open(_os.path.join(_repo, "config", "map.yaml"), encoding="utf-8") as fh:
                map_cfg = yaml.safe_load(fh)
        except OSError:
            return None
        tgt = sorted(set(self.sc.truth.targets))
        if tgt:
            map_cfg.setdefault("map", {})["targets"] = tgt
        return map_cfg

    def _swapped_tokens_ok(self, tokens: list[str], half: str, old: str,
                           new: str) -> tuple[bool, str]:
        """换洞后的整线 token 列表能否执行：``validate_route`` **0 error** 才放行。

        镜像 ``mission.py::_swapped_tokens_ok``（换洞会挪三格，可能把路线挪到雷/障上
        或制造斜切入洞，不能靠"看上去对"来判）。
        """
        map_cfg = self._repo_map_cfg()
        if map_cfg is None:
            return False, "本地地图不可用 → 不做换洞（保守：读得到地图才允许改路线）"
        try:
            route = parse_mission([str(t) for t in tokens])
        except Exception as exc:                     # noqa: BLE001
            return False, f"换洞后路线解析失败（{type(exc).__name__}: {exc}）"
        blocks = [w.block for w in route.waypoints]
        if new not in blocks or old in blocks:
            return False, f"换洞后 token 列表不符（{old} 仍在 / {new} 缺失）"
        try:
            issues = validate_route(route, map_cfg=map_cfg, cfg=self.cfg)
        except Exception as exc:                     # noqa: BLE001
            return False, f"换洞后校验异常（{type(exc).__name__}: {exc}）"
        errors = [str(i) for i in issues if getattr(i, "level", "") == "error"]
        if errors:
            return False, "换洞后校验未过：" + "；".join(errors[:3])
        return True, ""

    def _plan(self, cross: dict[str, str | None]) -> tuple[list[str], dict[str, str], str]:
        """调用**真规划器** ``plan_route_tracker_safe``（只读）。

        authority 模式：内存副本把池扩充为 候选 ∪ {情报格}（现规划器 R008 已放宽，
        此扩池主要影响缺省洞挑选）；非 authority 模式：t6 前口径（候选∩地图桥洞，
        情报格不在池内 → 情报在采纳门已判 rejected，这里 cross=None 走规划器默认）。
        """
        sc = self.sc
        map_cfg = self._repo_map_cfg() or {"map": {}}
        flat = dict(self.cfg._flat)
        if sc.authority_gate:
            for half in ("lower", "upper"):
                cell = cross[half]
                if cell:
                    flat[f"route.cross_{half}_candidates"] = sorted(
                        set(flat.get(f"route.cross_{half}_candidates") or []) | {cell})
                    bridges = list(map_cfg["map"].get("bridges") or [])
                    if cell not in bridges:
                        map_cfg["map"]["bridges"] = bridges + [cell]
        cfg2 = Config(flat, source=f"{self.cfg.source} + authority(intel)")
        try:
            res = plan_route_tracker_safe(map_cfg, cfg2,
                                          cross_lower=cross.get("lower"),
                                          cross_upper=cross.get("upper"),
                                          max_seconds=30.0)
        except Exception as exc:                     # noqa: BLE001  规划失败 → 手动回退
            tokens, crosses, note = self._manual_rewrite(list(self.baseline), cross)
            return tokens, crosses, f"规划器失败（{type(exc).__name__}: {exc}）→ 回退手动换洞：{note}"
        self._planner_calls += 1
        crossing = dict(res.crossing or {})
        crosses = {"lower": str(crossing.get("lower", "")), "upper": str(crossing.get("upper", ""))}
        return [str(t) for t in res.tokens], crosses, \
            f"tracker-safe 规划（{len(res.tokens)} token）穿洞 {crosses['lower']}/{crosses['upper']}"

    def _manual_rewrite(self, tokens: list[str],
                        cross: dict[str, str | None]) -> tuple[list[str], dict[str, str], str]:
        """镜像 ``mission.py::rewrite_crossing_tokens`` 三连换洞 + ``::_swapped_tokens_ok``
        validate 门禁（每个半场独立：形状不符 / 边界断链 / 校验未过 → 该半场不换）。
        """
        crosses: dict[str, str] = {}
        for half, cell in cross.items():
            if not cell:
                continue
            old = self._detect_cross(tokens, half)
            if not old or old == cell:
                continue
            new = _rewrite_triple(tokens, old, cell)
            if new is None:
                continue
            ok, why = self._swapped_tokens_ok(new, half, old, cell)
            if not ok:
                print(f"⚠️ [{self.sc.name}] 换洞 {old}→{cell} 校验未过 → 该半场走默认 {old}"
                      f"（{why}）", file=sys.stderr)
                continue
            tokens = new
            crosses[half] = cell
        return tokens, crosses, "手动三连换洞（形状不符/校验未过的半场不换）"

    @staticmethod
    def _detect_cross(tokens: list[str], half: str) -> str | None:
        band = CROSS_LOWER_ROWS if half == "lower" else CROSS_UPPER_ROWS
        for tok in tokens:
            text = str(tok).strip().upper()
            base = text.strip("*").rstrip(",y").strip(",")
            if base.startswith(CROSS_COLUMN):
                try:
                    row = block_to_index(base)[0] + 1
                except ValueError:
                    continue
                if band[0] <= row <= band[1]:
                    return base
        return None

    # ------------------------------------------------------------------ #
    #  桥前等待（改进变体：现车代码没有）
    # ------------------------------------------------------------------ #
    def bridge_wait_decision(self, joint_arrival_approach: float,
                             delivered: list[DeliveredMessage]) -> RouteVersion | None:
        """穿洞**入口格**（approach = 穿洞三连前格，见 ``approach_cell``）前：
        等两半场情报新鲜（≤上限），齐了重规划；超限按已得情报。

        与现车 ``mission.py`` 桥前等待段（``_bridge_wait_cap``）同型。

        :param joint_arrival_approach: 车到达入口格的 joint 时刻（启动窗口已含）。
            现役 68-token 线 lower 入口 = L7（旧版硬编码 L10 在换线后静默失效，已废）。
        :return: 新路线版本（若与现版本不同）；无需改道 → None。
        """
        if not self.sc.wait_at_bridge:
            return None
        cap = self.sc.mission.wait_cap_s
        horizon = joint_arrival_approach + cap
        complete_t: float | None = None
        for msg in delivered:
            if msg.recv_t > horizon + 1e-9:
                break
            self.gate.feed(msg.recv_t, msg.payload)
            snap = self.gate.intel_snapshot(msg.recv_t)
            if complete_t is None and all(v.from_drone for v in snap.values()):
                complete_t = msg.recv_t
                break                       # 两半场齐了 → 立即决策
        t_decision = complete_t if complete_t is not None else horizon
        snap0 = self.gate.intel_snapshot(joint_arrival_approach)
        if complete_t is not None:
            wait = max(0.0, complete_t - joint_arrival_approach)
        else:
            wait = cap if not all(v.from_drone for v in snap0.values()) else 0.0
        self.result.wait_lower_s += wait
        snap = self.gate.intel_snapshot(t_decision)
        cross = {half: (snap[half].cell if snap[half].from_drone else None)
                 for half in ("lower", "upper")}
        if all(c is None for c in cross.values()):
            return None                       # 什么情报都没有 → 维持现路线（默认格直冲）
        tokens, crosses, note = self._plan(cross)
        current = self.result.versions[-1].tokens
        if [str(t) for t in tokens] == [str(t) for t in current]:
            return None
        return RouteVersion(tokens=[str(t) for t in tokens], kind="bridge_replan",
                            crosses=crosses, note=f"L10 桥前重规划：{note}")


def _rewrite_triple(tokens: list[str], old: str, new: str) -> list[str] | None:
    """镜像 ``mission.py::rewrite_crossing_tokens``：三连同行换行。

    ★ v6 审计补充（第六图 B3 断链教训）：三连整体挪行会改动两个**边界对**
    （i-2→i-1 与 i+1→i+2）。改写后先做 **8 邻快筛**：边界对跳格（切比雪夫距离 ≥2，
    如现役线的 ``M7→L9``）= 物理断链 → 拒绝改写返回 None；半格斜切（切比雪夫 1，
    如 ``I5→J4``）放行，交由 ``validate_route``（R009 斜切入洞 ≤10° 等）与执行层裁决
    ——与 ``mission.py::_swapped_tokens_ok`` 的"真校验放行"口径一致。
    """
    hits = [i for i, t in enumerate(tokens)
            if str(t).strip().upper().strip("*").rstrip(",y").strip(",") == old]
    if len(hits) != 1:
        return None
    i = hits[0]
    if i == 0 or i + 1 >= len(tokens):
        return None
    try:
        old_row, old_col = block_to_index(old)
        new_row = block_to_index(new)[0]
        k_col = block_to_index(CROSS_COLUMN + "1")[1]
    except ValueError:
        return None
    if old_col != k_col:
        return None
    out = list(tokens)
    for j in (i - 1, i, i + 1):
        text = str(out[j]).strip().upper()
        # 后缀（`*` 停站 / `,y` 朝向 / `R:` 前缀）只出现在洞格本身，三连两侧是纯格名；
        # 这里按"洞格保留后缀、邻格纯格名"的现役路线形状处理（现役 68 token：`… L7 K7 J7 …`）。
        prefix = "R:" if text.startswith("R:") else ""
        base = text[2:] if prefix else text
        base = base.strip("*").rstrip(",y").strip(",")
        suffix = text[len(prefix) + len(base):]
        try:
            row, col = block_to_index(base)
        except ValueError:
            return None
        if row != old_row:
            return None
        if j != i and (col == k_col or abs(col - k_col) != 1):
            return None
        new_base = f"{'ABCDEFGHIJKLMN'[col]}{new_row + 1}"
        out[j] = prefix + new_base + (suffix if j == i else "")
    # ★ 边界对 8 邻快筛（改写后）：三连整体挪行只可能破坏 i-2→i-1 与 i+1→i+2 两对，
    #   跳格（切比雪夫 ≥2）= 断链，物理不可达 → 拒绝（如现役线 K7→K9 的 M7→L9）。
    for j_edge, k_edge in ((i - 1, i - 2), (i + 1, i + 2)):
        if k_edge < 0 or k_edge >= len(out):
            continue
        b_in, b_out = _pure_block(out[j_edge]), _pure_block(out[k_edge])
        if b_in is None or b_out is None:
            continue                     # 无法解析（PIVOT 等）→ 交给 validate_route 裁决
        r_in, c_in = block_to_index(b_in)
        r_out, c_out = block_to_index(b_out)
        chebyshev = max(abs(r_in - r_out), abs(c_in - c_out))
        if chebyshev > 1:
            print(f"⚠️ 三连换洞 {old}→{new} 拒绝：边界对 {b_out}->{b_in} 跳格断链"
                  f"（切比雪夫距离 {chebyshev} > 1），整线不换该洞", file=sys.stderr)
            return None
    return out
