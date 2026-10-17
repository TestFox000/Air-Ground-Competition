# -*- coding: utf-8 -*-
"""车侧采纳门与任务流**镜像**（t6 BridgeIntel 口径，逐条 file:line 出处）。

镜像而非 import 的理由：``dzbrain.tasks.mission`` 属车侧主线（本任务只读、不 import
其运行时状态机）；本模块按 ``mission.py`` 的采纳顺序**复述**判定，所有行号出处写在注释里。

## 镜像的判定（``tasks/mission.py``）

- 采纳顺序 ``_intel_of``（mission.py:1490-1524）：
  disabled（drone.enabled=false）→ no_link（无 P_DRONE）→ offline（无心跳，即使情报
  仍在过期窗口内也回落）→ missing（该半场从未收到）→ stale（age > ``drone.info_stale_s``）
  → rejected（``_intel_cell_ok`` 三道校验失败）→ drone（新鲜情报采纳）。
- ``_intel_cell_ok``（mission.py:1391-1410）：① 在 K 列；② 行号落在本半场行段
  （lower 7~11 / upper 1~6）；③ 在 ``_bridge_pool``（候选集合 ∩ 地图桥洞）内。
- 发车窗口 ``_should_wait_for_intel``（mission.py:1557-1572）：有无人机链路且两半场情报
  未齐时，最多等到 ``mission.bridge_info_wait_s``（本图 2.0 s）；无链路 → 不等。
- 路线装载 ``_load_startup_route``（mission.py:1782-1859）：情报新鲜的半场把格名透传给
  规划器（``cross_lower/cross_upper``），否则 None（规划器走默认格）；自动规划失败 →
  ``fallback_to_tokens`` 回退 route.tokens 并做三连换洞（``_apply_intel_to_tokens``）。
  ★ 关键事实：**执行中的 token 列表只在发车时定一次**——桥前改道在本镜像里是
  ``wait_at_bridge`` 改进变体（现车代码没有）。

## 改进变体（本包的建模，标注非现车行为）

- ``authority_gate``：采纳门去掉 ③（候选∩地图桥洞锁死），只做 ①列 ②行段；规划器调用时
  把池扩充为 候选 ∪ {情报格}（等价地：给规划器喂一份 bridges 含情报格的内存地图）。
  —— 依据：planner.py:2159-2164 要求显式穿洞格必须在池内，否则 PlannerError；而池来自
  ``route.cross_*_candidates ∩ map.bridges``，桥洞每场随机后地图必过期 → 现车代码
  **任何候选配置都无法采纳与地图矛盾的无人机情报**。
- ``wait_at_bridge``：K 列三连入口格（L10）前等待"两半场情报都新鲜"，等齐后重规划
  （一次覆盖两处穿洞），上限 ``mission.wait_cap_s``；超限按"已得情报"出发（入口新鲜则
  换入口、出口走默认）。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import yaml

from dzbrain.core.config import Config
from dzbrain.route.planner import plan_route_tracker_safe
from dzbrain.tools.sim import parse_map_yaml

from .geo import block_to_index
from .link import DeliveredMessage
from .scenarios import Scenario

CROSS_LOWER_ROWS = (7, 11)
CROSS_UPPER_ROWS = (1, 6)
CROSS_COLUMN = "K"


@dataclass
class BridgeRecord:
    half: str
    cell: str
    ts: float              # 车侧**收到时刻**（drone.py:284：载荷 ts 车侧不用）
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

    def _plan(self, cross: dict[str, str | None]) -> tuple[list[str], dict[str, str], str]:
        """调用**真规划器** ``plan_route_tracker_safe``（只读）。

        authority 模式：内存副本把池扩充为 候选 ∪ {情报格}（等价于放宽 planner.py:2159-2164
        的池校验）；非 authority 模式：现车口径（候选∩地图桥洞，情报格不在池内 → 情报
        在采纳门已判 rejected，这里 cross=None 走规划器默认）。
        """
        sc = self.sc
        import os as _os
        _repo = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                                               "..", "..", "..", "dzrobot_v2"))
        map_cfg = yaml.safe_load(open(_os.path.join(_repo, "config", "map.yaml"),
                                      encoding="utf-8"))
        # t28 ③：规划器地图里的靶格跟情景真值走（默认情景 == map.yaml 靶格，逐位不变；
        # 覆盖后规划器绕开新靶格——只改内存副本，绝不落 config/map.yaml）
        tgt = sorted(set(sc.truth.targets))
        if tgt:
            map_cfg["map"]["targets"] = tgt
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
        """镜像 ``rewrite_crossing_tokens``（mission.py:288-343）三连换洞。"""
        crosses: dict[str, str] = {}
        for half, cell in cross.items():
            if not cell:
                continue
            old = self._detect_cross(tokens, half)
            if not old or old == cell:
                continue
            new = _rewrite_triple(tokens, old, cell)
            if new is not None:
                tokens = new
                crosses[half] = cell
        return tokens, crosses, "手动三连换洞（形状不符的半场不换）"

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
    def bridge_wait_decision(self, joint_arrival_l10: float,
                             delivered: list[DeliveredMessage]) -> RouteVersion | None:
        """L10 入口格前：等两半场情报新鲜（≤上限），齐了重规划；超限按已得情报。

        :param joint_arrival_l10: 车到达 L10 的 joint 时刻（启动窗口已含）。
        :return: 新路线版本（若与现版本不同）；无需改道 → None。
        """
        if not self.sc.wait_at_bridge:
            return None
        cap = self.sc.mission.wait_cap_s
        horizon = joint_arrival_l10 + cap
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
        snap0 = self.gate.intel_snapshot(joint_arrival_l10)
        if complete_t is not None:
            wait = max(0.0, complete_t - joint_arrival_l10)
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
    """镜像 ``rewrite_crossing_tokens``（mission.py:288-343）：三连同行换行。"""
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
        # 这里按"洞格保留后缀、邻格纯格名"的现役路线形状处理（t31 路线 N10 M10 L10 K10 J10…）。
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
    return out
