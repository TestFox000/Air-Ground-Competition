# -*- coding: utf-8 -*-
r"""DroneTask —— 无人机链路（5 Hz），架构 §7.2 / §7.3 的车侧消费端。

无人机是**硬输入**：只有它能从空中看到桥顶标识（每局随机的一个桥洞格），
以及全场靶区/雷/障的布置。所以本任务的职责是"把情报变成可用的状态"，
**不做路线决策**（选格是 MissionTask/RouteBuilder 的事，见 §7.3）：

| 方向 | 载荷 | 本任务的动作 |
|---|---|---|
| 收 | `heartbeat` | 更新心跳时间；超时/恢复各发一次 `EV_DRONE_OFFLINE`/`EV_DRONE_ONLINE` |
| 收 | `bridge_passable` `{half,block,confidence,ts}` | 写 `P_DRONE`（`bridge_of()` 供 Mission 取格） |
| 收 | `target_zone` `{blocks:[...]}` | 与本地 `config/map.yaml` 的 `targets` 比对 → `field_diff` |
| 收 | `field_verify` `{mines,obstacles,targets?,coverage?,slice?,scan_id?}` | 同上（雷/障/靶差异）；带 `coverage` 时按**片内作用域**判定负证据 + 记覆盖账本（G5-2） |
| 收 | `field_layout` `{version,start,end,mines,obstacles,targets,bridges,barrier_*}` | 校验后写 `P_FIELD_LAYOUT`（**规划真源**；车端不再读地图文件，见下节） |
| 收 | 未知 `type` / 非法 JSON / 缺字段 | **记日志不崩**（旧代码"一条坏消息杀一个回调"的教训） |
| 发 | `car_state` `{type,block,yaw_deg,phase,ts,seq}` | 1~2 Hz 周期上报（block 从 `P_POSE` 推） |
| 发 | `ack` `{type:"ack",of:seq}` | 对带 `seq` 的消息回执 |

## 覆盖语义与 `EV_FIELD_VERIFY` 消费方（G5-2 / G5-3，本卡新增）

机侧 t5a 已在 `field_verify` 上附带三个 **additive** 键（`dzsmartscout/tools/perception/protocol.py`）：
`coverage{cells,complete,mode}` / `slice{id,index,of,route}` / `scan_id`。本任务在车侧消费它们：

* **覆盖账本** `CoverageLedger`：按 `(scan_id, slice.id)` 幂等去重（BEST_EFFORT 必须周期重发）；
  只有 `complete=True` 的片才把 `cells` 记入「已探明」，比例分母 = `geometry.MAP_COLS*MAP_ROWS`（154）。
  `complete=False` ⇒ 片内其余仍是 UNKNOWN，**不产生「本地要素消失」的差异**（负证据只在完整片里成立）。
* **核对消费方** `FieldVerifyConsumer`：订阅 `EV_FIELD_VERIFY`（生产订阅方，不再"零订阅"），
  把雷/障/靶差异折算成决策可见汇总，由本任务写入黑盒键 `P_FIELD_VERIFY` / `P_DRONE_COVERAGE`。
* 处置策略 `drone.field_verify_policy` ∈ `off|record|alert|hold`（**缺省 record = 只记录不阻断**）：
  `off` 关闭消费方（**不是总开关**；与 `coverage_enabled=false` 同时设置时，行为与加特性前
  逐位一致，A/B 反证用的就是这个组合）；`alert`/`hold` 只在汇总里置位 `alert`/`hold`
  供上层裁决，本任务**不自己停车、不改路线**。
* 覆盖开关 `drone.coverage_enabled`（缺省 true）：false ⇒ 完全忽略 `coverage` 字段（旧行为）。

⚠️ **上面两键不在 `core/config.py` 的 SCHEMA 里**（本卡写域不含它）：一律 `cfg.get(key, default)`；
   现场不写 `robot.yaml` 时按缺省值生效。建议的 SCHEMA 规格见 `_tmp_session/drone_gap/REPORT.md`。

★ 失效兜底（§7.2）：心跳丢失 > `drone.heartbeat_timeout_s`（默认 3 s）
  → `online=False` + 一次告警；**任务不阻塞、不改变默认分支/默认地图**
  （"宁可漏打，不可卡死"）。情报过期 > `drone.info_stale_s` → `bridge_of()` 返回 None
  （由 `DroneState.bridge_of` 现成实现，本任务只负责把 `bridge_ts` 写准）。
★ `drone.enabled=False` → **整任务空转**：不读 `bridge`、不发 `bridge`、不写 `P_DRONE`。

## 场地布局 `field_layout`（车端不留地图文件）

用户 2026-10-18 定版：「车机内不留地图文件，跑图的规划依据由无人机下发信息」。
本任务只做**接收 + 校验 + 落黑盒**（`P_FIELD_LAYOUT`）；规划是装配侧/任务侧的职责
（`main.plan_route_from_field_layout()` 读该键 → `planner.load_map(dict)` → `plan_route`）。
**没有布局就不规划、不出发**；绝不回退到本地 `config/map.yaml` / 旧 `route.tokens`。

报文（JSON over `drone_in`，未知字段忽略）：

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `type` | str | ✅ | 固定 `"field_layout"` |
| `version` | int | - | 布局协议版本（缺省 0；高于车端支持版本只告警，已知字段照收） |
| `ts` | float | - | 机侧时间戳（仅证据；坏值按 0 处理，不影响接收） |
| `start` / `end` | 格名 | ✅ | 起终点（必须在 A~N × 1~11 内） |
| `mines` / `obstacles` / `targets` | [格名] | ✅ | 可为空列表，但**键必须存在**（半张布局不能拿去规划） |
| `bridges` | [格名] | ✅ | **至少 1 个**（本局穿洞格） |
| `barrier_col_from` | "D" | - | 隔板起始列（缺省 `geo.BARRIER_COL_FROM`） |
| `barrier_row_hi` / `barrier_row_lo` | int | - | 隔板两侧行号（缺省 6/7；必须 1~11 且 `hi < lo`） |

★ 拒收语义：任一必填/格式字段非法 ⇒ **整份拒收**（`layout_rejects+1` + 节流日志），
  **保留上一份好布局**（坏报文绝不清空、不覆盖）。校验纯内存、不依赖本地地图。

## 本地地图从哪来（★ 接口现状，需要装配侧知道）

`main.build_tasks()` 目前只传 `(bridge, bb, bus, cfg)`，**不传 `map_cfg`**，
所以本任务按以下顺序自取本地地图（与 `main.load_and_check` 的默认路径一致）：
  1. 构造参数 `local_map=`（测试/将来装配侧显式注入，最优先）；
  2. `cfg.get("drone.map_path")` 指定的 YAML 文件；
  3. 仓库默认 `<repo>/config/map.yaml`（`src/dzbrain/dzbrain/tasks/drone.py` 的上溯 4 层）。
三个来源都没有 → `map_loaded=False`，只记一条日志，**核对功能降级为"不核对"**
（绝不因为拿不到地图就让任务起不来）。

⚠️ 本卡新增两键**未登记 SCHEMA**（写域不含 `core/config.py`，一律 `cfg.get(key, default)`）：
  `drone.field_verify_policy`（缺省 `record`）/ `drone.coverage_enabled`（缺省 true）。
  `drone.map_path` / `drone.field_diff_max` / `drone.ack_enabled` 已随其它卡登记进 SCHEMA；
  `drone.enabled` / `topic_in` / `topic_out` / `heartbeat_timeout_s` / `info_stale_s` 亦在。
"""

from __future__ import annotations

import json
import pathlib
from typing import Any, Callable, Iterable

from ..core import geometry as geo
from ..core.blackboard import (P_DRONE, P_MISSION_PHASE, P_POSE, Blackboard)  # noqa: F401
from ..core.config import Config
from ..core.events import (EV_BRIDGE_INFO, EV_DRONE_OFFLINE, EV_DRONE_ONLINE,
                           EV_FIELD_VERIFY, EventBus)
from ..core.logging import Throttle
from ..core.state import DroneState
from ..core.task import TaskThread
from ..io import contract

__all__ = ["DroneTask", "DRONE_PAYLOAD_TYPES", "default_map_path",
           "CoverageLedger", "FieldVerifyConsumer",
           "FIELD_VERIFY_POLICIES", "P_FIELD_VERIFY", "P_DRONE_COVERAGE",
           "P_FIELD_LAYOUT", "FIELD_LAYOUT_VERSION", "normalize_field_layout"]

#: 已知的无人机报文类型（其余一律"未知类型"，记日志不崩）
DRONE_PAYLOAD_TYPES: tuple[str, ...] = (
    "heartbeat", "bridge_passable", "target_zone", "field_verify", "field_layout",
)

_BRIDGE_HALVES = ("lower", "upper")
_INF = float("inf")


def default_map_path() -> pathlib.Path:
    """仓库默认地图路径：`<repo>/config/map.yaml`。

    ★ 层数不是猜的：本文件在 `<repo>/src/dzbrain/dzbrain/tasks/drone.py`，
      所以 `parents` = [tasks, dzbrain, dzbrain(pkg 根), src, <repo>] → `parents[4]` 才是仓库根。
      （`main.py` 在 `<repo>/src/dzbrain/dzbrain/main.py`，它的 `parents[3]` 正好是仓库根
       —— 两个模块深度不同，**不能照抄同一个下标**；下面还带一道存在性校验兜底。）
    """
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "config" / "map.yaml"
        if candidate.is_file():
            return candidate
    return here.parents[4] / "config" / "map.yaml"


# --------------------------------------------------------------------------- #
#  场地布局 field_layout（车端不留地图文件 → 规划依据由无人机下发）
# --------------------------------------------------------------------------- #
#: 车侧黑盒键：无人机下发的**完整场地布局**（dict，形状见 `normalize_field_layout`）。
#: ★ 键名常量暂时放在本模块（与 `P_FIELD_VERIFY` 同一做法，本卡写域不含
#:   `core/blackboard.py`）；语义稳定后应迁到那里统一登记。
P_FIELD_LAYOUT = "field_layout"

#: 车端认识的 `field_layout` 协议版本上限。更高版本**只告警**并照收已知字段
#: （前向兼容：新增字段默认被忽略，不会让整车停摆）。
FIELD_LAYOUT_VERSION = 1

#: `field_layout` 里必须出现（可为空表）的格名列表键：半张布局不能拿去规划，
#: 缺键 = 拒收，而不是"按空表猜"。
_LAYOUT_CELL_KEYS: tuple[str, ...] = ("mines", "obstacles", "targets")


def _layout_cells(value: Any, key: str, *,
                  allow_empty: bool = True) -> tuple[list[str] | None, str]:
    """归一化一个格名列表字段 → `(大写去重列表, 错误原因)`；非法返回 `(None, 原因)`。"""
    if not isinstance(value, (list, tuple)):
        return None, f"{key} 必须是格名列表（收到 {type(value).__name__}）"
    out: list[str] = []
    seen: set[str] = set()
    for item in value:
        if not isinstance(item, str):
            return None, f"{key} 含非字符串条目 {item!r}"
        text = item.strip().upper()
        if not text:
            return None, f"{key} 含空格名"
        try:
            geo.block_to_index(text)
        except ValueError as exc:
            return None, f"{key} 含非法格名 {text!r}（{exc}）"
        if text not in seen:
            seen.add(text)
            out.append(text)
    if not out and not allow_empty:
        return None, f"{key} 不能为空"
    return out, ""


def normalize_field_layout(payload: Any) -> tuple[dict[str, Any] | None, str]:
    """校验并归一化 `field_layout` 报文 → `(layout, reason)`（成功时 reason == ""）。

    ★ 纯内存、无 I/O、不依赖本地地图（车端可以完全没有 `config/map.yaml`）。
    ★ 只取规划需要的字段；**未知字段一律忽略**（机侧加字段不该让车侧拒收）。
    ★ 任一必填/格式字段非法 ⇒ 整份拒收（返回 `None` + 原因）：宁可不更新布局，
      也不要把"缺了雷/障的半张图"喂给规划器（那会产出漏压雷/撞桶的路线）。
    """
    if not isinstance(payload, dict):
        return None, f"报文不是 JSON 对象（{type(payload).__name__}）"
    out: dict[str, Any] = {}

    raw_version = payload.get("version", 0)
    try:
        out["version"] = int(raw_version if raw_version is not None else 0)
    except (TypeError, ValueError):
        return None, f"version 不是整数: {raw_version!r}"

    try:                                        # ts 只作证据，坏值不拒整份布局
        out["ts"] = round(float(payload.get("ts") or 0.0), 3)
    except (TypeError, ValueError):
        out["ts"] = 0.0

    for key in ("start", "end"):
        raw = payload.get(key)
        if not isinstance(raw, str) or not raw.strip():
            return None, f"缺少 {key}"
        text = raw.strip().upper()
        try:
            geo.block_to_index(text)
        except ValueError as exc:
            return None, f"{key} 非法: {text!r}（{exc}）"
        out[key] = text

    for key in _LAYOUT_CELL_KEYS:
        cells, err = _layout_cells(payload.get(key), key)
        if cells is None:
            return None, err
        out[key] = cells

    cells, err = _layout_cells(payload.get("bridges"), "bridges", allow_empty=False)
    if cells is None:
        return None, err
    out["bridges"] = cells

    col = str(payload.get("barrier_col_from") or geo.BARRIER_COL_FROM).strip().upper()
    if col not in geo.COL_LETTERS:
        return None, f"barrier_col_from 非法: {col!r}（应为 A~{geo.COL_LETTERS[-1]}）"
    out["barrier_col_from"] = col

    for key, fallback in (("barrier_row_hi", geo.BARRIER_ROW_HI),
                          ("barrier_row_lo", geo.BARRIER_ROW_LO)):
        raw = payload.get(key)
        raw = fallback if raw is None else raw          # 显式 null = 用缺省，不整份拒收
        try:
            value = int(raw)
        except (TypeError, ValueError):
            return None, f"{key} 不是整数: {raw!r}"
        if not 1 <= value <= geo.MAP_ROWS:
            return None, f"{key} 越界: {value}（应为 1~{geo.MAP_ROWS}）"
        out[key] = value
    if out["barrier_row_hi"] >= out["barrier_row_lo"]:
        return None, (f"barrier_row_hi({out['barrier_row_hi']}) 必须小于 "
                      f"barrier_row_lo({out['barrier_row_lo']})")

    return out, ""


# --------------------------------------------------------------------------- #
#  覆盖语义 / 核对消费方（G5-2 / G5-3）
# --------------------------------------------------------------------------- #
#: 车侧黑盒键：`field_verify` 消费方汇总（dict，见 `FieldVerifyConsumer.summary`）。
#: ★ 键名常量暂时放在本模块（本卡写域只有 drone.py）；**语义稳定后应迁到
#:   `core/blackboard.py`** 统一登记（与 `P_MOTION_LIMIT` 同一做法）。
P_FIELD_VERIFY = "field_verify"
#: 车侧黑盒键：无人机覆盖进度（dict，见 `CoverageLedger.summary`）。
P_DRONE_COVERAGE = "drone_coverage"

#: `field_verify` 处置策略白名单（缺省 `record`，只记录不阻断）：
#:   - `off`    ：不创建消费方（**不是总开关**：coverage 解析由 `drone.coverage_enabled`
#:                单独控制；两者都关 = 行为与加特性前逐位一致，A/B 反证用的就是这个组合）
#:   - `record` ：默认：汇总/留痕，`hold=False`
#:   - `alert`  ：额外 `alert=True`（供遥测/上层告警）
#:   - `hold`   ：额外 `hold=True`（**建议阻断信号**；本任务不自动执行，由上层裁决）
FIELD_VERIFY_POLICIES: tuple[str, ...] = ("off", "record", "alert", "hold")


class CoverageLedger:
    """片级覆盖账本（G5-2）：幂等去重 + 只把完整片的 cells 记为「已探明」。

    语义与机侧 `protocol.py` / 决策层方案 §4.2、§5.1 对齐：

    * `coverage.cells` = 本片几何覆盖的格（格心到航段折线 ≤ corridor）；
    * `complete=True` 才授权负证据（片内未报 = ABSENT）；`complete=False` 的片
      **只**能升可见要素，片内其余仍是 UNKNOWN；
    * 同一 `(scan_id, slice.id)` 是幂等键 —— BEST_EFFORT 链路上重发是常态，
      重复片**不得**重复触发核对/事件（决策层方案 §5.1 规则 4）。

    纯数据、无 I/O，可脱离任务线程单测；后续可并入 IntelLedger（决策层方案 §5）。
    """

    def __init__(self, *, cells_total: int | None = None) -> None:
        #: 覆盖率分母：本车网格总格数（14×11=154）。现场换网格时唯一改这里/构造参数。
        self.cells_total = int(cells_total or (geo.MAP_COLS * geo.MAP_ROWS))
        self.slices: dict[tuple[str, str], dict[str, Any]] = {}
        #: 只有 `complete=True` 的片才进这里（唯一「已探明」来源，用于负证据/进度）。
        self.cells_done: set[str] = set()
        #: 所有片（含不完整片）的 cells 并集，仅诊断用（不等于已探明）。
        self.cells_seen: set[str] = set()
        self.events = 0
        self.duplicates = 0
        self.updated_at = 0.0

    # ---------------- 更新 ----------------
    def update(self, cov: contract.Coverage, now: float) -> tuple[str, dict[str, Any]]:
        """记一条 coverage；返回 `(status, summary)`，status ∈ `new|duplicate`。"""
        self.events += 1
        key = cov.key
        if key is not None and key in self.slices:
            self.duplicates += 1
            return "duplicate", self.summary()
        rec: dict[str, Any] = {
            "scan_id": cov.scan_id,
            "slice_id": cov.slice.id if cov.slice is not None else "",
            "index": int(cov.slice.index) if cov.slice is not None else 0,
            "of": int(cov.slice.of) if cov.slice is not None else 0,
            "complete": bool(cov.complete),
            "mode": cov.mode,
            "n_cells": len(cov.cells),
            "dropped": int(cov.dropped),
            "ts": round(float(now), 3),
        }
        slot = key if key is not None else ("<unkeyed>", f"#{self.events}")
        self.slices[slot] = rec
        self.cells_seen.update(cov.cells)
        if cov.complete:
            self.cells_done.update(cov.cells)
        self.updated_at = float(now)
        return "new", self.summary()

    # ---------------- 快照 ----------------
    def _done(self) -> bool:
        """某一次扫描的全部片都已收到且 `complete=True`（= 探索闭环）。"""
        groups: dict[str, list[dict[str, Any]]] = {}
        for rec in self.slices.values():
            if int(rec.get("of") or 0) > 0 and rec.get("scan_id"):
                groups.setdefault(str(rec["scan_id"]), []).append(rec)
        for recs in groups.values():
            of = max(int(r["of"]) for r in recs)
            got = {int(r["index"]) for r in recs}
            if got == set(range(1, of + 1)) and all(bool(r["complete"]) for r in recs):
                return True
        return False

    def summary(self) -> dict[str, Any]:
        edges = [r for r in self.slices.values() if int(r.get("of") or 0) > 0]
        scan_id, of = "", 0
        if edges:
            latest = max(edges, key=lambda r: str(r.get("scan_id") or ""))
            scan_id = str(latest.get("scan_id") or "")
            of = max(int(r["of"]) for r in edges if str(r.get("scan_id") or "") == scan_id)
        return {
            "cells_done": len(self.cells_done),
            "cells_seen": len(self.cells_seen),
            "cells_total": self.cells_total,
            "ratio": (round(len(self.cells_done) / self.cells_total, 4)
                      if self.cells_total else 0.0),
            "slices": len(self.slices),
            "complete_slices": sum(1 for r in self.slices.values() if r["complete"]),
            "scan_id": scan_id,
            "of": of,
            "done": self._done(),
            "events": self.events,
            "duplicates": self.duplicates,
            "updated_at": round(self.updated_at, 3),
        }


class FieldVerifyConsumer:
    """`EV_FIELD_VERIFY` 的生产订阅方（G5-3）：把雷/障/靶差异折算成决策可见汇总。

    ★ 为什么订阅事件而不是在 `_diff` 里直接累加：
      事件总线是跨任务唯一低频通道；未来 Mission / ReplanGate 也能订阅同一事件，
      不必依赖 DroneTask 私有状态。当前由 DroneTask 在自己的 5 Hz tick 里
      `drain()`（不额外起线程、不阻塞控制回路）。

    分类（tag 约定见 `DroneTask._diff`）：
      * `target_zone` / `field_verify.targets` → **靶**类
      * `field_verify.mines` / `field_verify.obstacles` → **雷/障**类
      * 其余 tag → `other`（只累计，不分类）

    只读事件、不写黑盒：黑盒由 DroneTask 统一写（谁的状态谁写）。
    """

    NAME = "drone.field_verify"

    def __init__(self, bus: EventBus, *, policy: str = "record",
                 log: Callable[[str], None] = print, rate_limit_s: float = 1.0) -> None:
        self.policy = policy if policy in FIELD_VERIFY_POLICIES else "record"
        self._log = log
        self._warn = Throttle(float(rate_limit_s))
        self._sub = bus.subscribe(self.NAME, [EV_FIELD_VERIFY])
        self.events = 0
        self.by_tag: dict[str, int] = {}
        self.hazard_new: set[str] = set()      # 无人机看到、本地图没有的雷/障
        self.hazard_gone: set[str] = set()     # 本地图有、完整片判定为 ABSENT 的雷/障
        self.target_new: set[str] = set()
        self.target_gone: set[str] = set()
        self.other: set[str] = set()
        self.last: dict[str, Any] | None = None
        self.updated_at = 0.0

    # ---------------- 消费 ----------------
    def drain(self, now: float) -> int:
        """取完当前事件队列并逐条汇总；返回本次消费条数（0 = 无新事件）。"""
        events = self._sub.poll(64)
        for ev in events:
            self._consume(ev, now)
        return len(events)

    def _consume(self, ev, now: float) -> None:
        data = ev.data if isinstance(ev.data, dict) else {}
        tag = str(data.get("tag") or "")
        only_remote = [str(x) for x in (data.get("only_remote") or [])]
        only_local = [str(x) for x in (data.get("only_local") or [])]
        self.events += 1
        self.by_tag[tag] = self.by_tag.get(tag, 0) + 1
        kind = self._kind_of(tag)
        if kind == "target":
            self.target_new.update(only_remote)
            self.target_gone.update(only_local)
        elif kind == "hazard":
            self.hazard_new.update(only_remote)
            self.hazard_gone.update(only_local)
        else:
            self.other.update(only_remote)
            self.other.update(only_local)
        self.last = {
            "tag": tag,
            "remote": sorted(str(x) for x in (data.get("remote") or [])),
            "local": sorted(str(x) for x in (data.get("local") or [])),
            "only_remote": sorted(only_remote),
            "only_local": sorted(only_local),
            "scoped": bool(data.get("scoped")),
            "complete": data.get("complete"),
            "scan_id": str(data.get("scan_id") or ""),
            "slice_id": str(data.get("slice_id") or ""),
            "out_of_scope_local": int(data.get("out_of_scope_local") or 0),
            "ts": round(float(now), 3),
        }
        self.updated_at = float(now)
        if self.policy in ("alert", "hold") and self._warn.allow("fv", now):
            self._log(f"⚠️ 核对情报汇总（policy={self.policy}）：事件 {self.events} 条，"
                      f"新增雷/障 {sorted(self.hazard_new)} 靶 {sorted(self.target_new)}")

    @staticmethod
    def _kind_of(tag: str) -> str:
        low = tag.lower()
        if "target" in low:
            return "target"
        if "mines" in low or "obstacles" in low or "hazard" in low:
            return "hazard"
        return "other"

    # ---------------- 快照 ----------------
    def summary(self) -> dict[str, Any]:
        return {
            "policy": self.policy,
            "events": self.events,
            "by_tag": dict(sorted(self.by_tag.items())),
            "hazard_new": sorted(self.hazard_new),
            "hazard_gone": sorted(self.hazard_gone),
            "target_new": sorted(self.target_new),
            "target_gone": sorted(self.target_gone),
            "other": sorted(self.other),
            "verdict": "diff" if self.events else "none",
            #: 决策可见的风险分级：risk = 无人机报了本地图没有的雷/障（路线可能撞），
            #: warn = 其它不一致（本地要素消失/靶区差异），info = 仅记录。
            "severity": ("risk" if self.hazard_new else
                         "warn" if (self.hazard_gone or self.target_new or self.target_gone)
                         else "info" if self.events else "none"),
            "hold": bool(self.policy == "hold" and self.events),
            "alert": bool(self.policy in ("alert", "hold") and self.events),
            "last": dict(self.last) if self.last is not None else None,
            "updated_at": round(self.updated_at, 3),
        }

    def unsubscribe(self) -> None:
        """退出时解订阅（测试里同一总线反复建任务时避免泄漏）。"""
        try:
            self._sub.unsubscribe()
        except Exception:                       # noqa: BLE001  解订阅失败不该杀任务
            pass


class DroneTask(TaskThread):
    """无人机情报收发（5 Hz）。

    写 `P_DRONE`（兼容旧口径）+ `P_DRONE_COVERAGE`（覆盖进度，受 `drone.coverage_enabled`
    控制）+ `P_FIELD_VERIFY`（差异消费汇总，受 `drone.field_verify_policy` 控制）。
    消息解析、心跳与车状态回传与旧版一致。
    """

    def __init__(self, bridge, bb: Blackboard, bus: EventBus, cfg: Config, *,
                 log: Callable[[str], None] = print, group=None,
                 clock: Callable[[], float] | None = None,
                 local_map: dict | None = None) -> None:
        super().__init__("DroneTask", 5.0, bus=bus, blackboard=bb, logger=log)
        self.bridge = bridge
        self.cfg = cfg
        self.group = group
        if clock is None:
            import time as _time
            clock = _time.monotonic
        self.clock = clock

        # ---------------- 配置（缺键走显式默认值，见模块 docstring ⚠️） ----
        self.enabled = bool(cfg.get("drone.enabled"))
        self.heartbeat_timeout_s = float(cfg.get("drone.heartbeat_timeout_s") or 3.0)
        self.info_stale_s = float(cfg.get("drone.info_stale_s") or 30.0)
        self.car_state_period_s = float(cfg.get("drone.car_state_period_s", 0.7))
        self.ack_enabled = bool(cfg.get("drone.ack_enabled", True))
        self.diff_max = max(1, int(cfg.get("drone.field_diff_max", 8)))
        self._rate_limit_s = float(cfg.get("logging.rate_limit_s") or 1.0)

        # ---------------- 覆盖语义 / 核对消费方（G5-2 / G5-3，缺键走显式默认） ----
        # ★ 两键**未登记 SCHEMA**（本卡写域不含 core/config.py）：不写 robot.yaml 即按缺省生效。
        self.coverage_enabled = bool(cfg.get("drone.coverage_enabled", True))
        raw_policy = str(cfg.get("drone.field_verify_policy", "") or "").strip().lower()
        self.field_verify_policy = (raw_policy if raw_policy in FIELD_VERIFY_POLICIES
                                    else "record")
        self._raw_policy = raw_policy
        self._policy_invalid = bool(raw_policy) and raw_policy not in FIELD_VERIFY_POLICIES
        self.coverage_ledger: CoverageLedger | None = (
            CoverageLedger() if self.coverage_enabled else None)
        self.field_verify: FieldVerifyConsumer | None = None
        self.coverage_updates = 0
        self.coverage_bad = 0
        self.coverage_duplicates = 0

        # ---------------- 本地地图（核对真源） ----------------
        self._local_map: dict | None = local_map
        self.map_loaded = bool(local_map)
        self._map_source = "构造参数" if local_map else ""
        self.targets: frozenset[str] = frozenset()
        self.mines: frozenset[str] = frozenset()
        self.obstacles: frozenset[str] = frozenset()
        self.bridges: frozenset[str] = frozenset()

        # ---------------- 场地布局（field_layout，车端不留地图文件） ----------------
        #: 最近一份**已校验**的布局（dict；原样写黑盒，供 main/任务侧规划）。
        self.field_layout: dict | None = None
        self.layout_updates = 0
        self.layout_rejects = 0
        self.layout_ts = 0.0

        # ---------------- 链路状态 ----------------
        self.online = False
        self.last_heartbeat = 0.0
        self.messages = 0
        self.bad_messages = 0                # 非法 JSON / 非字符串
        self.unknown_types: dict[str, int] = {}
        self.missing_fields = 0
        self.field_diffs: list[str] = []
        self.bridge_half: str | None = None
        self.bridge_block: str | None = None
        self.bridge_confidence = 0.0
        self.bridge_ts = 0.0
        self.bridge_updates = 0
        self.last_type = ""
        self.acks_sent = 0
        self.car_states_sent = 0

        self._seq = 0
        self._next_car_state_at: float | None = None
        self._last_phase = ""
        self._started_at: float | None = None
        self._warn = Throttle(self._rate_limit_s)

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        self._started_at = self.clock()
        self._next_car_state_at = self._started_at
        self._load_local_map()
        if not self.enabled:
            self._log("无人机链路已禁用（drone.enabled=False）→ 整任务空转：不读不发")
            return
        self._log(f"无人机链路：心跳超时 {self.heartbeat_timeout_s:g}s"
                  f" 情报过期 {self.info_stale_s:g}s 车状态 {1.0 / self.car_state_period_s:.1f}Hz"
                  f" 本地地图={'已载入(' + self._map_source + ')' if self.map_loaded else '缺失→不核对'}")
        if not self.map_loaded:
            self._log("⚠️ 未找到本地地图（config/map.yaml）→ 靶区/雷障核对降级为不核对")
        if self._policy_invalid:
            self._log(f"⚠️ drone.field_verify_policy={self._raw_policy!r} 非法"
                      f"（可选 {FIELD_VERIFY_POLICIES}）→ 按 record")
        if self.field_verify_policy != "off":
            self.field_verify = FieldVerifyConsumer(
                self.bus, policy=self.field_verify_policy, log=self._log,
                rate_limit_s=self._rate_limit_s)
            self._log(f"核对消费方：policy={self.field_verify_policy}"
                      f" 覆盖账本={'开' if self.coverage_enabled else '关'}"
                      f"（默认只记录不阻断；EV_FIELD_VERIFY → {P_FIELD_VERIFY}/{P_DRONE_COVERAGE}）")

    def teardown(self) -> None:
        if self.field_verify is not None:
            self.field_verify.unsubscribe()
        line = (f"退出：消息 {self.messages} 条（坏 {self.bad_messages}）"
                f" 桥洞情报 {self.bridge_updates} 次 车状态 {self.car_states_sent} 条"
                f" ack {self.acks_sent} 条 差异 {len(self.field_diffs)} 条"
                f" 在线={'是' if self.online else '否'}")
        if self.field_verify is not None or self.coverage_updates:
            line += (f" ｜覆盖 {len(self.coverage_ledger.cells_done) if self.coverage_ledger else 0}"
                     f"/{self.coverage_ledger.cells_total if self.coverage_ledger else 0}"
                     f" 片 {self.coverage_updates}（重发 {self.coverage_duplicates}"
                     f"/坏 {self.coverage_bad}）"
                     f" ｜核对事件 {self.field_verify.events if self.field_verify else 0}")
        if self.layout_updates or self.layout_rejects:
            line += (f" ｜场地布局 {self.layout_updates} 份"
                     f"（拒收 {self.layout_rejects}）")
        self._log(line)

    # ------------------------------------------------------------------ #
    #  只读接口（测试/遥测）
    # ------------------------------------------------------------------ #
    def state(self) -> DroneState:
        """当前 `P_DRONE` 的内容（最近一拍写的快照）。"""
        return DroneState(
            online=self.online, last_heartbeat=self.last_heartbeat,
            bridge_half=self.bridge_half, bridge_block=self.bridge_block,
            bridge_confidence=self.bridge_confidence, bridge_ts=self.bridge_ts,
            field_diff=tuple(self.field_diffs), messages=self.messages,
            last_type=self.last_type,
        )

    def bridge_of(self, half: str, *, now: float | None = None) -> str | None:
        """便捷直读（★ 权威实现在 `DroneState.bridge_of`，这里只是转发）。"""
        now = self.clock() if now is None else now
        return self.state().bridge_of(half, now=now, stale_s=self.info_stale_s)

    def field_verify_summary(self) -> dict | None:
        """核对消费汇总（决策可见信号）；消费方未启用（policy=off/任务禁用）返回 `None`。

        额外补两个**基于覆盖账本**的决策信号（仅当本地地图已加载）：
          * `local_unverified`  ：本地图里的雷/障格中，**没有任何完整片判读**过的格
            （= 地图先验，不可当作"现场没有"）；
          * `targets_unverified`：同理的靶格。
        """
        consumer = self.field_verify
        if consumer is None:
            return None
        summary = consumer.summary()
        ledger = self.coverage_ledger
        if ledger is not None and self.map_loaded:
            done = ledger.cells_done
            summary["local_unverified"] = sorted((self.mines | self.obstacles) - done)
            summary["targets_unverified"] = sorted(self.targets - done)
        return summary

    def field_verify_gate(self) -> dict:
        """给上层（Mission/将来闸门）读的**处置建议**：本任务自己绝不执行。

        默认 `policy=record` ⇒ `hold=False`（只记录不阻断）。`policy=hold` 时
        只在收到差异后置 `hold=True`，是否停车/等待由上层裁决。
        """
        summary = self.field_verify_summary()
        if summary is None:
            return {"hold": False, "verdict": "none", "reason": "核对消费方未启用"}
        return {
            "hold": bool(summary.get("hold")),
            "verdict": str(summary.get("verdict") or "none"),
            "severity": str(summary.get("severity") or "none"),
            "policy": str(summary.get("policy") or "record"),
            "reason": ("policy=hold 且已收到核对差异 → 建议上层复核"
                       if summary.get("hold")
                       else f"policy={summary.get('policy')}：只记录不阻断"),
        }

    def coverage_summary(self) -> dict | None:
        """覆盖进度汇总（黑盒 `P_DRONE_COVERAGE` 的同源快照）；关闭覆盖时返回 `None`。"""
        return None if self.coverage_ledger is None else self.coverage_ledger.summary()

    # ------------------------------------------------------------------ #
    #  单拍
    # ------------------------------------------------------------------ #
    def tick(self) -> None:
        now = self.clock()
        if not self.enabled:
            return                                  # ★ 空转：一个 bridge 调用都不发
        self._drain_inbox(now)
        self._drain_field_verify(now)
        self._check_heartbeat(now)
        self._maybe_send_car_state(now)
        self._publish(now)

    # ------------------------------------------------------------------ #
    #  收
    # ------------------------------------------------------------------ #
    def _drain_inbox(self, now: float) -> None:
        try:
            raw_items = self.bridge.take_drone_messages(16) or []
        except Exception as exc:                    # noqa: BLE001  读失败不该杀任务
            if self._warn.allow("inbox", now):
                self._log(f"读取无人机消息失败（忽略本拍）：{type(exc).__name__}: {exc}")
            return
        for raw in raw_items:
            self._handle_raw(raw, now)

    def _handle_raw(self, raw: Any, now: float) -> None:
        """单条报文的解析入口。★ 任何一条坏消息都只记日志，绝不上抛。"""
        if not isinstance(raw, str):
            self.bad_messages += 1
            if self._warn.allow("bad_type", now):
                self._log(f"⚠️ 无人机消息类型非字符串（丢弃）：{type(raw).__name__}")
            return
        try:
            payload = json.loads(raw)
        except Exception as exc:                    # noqa: BLE001  非法 JSON
            self.bad_messages += 1
            if self._warn.allow("bad_json", now):
                self._log(f"⚠️ 无人机消息非法 JSON（丢弃）：{type(exc).__name__}: "
                          f"{raw[:80]!r}")
            return
        if not isinstance(payload, dict):
            self.bad_messages += 1
            if self._warn.allow("bad_shape", now):
                self._log(f"⚠️ 无人机消息不是 JSON 对象（丢弃）：{type(payload).__name__}")
            return

        self.messages += 1
        msg_type = str(payload.get("type") or "").strip()
        self.last_type = msg_type
        if msg_type == "heartbeat":
            self._on_heartbeat(now)
        elif msg_type == "bridge_passable":
            self._on_bridge_passable(payload, now)
        elif msg_type == "target_zone":
            self._on_target_zone(payload, now)
        elif msg_type == "field_verify":
            self._on_field_verify(payload, now)
        elif msg_type == "field_layout":
            self._on_field_layout(payload, now)
        else:
            self.unknown_types[msg_type] = self.unknown_types.get(msg_type, 0) + 1
            if self._warn.allow(f"unknown:{msg_type}", now):
                self._log(f"⚠️ 未知无人机消息类型 {msg_type!r}（忽略，累计 "
                          f"{self.unknown_types[msg_type]} 条）：{raw[:80]!r}")
        self._maybe_ack(payload, now)

    # ---- 心跳 ----
    def _on_heartbeat(self, now: float) -> None:
        # ★ 顺序要紧：先算"离线了多久"，再刷新 last_heartbeat。
        #   反过来的话 silent_s 恒为 0.0（恢复事件就丢了唯一有价值的证据）。
        silent = self._silent_s(now)
        self.last_heartbeat = now
        if not self.online:
            self.online = True
            self.bus.publish(EV_DRONE_ONLINE, source=self.name,
                             silent_s=(None if silent == _INF else round(silent, 2)))
            self._log(f"无人机上线（距上次心跳 {self._fmt_age(silent)}）")

    def _maybe_ack(self, payload: dict, now: float) -> None:
        if not self.ack_enabled:
            return
        seq = payload.get("seq")
        if seq is None and not payload.get("ack"):
            return                                  # 没要求回执就不回（省链路）
        if seq is None:
            return
        try:
            body = json.dumps({"type": "ack", "of": seq, "ts": round(now, 3)},
                              ensure_ascii=False, separators=(",", ":"))
            self.bridge.publish_drone(body)
            self.acks_sent += 1
        except Exception as exc:                    # noqa: BLE001
            if self._warn.allow("ack", now):
                self._log(f"回执发送失败：{type(exc).__name__}: {exc}")

    # ---- 桥洞情报 ----
    def _on_bridge_passable(self, payload: dict, now: float) -> None:
        half = str(payload.get("half") or "").strip().lower()
        block = str(payload.get("block") or "").strip().upper()
        if half not in _BRIDGE_HALVES or not block:
            self.missing_fields += 1
            self._warn_missing("bridge_passable", "half/block", payload, now)
            return
        # ★ 刻意**不**校验"格名是否在地图内"：穿洞格每局随机，无人机才是权威；
        #   本地地图里没有这一格不代表情报是坏的（路线校验会在 Mission 侧兜住）。
        conf = self._as_float(payload.get("confidence"), 0.0)
        self.bridge_half, self.bridge_block = half, block
        self.bridge_confidence, self.bridge_ts = conf, now
        self.bridge_updates += 1
        self.bus.publish(EV_BRIDGE_INFO, source=self.name, half=half, block=block,
                         confidence=conf, ts=now)
        self._log(f"桥洞情报：{half} 半场 → {block}（置信度 {conf:.2f}，"
                  f"第 {self.bridge_updates} 次）")

    # ---- 靶区核对 ----
    def _on_target_zone(self, payload: dict, now: float) -> None:
        blocks = payload.get("blocks")
        if not isinstance(blocks, (list, tuple)) or not blocks:
            self.missing_fields += 1
            self._warn_missing("target_zone", "blocks", payload, now)
            return
        got, malformed = self._norm_blocks(blocks)
        if malformed:
            self.missing_fields += 1
            self._warn_missing("target_zone", f"{malformed} 个畸形格名（已忽略）",
                               payload, now)
        if got:
            self._diff("target_zone", got, self.targets, now)

    # ---- 完整场地布局（field_layout；车端不留地图文件） ----
    def _on_field_layout(self, payload: dict, now: float) -> None:
        """校验并落 `P_FIELD_LAYOUT`。坏报文只记日志：**不清空、不覆盖上一份好布局**。"""
        layout, reason = normalize_field_layout(payload)
        if layout is None:
            self.layout_rejects += 1
            if self._warn.allow("layout_bad", now):
                self._log(f"⚠️ 无人机 field_layout 非法（已整份忽略，保留上一份布局；"
                          f"累计拒收 {self.layout_rejects}）：{reason}")
            return
        if int(layout.get("version") or 0) > FIELD_LAYOUT_VERSION:
            if self._warn.allow("layout_ver", now):
                self._log(f"⚠️ field_layout version={layout.get('version')} 高于车端支持 "
                          f"{FIELD_LAYOUT_VERSION} → 只按已知字段解析（未知字段忽略）")
        self.field_layout = layout
        self.layout_updates += 1
        self.layout_ts = float(now)
        self.bb.set(P_FIELD_LAYOUT, layout, source=self.name, ts=now)
        self._log(f"场地布局：起 {layout['start']} 终 {layout['end']} "
                  f"雷 {len(layout['mines'])} 障 {len(layout['obstacles'])} "
                  f"靶 {len(layout['targets'])} 桥洞 {layout['bridges']} "
                  f"隔板 {layout['barrier_col_from']}{layout['barrier_row_hi']}/"
                  f"{layout['barrier_row_lo']}（第 {self.layout_updates} 份，v{layout['version']}）")

    # ---- 雷/障/靶核对（+ 可选的片级覆盖） ----
    def _on_field_verify(self, payload: dict, now: float) -> None:
        coverage, duplicate = self._take_coverage(payload, now)
        if duplicate:
            # ★ 幂等：同一 (scan_id, slice.id) 的重发整条忽略（BEST_EFFORT 上重发是常态），
            #   不重复核对、不重复发 EV_FIELD_VERIFY、不重复记 field_diff。
            return
        touched = False
        for key, local in (("mines", self.mines), ("obstacles", self.obstacles),
                           ("targets", self.targets)):
            if key not in payload:
                continue
            raw = payload.get(key)
            if not isinstance(raw, (list, tuple)):
                self.missing_fields += 1
                self._warn_missing("field_verify", key, payload, now)
                continue
            got, malformed = self._norm_blocks(raw)
            if malformed:
                self.missing_fields += 1
                self._warn_missing("field_verify", f"{key} 含 {malformed} 个畸形格名",
                                   payload, now)
            touched = True
            self._diff(f"field_verify.{key}", got, local, now, coverage=coverage)
        if not touched and coverage is None:
            # 旧报文口径：既没有 mines/obstacles/targets，也没有 coverage ⇒ 空消息。
            #   ★ 带合法 coverage 的"只报进度"报文不算缺失字段（它是 t5a 的合法形状）。
            self.missing_fields += 1
            self._warn_missing("field_verify", "mines/obstacles", payload, now)

    # ---- 覆盖字段（G5-2） ----
    def _take_coverage(self, payload: dict, now: float) -> tuple[contract.Coverage | None, bool]:
        """解析并登记 `coverage`；返回 `(coverage, 是否重复片)`。

        * 关闭 `drone.coverage_enabled` / 报文无 `coverage` 键 ⇒ `(None, False)`，
          **零副作用**（旧报文兼容的关键路径）。
        * 形状非法 ⇒ `(None, False)` + `coverage_bad` 计数，其余字段照旧处理。
        * 同一 `(scan_id, slice.id)` 重发 ⇒ `(coverage, True)`，调用方整条忽略。
        """
        if not self.coverage_enabled or self.coverage_ledger is None:
            return None, False
        if "coverage" not in payload:
            return None, False
        cov = contract.decode_coverage(
            payload.get("coverage"), scan_id=payload.get("scan_id"),
            slice_info=payload.get("slice"))
        if cov is None:
            self.coverage_bad += 1
            if self._warn.allow("coverage_bad", now):
                self._log("⚠️ field_verify.coverage 形状非法（忽略该字段，其余字段照旧处理）")
            return None, False
        status, summary = self.coverage_ledger.update(cov, now)
        if status == "duplicate":
            self.coverage_duplicates += 1
            if self._warn.allow("coverage_dup", now):
                self._log(f"覆盖片重发（幂等忽略）：scan_id={cov.scan_id or '-'} "
                          f"slice={(cov.slice.id if cov.slice is not None else '-')}"
                          f"（累计重发 {self.coverage_duplicates}）")
            return cov, True
        self.coverage_updates += 1
        self.bb.set(P_DRONE_COVERAGE, summary, source=self.name, ts=now)
        if cov.problems and self._warn.allow("coverage_note", now):
            self._log("⚠️ coverage 已降级处理：" + "；".join(cov.problems))
        return cov, False

    def _drain_field_verify(self, now: float) -> None:
        """把本拍攒下的 `EV_FIELD_VERIFY` 消费成决策可见汇总（G5-3）。"""
        consumer = self.field_verify
        if consumer is None:
            return
        if consumer.drain(now):
            self.bb.set(P_FIELD_VERIFY, self.field_verify_summary(), source=self.name, ts=now)

    # ---- 差异登记 ----
    def _diff(self, tag: str, got: frozenset[str], local: frozenset[str], now: float,
              *, coverage: contract.Coverage | None = None) -> None:
        """登记差异并发 `EV_FIELD_VERIFY`。

        ★ 两种口径（**旧报文走第一条，输出逐位不变**）：

        1. **无 coverage（旧机侧）**：`got` 视为全图清单，与本地全量集合做差集。
        2. **有 coverage（t5a 片级报文）**：`got` 只是**本片**看得见的要素，所以
           "本地有、无人机没有"只能在**本片 cells 内**下结论，且仅当 `complete=True`：
           `complete=False` 时片内其余仍是 UNKNOWN（**不得**报成本地要素消失）。
           `only_remote` 不受片限制：无人机看到本地图没有的雷/障，不管在不在片内都是真情报。
        """
        if not self.map_loaded:
            self._record_diff(f"{tag}:收到{sorted(got)} 本地地图缺失→未核对")
            return

        if coverage is not None and not coverage.cells:
            # cells 为空的 coverage 无判读范围 ⇒ 不做任何差异判定（保守：未知≠不存在）。
            if self._warn.allow(f"coverage_empty:{tag}", now):
                self._log(f"⚠️ {tag} 带 coverage 但 cells 为空 → 不做差异判定（全部按 UNKNOWN）")
            return

        if coverage is not None and coverage.cells:
            scope = frozenset(coverage.cells)
            local_scoped = local & scope
            only_remote = sorted(got - local)
            only_local = sorted(local_scoped - got) if coverage.complete else []
            out_of_scope = len(local - local_scoped)
            if not only_remote and not only_local:
                note = "" if coverage.complete else "（片不完整：片内未报 ≠ 不存在）"
                extra = (f"｜片外 {out_of_scope} 个本地要素未判读" if out_of_scope else "")
                self._log(f"核对一致：{tag} 片内 {sorted(local_scoped)}{note}{extra}")
                return
            text = (f"{tag}[片内]:仅无人机{only_remote} 仅本地{only_local}"
                    f"（无人机 {sorted(got)} / 片内本地 {sorted(local_scoped)}"
                    f" / 片外 {out_of_scope} 未判读"
                    f" / 片={'完整' if coverage.complete else '不完整'}）")
            self._record_diff(text)
            self.bus.publish(
                EV_FIELD_VERIFY, source=self.name, tag=tag,
                remote=sorted(got), local=sorted(local_scoped),
                only_remote=only_remote, only_local=only_local,
                scoped=True, complete=bool(coverage.complete),
                cells=sorted(scope), scan_id=coverage.scan_id,
                slice_id=(coverage.slice.id if coverage.slice is not None else ""),
                out_of_scope_local=out_of_scope)
            self._log(f"⚠️ 情报不一致：{text}")
            return

        # ---- 旧口径（无 coverage）：以下代码与加特性前逐字相同 ----
        if got == local:
            self._log(f"核对一致：{tag} {sorted(got)}")
            return
        only_remote = sorted(got - local)
        only_local = sorted(local - got)
        text = (f"{tag}:仅无人机{sorted(only_remote)} 仅本地{sorted(only_local)}"
                f"（无人机 {sorted(got)} / 本地 {sorted(local)}）")
        self._record_diff(text)
        self.bus.publish(EV_FIELD_VERIFY, source=self.name, tag=tag,
                         remote=sorted(got), local=sorted(local),
                         only_remote=only_remote, only_local=only_local)
        self._log(f"⚠️ 情报不一致：{text}")

    def _record_diff(self, text: str) -> None:
        self.field_diffs.append(text)
        if len(self.field_diffs) > self.diff_max:
            del self.field_diffs[:-self.diff_max]

    @staticmethod
    def _norm_blocks(blocks: Iterable[Any]) -> tuple[frozenset[str], int]:
        """归一化格名列表 → `(格名集合, 畸形条目数)`。

        ★ 这里**不校验格名在地图内**：情报与本地地图不一致本来就可能是
          "无人机看到了本地没有的格子"（靶区确实量到 H12、或本地识图漏了一格），
          把这种条目当"非法消息"丢掉，就等于**核对功能永远报不出差异**
          —— 那正是本任务存在的理由。所以只挡明显畸形（空串/非字符串），
          格号本身留给 `_diff` 去比对并告警。
        """
        out: set[str] = set()
        malformed = 0
        for item in blocks:
            if not isinstance(item, str):
                malformed += 1
                continue
            text = item.strip().upper()
            if not text:
                malformed += 1
                continue
            out.add(text)
        return frozenset(out), malformed

    def _warn_missing(self, msg_type: str, what: str, payload: dict, now: float) -> None:
        if self._warn.allow(f"missing:{msg_type}:{what}", now):
            self._log(f"⚠️ 无人机 {msg_type} 缺字段/字段非法（{what}，已忽略）：{payload}")

    # ---- 心跳超时 ----
    def _check_heartbeat(self, now: float) -> None:
        if not self.online:
            return                                  # ★ 只在状态变化时发一次（不刷屏）
        if self._silent_s(now) <= self.heartbeat_timeout_s:
            return
        self.online = False
        silent = self._silent_s(now)
        self.bus.publish(EV_DRONE_OFFLINE, source=self.name,
                         silent_s=(None if silent == _INF else round(silent, 2)),
                         timeout_s=self.heartbeat_timeout_s)
        self._log(f"⚠️ 无人机离线：{self._fmt_age(silent)} 无心跳"
                  f"（阈值 {self.heartbeat_timeout_s:g}s）→ 不阻塞任务，"
                  f"保持默认分支/默认地图")

    def _silent_s(self, now: float) -> float:
        return _INF if not self.last_heartbeat else (now - self.last_heartbeat)

    @staticmethod
    def _fmt_age(age: float) -> str:
        return "从未" if age == _INF else f"{age:.2f}s"

    # ------------------------------------------------------------------ #
    #  发
    # ------------------------------------------------------------------ #
    def _maybe_send_car_state(self, now: float) -> None:
        phase = str(self.bb.get(P_MISSION_PHASE) or "")
        due = self._next_car_state_at is None or now >= self._next_car_state_at
        changed = phase != self._last_phase
        if not (due or changed):
            return
        if self._send_car_state(now, phase):
            self._next_car_state_at = now + self.car_state_period_s
            self._last_phase = phase

    def _send_car_state(self, now: float, phase: str) -> bool:
        pose = self.bb.get(P_POSE)
        block = ""
        yaw_deg = 0.0
        if isinstance(pose, geo.Pose):
            block = pose.block()
            yaw_deg = round(geo.deg(pose.yaw), 1)
        self._seq += 1
        body = json.dumps({
            "type": "car_state", "block": block, "yaw_deg": yaw_deg,
            "phase": phase, "ts": round(now, 3), "seq": self._seq,
        }, ensure_ascii=False, separators=(",", ":"))
        try:
            self.bridge.publish_drone(body)
        except Exception as exc:                    # noqa: BLE001  发不出去也不许崩
            if self._warn.allow("car_state", now):
                self._log(f"车状态上报失败（下拍重试）：{type(exc).__name__}: {exc}")
            return False
        self.car_states_sent += 1
        return True

    # ------------------------------------------------------------------ #
    #  落盘 / 地图
    # ------------------------------------------------------------------ #
    def _publish(self, now: float) -> None:
        self.bb.set(P_DRONE, self.state(), source=self.name, ts=now)

    def _load_local_map(self) -> None:
        cfg_map = self._local_map
        if cfg_map is None:
            path = self.cfg.get("drone.map_path")
            candidates = [pathlib.Path(path)] if path else [default_map_path()]
            for cand in candidates:
                cfg_map = self._read_yaml(cand)
                if cfg_map:
                    self._map_source = str(cand)
                    break
        if not cfg_map:
            return
        body = cfg_map.get("map") if isinstance(cfg_map.get("map"), dict) else cfg_map
        self._local_map = cfg_map
        self.map_loaded = True
        if not self._map_source:
            self._map_source = "构造参数"
        self.targets = self._block_set(body.get("targets"))
        self.mines = self._block_set(body.get("mines"))
        self.obstacles = self._block_set(body.get("obstacles"))
        self.bridges = self._block_set(body.get("bridges"))

    def _read_yaml(self, path: pathlib.Path) -> dict | None:
        try:
            if not path.is_file():
                return None
            import yaml
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            return data if isinstance(data, dict) else None
        except Exception as exc:                    # noqa: BLE001  地图坏了不该杀任务
            self._log(f"⚠️ 读取本地地图失败 {path}：{type(exc).__name__}: {exc}")
            return None

    @staticmethod
    def _block_set(items: Any) -> frozenset[str]:
        if not isinstance(items, (list, tuple)):
            return frozenset()
        return frozenset(str(x).strip().upper() for x in items if str(x).strip())

    @staticmethod
    def _as_float(value: Any, default: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default
