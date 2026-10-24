# -*- coding: utf-8 -*-
r"""协议发射器：把感知结果编成**车侧能吃的 JSON**。

## 契约来源（逐条对照，不靠记忆）

消费端：``dzrobot_v2/src/dzbrain/dzbrain/tasks/drone.py``（1196 行，`DroneTask`，5 Hz）

| 报文 | 车侧要求的字段 | 车侧行为 | 出处 |
|---|---|---|---|
| ``heartbeat`` | ``type`` | 刷新心跳；>3 s 无心跳 → ``online=False`` + 一次告警 | ``drone.py:228,244-253,379-391``；``robot.yaml`` ``heartbeat_timeout_s: 3.0`` |
| ``bridge_passable`` | ``half``∈{lower,upper}、``block``非空、``confidence`` | 写 ``P_DRONE``，``bridge_of(half)`` 供 Mission 取格；>30 s 过期返回 None | ``drone.py:273-289``、``info_stale_s: 30.0`` |
| ``target_zone`` | ``blocks``（非空 list） | 与本地 ``map.yaml.targets`` 做差集 → ``field_diff`` | ``drone.py:292-304`` |
| ``field_verify`` | ``mines`` / ``obstacles``（list） | 同上（雷/障） | ``drone.py:307-326`` |
| ``field_verify`` + ``coverage``/``slice``/``scan_id`` | 旧车侧**不读**这三个键（``payload.get``） | 记录新键、按旧规则处理 | ``drone.py:307-326``；语义见 ``docs/黑图探索与动态规划_决策层方案.md`` §4.2 / §5.1 |
| ``field_layout`` v2 | ``version``/``start``/``end``/``known``/``mines``/``obstacles``/``targets``/``bridges`` 必填；``target_cell``/``segment_id``/``barrier_*`` 可选 | 校验过 → 写 ``P_FIELD_LAYOUT``（**规划真源**，车端不留地图文件）；version 单调：旧版/同版异内容**整份拒收** | ``drone.py:181-294``；``FIELD_LAYOUT_VERSION=2`` |
| 任意带 ``seq`` | — | 回一条 ``ack{type:"ack",of:seq}``（**发送成功的硬证据**） | ``drone.py:255-270`` |

**四个容易踩的坑（都是从 ``drone.py`` 读出来的，不是推测）**：

1. ``half`` 会被 ``.strip().lower()`` 后校验，``block`` 会被 ``.strip().upper()``；
   但**格名是否在地图内不校验**（``drone.py:280-281`` 明写"穿洞格每局随机，无人机才是权威"）。
2. **载荷里的 ``ts`` 车侧不使用**：``_on_bridge_passable`` 用的是**收到时刻**
   ``self.bridge_ts = now``（``drone.py:284``）。所以"情报 30 s 过期"是从**车收到**算起，
   周期性重发同一结论是**有效的**续期手段（不要以为发了旧时间戳就会立刻过期）。
3. ``target_zone`` 的 ``blocks`` 为空列表 = 车侧记一次 ``missing_fields``（``:294``），
   等于白发一条；要么不发，要么别发空表。
4. 未知 ``type`` 只记日志不崩（``:236-240``），所以**新增字段是安全的、新增类型不安全**
   （会被当成未知类型忽略）。本模块只产出上述 5 种类型。

话题与 QoS（``dzrobot_v2/src/dzbrain/dzbrain/io/contract.py:40-41,94-96``）：
``/dzlink/drone_to_car`` / ``std_msgs/String`` / **depth 10 · BEST_EFFORT**。
QoS 配错（机侧 RELIABLE）会**静默收不到**——跨机 ROS 2 最经典的失败模式。

## 覆盖 / 负证据（t5a，2026-10-17）

``build_field_verify`` 新增三个 **additive** 键，让「扫过且为空」可表达：

```json
{"type":"field_verify","mines":[],"obstacles":[],
 "coverage":{"cells":["A1","A2","A3"],"complete":true,"mode":"strip"},
 "slice":{"id":"S1","index":1,"of":7,"route":"N8>M8>L8>K8"},
 "scan_id":"20261017T0930_a1b2c3","ts":1792331234.5,"seq":7}
```

* ``coverage.complete=true``  ⇒ 片内 ``cells`` 已判读完毕，未出现的要素 = **ABSENT**；
  ``complete=false`` ⇒ 只报看得见的，其余仍是 **UNKNOWN**（**不得**当 ABSENT，
  :func:`coverage_absent_cells` 对这种片返回 ``None``）。
* ``scan_id`` 是**幂等键**：BEST_EFFORT 必须周期重发同一片，车侧按
  ``(scan_id, slice.id)`` 去重（决策层方案 §5.1 第 4 条）。重发**必须**复用同一个
  ``scan_id``（CLI 会打印，``--scan-id`` 显式指定）。
* 旧车侧对这 3 个键**完全不看**（字段读取是 ``payload.get``）⇒ 安全；
  但**空表**会被旧车侧 ``_diff`` 读成「本地有、无人机没有」⇒ 记一条差异
  （``EV_FIELD_VERIFY`` 目前**零订阅**，只进遥测；新语义由 t5b 台账消费）。
  过渡期可用 ``--omit-empty-classes`` 不发空表（代价：旧车侧记 ``missing_fields``）。
"""

from __future__ import annotations

import json
import time
import uuid
from typing import Any, Iterable, Mapping, Sequence

from .grid import GridSpec, RegistrationError

__all__ = [
    "DRONE_TO_CAR_TOPIC",
    "CAR_TO_DRONE_TOPIC",
    "PAYLOAD_TYPES",
    "BRIDGE_HALVES",
    "COVERAGE_MODES",
    "SCAN_ID_MAX_LEN",
    "build_heartbeat",
    "build_bridge_passable",
    "build_target_zone",
    "build_field_verify",
    "build_field_layout",
    "FIELD_LAYOUT_VERSION",
    "encode",
    "encode_all",
    "validate_payload",
    "mirror_car_side_parse",
    "recommended_cadence",
    "make_scan_id",
    "coverage_absent_cells",
    "field_verify_absent",
]

DRONE_TO_CAR_TOPIC = "/dzlink/drone_to_car"
CAR_TO_DRONE_TOPIC = "/dzlink/car_to_drone"
QOS_DRONE_LINK = {"depth": 10, "reliability": "best_effort"}
PAYLOAD_TYPES = ("heartbeat", "bridge_passable", "target_zone", "field_verify",
                 "field_layout")
#: ``field_layout`` 协议版本上限，**与车侧** ``drone.py FIELD_LAYOUT_VERSION``
#: **保持相等**（契约测试断言两边一致，漂移 = 测试红）。
FIELD_LAYOUT_VERSION = 2
#: ``field_layout`` 里必须 ⊆ ``known`` 的要素键（车侧逐键强制）；
#: ``bridges`` 照图不受限（桥洞格每局随机、可能在未观测区，无人机是权威）。
_LAYOUT_FEATURE_KEYS: tuple[str, ...] = ("mines", "obstacles", "targets")
BRIDGE_HALVES = ("lower", "upper")
#: ``coverage.mode`` 白名单（语义见 ``coverage.py`` 模块头）
COVERAGE_MODES = ("strip", "point", "full")
#: ``scan_id`` 上限（车侧若存表，短 id 更省；64 已远超一个扫描期）
SCAN_ID_MAX_LEN = 64


def _now(ts: float | None) -> float:
    return round(float(time.time() if ts is None else ts), 3)


def _norm_scan_id(scan_id: Any) -> str:
    """``scan_id``：非空、无空白、≤ :data:`SCAN_ID_MAX_LEN`。"""
    text = str(scan_id or "").strip()
    if not text:
        raise ValueError("scan_id 不能为空（BEST_EFFORT 重发靠它去重，见决策层方案 §5.1-4）")
    if len(text) > SCAN_ID_MAX_LEN:
        raise ValueError(f"scan_id 超长（{len(text)} > {SCAN_ID_MAX_LEN}）")
    if any(ch.isspace() for ch in text):
        raise ValueError(f"scan_id 不能含空白：{text!r}")
    return text


def make_scan_id(ts: float | None = None, *, token: str | None = None) -> str:
    """生成 ``YYYYMMDDTHHMM_<6位>`` 形式的扫描号（示例见决策层方案 §4.2）。

    ``token`` 由调用方给**确定性**短串（CLI 用图片+片清单的哈希）时，同一分钟内
    同样输入会得到同一个 ``scan_id`` ⇒ 重发即幂等；不传则随机 6 位。
    """
    moment = time.localtime(float(time.time() if ts is None else ts))
    base = time.strftime("%Y%m%dT%H%M", moment)
    tok = "".join(ch for ch in str(token or "")
                  if ch.isascii() and ch.isalnum())[:6]
    return f"{base}_{tok or uuid.uuid4().hex[:6]}"


def _norm_cells(cells: Any, spec: GridSpec | None, *, key: str = "coverage.cells") -> list[str]:
    """覆盖格清单：非空、无重复、无空串；``spec`` 给出时逐格校验在网格内。"""
    if not isinstance(cells, (list, tuple)):
        raise ValueError(f"{key} 必须是 list（收到 {type(cells).__name__}）")
    out: list[str] = []
    seen: set[str] = set()
    for item in cells:
        text = str(item).strip().upper()
        if not text:
            raise ValueError(f"{key} 含空格名")
        if text in seen:
            raise ValueError(f"{key} 有重复格：{text}")
        if spec is not None:
            try:
                spec.block_to_index(text)
            except RegistrationError as exc:
                raise ValueError(f"{key} 含非法格名：{text}（{exc}）") from exc
        seen.add(text)
        out.append(text)
    if not out:
        raise ValueError(f"{key} 不能为空（空 coverage = 什么都没判读，别发）")
    return out


def _norm_coverage(coverage: Any, spec: GridSpec | None) -> dict:
    """``coverage{cells,complete,mode}`` 的发送端严格校验（顺序原样保留）。"""
    if not isinstance(coverage, Mapping):
        raise ValueError(f"coverage 必须是 dict（收到 {type(coverage).__name__}）")
    cells = _norm_cells(coverage.get("cells"), spec)
    complete = coverage.get("complete")
    if not isinstance(complete, bool):
        raise ValueError("coverage.complete 必须是 bool"
                         "（true=片内判读完；false=只报看见的，其余 UNKNOWN）")
    mode = str(coverage.get("mode") or "strip").strip().lower()
    if mode not in COVERAGE_MODES:
        raise ValueError(f"coverage.mode 只能是 {COVERAGE_MODES}，收到 {mode!r}")
    return {"cells": cells, "complete": complete, "mode": mode}


def _norm_slice(slice_info: Any, spec: GridSpec | None) -> dict:
    """``slice{id,index,of,route}``：id 非空、``1 ≤ index ≤ of``、route 逐点合法。"""
    if not isinstance(slice_info, Mapping):
        raise ValueError(f"slice 必须是 dict（收到 {type(slice_info).__name__}）")
    sid = str(slice_info.get("id") or "").strip().upper()
    if not sid:
        raise ValueError("slice.id 不能为空（车侧按 (scan_id, slice.id) 去重）")
    if len(sid) > 16:
        raise ValueError(f"slice.id 太长（{len(sid)} > 16）：{sid!r}")
    index, of = slice_info.get("index"), slice_info.get("of")
    for name, value in (("index", index), ("of", of)):
        if not isinstance(value, int) or isinstance(value, bool):
            raise ValueError(f"slice.{name} 必须是 int（收到 {value!r}）")
    if not (1 <= index <= of):
        raise ValueError(f"slice.index/of 非法：{index}/{of}（要求 1 ≤ index ≤ of）")
    out: dict = {"id": sid, "index": int(index), "of": int(of)}
    route = slice_info.get("route")
    if route not in (None, ""):
        parts = [p.strip().upper() for p in str(route).split(">") if p.strip()]
        if not parts:
            raise ValueError("slice.route 给了但解析不出航点")
        if spec is not None:
            for wp in parts:
                try:
                    spec.block_to_index(wp)
                except RegistrationError as exc:
                    raise ValueError(f"slice.route 含非法航点：{wp}（{exc}）") from exc
        out["route"] = ">".join(parts)
    return out


def _norm_confidence(confidence: Any, spec: GridSpec | None) -> dict:
    """``confidence{格名: 0~1}``（可选的逐格置信度，供台账做劣化阈值）。"""
    if not isinstance(confidence, Mapping):
        raise ValueError(f"confidence 必须是 dict（收到 {type(confidence).__name__}）")
    out: dict[str, float] = {}
    for block, value in confidence.items():
        text = str(block).strip().upper()
        if spec is not None:
            try:
                spec.block_to_index(text)
            except RegistrationError as exc:
                raise ValueError(f"confidence 含非法格名：{text}（{exc}）") from exc
        try:
            num = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"confidence[{text}] 不是数：{value!r}") from exc
        if not 0.0 <= num <= 1.0:
            raise ValueError(f"confidence[{text}] 超出 [0,1]：{num}")
        out[text] = round(num, 3)
    if not out:
        raise ValueError("confidence 不能是空表（要不就别发这个键）")
    return out


def _norm_blocks(blocks: Iterable[Any], spec: GridSpec | None = None,
                 *, strict: bool = True) -> list[str]:
    """格名归一化：大写去空格；``strict`` 时校验列字母/行号在网格内。

    注意**不要**用它去挡"地图里没有的格"——``drone.py:351-372`` 明确不校验格名
    是否在地图内，把这类条目丢掉等于让核对功能永远报不出差异。
    """
    out: list[str] = []
    for item in blocks or ():
        text = str(item).strip().upper()
        if not text:
            continue
        if strict and spec is not None:
            try:
                spec.block_to_index(text)
            except RegistrationError:
                continue
        if text not in out:
            out.append(text)
    return out


# --------------------------------------------------------------------------- #
#  构造
# --------------------------------------------------------------------------- #
def build_heartbeat(*, ts: float | None = None, seq: int | None = None) -> dict:
    """心跳（建议 1 Hz；车侧 3 s 超时）。"""
    msg = {"type": "heartbeat", "ts": _now(ts)}
    if seq is not None:
        msg["seq"] = int(seq)
    return msg


def build_bridge_passable(half: str, block: str, confidence: float = 0.0,
                          *, ts: float | None = None, seq: int | None = None) -> dict:
    """桥洞通行格：``{"type":"bridge_passable","half":...,"block":...,"confidence":...,"ts":...}``."""
    h = str(half).strip().lower()
    if h not in BRIDGE_HALVES:
        raise ValueError(f"half 只能是 {BRIDGE_HALVES}，收到 {half!r}")
    b = str(block).strip().upper()
    if not b:
        raise ValueError("block 不能为空")
    msg = {"type": "bridge_passable", "half": h, "block": b,
           "confidence": round(float(confidence), 3), "ts": _now(ts)}
    if seq is not None:
        msg["seq"] = int(seq)
    return msg


def build_target_zone(blocks: Sequence[str], *, spec: GridSpec | None = None,
                      ts: float | None = None, seq: int | None = None) -> dict:
    """靶区格名列表（空表会被车侧记 `missing_fields` ⇒ 直接抛错，别发空表）。"""
    bl = _norm_blocks(blocks, spec)
    if not bl:
        raise ValueError("target_zone.blocks 不能为空（drone.py:294 会记 missing_fields）")
    msg = {"type": "target_zone", "blocks": bl, "ts": _now(ts)}
    if seq is not None:
        msg["seq"] = int(seq)
    return msg


def build_field_verify(mines: Sequence[str] | None = None,
                       obstacles: Sequence[str] | None = None, *,
                       spec: GridSpec | None = None, diff: Sequence[str] | None = None,
                       coverage: Mapping | None = None, slice: Mapping | None = None,  # noqa: A002
                       scan_id: str | None = None,
                       confidence: Mapping | None = None,
                       ts: float | None = None, seq: int | None = None) -> dict:
    """雷/障核对（+ 可选「片级覆盖」三键）。

    * 旧调用 ``build_field_verify(mines, obstacles)`` 的产物**逐字节不变**；
    * 给 ``coverage`` 时必须同时给 ``slice`` 与 ``scan_id``（三者是一组，
      缺任何一个车侧/台账都无法解释"这是哪一片、能不能当 ABSENT"）；
    * **空片（扫过且什么都没有）必须走这里**：``mines=[]`` / ``obstacles=[]``
      + ``coverage{complete:true}`` —— 旧接口 ``build_field_verify()`` 无参仍抛错，
      因为"什么都不发"永远不等于"扫过是空的"。
    """
    msg: dict = {"type": "field_verify"}
    if mines is not None:
        msg["mines"] = _norm_blocks(mines, spec)
    if obstacles is not None:
        msg["obstacles"] = _norm_blocks(obstacles, spec)
    if diff is not None:
        msg["diff"] = [str(d) for d in diff]
    if coverage is not None:
        msg["coverage"] = _norm_coverage(coverage, spec)
        if slice is None:
            raise ValueError("带 coverage 必须同时给 slice（片级汇总，决策层方案 §4.2）")
        if scan_id is None:
            raise ValueError("带 coverage 必须同时给 scan_id（幂等键，§5.1 第 4 条）")
        msg["slice"] = _norm_slice(slice, spec)
        msg["scan_id"] = _norm_scan_id(scan_id)
    elif slice is not None or scan_id is not None:
        raise ValueError("slice/scan_id 只能与 coverage 一起发（否则无从解释）")
    if confidence is not None:
        msg["confidence"] = _norm_confidence(confidence, spec)
    if mines is None and obstacles is None and coverage is None:
        raise ValueError("field_verify 至少要带 mines / obstacles / coverage 之一")
    msg["ts"] = _now(ts)
    if seq is not None:
        msg["seq"] = int(seq)
    return msg




# --------------------------------------------------------------------------- #
#  field_layout v2（AGC3 / B2：机端 → 车端的「本局规划真源」报文）
# --------------------------------------------------------------------------- #
def _norm_layout_cells(cells: Any, spec: GridSpec, *, key: str,
                       allow_empty: bool = True) -> list[str]:
    """``field_layout`` 专用格名归一化：**非法格直接抛错**。

    与 :func:`_norm_blocks` 的差别是有意的：车侧 ``normalize_field_layout``
    对非法格名**整份拒收**；发送端若静默丢格，等于发出一张缺格布局让车照跑。
    大写、去重、逐格 ``spec.block_to_index`` 校验。
    """
    if not isinstance(cells, (list, tuple)):
        raise ValueError(f"{key} 必须是格名列表（收到 {type(cells).__name__}）")
    out: list[str] = []
    for item in cells:
        text = str(item).strip().upper()
        if not text:
            raise ValueError(f"{key} 含空格名")
        try:
            spec.block_to_index(text)
        except RegistrationError as exc:
            raise ValueError(f"{key} 含非法格名：{text}（{exc}）") from exc
        if text not in out:
            out.append(text)
    if not out and not allow_empty:
        raise ValueError(f"{key} 不能为空")
    return out


def build_field_layout(*, version: int, start: str, end: str,
                       known: Sequence[str], bridges: Sequence[str],
                       spec: GridSpec,
                       mines: Sequence[str] = (), obstacles: Sequence[str] = (),
                       targets: Sequence[str] = (),
                       barrier_col_from: str | None = None,
                       barrier_row_hi: int | None = None,
                       barrier_row_lo: int | None = None,
                       target_cell: str = "", segment_id: int | None = None,
                       ts: float | None = None, seq: int | None = None) -> dict:
    """场地布局 v2（``type="field_layout"``）—— 本局**规划真源**报文。

    车侧校验真源：``dzrobot_v2/.../tasks/drone.py:181 normalize_field_layout``
    （1196 行版）。以下每条都对照车侧代码，不是推测：

    * ``version`` 单调：车侧对「旧版」记 ``layout_stale`` 拒收、「同版异内容」
      记 ``layout_rejects`` 拒收（``drone.py:69-71``）⇒ **改内容必须升 version**；
      同版同内容周期重发是安全心跳（不重写、不重复计数）。
    * ``known`` 非空 ⇒ v2（车端据此开**步进循环**；无 known 键 = v1「整图已知」
      旧语义，且归一化后**不补键**——``drone.py:217-229``）。本构造器恒发 known；
      要 v1 旧行为请别用本函数。
    * ``mines``/``obstacles``/``targets`` 键恒带（可空表）且必须 **⊆ known**
      —— 半张布局拿去规划 = 漏压雷路线，车侧宁可整份拒收（``:231-239``）。
    * ``bridges`` 至少 1 个、**不受 known 限制**（``:241-244``；桥洞格每局随机，
      可能落在未观测区，车侧正是这么放行的）。
    * ``target_cell`` 空 = 车只等不做；非空必须 ∈ ``known``（``:266-280``）。
    * ``segment_id`` 缺省取 ``version`` 且**恒显式发送**：车侧 docstring 写
      「缺省=version」但代码实际缺省 0（``:283-292``）——发送端不留歧义；
      这处漂移已在 AGC3 卡登记，待车端卡裁决。
    * ``barrier_*`` 三键**要么全给要么全不给**（不给 = 车侧吃 geo 缺省 D/6/7；
      只给一部分时另一部分吃缺省再查 hi<lo，发送端预测不了 ⇒ 禁止）。
    * ``spec`` **必填**：车侧对非法格名整份拒收，发送端没有网格表就无权发布局。
    """
    if isinstance(version, bool) or not isinstance(version, int):
        raise ValueError(f"version 必须是 int（收到 {version!r}）")
    if version < 0:
        raise ValueError(f"version 不能为负：{version}")
    known_l = _norm_layout_cells(known, spec, key="known", allow_empty=False)
    known_set = set(known_l)
    msg: dict = {
        "type": "field_layout",
        "version": version,
        "start": _norm_layout_cells([start], spec, key="start",
                                    allow_empty=False)[0],
        "end": _norm_layout_cells([end], spec, key="end",
                                  allow_empty=False)[0],
        "known": known_l,
    }
    for key, raw in (("mines", mines), ("obstacles", obstacles),
                     ("targets", targets)):
        cells = _norm_layout_cells(raw, spec, key=key)
        missing = [c for c in cells if c not in known_set]
        if missing:
            raise ValueError(f"{key} 含未观测格 {missing[:4]}（v2 要求 ⊆ known，"
                             f"车侧 drone.py:235-238 整份拒收）")
        msg[key] = cells
    msg["bridges"] = _norm_layout_cells(bridges, spec, key="bridges",
                                        allow_empty=False)
    trio = (barrier_col_from is not None, barrier_row_hi is not None,
            barrier_row_lo is not None)
    if any(trio):
        if not all(trio):
            raise ValueError("barrier_col_from/barrier_row_hi/barrier_row_lo "
                             "必须三键同给或同不给（车侧缺省各自独立回落，"
                             "给一半无法预测合成结果）")
        col = str(barrier_col_from).strip().upper()
        if col not in spec.col_letters:
            raise ValueError(f"barrier_col_from 非法：{col!r}（列字母表 "
                             f"{spec.col_letters[0]}~{spec.col_letters[-1]}）")
        for name, value in (("barrier_row_hi", barrier_row_hi),
                            ("barrier_row_lo", barrier_row_lo)):
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{name} 必须是 int（收到 {value!r}）")
        if not (1 <= barrier_row_hi < barrier_row_lo <= spec.rows):
            raise ValueError(f"挡板行区间非法：{barrier_row_hi}/{barrier_row_lo}"
                             f"（要求 1 ≤ hi < lo ≤ {spec.rows}）")
        msg["barrier_col_from"] = col
        msg["barrier_row_hi"] = barrier_row_hi
        msg["barrier_row_lo"] = barrier_row_lo
    tcell = str(target_cell or "").strip().upper()
    if tcell:
        try:
            spec.block_to_index(tcell)
        except RegistrationError as exc:
            raise ValueError(f"target_cell 非法：{tcell}（{exc}）") from exc
        if tcell not in known_set:
            raise ValueError(f"target_cell {tcell} 不在 known 内"
                             f"（车无法只走已知格到达，drone.py:278-279 拒收）")
    msg["target_cell"] = tcell
    seg = version if segment_id is None else segment_id
    if isinstance(seg, bool) or not isinstance(seg, int) or seg < 0:
        raise ValueError(f"segment_id 必须是非负 int（收到 {segment_id!r}）")
    msg["segment_id"] = seg
    msg["ts"] = _now(ts)
    if seq is not None:
        msg["seq"] = int(seq)
    return msg


# --------------------------------------------------------------------------- #
#  编码 / 校验
# --------------------------------------------------------------------------- #
def encode(payload: dict) -> str:
    """编码成 ``std_msgs/String.data``。分隔符风格与车侧 ``publish_drone`` 一致。"""
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def encode_all(payloads: Iterable[dict]) -> list[str]:
    return [encode(p) for p in payloads]


def validate_payload(payload: Any, spec: GridSpec | None = None) -> list[str]:
    """发送端自检：返回问题列表（空 = 车侧会正常处理）。

    这是**发送端的**严格版（比车侧严：``block`` 也查网格），用来在发射前拦住笔误。
    想完全照车侧口径（不查格名）就传 ``spec=None``。
    """
    problems: list[str] = []
    if not isinstance(payload, dict):
        return [f"载荷不是 dict：{type(payload).__name__}"]
    mtype = str(payload.get("type") or "").strip()
    if mtype not in PAYLOAD_TYPES:
        return [f"未知 type={mtype!r}（车侧会当未知类型忽略，见 drone.py:236-240）"]
    if mtype == "bridge_passable":
        half = str(payload.get("half") or "").strip().lower()
        block = str(payload.get("block") or "").strip().upper()
        if half not in BRIDGE_HALVES:
            problems.append(f"half 非法：{payload.get('half')!r}")
        if not block:
            problems.append("block 为空")
        elif spec is not None:
            try:
                spec.block_to_index(block)
            except RegistrationError as exc:
                problems.append(f"block 不在网格内：{exc}")
        if "confidence" in payload:
            try:
                c = float(payload["confidence"])
                if not 0.0 <= c <= 1.0:
                    problems.append(f"confidence 超出 [0,1]：{c}")
            except (TypeError, ValueError):
                problems.append(f"confidence 不是数：{payload.get('confidence')!r}")
    elif mtype == "target_zone":
        blocks = payload.get("blocks")
        if not isinstance(blocks, (list, tuple)) or not blocks:
            problems.append("blocks 必须是非空 list（空表 = 车侧 missing_fields）")
        elif spec is not None:
            for b in blocks:
                try:
                    spec.block_to_index(str(b))
                except RegistrationError as exc:
                    problems.append(f"blocks 含非法格名：{exc}")
    elif mtype == "field_verify":
        has_cov = "coverage" in payload
        if "mines" not in payload and "obstacles" not in payload and not has_cov:
            problems.append("mines/obstacles/coverage 至少要给一个")
        for key in ("mines", "obstacles"):
            if key in payload:
                raw = payload[key]
                if not isinstance(raw, (list, tuple)):
                    problems.append(f"{key} 必须是 list")
                elif spec is not None:
                    for b in raw:
                        try:
                            spec.block_to_index(str(b))
                        except RegistrationError as exc:
                            problems.append(f"{key} 含非法格名：{exc}")
        if has_cov:
            try:
                _norm_coverage(payload.get("coverage"), spec)
            except ValueError as exc:
                problems.append(f"coverage：{exc}")
            try:
                _norm_slice(payload.get("slice"), spec)
            except ValueError as exc:
                problems.append(f"slice：{exc}")
            try:
                _norm_scan_id(payload.get("scan_id"))
            except ValueError as exc:
                problems.append(f"scan_id：{exc}")
        elif "slice" in payload or "scan_id" in payload:
            problems.append("slice/scan_id 必须与 coverage 同时出现")
        if "confidence" in payload:
            try:
                _norm_confidence(payload.get("confidence"), spec)
            except ValueError as exc:
                problems.append(f"confidence：{exc}")
    elif mtype == "field_layout":
        # 车侧真源 drone.py:181-294；发送端严格版（spec 给定时查网格）
        ver = payload.get("version", 0)
        if isinstance(ver, bool) or not isinstance(ver, int) or ver < 0:
            problems.append(f"version 非法：{payload.get('version')!r}")
        known = payload.get("known")
        known_set: set = set()
        if known is None:
            problems.append("无 known 键（= v1 整图已知语义；发 v2 必须带非空 known）")
        elif not isinstance(known, (list, tuple)) or not known:
            problems.append("known 为空（v2 至少要有一个已观测格）")
        else:
            known_set = {str(c).strip().upper() for c in known}
        def _cell_ok(name: str) -> bool:
            if spec is None:
                return True
            try:
                spec.block_to_index(name)
                return True
            except RegistrationError:
                return False
        for key in ("start", "end"):
            v = payload.get(key)
            if not isinstance(v, str) or not v.strip():
                problems.append(f"缺少 {key}")
            elif not _cell_ok(v.strip().upper()):
                problems.append(f"{key} 不在网格内：{v!r}")
        for key in ("mines", "obstacles", "targets"):
            if key not in payload:
                problems.append(f"缺 {key} 键（缺键 = 车侧整份拒收，不是按空表猜）")
                continue
            raw = payload[key]
            if not isinstance(raw, (list, tuple)):
                problems.append(f"{key} 必须是 list")
                continue
            for c in raw:
                cn = str(c).strip().upper()
                if not _cell_ok(cn):
                    problems.append(f"{key} 含非法格名：{c!r}")
                elif known_set and cn not in known_set:
                    problems.append(f"{key} 含未观测格 {cn}（v2 要求 ⊆ known）")
        bridges = payload.get("bridges")
        if not isinstance(bridges, (list, tuple)) or not bridges:
            problems.append("bridges 至少要 1 个（本局穿洞格）")
        hi, lo = payload.get("barrier_row_hi"), payload.get("barrier_row_lo")
        if (hi is None) != (lo is None):
            problems.append("barrier_row_hi/lo 要同给或同不给")
        elif hi is not None:
            try:
                if not (1 <= int(hi) < int(lo)):
                    problems.append(f"barrier_row_hi({hi}) 必须 < barrier_row_lo({lo})")
            except (TypeError, ValueError):
                problems.append("barrier_row_* 不是整数")
        tcell = str(payload.get("target_cell") or "").strip().upper()
        if tcell:
            if not _cell_ok(tcell):
                problems.append(f"target_cell 不在网格内：{tcell!r}")
            elif known_set and tcell not in known_set:
                problems.append(f"target_cell {tcell} 不在 known 内")
        seg = payload.get("segment_id")
        if seg is not None and (isinstance(seg, bool) or not isinstance(seg, int) or seg < 0):
            problems.append(f"segment_id 非法：{seg!r}")
    return problems


# --------------------------------------------------------------------------- #
#  车侧解析镜像（用于"证明车能吃" —— 是镜像是复述，不是真跑车侧代码）
# --------------------------------------------------------------------------- #
def mirror_car_side_parse(raw: str, spec: GridSpec | None = None) -> dict:
    """**镜像** ``DroneTask._handle_raw`` 的判定：这条报文车侧会怎么处理。

    行号出处已在下面注释里逐条标出；本函数只做逻辑复述，不 Import 车侧代码
    （``dzrobot_v2`` 是别人的仓库，本工具链不依赖它）。
    """
    res: dict = {"raw_ok": True, "type": "", "action": "", "notes": []}
    if not isinstance(raw, str):
        return {"raw_ok": False, "type": "", "action": "丢弃(bad_messages+)",
                "notes": ["drone.py:206-210 非字符串"]}
    try:
        payload = json.loads(raw)
    except Exception as exc:                              # noqa: BLE001
        return {"raw_ok": False, "type": "", "action": "丢弃(bad_messages+)",
                "notes": [f"drone.py:211-218 非法 JSON：{type(exc).__name__}"]}
    if not isinstance(payload, dict):
        return {"raw_ok": False, "type": "", "action": "丢弃(bad_messages+)",
                "notes": ["drone.py:219-223 非 JSON 对象"]}
    mtype = str(payload.get("type") or "").strip()
    res["type"] = mtype
    if mtype == "heartbeat":
        res["action"] = "刷新心跳 / 可能发 EV_DRONE_ONLINE"
        res["notes"].append("drone.py:244-253")
    elif mtype == "bridge_passable":
        half = str(payload.get("half") or "").strip().lower()
        block = str(payload.get("block") or "").strip().upper()
        if half not in BRIDGE_HALVES or not block:
            res["action"] = "记 missing_fields（忽略）"
            res["notes"].append("drone.py:274-278 half/block 校验")
        else:
            res["action"] = f"写 P_DRONE：{half} 半场 → {block}；发 EV_BRIDGE_INFO"
            res["notes"].append("drone.py:273-289；ts 用**收到时刻**而非载荷 ts")
    elif mtype == "target_zone":
        blocks = payload.get("blocks")
        if not isinstance(blocks, (list, tuple)) or not blocks:
            res["action"] = "记 missing_fields（忽略）"
            res["notes"].append("drone.py:293-297")
        else:
            norm = [str(b).strip().upper() for b in blocks if str(b).strip()]
            res["action"] = f"与本地 targets 差集核对：{norm}"
            res["notes"].append("drone.py:298-304（畸形条目计入 missing_fields）")
    elif mtype == "field_verify":
        keys = [k for k in ("mines", "obstacles") if k in payload]
        cov = payload.get("coverage")
        if not keys:
            res["action"] = "记 missing_fields（忽略）"
            res["notes"].append("drone.py:324-326")
            if isinstance(cov, Mapping):
                res["notes"].append(
                    "带 coverage 也救不了：旧车侧只认 mines/obstacles ⇒ 空片必须显式发 "
                    "mines:[]/obstacles:[]（或等 t5b 台账消费 coverage）")
        else:
            res["action"] = f"与本地 {keys} 差集核对"
            res["notes"].append("drone.py:307-323")
            empty = [k for k in keys if not payload.get(k)]
            if empty:
                res["notes"].append(
                    f"{empty} 是空表：旧车侧会读成「本地有、无人机没有」⇒ "
                    "一条 diff（EV_FIELD_VERIFY 零订阅，只进遥测）")
            if isinstance(cov, Mapping):
                res["notes"].append(
                    "coverage/slice/scan_id：旧车侧不读（payload.get）；"
                    "片内 ABSENT 由 t5b 台账按 complete 语义消费")
    elif mtype == "field_layout":
        # 车侧真源 drone.py:181-294 normalize_field_layout（镜像只到 v1/v2 分流，
        # 字段级判定以 validate_payload + 契约测试里的真函数为准）
        if "known" in payload:
            res["action"] = "v2：校验通过 → 写 P_FIELD_LAYOUT（规划真源）+ 步进循环"
            res["notes"].append("drone.py:217-229（known 在 ⇒ v2）；version 单调拒旧/拒同版异内容 :69-71")
        else:
            res["action"] = "v1：校验通过 → 写 P_FIELD_LAYOUT（旧『规划一次』路径）"
            res["notes"].append("drone.py:217-229（无 known ⇒ v1 整图已知语义，不补键）")
        res["notes"].append("任一必填/格式字段非法 ⇒ 整份拒收并保留上一份好布局 :67-68")
    else:
        res["action"] = "未知类型：记日志（不崩）"
        res["notes"].append("drone.py:236-240")
    seq = payload.get("seq")
    if seq is not None:
        res["notes"].append(f"带 seq={seq}：车侧会回 ack{{of:{seq}}}（drone.py:255-270）")
    return res


def coverage_absent_cells(coverage: Mapping, reported: Iterable[str] = (), *,
                          spec: GridSpec | None = None) -> list[str] | None:
    """由 ``coverage`` 推 ABSENT 格清单 —— **负面证据的唯一来源**。

    * ``complete=True``  ⇒ 返回 ``coverage.cells`` 中**未**被报出的格（= ABSENT）；
    * ``complete=False`` ⇒ 返回 ``None``（= UNKNOWN，**不得**当 ABSENT）；
    * 缺 ``complete`` 键 ⇒ 抛 ``ValueError``（分不清"空的"与"没扫"，不许猜）。

    消费端（t5b 台账）必须只用这个函数判空，别自己写 ``set(cells) - set(detected)``——
    那会把不完整的片也算成空的。
    """
    if not isinstance(coverage, Mapping):
        raise ValueError(f"coverage 必须是 dict（收到 {type(coverage).__name__}）")
    cells = _norm_cells(coverage.get("cells"), spec)
    complete = coverage.get("complete")
    if complete is None:
        raise ValueError("coverage 缺 complete：没有它就分不清『空的』与『没扫』")
    if not bool(complete):
        return None
    seen = {str(b).strip().upper() for b in reported or ()}
    return [c for c in cells if c not in seen]


def field_verify_absent(payload: Mapping, *, spec: GridSpec | None = None) -> list[str] | None:
    """从一条 ``field_verify`` 载荷推 ABSENT（把 mines/obstacles/targets 当作已报出）。"""
    if not isinstance(payload, Mapping) or str(payload.get("type") or "") != "field_verify":
        raise ValueError("只接受 type=field_verify 的载荷")
    if "coverage" not in payload:
        raise ValueError("载荷没有 coverage ⇒ 无负证据语义（旧报文，只能报差异）")
    reported: list[str] = []
    for key in ("mines", "obstacles", "targets"):
        raw = payload.get(key)
        if isinstance(raw, (list, tuple)):
            reported.extend(str(b) for b in raw)
    return coverage_absent_cells(payload.get("coverage"), reported, spec=spec)


def recommended_cadence(*, heartbeat_hz: float = 1.0) -> dict:
    """建议的发布节奏（依据车侧的 3 s 心跳超时 / 30 s 情报过期）。"""
    return {
        "heartbeat": f"{heartbeat_hz:g} Hz（车侧 3 s 超时，心跳是唯一保活手段）",
        "bridge_passable": "首次检到立即发；此后每次重检到同结论也重发（续 30 s 窗口）",
        "target_zone": "一次检测一条，带 seq 要 ack",
        "field_verify": "一片一条（带 coverage/slice/scan_id）；重发同一片必须复用 scan_id",
        "field_layout": "每局首扫定版后 version+1 发一次；同版同内容可周期重发当心跳；改内容必须升 version（同版异内容车侧拒收）",
        "topic": DRONE_TO_CAR_TOPIC,
        "qos": QOS_DRONE_LINK,
        "note": "发车前必须发（Mission 最多等 2 s，等不到就用默认 K9(下)/K5(上)）",
    }
