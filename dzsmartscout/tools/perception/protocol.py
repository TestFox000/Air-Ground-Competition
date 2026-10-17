# -*- coding: utf-8 -*-
r"""协议发射器：把感知结果编成**车侧能吃的 JSON**。

## 契约来源（逐条对照，不靠记忆）

消费端：``dzrobot_v2/src/dzbrain/dzbrain/tasks/drone.py``（484 行，`DroneTask`，5 Hz）

| 报文 | 车侧要求的字段 | 车侧行为 | 出处 |
|---|---|---|---|
| ``heartbeat`` | ``type`` | 刷新心跳；>3 s 无心跳 → ``online=False`` + 一次告警 | ``drone.py:228,244-253,379-391``；``robot.yaml`` ``heartbeat_timeout_s: 3.0`` |
| ``bridge_passable`` | ``half``∈{lower,upper}、``block``非空、``confidence`` | 写 ``P_DRONE``，``bridge_of(half)`` 供 Mission 取格；>30 s 过期返回 None | ``drone.py:273-289``、``info_stale_s: 30.0`` |
| ``target_zone`` | ``blocks``（非空 list） | 与本地 ``map.yaml.targets`` 做差集 → ``field_diff`` | ``drone.py:292-304`` |
| ``field_verify`` | ``mines`` / ``obstacles``（list） | 同上（雷/障） | ``drone.py:307-326`` |
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
   （会被当成未知类型忽略）。本模块只产出上述 4 种类型。

话题与 QoS（``dzrobot_v2/src/dzbrain/dzbrain/io/contract.py:40-41,94-96``）：
``/dzlink/drone_to_car`` / ``std_msgs/String`` / **depth 10 · BEST_EFFORT**。
QoS 配错（机侧 RELIABLE）会**静默收不到**——跨机 ROS 2 最经典的失败模式。
"""

from __future__ import annotations

import json
import time
from typing import Any, Iterable, Sequence

from .grid import GridSpec, RegistrationError

__all__ = [
    "DRONE_TO_CAR_TOPIC",
    "CAR_TO_DRONE_TOPIC",
    "PAYLOAD_TYPES",
    "BRIDGE_HALVES",
    "build_heartbeat",
    "build_bridge_passable",
    "build_target_zone",
    "build_field_verify",
    "encode",
    "encode_all",
    "validate_payload",
    "mirror_car_side_parse",
    "recommended_cadence",
]

DRONE_TO_CAR_TOPIC = "/dzlink/drone_to_car"
CAR_TO_DRONE_TOPIC = "/dzlink/car_to_drone"
QOS_DRONE_LINK = {"depth": 10, "reliability": "best_effort"}
PAYLOAD_TYPES = ("heartbeat", "bridge_passable", "target_zone", "field_verify")
BRIDGE_HALVES = ("lower", "upper")


def _now(ts: float | None) -> float:
    return round(float(time.time() if ts is None else ts), 3)


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
                       ts: float | None = None, seq: int | None = None) -> dict:
    """雷/障核对。至少要给一个键（两个都不给 = 车侧 ``missing_fields``）。"""
    msg: dict = {"type": "field_verify", "ts": _now(ts)}
    if mines is not None:
        msg["mines"] = _norm_blocks(mines, spec)
    if obstacles is not None:
        msg["obstacles"] = _norm_blocks(obstacles, spec)
    if len(msg) == 2:                       # 只有 type+ts
        raise ValueError("field_verify 至少要带 mines 或 obstacles 之一")
    if diff is not None:
        msg["diff"] = [str(d) for d in diff]
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
        if "mines" not in payload and "obstacles" not in payload:
            problems.append("mines/obstacles 至少要给一个")
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
        if not keys:
            res["action"] = "记 missing_fields（忽略）"
            res["notes"].append("drone.py:324-326")
        else:
            res["action"] = f"与本地 {keys} 差集核对"
            res["notes"].append("drone.py:307-323")
    else:
        res["action"] = "未知类型：记日志（不崩）"
        res["notes"].append("drone.py:236-240")
    seq = payload.get("seq")
    if seq is not None:
        res["notes"].append(f"带 seq={seq}：车侧会回 ack{{of:{seq}}}（drone.py:255-270）")
    return res


def recommended_cadence(*, heartbeat_hz: float = 1.0) -> dict:
    """建议的发布节奏（依据车侧的 3 s 心跳超时 / 30 s 情报过期）。"""
    return {
        "heartbeat": f"{heartbeat_hz:g} Hz（车侧 3 s 超时，心跳是唯一保活手段）",
        "bridge_passable": "首次检到立即发；此后每次重检到同结论也重发（续 30 s 窗口）",
        "target_zone": "一次检测一条，带 seq 要 ack",
        "field_verify": "一次检测一条，带 seq 要 ack",
        "topic": DRONE_TO_CAR_TOPIC,
        "qos": QOS_DRONE_LINK,
        "note": "发车前必须发（Mission 最多等 2 s，等不到就用默认 K10(下)/K5(上)）",
    }
