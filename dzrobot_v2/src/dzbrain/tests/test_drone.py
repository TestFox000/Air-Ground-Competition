# -*- coding: utf-8 -*-
r"""DroneTask（无人机链路）测试 —— 纯脚本，无 pytest。

运行：python src\dzbrain\tests\test_drone.py

覆盖（任务书七条 + 失效兜底）：
  1. 合法 bridge_passable → P_DRONE.bridge_of("lower")=="K9"；
  2. 心跳超时 → online=False 且**只发一次** EV_DRONE_OFFLINE（再跑 20 拍不重复）；
  3. 恢复（新心跳）→ EV_DRONE_ONLINE；
  4. 情报过期（> drone.info_stale_s）→ bridge_of() 返回 None（用现成方法）；
  5. 非法 JSON / 未知 type / 缺字段 / 非字符串 → 不崩、记日志、计数正确；
  6. target_zone 与本地 map.targets 不一致 → field_diff 有内容（一致时不报）；
  7. 车状态上报被发出（合法 JSON、type=="car_state"）；
  8. 附加：ack 回执、field_verify 雷/障差异、drone.enabled=False 整任务空转、
     本地地图三级来源（构造参数 / cfg 路径 / 仓库默认 config/map.yaml）。
  9. ⑩ 覆盖语义（G5-2）：片级 coverage 账本 / complete 门控 / 幂等重发 / 比例 154；
 10. ⑪ EV_FIELD_VERIFY 生产订阅方（G5-3）：总线事件 → 黑盒汇总 / policy record|alert|hold|off；
 11. ⑫ 旧报文兼容 + 关掉新逻辑行为不变（旧 JSON、无新键 → 决策可观测逐位相同）；
 12. ⑬ 畸形 coverage 降级：不崩、计数正确、其余字段照旧处理。
"""

from __future__ import annotations

import json
import pathlib
import sys
import tempfile

import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:                                    # 车端/Windows 控制台默认不是 UTF-8
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from dzbrain.core import geometry as geo                                       # noqa: E402
from dzbrain.core.blackboard import (P_DRONE, P_MISSION_PHASE, P_POSE,         # noqa: E402
                                     P_ROUTE, P_SAFETY, P_TELEMETRY,
                                     P_FIELD_LAYOUT as CORE_P_FIELD_LAYOUT,
                                     P_FIELD_VERIFY as CORE_P_FIELD_VERIFY, Blackboard)
from dzbrain.core.config import Config                                         # noqa: E402
from dzbrain.core.events import (EV_BRIDGE_INFO, EV_DRONE_OFFLINE,             # noqa: E402
                                 EV_DRONE_ONLINE, EV_FIELD_VERIFY, EventBus)
from dzbrain.core.state import DroneState                                      # noqa: E402
from dzbrain.io import contract                                            # noqa: E402
from dzbrain.tasks.drone import (P_DRONE_COVERAGE, P_FIELD_LAYOUT,            # noqa: E402
                                 P_FIELD_VERIFY, DRONE_PAYLOAD_TYPES,
                                 FIELD_LAYOUT_VERSION,
                                 CoverageLedger, DroneTask, FieldVerifyConsumer,
                                 default_map_path, normalize_field_layout)
from fakes import FakeBridge, FakeClock                                        # noqa: E402
from harness import check, finish, section                                     # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]          # <ws> = dzrobot_v2/
MAP_PATH = WS / "config" / "map.yaml"
CONFIG_PATH = WS / "config" / "robot.yaml"

#: 与现场 `config/map.yaml` 同口径的本地地图（靶区两格相连、雷/障各若干）
LOCAL_MAP = {
    "map": {
        "name": "测试地图",
        "mines": ["H1", "G2", "C4"],
        "obstacles": ["D1", "B3"],
        "targets": ["F11", "G11"],
        "bridges": ["K4", "K9"],
    }
}


# --------------------------------------------------------------------------- #
#  脚手架
# --------------------------------------------------------------------------- #
def build(overrides: dict | None = None, *, local_map=LOCAL_MAP, seed_pose=True,
          drone_extra: dict | None = None, log=None):
    """造 (task, bridge, bb, bus, clock, events)。事件全量订阅，便于断言"只发一次"。

    `overrides` 对同名字典做**合并**（旧用法 `build({"drone": {"enabled": False}})`
    语义不变：其余键保持缺省）；`drone_extra` 是追加到 `drone.*` 的便捷入口；
    `log` 传入列表可收集任务日志（⑫ 兼容性断言用）。
    """
    clock = FakeClock()
    bridge = FakeBridge(clock)
    bb = Blackboard()
    bus = EventBus()
    data: dict = {
        "drone": {"enabled": True, "heartbeat_timeout_s": 3.0, "info_stale_s": 30.0,
                  **(drone_extra or {})},
        "mission": {"auto_start": False},
        "logging": {"rate_limit_s": 1.0},
        "telemetry": {},
    }
    for section_key, value in (overrides or {}).items():
        if isinstance(value, dict) and isinstance(data.get(section_key), dict):
            data[section_key] = {**data[section_key], **value}
        else:
            data[section_key] = value
    cfg = Config.from_dict(data)
    if seed_pose:
        bb.set(P_POSE, geo.Pose(*geo.block_to_meters("N10"), yaw=geo.HEADING_Y),
               source="test")
    bb.set(P_MISSION_PHASE, "DRIVE", source="test")
    sub = bus.subscribe("watch", None)             # 全量订阅
    task = DroneTask(bridge, bb, bus, cfg, log=(log or (lambda m: None)),
                     clock=clock.now, local_map=local_map)
    task.setup()
    return task, bridge, bb, bus, clock, sub


def feed(task, clock, n: int = 1, dt: float = 0.2) -> None:
    for _ in range(n):
        clock.advance(dt)
        task.tick()


def events_of(sub, ev_type: str | None = None) -> list:
    """取事件；`ev_type=None` 表示"取走并丢弃"（清队列，用于阶段性断言）。"""
    out = sub.poll(512)
    return out if ev_type is None else [e for e in out if e.type == ev_type]


def push(bridge, payload) -> None:
    bridge.push_drone(payload if isinstance(payload, str)
                      else json.dumps(payload, ensure_ascii=False))


# --------------------------------------------------------------------------- #
section("① 合法 bridge_passable → P_DRONE.bridge_of('lower') == 'K9'")
task, bridge, bb, bus, clock, sub = build()
push(bridge, {"type": "heartbeat", "seq": 1})
push(bridge, {"type": "bridge_passable", "half": "lower", "block": "K9",
              "confidence": 0.92, "ts": 1000.0})
feed(task, clock)
st = bb.get(P_DRONE)
check("P_DRONE 类型是 DroneState", isinstance(st, DroneState), f"{type(st).__name__}")
check("bridge_of('lower') == 'K9'",
      st.bridge_of("lower", now=clock.now(), stale_s=30.0) == "K9",
      f"got {st.bridge_of('lower', now=clock.now(), stale_s=30.0)!r}")
check("半场不匹配时返回 None（'upper' 没有情报）",
      st.bridge_of("upper", now=clock.now(), stale_s=30.0) is None)
check("confidence 落盘 0.92", abs(st.bridge_confidence - 0.92) < 1e-9,
      f"got {st.bridge_confidence}")
check("bridge_ts == 本拍时钟", abs(st.bridge_ts - clock.now()) < 1e-9,
      f"got {st.bridge_ts} now={clock.now()}")
check("心跳让 online=True", st.online is True)
check("消息计数 messages == 2", task.messages == 2, f"got {task.messages}")
check("last_type == 'bridge_passable'", task.last_type == "bridge_passable",
      f"got {task.last_type!r}")
check("发过 EV_BRIDGE_INFO（Mission 用它选穿洞格）",
      len(events_of(sub, EV_BRIDGE_INFO)) == 1)
check("便捷直读 task.bridge_of('lower') 同值", task.bridge_of("lower") == "K9")

# --------------------------------------------------------------------------- #
section("② 心跳超时 → online=False 且只发一次 EV_DRONE_OFFLINE（再跑 20 拍不重复）")
task, bridge, bb, bus, clock, sub = build()
push(bridge, {"type": "heartbeat"})
feed(task, clock)                                   # t=1000.2，online=True
check("首次心跳只发 ONLINE、不发 OFFLINE",
      len(events_of(sub, EV_DRONE_OFFLINE)) == 0 and task.online is True,
      f"events={[e.type for e in events_of(sub)]}")
feed(task, clock, n=10, dt=0.2)                     # +2.0 s，未到 3 s 阈值
check("2.0 s 无心跳时仍在线", bb.get(P_DRONE).online is True,
      f"silent={task._silent_s(clock.now())}")
check("未超时前不发离线事件", len(events_of(sub, EV_DRONE_OFFLINE)) == 0)
feed(task, clock, n=6, dt=0.2)                      # 累计 3.2 s > 3.0 s
offline_first = events_of(sub, EV_DRONE_OFFLINE)
check("超时后 online=False", bb.get(P_DRONE).online is False)
check("发出 1 条 EV_DRONE_OFFLINE", len(offline_first) == 1,
      f"got {len(offline_first)}")
check("事件带 silent_s / timeout_s 证据",
      offline_first and abs(offline_first[0].data["timeout_s"] - 3.0) < 1e-9
      and offline_first[0].data["silent_s"] >= 3.0,
      f"got {offline_first[0].data if offline_first else None}")
feed(task, clock, n=20, dt=0.2)                     # 再跑 20 拍（+4 s）
more = events_of(sub, EV_DRONE_OFFLINE)
check("★ 再跑 20 拍不重复发（只在状态变化时发一次）", len(more) == 0,
      f"got {len(more)} 条")
check("持续离线时 P_DRONE.online 保持 False", bb.get(P_DRONE).online is False)

section("③ 恢复 → EV_DRONE_ONLINE")
push(bridge, {"type": "heartbeat"})
feed(task, clock)
online_events = events_of(sub, EV_DRONE_ONLINE)
check("恢复发出 1 条 EV_DRONE_ONLINE", len(online_events) == 1,
      f"got {len(online_events)}")
check("online 回到 True", bb.get(P_DRONE).online is True)
check("事件带离线时长 silent_s >= 3 s（★ 顺序 bug 回归：不能恒为 0）",
      online_events and online_events[0].data["silent_s"] >= 3.0,
      f"got {online_events[0].data if online_events else None}")
push(bridge, {"type": "heartbeat"})
feed(task, clock)
check("在线期间再收心跳不重复发 ONLINE",
      len(events_of(sub, EV_DRONE_ONLINE)) == 0)

# --------------------------------------------------------------------------- #
section("④ 情报过期 → bridge_of() 返回 None")
task, bridge, bb, bus, clock, sub = build()
push(bridge, {"type": "bridge_passable", "half": "upper", "block": "K4",
              "confidence": 0.8})
feed(task, clock)
st = bb.get(P_DRONE)
check("新鲜时可用（29 s < info_stale_s 30 s）",
      st.bridge_of("upper", now=clock.now() + 29.0, stale_s=30.0) == "K4")
check("过期时返回 None（31 s > 30 s）",
      st.bridge_of("upper", now=clock.now() + 31.0, stale_s=30.0) is None)
check("此刻任务直读也为 None（内部用同一个判据）",
      task.bridge_of("upper", now=clock.now() + 31.0) is None)
check("情报含未在地图内的格名（K4 在本地图里）也能落地——无人机是权威",
      st.bridge_block == "K4", f"got {st.bridge_block!r}")

# --------------------------------------------------------------------------- #
section("⑤ 非法 JSON / 未知 type / 缺字段 / 非字符串 → 不崩、记日志")
task, bridge, bb, bus, clock, sub = build()
push(bridge, "{不是合法 JSON")                        # 非法 JSON
push(bridge, {"type": "weather_report", "temp": 25})  # 未知 type
push(bridge, {"type": "bridge_passable", "half": "middle", "block": "K9"})  # 半场非法
push(bridge, {"type": "bridge_passable", "half": "lower"})                  # 缺 block
push(bridge, {"type": "bridge_passable", "half": "lower", "block": "Z9"})   # 非法格名
push(bridge, {"type": "target_zone"})                                      # 缺 blocks
push(bridge, 12345)                                     # 非字符串
feed(task, clock)                                       # ★ 不抛异常 = 通过
st = bb.get(P_DRONE)
check("非法 JSON 计数 bad_messages >= 2（含非字符串）", task.bad_messages == 2,
      f"got {task.bad_messages}")
check("未知 type 被登记（不是静默丢弃）",
      task.unknown_types.get("weather_report") == 1, f"got {task.unknown_types}")
check("缺字段计数 == 3（半场非法 / 缺 block / target_zone 缺 blocks）",
      task.missing_fields == 3, f"got {task.missing_fields}")
check("合法消息计数 = 解析成功的 JSON 对象（4 条 bridge_passable + 1 条 target_zone）",
      task.messages == 5, f"got {task.messages}")
check("半场取值的合法性仍是硬判据（'middle' 被拒，保留上一次合法半场）",
      st.bridge_half == "lower", f"got {st.bridge_half!r}")
check("★ 不校验'格名在不在本地地图'：Z9 在场地外，但无人机是权威 → 不丢",
      st.bridge_block == "Z9", f"got {st.bridge_block!r}")
check("未知 type 也尝试回执（带 seq）无异常", task.acks_sent == 0)
feed(task, clock, n=3)
check("坏消息之后任务照常跑（P_DRONE 仍在更新）", bb.get(P_DRONE) is not None)

# --------------------------------------------------------------------------- #
section("⑥ target_zone 与本地 map 不一致 → field_diff 有内容")
task, bridge, bb, bus, clock, sub = build()
push(bridge, {"type": "target_zone", "blocks": ["F11", "G11"]})   # 与本地一致
feed(task, clock)
check("一致时 field_diff 为空", bb.get(P_DRONE).field_diff == (),
      f"got {bb.get(P_DRONE).field_diff}")
check("一致时也不发 EV_FIELD_VERIFY", len(events_of(sub, EV_FIELD_VERIFY)) == 0)
push(bridge, {"type": "target_zone", "blocks": ["G11", "", 7]})    # 畸形条目（另一拍）
feed(task, clock)
check("畸形条目计入 missing_fields（不静默）", task.missing_fields == 1,
      f"got {task.missing_fields}")
check("畸形条目被剔除后剩下的 {G11} 与本地 {F11,G11} 确实不同 → 如实报差异",
      len(bb.get(P_DRONE).field_diff) == 1, f"got {bb.get(P_DRONE).field_diff}")
push(bridge, {"type": "target_zone", "blocks": ["G11", "H12"]})   # 不一致（新开一段核对）
task.field_diffs.clear()
events_of(sub)                                    # 清事件队列（上一条畸形消息也发过 verify）
feed(task, clock)
diffs = bb.get(P_DRONE).field_diff
check("不一致时 field_diff 有 1 条", len(diffs) == 1, f"got {diffs}")
check("差异文本含两侧证据（仅无人机 / 仅本地）",
      "H12" in diffs[0] and "F11" in diffs[0] and "仅无人机" in diffs[0]
      and "仅本地" in diffs[0], f"got {diffs[0]!r}")
verify = events_of(sub, EV_FIELD_VERIFY)
check("发出 EV_FIELD_VERIFY（tag/remote/local 齐备）",
      len(verify) == 1 and verify[0].data["tag"] == "target_zone"
      and verify[0].data["only_remote"] == ["H12"]
      and verify[0].data["only_local"] == ["F11"],
      f"got {verify[0].data if verify else None}")

section("⑥b field_verify：雷/障差异同样记进 field_diff")
push(bridge, {"type": "field_verify", "mines": ["H1", "G2", "C4"],
              "obstacles": ["D1", "Z3"]})
feed(task, clock)
diffs = bb.get(P_DRONE).field_diff
check("雷一致 + 障不一致 → 只多 1 条差异", len(diffs) == 2, f"got {diffs}")
check("雷一致时不产生差异（只有 obstacles 那条）",
      sum(1 for d in diffs if "mines" in d) == 0, f"got {diffs}")
check("障差异定位到 field_verify.obstacles", "obstacles" in diffs[-1],
      f"got {diffs[-1]!r}")
check("差异列表有上限（diff_max=8）", len(diffs) <= task.diff_max, f"got {len(diffs)}")

# --------------------------------------------------------------------------- #
section("⑦ 车状态上报被发出（合法 JSON、type=='car_state'）")
task, bridge, bb, bus, clock, sub = build()
feed(task, clock)
check("drone_out 非空", len(bridge.drone_out) >= 1, f"got {bridge.drone_out}")
body = json.loads(bridge.drone_out[0])
check("是合法 JSON 且 type == 'car_state'", body.get("type") == "car_state",
      f"got {body}")
check("block 从 P_POSE 推出（N10）", body.get("block") == "N10", f"got {body}")
check("含 yaw_deg（地图朝向 90°）", abs(body.get("yaw_deg", 0) - 90.0) < 0.2,
      f"got {body.get('yaw_deg')}")
check("含 phase（从 P_MISSION_PHASE 读）", body.get("phase") == "DRIVE",
      f"got {body.get('phase')}")
check("含 ts 与 seq", isinstance(body.get("ts"), (int, float))
      and isinstance(body.get("seq"), int), f"got {body}")
n_before = len(bridge.drone_out)
feed(task, clock, n=4, dt=0.2)                       # +0.8 s（周期 0.7 s）→ 应再发 1 条
check("按 drone.car_state_period_s 节流（0.8 s 只多 1 条）",
      len(bridge.drone_out) == n_before + 1,
      f"got {len(bridge.drone_out) - n_before} 条")
bb.set(P_MISSION_PHASE, "STOP", source="test")
clock.advance(0.05)
task.tick()
check("阶段变化立即补发一条（不等周期）",
      json.loads(bridge.drone_out[-1]).get("phase") == "STOP",
      f"got {bridge.drone_out[-1]}")

section("⑦c car_state 的 done/done_seq 透传到机端（D1b）")
task, bridge, bb, bus, clock, sub = build()
bb.set("car_state", {
    "phase": "SEGMENT_DONE", "segment_id": 3, "known_count": 154,
    "target_cell": "F4", "stop_kind": "stop", "version": 10,
    "reason": "段完成", "done_seq": 3,
    "done": {"segment_id": 3, "block": "F4", "stop_kind": "stop",
             "skipped": False, "elapsed_s": 12.0, "seq": 3, "replay": True},
}, source="test")
feed(task, clock)
body = json.loads(bridge.drone_out[-1])
check("phase 来自 P_CAR_STATE", body.get("phase") == "SEGMENT_DONE",
      f"got {body.get('phase')}")
check("done_seq 透传", body.get("done_seq") == 3, f"got {body}")
check("done 透传（block/seq/replay）",
      isinstance(body.get("done"), dict) and body["done"].get("block") == "F4"
      and body["done"].get("seq") == 3 and body["done"].get("replay") is True,
      f"got {body.get('done')}")

section("⑦b 收到带 seq 的消息 → 回 ack")
task, bridge, bb, bus, clock, sub = build()
push(bridge, {"type": "bridge_passable", "half": "lower", "block": "K9", "seq": 1234})
feed(task, clock)
acks = [json.loads(x) for x in bridge.drone_out if json.loads(x).get("type") == "ack"]
check("发出 1 条 ack", len(acks) == 1 and task.acks_sent == 1, f"got {acks}")
check("ack.of == 1234", acks and acks[0].get("of") == 1234, f"got {acks}")
task, bridge, bb, bus, clock, sub = build()
push(bridge, {"type": "heartbeat"})                # 没带 seq → 不回执
feed(task, clock)
check("没带 seq 的消息不回执（省链路）",
      all(json.loads(x).get("type") != "ack" for x in bridge.drone_out),
      f"got {bridge.drone_out}")

# --------------------------------------------------------------------------- #
section("⑧ drone.enabled=False → 整任务空转（不读不发不写）")
task, bridge, bb, bus, clock, sub = build({"drone": {"enabled": False}})
push(bridge, {"type": "bridge_passable", "half": "lower", "block": "K9"})
writes_before = bb.writes
feed(task, clock, n=10, dt=0.2)
check("没有写 P_DRONE", bb.get(P_DRONE) is None)
check("没有发任何车状态", bridge.drone_out == [], f"got {bridge.drone_out}")
check("没有消费收件箱（消息还在）", len(bridge.take_drone_messages(16)) == 1)
check("黑盒写入次数没变（一个键都没写）", bb.writes == writes_before,
      f"writes {writes_before} → {bb.writes}")
check("没有发上线/离线事件",
      len(events_of(sub, EV_DRONE_ONLINE)) == 0
      and len(events_of(sub, EV_DRONE_OFFLINE)) == 0)

# --------------------------------------------------------------------------- #
section("⑨ 本地地图三级来源")
check("默认路径指向 <repo>/config/map.yaml 且存在",
      default_map_path().name == "map.yaml" and default_map_path().is_file(),
      f"got {default_map_path()}")
task, bridge, bb, bus, clock, sub = build(local_map=None)
check("不传 local_map → 自动读仓库默认地图（loaded=True）", task.map_loaded is True,
      f"got {task.map_loaded} source={task._map_source}")
# ★★ t22 重钉（车端不留地图文件同卡）：**期望值从 map.yaml 现场读**，不再硬编码某张图。
#   原断言钉 {E5,F5}（第三张图），换到第六张图（D5+D6）后变成"数据过期红"——判据本身
#   （"靶区从 map.yaml 读出、且雷/障计数与图一致"）不变，只是样本改为数据驱动。
_map_yaml = yaml.safe_load(MAP_PATH.read_text(encoding="utf-8"))["map"]
_exp_targets = frozenset(str(x).strip().upper() for x in _map_yaml["targets"])
_exp_mines = frozenset(str(x).strip().upper() for x in _map_yaml["mines"])
_exp_obstacles = frozenset(str(x).strip().upper() for x in _map_yaml["obstacles"])
check("靶区从 map.yaml 读出（与当前图逐格一致）",
      task.targets == _exp_targets, f"got {sorted(task.targets)} 期望 {sorted(_exp_targets)}")
check("雷/障集合与 map.yaml 逐格一致",
      task.mines == _exp_mines and task.obstacles == _exp_obstacles,
      f"mines={len(task.mines)} obstacles={len(task.obstacles)}"
      f" 期望 {len(_exp_mines)}/{len(_exp_obstacles)}")
skip, bridge2, bb2, bus2, clock2, sub2 = build(
    {"drone": {"map_path": str(pathlib.Path(__file__).resolve().parent
                               / "fixtures" / "map_field.yaml")}},
    local_map={"map": {"targets": ["A1"]}})
check("构造参数优先（显式注入覆盖文件）",
      skip.targets == frozenset({"A1"}), f"got {sorted(skip.targets)}")

# --------------------------------------------------------------------------- #
section("⑩ 覆盖语义（G5-2）：片级 coverage 账本 / complete 门控 / 幂等 / 比例")
#   机侧 t5a 的 field_verify 形状：
#     coverage{cells,complete,mode} + slice{id,index,of,route} + scan_id
#   车侧语义：complete=True 才允许「片内未报 = ABSENT」；同一 (scan_id,slice.id) 幂等。
local = {"map": {"mines": ["H1", "G2", "C4"], "obstacles": ["D1", "B3"],
                 "targets": ["F11", "G11"]}}
task, bridge, bb, bus, clock, sub = build(local_map=local)
push(bridge, {"type": "heartbeat"})
push(bridge, {"type": "field_verify", "mines": [], "obstacles": [],
              "coverage": {"cells": ["A1", "A2", "B1"], "complete": True, "mode": "strip"},
              "slice": {"id": "S1", "index": 1, "of": 2, "route": "A1>A2>B1"},
              "scan_id": "SCAN-A", "seq": 10})
feed(task, clock)
cov = bb.get(P_DRONE_COVERAGE)
check("首片后写入 P_DRONE_COVERAGE", isinstance(cov, dict), f"{cov}")
check("cells_done = 3（完整片才计已探明）", cov["cells_done"] == 3, f"{cov}")
check("覆盖率分母 = 154（geometry 14×11）",
      cov["cells_total"] == 154 and abs(cov["ratio"] - round(3 / 154, 4)) < 1e-12, f"{cov}")
check("complete_slices=1 / done=False（还差 S2）",
      cov["complete_slices"] == 1 and cov["done"] is False, f"{cov}")
check("coverage_updates=1 / duplicates=0 / bad=0",
      task.coverage_updates == 1 and task.coverage_duplicates == 0
      and task.coverage_bad == 0, f"{task.coverage_updates}")
check("★ 片内既无本地要素、也无报出 ⇒ 不产生差异（旧口径会误报）",
      task.field_diffs == [], f"{task.field_diffs}")

push(bridge, {"type": "field_verify", "mines": ["C4"], "obstacles": [],
              "coverage": {"cells": ["C4"], "complete": True},
              "slice": {"id": "S2", "index": 2, "of": 2, "route": "C4"},
              "scan_id": "SCAN-A", "seq": 11})
feed(task, clock)
cov = bb.get(P_DRONE_COVERAGE)
check("第二片后 cells_done=4 且 done=True（1..of 齐全 + 全 complete）",
      cov["cells_done"] == 4 and cov["done"] is True, f"{cov}")
check("片内一致（本地 C4 被报出）不产生差异", task.field_diffs == [], f"{task.field_diffs}")
push(bridge, {"type": "field_verify", "mines": ["C4"], "obstacles": [],
              "coverage": {"cells": ["C4"], "complete": True},
              "slice": {"id": "S2", "index": 2, "of": 2, "route": "C4"},
              "scan_id": "SCAN-A", "seq": 12})
feed(task, clock)
check("★ 同一 (scan_id,slice.id) 重发幂等：duplicates=1，updates 不变",
      task.coverage_duplicates == 1 and task.coverage_updates == 2,
      f"dup={task.coverage_duplicates} up={task.coverage_updates}")
check("重发不重复记差异/不发新事件",
      task.field_diffs == [] and len(events_of(sub, EV_FIELD_VERIFY)) == 0)

# ---- complete=False：片内未报 ≠ 本地要素消失 ----
task, bridge, bb, bus, clock, sub = build(local_map=local)
push(bridge, {"type": "field_verify", "mines": [], "obstacles": [],
              "coverage": {"cells": ["H1"], "complete": False},
              "slice": {"id": "S1", "index": 1, "of": 2},
              "scan_id": "SCAN-B", "seq": 20})
feed(task, clock)
cov = bb.get(P_DRONE_COVERAGE)
check("★ complete=False 的片不进 cells_done（cells_seen 仍记，供诊断）",
      cov["cells_done"] == 0 and cov["cells_seen"] == 1, f"{cov}")
check("★ 反证：不完整片的空表**不**报『本地 H1 消失』",
      task.field_diffs == [] and len(events_of(sub, EV_FIELD_VERIFY)) == 0,
      f"diffs={task.field_diffs}")
push(bridge, {"type": "field_verify", "mines": [], "obstacles": [],
              "coverage": {"cells": ["H1"], "complete": True},
              "slice": {"id": "S1", "index": 1, "of": 1},
              "scan_id": "SCAN-B2", "seq": 21})
feed(task, clock)
verify = events_of(sub, EV_FIELD_VERIFY)
check("★ 对照：complete=True 的完整片才允许报『H1 消失』",
      len(verify) == 1 and verify[0].data["tag"] == "field_verify.mines"
      and verify[0].data["only_local"] == ["H1"]
      and verify[0].data["complete"] is True and verify[0].data["scoped"] is True,
      f"{verify[0].data if verify else None}")
check("差异带片级作用域证据（cells/scan_id/slice_id）",
      verify and verify[0].data["cells"] == ["H1"]
      and verify[0].data["scan_id"] == "SCAN-B2"
      and verify[0].data["slice_id"] == "S1", f"{verify[0].data if verify else None}")
check("已探明比例随完整片累计（1/154）",
      abs(bb.get(P_DRONE_COVERAGE)["ratio"] - round(1 / 154, 4)) < 1e-12)
# ★ 靶也可以并入 field_verify（决策层方案 §4.2 的可选键；机侧构造器暂未产出，
#   车侧先按 additive 接受）——同样受片作用域/complete 门控。
push(bridge, {"type": "field_verify", "targets": [],
              "coverage": {"cells": ["F11"], "complete": True},
              "slice": {"id": "S2", "index": 2, "of": 2},
              "scan_id": "SCAN-B2", "seq": 22})
feed(task, clock)
verify = events_of(sub, EV_FIELD_VERIFY)
check("★ field_verify.targets（可选键）按片作用域核对靶：仅本地 F11",
      len(verify) == 1 and verify[0].data["tag"] == "field_verify.targets"
      and verify[0].data["only_local"] == ["F11"]
      and verify[0].data["only_remote"] == [],
      f"{verify[0].data if verify else None}")

# ---- 纯函数：解码容错 ----
check("decode_coverage(None/非 dict) → None",
      contract.decode_coverage(None) is None
      and contract.decode_coverage("x") is None
      and contract.decode_coverage({"complete": True}) is None)
cov = contract.decode_coverage(
    {"cells": ["a1", "A1", "Z9", ""], "complete": True, "mode": "STRIP"},
    scan_id="S-1", slice_info={"id": "s9", "index": 1, "of": 2})
check("覆盖格归一化：大写/去重/丢越界与空串（dropped=3）",
      cov is not None and cov.cells == ("A1",) and cov.dropped == 3
      and cov.mode == "strip", f"{cov}")
check("幂等键 = (scan_id, slice.id) 且 slice.id 大写",
      cov.key == ("S-1", "S9"), f"{cov.key if cov else None}")
cov_missing = contract.decode_coverage(
    {"cells": ["A1"]}, scan_id="S-1",
    slice_info={"id": "s9", "index": 1, "of": 2})
check("缺 complete ⇒ 按 UNKNOWN（False）+ problems 记录（绝不猜成空的）",
      cov_missing is not None and cov_missing.complete is False
      and any("complete" in p for p in cov_missing.problems),
      f"{cov_missing.problems if cov_missing else None}")

# --------------------------------------------------------------------------- #
section("⑪ EV_FIELD_VERIFY 生产订阅方（G5-3）：事件 → 黑盒汇总 / policy 四档")
task, bridge, bb, bus, clock, sub = build()
feed(task, clock)
check("默认 policy=record（只记录不阻断）",
      task.field_verify_policy == "record" and task.field_verify is not None)
stats = bus.stats()
check("★ 总线上出现生产订阅方 drone.field_verify（不再是零订阅）",
      "drone.field_verify" in (stats.get("subscribers", {}).get(EV_FIELD_VERIFY) or []),
      f"{stats.get('subscribers', {}).get(EV_FIELD_VERIFY)}")
bus.publish(EV_FIELD_VERIFY, source="probe", tag="target_zone",
            remote=["H12"], local=["F11", "G11"],
            only_remote=["H12"], only_local=["F11"])
clock.advance(0.2)
task.tick()
st = bb.get(P_FIELD_VERIFY)
check("消费总线事件并写 P_FIELD_VERIFY（不依赖自己发的报文）",
      isinstance(st, dict) and st["events"] == 1, f"{st}")
check("靶类分桶：target_new/target_gone",
      st["by_tag"] == {"target_zone": 1}
      and st["target_new"] == ["H12"] and st["target_gone"] == ["F11"], f"{st}")
check("默认 record：verdict=diff 但 hold=False、alert=False",
      st["verdict"] == "diff" and st["hold"] is False and st["alert"] is False, f"{st}")
check("决策风险分级：靶区不一致 ⇒ severity=warn（不是撞雷级 risk）",
      st["severity"] == "warn", f"{st.get('severity')}")
check("field_verify_gate 同判『只记录不阻断』",
      task.field_verify_gate()["hold"] is False, f"{task.field_verify_gate()}")
check("task.field_verify_summary() 与黑盒键同源",
      task.field_verify_summary()["events"] == 1)

# ---- policy=alert / hold / off ----
task_a, bridge_a, bb_a, bus_a, clock_a, sub_a = build(
    drone_extra={"field_verify_policy": "alert"})
bus_a.publish(EV_FIELD_VERIFY, source="probe", tag="field_verify.obstacles",
              remote=["Z3"], local=[], only_remote=["Z3"], only_local=[])
clock_a.advance(0.2)
task_a.tick()
st_a = bb_a.get(P_FIELD_VERIFY)
check("policy=alert ⇒ alert=True / hold=False，雷障进 hazard_new",
      st_a["alert"] is True and st_a["hold"] is False
      and st_a["hazard_new"] == ["Z3"], f"{st_a}")
check("决策风险分级：本地图没有的障 ⇒ severity=risk（最高级）",
      st_a["severity"] == "risk", f"{st_a.get('severity')}")

task_h, bridge_h, bb_h, bus_h, clock_h, sub_h = build(
    drone_extra={"field_verify_policy": "hold"})
bus_h.publish(EV_FIELD_VERIFY, source="probe", tag="field_verify.mines",
              remote=["A9"], local=[], only_remote=["A9"], only_local=[])
clock_h.advance(0.2)
task_h.tick()
st_h = bb_h.get(P_FIELD_VERIFY)
check("policy=hold ⇒ hold=True（建议阻断信号，但任务自己不执行）",
      st_h["hold"] is True and task_h.field_verify_gate()["hold"] is True, f"{st_h}")
check("hold 只置信号：任务不写安全状态、不改路线（P_SAFETY/P_ROUTE 未动）",
      bb_h.get(P_SAFETY) is None and bb_h.get(P_ROUTE) is None)

task_o, bridge_o, bb_o, bus_o, clock_o, sub_o = build(
    drone_extra={"field_verify_policy": "off"})
bus_o.publish(EV_FIELD_VERIFY, source="probe", tag="target_zone",
              remote=["H12"], local=[], only_remote=["H12"], only_local=[])
clock_o.advance(0.2)
task_o.tick()
check("policy=off ⇒ 不创建消费方、不写汇总键、总线上无订阅",
      task_o.field_verify is None and bb_o.get(P_FIELD_VERIFY) is None
      and task_o.field_verify_summary() is None
      and "drone.field_verify" not in
      (bus_o.stats().get("subscribers", {}).get(EV_FIELD_VERIFY) or []))

# --------------------------------------------------------------------------- #
section("⑫ 旧报文兼容 + 关掉新逻辑行为不变（旧 JSON 无新键）")
LEGACY = [
    {"type": "heartbeat", "seq": 1},
    {"type": "bridge_passable", "half": "lower", "block": "K9",
     "confidence": 0.9, "seq": 2},
    {"type": "target_zone", "blocks": ["F11", "G11"], "seq": 3},
    {"type": "target_zone", "blocks": ["G11", "H12"], "seq": 4},
    {"type": "field_verify", "mines": ["H1", "G2", "C4"],
     "obstacles": ["D1", "Z3"], "seq": 5},
    {"type": "weather_report", "temp": 25, "seq": 6},
    "{不是合法 JSON",
    12345,
]


def run_legacy(**kw):
    task, bridge, bb, bus, clock, sub = build(**kw)
    for item in LEGACY:
        push(bridge, item)
    feed(task, clock, n=3)
    events = [(e.type, e.source, e.data) for e in sub.poll(512)]
    return task, bridge, bb, bus, events


legacy_on = run_legacy()
legacy_off = run_legacy(drone_extra={"field_verify_policy": "off",
                                     "coverage_enabled": False})
task_on, bridge_on, bb_on, bus_on, ev_on = legacy_on
task_off, bridge_off, bb_off, bus_off, ev_off = legacy_off
check("旧报文照常解析：同一 P_DRONE（online/桥洞/差异条数/计数全等）",
      bb_on.get(P_DRONE) == bb_off.get(P_DRONE),
      f"on={bb_on.get(P_DRONE)} off={bb_off.get(P_DRONE)}")
check("事件流逐条相同（类型/来源/载荷）", ev_on == ev_off,
      f"on={[(e[0], e[2].get('tag')) for e in ev_on]} vs "
      f"off={[(e[0], e[2].get('tag')) for e in ev_off]}")
check("计数逐位相同（messages/bad/missing/unknown/bridge/ack/state）",
      (task_on.messages, task_on.bad_messages, task_on.missing_fields,
       task_on.unknown_types, task_on.bridge_updates, task_on.acks_sent,
       task_on.car_states_sent)
      == (task_off.messages, task_off.bad_messages, task_off.missing_fields,
          task_off.unknown_types, task_off.bridge_updates, task_off.acks_sent,
          task_off.car_states_sent),
      f"on={task_on.messages}/{task_on.bad_messages}/{task_on.missing_fields}")
check("车→无人机回传逐条相同（car_state/ack 的 JSON）",
      [json.loads(x) for x in bridge_on.drone_out]
      == [json.loads(x) for x in bridge_off.drone_out],
      f"on={len(bridge_on.drone_out)} off={len(bridge_off.drone_out)}")
check("默认档新增的汇总键只是 additive（off 档无此键）",
      bb_on.get(P_FIELD_VERIFY) is not None and bb_off.get(P_FIELD_VERIFY) is None)
check("★ 关掉新逻辑后黑盒键集合与旧口径一致（只有 pose/phase/drone）",
      set(bb_off.snapshot()) == {P_POSE, P_MISSION_PHASE, P_DRONE},
      f"{sorted(bb_off.snapshot())}")
check("off 档不写任何覆盖键/计数为 0",
      bb_off.get(P_DRONE_COVERAGE) is None and task_off.coverage_updates == 0
      and task_off.coverage_bad == 0)
# ★ 覆盖关闭 + 旧口径反证：同样的空表在「关掉覆盖」时恢复旧行为（误报本地全消失）
task_c, bridge_c, bb_c, bus_c, clock_c, sub_c = build(
    drone_extra={"coverage_enabled": False})
push(bridge_c, {"type": "field_verify", "mines": [], "obstacles": [],
                "coverage": {"cells": ["H1"], "complete": True},
                "slice": {"id": "S1", "index": 1, "of": 1},
                "scan_id": "SCAN-OFF", "seq": 30})
feed(task_c, clock_c)
check("覆盖关闭 ⇒ 不解析 coverage（零副作用、无覆盖键/计数）",
      bb_c.get(P_DRONE_COVERAGE) is None and task_c.coverage_updates == 0
      and task_c.coverage_bad == 0)
check("覆盖关闭 ⇒ 空表走旧口径（误报『本地 3 雷 + 2 障全消失』，反证新逻辑关掉即旧行为）",
      len(bb_c.get(P_DRONE).field_diff) == 2, f"{bb_c.get(P_DRONE).field_diff}")

# --------------------------------------------------------------------------- #
section("⑬ 畸形 coverage 降级：不崩、计数正确、其余字段照旧处理")
task, bridge, bb, bus, clock, sub = build(local_map=local)
push(bridge, {"type": "field_verify", "mines": ["H1"], "obstacles": [],
              "coverage": "这不是 dict", "scan_id": "X", "seq": 40})
feed(task, clock)
check("coverage 非 dict ⇒ coverage_bad=1，mines/obstacles 仍按旧口径处理（2 条差异）",
      task.coverage_bad == 1 and len(bb.get(P_DRONE).field_diff) == 2
      and bb.get(P_DRONE_COVERAGE) is None,
      f"bad={task.coverage_bad} diffs={bb.get(P_DRONE).field_diff}")
push(bridge, {"type": "field_verify", "mines": [], "obstacles": [],
              "coverage": {"cells": "不是列表", "complete": True},
              "slice": {"id": "S1", "index": 1, "of": 1}, "seq": 41})
feed(task, clock)
check("coverage.cells 非 list ⇒ coverage_bad 递增且不崩", task.coverage_bad == 2)

task, bridge, bb, bus, clock, sub = build(local_map=local)
push(bridge, {"type": "field_verify", "mines": [], "obstacles": [],
              "coverage": {"cells": ["H1"]},          # 缺 complete
              "slice": {"id": "S1", "index": 1, "of": 1},
              "scan_id": "SCAN-P", "seq": 42})
feed(task, clock)
cov = bb.get(P_DRONE_COVERAGE)
check("缺 complete ⇒ 按 UNKNOWN：片被记账但 cells_done=0、不报差异",
      isinstance(cov, dict) and cov["cells_done"] == 0
      and task.field_diffs == [], f"{cov} / {task.field_diffs}")
push(bridge, {"type": "field_verify", "mines": [], "obstacles": [],
              "coverage": {"cells": ["G2"], "complete": True},
              "slice": {"id": "S1", "index": 1, "of": 1},
              "scan_id": "S" * 80, "seq": 43})       # scan_id 超长 ⇒ 丢去重键
feed(task, clock)
check("scan_id 超长 ⇒ 只丢去重键，覆盖数据照收",
      task.coverage_updates == 2 and task.coverage_bad == 0,
      f"up={task.coverage_updates} bad={task.coverage_bad}")
check("越界/畸形格名被剔除但不崩（H1 不完整不计、G2 完整计入）",
      bb.get(P_DRONE_COVERAGE)["cells_done"] == 1, f"{bb.get(P_DRONE_COVERAGE)}")

# --------------------------------------------------------------------------- #
section("⑭ field_layout（车端不留地图文件）：校验 → P_FIELD_LAYOUT，坏报文不清空")
check("DRONE_PAYLOAD_TYPES 收录 field_layout", "field_layout" in DRONE_PAYLOAD_TYPES,
      f"{DRONE_PAYLOAD_TYPES}")
check("FIELD_LAYOUT_VERSION >= 1", FIELD_LAYOUT_VERSION >= 1)

_LAYOUT = {
    "type": "field_layout", "version": 1, "ts": 1234.5,
    "start": "n10", "end": "n5",
    "mines": ["a2", "F4", "H4", "G5", "E8", "B10", "D11", "I11", "A2"],   # 小写 + 重复
    "obstacles": ["B1", "D1", "F3", "I6", "H7", "A8", "C9", "E9"],
    "targets": ["d5", "d6"],
    "bridges": ["k7", "k5"],
    "barrier_col_from": "d", "barrier_row_hi": 6, "barrier_row_lo": 7,
    "future_key": {"ignored": True},          # 前向兼容：未知字段必须被忽略
}
# 布局与本地地图无关：把 drone.map_path 指到一个不存在的文件 ⇒ map_loaded=False，
# 但 field_layout 仍必须照收（车端本来就不留地图文件）。
task, bridge, bb, bus, clock, sub = build(
    {"drone": {"map_path": str(pathlib.Path(tempfile.mkdtemp()) / "absent.yaml")}},
    local_map=None)
check("布局不依赖本地地图：map_loaded=False（车端无地图文件是正常态）",
      task.map_loaded is False, f"source={task._map_source!r}")
check("没收到布局前黑盒里没有 P_FIELD_LAYOUT", bb.get(P_FIELD_LAYOUT) is None)
push(bridge, {"type": "heartbeat"})
push(bridge, _LAYOUT)
feed(task, clock)
lay = bb.get(P_FIELD_LAYOUT)
check("合法布局 → 写 P_FIELD_LAYOUT（dict）", isinstance(lay, dict), f"{lay}")
check("start/end 大写归一", lay["start"] == "N10" and lay["end"] == "N5", f"{lay}")
check("雷列表归一：大写 + 去重（9 条含重复 → 8）",
      lay["mines"] == ["A2", "F4", "H4", "G5", "E8", "B10", "D11", "I11"], f"{lay['mines']}")
check("靶/桥洞同样归一", lay["targets"] == ["D5", "D6"] and lay["bridges"] == ["K7", "K5"],
      f"{lay['targets']} / {lay['bridges']}")
check("隔板三键归一（列大写、行号整数）",
      (lay["barrier_col_from"], lay["barrier_row_hi"], lay["barrier_row_lo"]) == ("D", 6, 7))
check("未知字段被忽略（前向兼容；不污染规划入参）", "future_key" not in lay)
check("计数正确：updates=1 / rejects=0 / last_type=field_layout",
      task.layout_updates == 1 and task.layout_rejects == 0
      and task.last_type == "field_layout", f"{task.layout_updates}/{task.layout_rejects}")
check("黑盒键有来源留痕（source=DroneTask）",
      bb.entry(P_FIELD_LAYOUT) is not None and bb.entry(P_FIELD_LAYOUT).source == "DroneTask")

_bad_layouts = [
    ("缺 start", {"type": "field_layout", "end": "N5", "mines": [], "obstacles": [],
                 "targets": [], "bridges": ["K4"]}),
    ("坏格名", dict(_LAYOUT, start="Z9")),
    ("空 bridges", dict(_LAYOUT, bridges=[])),
    ("mines 非列表", dict(_LAYOUT, mines="A2")),
    ("隔板行号相等", dict(_LAYOUT, barrier_row_hi=6, barrier_row_lo=6)),
    ("隔板行越界", dict(_LAYOUT, barrier_row_hi=0, barrier_row_lo=7)),
]
for _name, _payload in _bad_layouts:
    push(bridge, _payload)
feed(task, clock)
check("坏布局全部整份拒收（6 条），任务不崩", task.layout_rejects == 6,
      f"rejects={task.layout_rejects}")
check("★ 坏报文不清空、不覆盖上一份好布局",
      bb.get(P_FIELD_LAYOUT) == lay and task.layout_updates == 1,
      f"up={task.layout_updates} layout={bb.get(P_FIELD_LAYOUT)}")
check("布局解析不动 field_diff / missing_fields（坏布局不等于字段缺失）",
      task.field_diffs == [] and task.missing_fields == 0,
      f"diffs={task.field_diffs} missing={task.missing_fields}")
push(bridge, {"type": "bridge_passable", "half": "lower", "block": "K7"})
feed(task, clock)
check("★ 布局之后旧报文照常工作（bridge_passable 不受影响）",
      task.bridge_of("lower") == "K7")

# 可选字段容错：显式 null / 缺键 → 用缺省（不整份拒收）
_loose, _why = normalize_field_layout({
    "start": "N10", "end": "N5", "mines": [], "obstacles": [], "targets": [],
    "bridges": ["K7"], "barrier_col_from": None,
    "barrier_row_hi": None, "barrier_row_lo": None,
})
check("隔板三键显式 null → 用缺省 D/6/7（不整份拒收）",
      _loose is not None and (_loose["barrier_col_from"], _loose["barrier_row_hi"],
                              _loose["barrier_row_lo"]) == ("D", 6, 7),
      f"{_loose} / {_why!r}")

# 纯函数级反证：非法输入返回 (None, 原因)，不抛异常
for _bad in (None, 123, "x", {}, {"start": "N10", "end": "Z5"}):
    _norm, _why = normalize_field_layout(_bad)
    check(f"normalize_field_layout({_bad!r}) → None 且给出原因",
          _norm is None and bool(_why), f"{_norm} / {_why!r}")

# --------------------------------------------------------------------------- #
section("⑮ 主流程规划入口：有布局用内存 MapSpec（与文件同图逐 token 相同）/ 无布局不规划")
from dzbrain import main as M                                             # noqa: E402
from dzbrain.tasks.mission import plan_route_from_config                   # noqa: E402

check("main 与 drone 的黑盒键名同值（P_FIELD_LAYOUT，无拼写漂移）",
      M.P_FIELD_LAYOUT == P_FIELD_LAYOUT, f"{M.P_FIELD_LAYOUT!r} vs {P_FIELD_LAYOUT!r}")
check("P_FIELD_LAYOUT 常量已迁 core.blackboard（drone 模块 re-export 同名同值）",
      P_FIELD_LAYOUT == CORE_P_FIELD_LAYOUT == "field_layout",
      f"{P_FIELD_LAYOUT!r} vs {CORE_P_FIELD_LAYOUT!r}")
check("P_FIELD_VERIFY 常量已迁 core.blackboard（drone 模块 re-export 同名同值）",
      P_FIELD_VERIFY == CORE_P_FIELD_VERIFY == "field_verify",
      f"{P_FIELD_VERIFY!r} vs {CORE_P_FIELD_VERIFY!r}")
_layout_from_file, _why = normalize_field_layout(dict(_map_yaml, version=1, ts=0.0))
check("真实 map.yaml 的 map: 段能转成合法布局", _layout_from_file is not None, _why)

cfg = Config.load(str(CONFIG_PATH))
cfg.override("planner.engine", "tour")          # 两条入口统一引擎（tour 快、无门禁依赖）
cfg.override("planner.auto_plan_enabled", True)  # 布局模式的前置开关（本测试不动真配置）
ref = plan_route_from_config(cfg, map_path=str(MAP_PATH))
check("文件入口（对照臂）规划成功", ref.ok and bool(ref.tokens), f"{ref.error}")

bb_l = Blackboard()
bb_l.set(P_FIELD_LAYOUT, _layout_from_file, source="test")
out = M.plan_startup_route(cfg, bb_l, log=lambda m: None)
check("有布局 → plan_startup_route 规划成功", out is not None and out.ok,
      f"{getattr(out, 'error', '')}")
check("★ 内存 MapSpec 与文件入口**逐 token 相同**（同一张图、同一份配置）",
      out is not None and out.tokens == ref.tokens,
      f"{len(out.tokens) if out else 0} vs {len(ref.tokens)}")
check("规划来源标记为 drone:field_layout（不是文件路径）",
      out is not None and out.map_path == "drone:field_layout", f"{getattr(out, 'map_path', '')}")
check("路线里程/引擎与文件入口一致",
      out is not None and abs(out.route_len_m - ref.route_len_m) < 1e-9
      and out.engine == ref.engine,
      f"{out.route_len_m if out else 0} vs {ref.route_len_m}")

check("★ 无布局 → 不规划（返回 None；不读文件、不调用规划器）",
      M.plan_startup_route(cfg, Blackboard()) is None)
bb_bad = Blackboard()
bb_bad.set(P_FIELD_LAYOUT, {"start": "Z9", "end": "N5"}, source="test")
check("P_FIELD_LAYOUT 非法 → 同样不规划（防御性二次校验）",
      M.plan_startup_route(cfg, bb_bad) is None)

check("布局模式判定：无 --map + auto_plan=true → 布局模式",
      M._layout_mode_enabled(cfg, None) is True)
check("显式 --map → 文件模式（开发/仿真入口，不部署）",
      M._layout_mode_enabled(cfg, str(MAP_PATH)) is False)
cfg_no = Config.load(str(CONFIG_PATH))
cfg_no.override("planner.auto_plan_enabled", False)
check("auto_plan=false → 不是布局模式（走旧手写路线）",
      M._layout_mode_enabled(cfg_no, None) is False)

check("_wait_for_field_layout：已有布局 → 立即返回同一份 dict",
      M._wait_for_field_layout(bb_l, cfg, stop_flag={"stop": False},
                               alive=lambda: True, log=lambda m: None)
      == _layout_from_file)
cfg_wait0 = Config.from_dict({"drone": {"field_layout_wait_s": 0.0}})
check("_wait_for_field_layout：无布局 + 等待 0s ⇒ None（不规划、不出发）",
      M._wait_for_field_layout(Blackboard(), cfg_wait0, stop_flag={"stop": False},
                               alive=lambda: True, log=lambda m: None) is None)
check("_wait_for_field_layout：停机信号 ⇒ 立即 None（不再等满超时）",
      M._wait_for_field_layout(Blackboard(), Config.from_dict(
          {"drone": {"field_layout_wait_s": 999.0}}), stop_flag={"stop": True},
          alive=lambda: True, log=lambda m: None) is None)

# --------------------------------------------------------------------------- #
section("⑯ run_live 布局门（替身，不碰 ROS）：无布局 exit 3 / 规划失败 exit 4 / 成功写 route.tokens")
#   这是"车端无地图文件"的**上车行为**证据：规划发生在 build_tasks **之前**，
#   拿不到布局就绝不构造任务组（= 不出发），不回退 route.tokens / 地图文件。
import types                                                              # noqa: E402

import dzbrain.io.ros_io as _rz                                           # noqa: E402


class _FakeLiveBridge:
    def __init__(self, *a, **k):
        self.shutdown_brakes = []
        self.spun = 0

    def spin_async(self, threads: int = 4):
        self.spun += 1

    def shutdown(self, *, brake: bool = True):
        self.shutdown_brakes.append(brake)
        return 0

    # ---- 2026-10-18 新增的启动自检/位姿就绪门接口（run_live 会调；T-FIX）----
    def subscription_topics(self):
        return ["/odom/localization"]

    def topic_counts(self):
        return {"odom": {"publishers": 1, "subscribers": 1}}

    def pose(self):
        return object()

    def pose_age(self):
        return 0.05


class _FakeLiveDrone:
    def __init__(self, *a, **k):
        self.started = False
        self.stopped = False
        self.layout_updates = 0
        self.layout_rejects = 0

    def start(self):
        self.started = True

    def stop(self, timeout: float | None = 2.0):
        self.stopped = True
        return True

    def health(self):
        return types.SimpleNamespace(ticks=0, errors=0)


class _FakeLiveGroup:
    _tasks: list = []

    def start_all(self):
        self.started = True

    def stop_all(self, timeout: float = 2.0):
        return []


_saved = {name: getattr(M, name) for name in (
    "install_stop_signals", "build_tasks", "start_banner", "finish_report",
    "_live_loop", "_start_layout_listener", "_wait_for_field_layout",
    "plan_route_from_field_layout")}
_rz_saved = _rz.RosBridge
try:
    _rz.RosBridge = _FakeLiveBridge
    M.install_stop_signals = lambda *a, **k: []
    M.finish_report = lambda *a, **k: None
    M.start_banner = lambda *a, **k: None
    _calls: dict = {"build": 0, "banner": 0, "live": 0, "skip": None}

    def _fake_build(*a, **kw):
        _calls["build"] += 1
        _calls["skip"] = kw.get("skip")
        return _FakeLiveGroup(), []

    M.build_tasks = _fake_build
    M._live_loop = lambda *a, **kw: (_calls.__setitem__("live", _calls["live"] + 1), "stop")[1]
    M._start_layout_listener = lambda *a, **k: _FakeLiveDrone()

    # ---- A 无布局：不规划、不起任务组、exit 3 ----
    M._wait_for_field_layout = lambda *a, **k: None
    cfg_live = Config.from_dict({"planner": {"auto_plan_enabled": True},
                                 "drone": {"field_layout_wait_s": 0.0}})
    rc_none = M.run_live(cfg_live, log=lambda s: None)
    check("★ run_live 无布局 → 退出码 3（不规划、不出发）", rc_none == 3, f"rc={rc_none}")
    check("★ 无布局 → build_tasks 一次都不调用（任务组根本没起）",
          _calls["build"] == 0, f"build={_calls['build']}")

    # ---- B 布局规划失败：不写 route.tokens、不起任务组、exit 4 ----
    M._wait_for_field_layout = lambda *a, **k: _layout_from_file
    M.plan_route_from_field_layout = lambda *a, **k: types.SimpleNamespace(
        ok=False, tokens=[], error="PlannerError: 演示失败")
    cfg_fail = Config.from_dict({"planner": {"auto_plan_enabled": True},
                                 "route": {"tokens": ["N10", "N5,y"]}})
    rc_fail = M.run_live(cfg_fail, log=lambda s: None)
    check("★ 布局规划失败 → 退出码 4（不回退 route.tokens/地图文件）",
          rc_fail == 4, f"rc={rc_fail}")
    check("规划失败 → 原 route.tokens 一个字没改、任务组没起",
          cfg_fail.get("route.tokens") == ["N10", "N5,y"] and _calls["build"] == 0,
          f"tokens={cfg_fail.get('route.tokens')} build={_calls['build']}")

    # ---- C 规划成功：写 route.tokens + 关任务侧再规划 + build_tasks(skip=drone) ----
    _plan = types.SimpleNamespace(ok=True, tokens=["N10", "M10", "N5,y"],
                                  route_len_m=0.7, engine="tour", elapsed_ms=1.0)
    M.plan_route_from_field_layout = lambda *a, **k: _plan
    cfg_ok = Config.from_dict({"planner": {"auto_plan_enabled": True},
                               "route": {"tokens": ["旧", "路线"]}})
    rc_ok = M.run_live(cfg_ok, log=lambda s: None)
    check("★ 有布局 → 规划产物写入 route.tokens（交 MissionTask 执行）",
          cfg_ok.get("route.tokens") == _plan.tokens, f"{cfg_ok.get('route.tokens')}")
    check("★ 规划后关掉任务侧再规划（不会去碰 planner.map_path 文件）",
          cfg_ok.get("planner.auto_plan_enabled") is False,
          f"{cfg_ok.get('planner.auto_plan_enabled')}")
    check("★ 任务组用 skip={'drone'} 构造（无人机已在装配前单独启动）",
          _calls["build"] == 1 and _calls["skip"] == {"drone"}, f"{_calls}")
    check("布局链路正常收尾 → exit 0", rc_ok == 0, f"rc={rc_ok}")

    # ---- D 无人机被禁用：布局模式快速失败（不空等 30s、不起组） ----
    _calls["build"] = 0
    cfg_nodrone = Config.from_dict({"planner": {"auto_plan_enabled": True},
                                    "drone": {"enabled": False}})
    rc_nd = M.run_live(cfg_nodrone, log=lambda s: None)
    check("drone.enabled=False → 布局模式 exit 3（永远收不到布局）", rc_nd == 3, f"rc={rc_nd}")
    check("drone.enabled=False → 不起任务组", _calls["build"] == 0, f"build={_calls['build']}")
finally:
    _rz.RosBridge = _rz_saved
    for _name, _orig in _saved.items():
        setattr(M, _name, _orig)

# --------------------------------------------------------------------------- #
section("⑰ field_layout v2（协议 v2 / T28）：known / 特征子集 / target_cell / version 单调")
check("FIELD_LAYOUT_VERSION == 2（车端支持 v2 步进布局）", FIELD_LAYOUT_VERSION == 2)

_V1 = {
    "type": "field_layout", "version": 1, "ts": 10.0,
    "start": "N10", "end": "N5",
    "mines": ["H1"], "obstacles": ["D1"], "targets": ["F11"],
    "bridges": ["K7", "K5"],
}
task, bridge, bb, bus, clock, sub = build()
push(bridge, _V1)
feed(task, clock)
lay = bb.get(P_FIELD_LAYOUT)
check("★ v1 报文（无 known）照常接受，但归一化后**没有** known 键（旧'规划一次'路径不变）",
      isinstance(lay, dict) and "known" not in lay, f"{lay}")
check("v1 的 target_cell 为空 / segment_id 默认 0（不会误触发步进目标）",
      lay.get("target_cell") == "" and lay.get("segment_id") == 0, f"{lay}")

_V2_KNOWN = ["N10", "M10", "N9", "M9", "N8", "M8", "H7", "K7", "K5", "N5"]
_V2 = {
    "type": "field_layout", "version": 2, "ts": 20.0,
    "start": "N10", "end": "N5",
    "known": _V2_KNOWN,
    "mines": [], "obstacles": [], "targets": [],
    "bridges": ["K7", "K5"],
    "target_cell": "M10", "segment_id": 7,
}
push(bridge, _V2)
feed(task, clock)
lay = bb.get(P_FIELD_LAYOUT)
check("★ v2 合法布局：known/target_cell/segment_id 落盘",
      isinstance(lay, dict) and lay.get("known") == _V2_KNOWN
      and lay.get("target_cell") == "M10" and lay.get("segment_id") == 7, f"{lay}")
check("v2 被计入 layout_updates（v1 之后第 2 份）", task.layout_updates == 2,
      f"{task.layout_updates}")

_bad_v2 = [
    ("特征不在 known", dict(_V2, version=3, mines=["A2"])),
    ("target 不在 known", dict(_V2, version=3, target_cell="A1")),
    ("known 为空", dict(_V2, version=3, known=[])),
    ("known 含非法格名", dict(_V2, version=3, known=["Z9"])),
]
_rej0 = task.layout_rejects
for _name, _payload in _bad_v2:
    push(bridge, _payload)
feed(task, clock)
check(f"v2 非法布局整份拒收（{len(_bad_v2)} 条：特征∉known / target∉known / known 空 / 坏格名）",
      task.layout_rejects == _rej0 + len(_bad_v2), f"{task.layout_rejects}")
check("拒收后仍保留 v2 好布局", bb.get(P_FIELD_LAYOUT) == lay)

# ---- version 单调（协议 §3.1）----
push(bridge, dict(_V2, version=1, ts=30.0))          # 迟到旧版
feed(task, clock)
check("★ 迟到旧版（v1 < 当前 v2）被拒收且不清空当前布局",
      task.layout_stale == 1 and bb.get(P_FIELD_LAYOUT) == lay,
      f"stale={task.layout_stale}")
push(bridge, dict(_V2, ts=31.0))                    # 同版同内容（心跳）
feed(task, clock)
check("★ 同版同内容（仅 ts 不同）= 心跳重发：不计数、不重写",
      task.layout_updates == 2 and task.layout_rejects == _rej0 + len(_bad_v2),
      f"up={task.layout_updates} rej={task.layout_rejects}")
push(bridge, dict(_V2, version=2, target_cell="M9"))  # 同版异内容：违约
feed(task, clock)
check("★ 同版异内容被拒收（同版必须同内容）",
      task.layout_rejects == _rej0 + len(_bad_v2) + 1
      and bb.get(P_FIELD_LAYOUT) == lay, f"rej={task.layout_rejects}")

push(bridge, dict(_V2, version=3, ts=40.0, known=_V2_KNOWN + ["M11"],
                  target_cell="N5"))
feed(task, clock)
lay3 = bb.get(P_FIELD_LAYOUT)
check("更高版本（v3）被接受，known 单调增长到 11 格",
      lay3.get("version") == 3 and len(lay3.get("known") or []) == 11
      and lay3.get("target_cell") == "N5", f"{lay3}")
check("version 历史单调：2 → 3（旧版未覆盖）", task.layout_updates == 3,
      f"{task.layout_updates}")

finish()
