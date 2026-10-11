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
"""

from __future__ import annotations

import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:                                    # 车端/Windows 控制台默认不是 UTF-8
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from dzbrain.core import geometry as geo                                       # noqa: E402
from dzbrain.core.blackboard import (P_DRONE, P_MISSION_PHASE, P_POSE,         # noqa: E402
                                     P_TELEMETRY, Blackboard)
from dzbrain.core.config import Config                                         # noqa: E402
from dzbrain.core.events import (EV_BRIDGE_INFO, EV_DRONE_OFFLINE,             # noqa: E402
                                 EV_DRONE_ONLINE, EV_FIELD_VERIFY, EventBus)
from dzbrain.core.state import DroneState                                      # noqa: E402
from dzbrain.tasks.drone import DroneTask, default_map_path                    # noqa: E402
from fakes import FakeBridge, FakeClock                                        # noqa: E402
from harness import check, finish, section                                     # noqa: E402

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
def build(overrides: dict | None = None, *, local_map=LOCAL_MAP, seed_pose=True):
    """造 (task, bridge, bb, bus, clock, events)。事件全量订阅，便于断言"只发一次"。"""
    clock = FakeClock()
    bridge = FakeBridge(clock)
    bb = Blackboard()
    bus = EventBus()
    cfg = Config.from_dict({
        "drone": {"enabled": True, "heartbeat_timeout_s": 3.0, "info_stale_s": 30.0},
        "mission": {"auto_start": False},
        "logging": {"rate_limit_s": 1.0},
        "telemetry": {},
        **(overrides or {}),
    })
    if seed_pose:
        bb.set(P_POSE, geo.Pose(*geo.block_to_meters("N10"), yaw=geo.HEADING_Y),
               source="test")
    bb.set(P_MISSION_PHASE, "DRIVE", source="test")
    sub = bus.subscribe("watch", None)             # 全量订阅
    task = DroneTask(bridge, bb, bus, cfg, log=lambda m: None, clock=clock.now,
                     local_map=local_map)
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
check("靶区从 map.yaml 读出 = {F11,G11}", task.targets == frozenset({"F11", "G11"}),
      f"got {sorted(task.targets)}")
check("雷 8 个、障 8 个被读出",
      len(task.mines) == 8 and len(task.obstacles) == 8,
      f"mines={len(task.mines)} obstacles={len(task.obstacles)}")
skip, bridge2, bb2, bus2, clock2, sub2 = build(
    {"drone": {"map_path": str(pathlib.Path(__file__).resolve().parent
                               / "fixtures" / "map_field.yaml")}},
    local_map={"map": {"targets": ["A1"]}})
check("构造参数优先（显式注入覆盖文件）",
      skip.targets == frozenset({"A1"}), f"got {sorted(skip.targets)}")

finish()
