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
| 收 | `field_verify` `{mines,obstacles,diff}` | 同上（雷/障差异） |
| 收 | 未知 `type` / 非法 JSON / 缺字段 | **记日志不崩**（旧代码"一条坏消息杀一个回调"的教训） |
| 发 | `car_state` `{type,block,yaw_deg,phase,ts,seq}` | 1~2 Hz 周期上报（block 从 `P_POSE` 推） |
| 发 | `ack` `{type:"ack",of:seq}` | 对带 `seq` 的消息回执 |

★ 失效兜底（§7.2）：心跳丢失 > `drone.heartbeat_timeout_s`（默认 3 s）
  → `online=False` + 一次告警；**任务不阻塞、不改变默认分支/默认地图**
  （"宁可漏打，不可卡死"）。情报过期 > `drone.info_stale_s` → `bridge_of()` 返回 None
  （由 `DroneState.bridge_of` 现成实现，本任务只负责把 `bridge_ts` 写准）。
★ `drone.enabled=False` → **整任务空转**：不读 `bridge`、不发 `bridge`、不写 `P_DRONE`。

## 本地地图从哪来（★ 接口现状，需要装配侧知道）

`main.build_tasks()` 目前只传 `(bridge, bb, bus, cfg)`，**不传 `map_cfg`**，
所以本任务按以下顺序自取本地地图（与 `main.load_and_check` 的默认路径一致）：
  1. 构造参数 `local_map=`（测试/将来装配侧显式注入，最优先）；
  2. `cfg.get("drone.map_path")` 指定的 YAML 文件；
  3. 仓库默认 `<repo>/config/map.yaml`（`src/dzbrain/dzbrain/tasks/drone.py` 的上溯 4 层）。
三个来源都没有 → `map_loaded=False`，只记一条日志，**核对功能降级为"不核对"**
（绝不因为拿不到地图就让任务起不来）。

⚠️ SCHEMA 缺键（未改 `core/config.py`，一律 `cfg.get(key, default)`）：
  `drone.map_path` / `drone.field_diff_max` / `drone.ack_enabled`。
  `drone.enabled` / `topic_in` / `topic_out` / `heartbeat_timeout_s` / `info_stale_s`
  已在 SCHEMA 里。
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

__all__ = ["DroneTask", "DRONE_PAYLOAD_TYPES", "default_map_path"]

#: 已知的无人机报文类型（其余一律"未知类型"，记日志不崩）
DRONE_PAYLOAD_TYPES: tuple[str, ...] = (
    "heartbeat", "bridge_passable", "target_zone", "field_verify",
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


class DroneTask(TaskThread):
    """无人机情报收发（5 Hz，只写 `P_DRONE`）。"""

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

        # ---------------- 本地地图（核对真源） ----------------
        self._local_map: dict | None = local_map
        self.map_loaded = bool(local_map)
        self._map_source = "构造参数" if local_map else ""
        self.targets: frozenset[str] = frozenset()
        self.mines: frozenset[str] = frozenset()
        self.obstacles: frozenset[str] = frozenset()
        self.bridges: frozenset[str] = frozenset()

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

    def teardown(self) -> None:
        self._log(f"退出：消息 {self.messages} 条（坏 {self.bad_messages}）"
                  f" 桥洞情报 {self.bridge_updates} 次 车状态 {self.car_states_sent} 条"
                  f" ack {self.acks_sent} 条 差异 {len(self.field_diffs)} 条"
                  f" 在线={'是' if self.online else '否'}")

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

    # ------------------------------------------------------------------ #
    #  单拍
    # ------------------------------------------------------------------ #
    def tick(self) -> None:
        now = self.clock()
        if not self.enabled:
            return                                  # ★ 空转：一个 bridge 调用都不发
        self._drain_inbox(now)
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

    # ---- 雷/障核对 ----
    def _on_field_verify(self, payload: dict, now: float) -> None:
        touched = False
        for key, local in (("mines", self.mines), ("obstacles", self.obstacles)):
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
            self._diff(f"field_verify.{key}", got, local, now)
        if not touched:
            self.missing_fields += 1
            self._warn_missing("field_verify", "mines/obstacles", payload, now)

    # ---- 差异登记 ----
    def _diff(self, tag: str, got: frozenset[str], local: frozenset[str], now: float) -> None:
        if not self.map_loaded:
            self._record_diff(f"{tag}:收到{sorted(got)} 本地地图缺失→未核对")
            return
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
