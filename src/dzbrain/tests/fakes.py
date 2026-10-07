# -*- coding: utf-8 -*-
"""测试替身：FakeBridge / FakeClock / 造数据helper。

为什么需要：任务线程要能在**没有 ROS、没有车**的情况下测（这是"本地可验证"的前提）。
FakeBridge 与 `dzbrain.io.ros_io.RosBridge` 是**鸭子类型**关系 —— 方法名/返回类型一致，
所以任务代码在两边都能跑。
"""

from __future__ import annotations

import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from dzbrain.core import geometry as geo                # noqa: E402
from dzbrain.io.contract import AimStatus, OffsetObservation  # noqa: E402


class FakeClock:
    """可控时钟：任务注入 `clock=FakeClock().now`，测试可 `advance()` 快进。"""

    def __init__(self, start: float = 1000.0) -> None:
        self.t = float(start)

    def now(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += float(dt)
        return self.t


class FakeBridge:
    """RosBridge 的鸭子类型替身（可脚本化、可注入故障）。"""

    def __init__(self, clock: FakeClock | None = None) -> None:
        self.clock = clock or FakeClock()
        self._pose: geo.Pose | None = None
        self._pose_ts: float = 0.0
        self._vision: OffsetObservation | None = None
        self._aim_status = AimStatus.IDLE
        self._shoot_count = 0
        self._ammo: int | None = None
        self._hits = 0
        self._manual_override = False
        self._manual_gimbal = False
        self._drone: list[str] = []
        self._cmds: list[str] = []

        # 记录（测试断言用）
        self.commands: list[tuple[float, float]] = []
        self.brakes = 0
        self.scan_requests: list[int] = []
        self.fire_gate: list[bool] = []
        self.motion_states: list[bool] = []
        self.voices: list[int] = []
        self.drone_out: list[str] = []
        self.leads: list[float] = []

        # 故障注入
        self.pose_frozen = False          # True = pose_age 一直涨（模拟定位失联）

    # ---------------- 脚本化 ----------------
    def set_pose(self, x: float, y: float, yaw: float = 0.0) -> None:
        self._pose = geo.Pose(x, y, yaw)
        self._pose_ts = self.clock.now()

    def set_pose_at_block(self, block: str, yaw: float = 0.0) -> None:
        x, y = geo.block_to_meters(block)
        self.set_pose(x, y, yaw)

    def set_vision(self, dx: int = 0, dy: int = 0, cls: int = 1, box_w: int = 45,
                   box_h: int = 55) -> None:
        self._vision = OffsetObservation(dx=dx, dy=dy, cls=cls, box_w=box_w, box_h=box_h,
                                         ts=self.clock.now())

    def clear_vision(self) -> None:
        self._vision = None

    def set_aim_status(self, status: int) -> None:
        self._aim_status = status

    def add_shots(self, n: int = 1) -> None:
        self._shoot_count += int(n)

    def add_hits(self, n: int = 1) -> None:
        self._hits += int(n)

    def set_ammo(self, n: int | None) -> None:
        self._ammo = n

    def set_manual_override(self, flag: bool) -> None:
        self._manual_override = bool(flag)

    def push_drone(self, payload: str) -> None:
        self._drone.append(payload)

    def push_waypoint_command(self, cmd: str) -> None:
        self._cmds.append(cmd)

    # ---------------- 读 ----------------
    def pose(self) -> geo.Pose | None:
        return self._pose

    def pose_age(self) -> float:
        if self._pose is None:
            return float("inf")
        if self.pose_frozen:
            return float("inf")
        return self.clock.now() - self._pose_ts

    def vision(self, max_age_s: float = 0.5):
        obs = self._vision
        if obs is None:
            return None
        return obs if (self.clock.now() - obs.ts) <= max_age_s else None

    def vision_raw(self):
        return self._vision

    def aim_status(self) -> int:
        return self._aim_status

    def aim_status_name(self) -> str:
        return AimStatus.name(self._aim_status)

    def shoot_count(self) -> int:
        return self._shoot_count

    def ammo(self):
        return self._ammo

    def take_hits(self) -> int:
        n = self._hits
        self._hits = 0
        return n

    def manual_override(self) -> bool:
        return self._manual_override

    def manual_gimbal(self) -> bool:
        return self._manual_gimbal

    def take_drone_messages(self, max_items: int = 16) -> list[str]:
        out = self._drone[:max_items]
        del self._drone[:max_items]
        return out

    def take_waypoint_commands(self, max_items: int = 8) -> list[str]:
        out = self._cmds[:max_items]
        del self._cmds[:max_items]
        return out

    # ---------------- 写 ----------------
    def publish_command(self, linear_x: float, steer_deg: float) -> None:
        self.commands.append((float(linear_x), float(steer_deg)))

    def publish_brake(self) -> None:
        self.brakes += 1
        self.commands.append((0.0, 0.0))

    def publish_scan_request(self, mode: int) -> None:
        self.scan_requests.append(int(mode))

    def publish_fire_gate(self, allow: bool) -> None:
        self.fire_gate.append(bool(allow))

    def publish_motion_state(self, reverse: bool) -> None:
        self.motion_states.append(bool(reverse))

    def publish_voice(self, voice_id: int) -> None:
        self.voices.append(int(voice_id))

    def publish_drone(self, payload: str) -> None:
        self.drone_out.append(str(payload))

    def publish_lead(self, lead_px: float) -> None:
        self.leads.append(float(lead_px))

    # ---------------- 断言辅助 ----------------
    def last_command(self) -> tuple[float, float] | None:
        return self.commands[-1] if self.commands else None

    def moved_commands(self) -> list[tuple[float, float]]:
        return [c for c in self.commands if abs(c[0]) > 1e-9]


def make_config(overrides: dict | None = None):
    """造一份测试配置（用 dzbrain 的默认值 + 覆盖项）。"""
    from dzbrain.core.config import Config
    data = {
        "route": {"tokens": ["N10", "M10"]},
        "pursuit": {},
        "mission": {"auto_start": False},
    }
    if overrides:
        for k, v in overrides.items():
            data.setdefault(k, {}).update(v)
    return Config.from_dict(data)


def simple_line_route(a: str = "N10", b: str = "M10", **kw):
    """两格直线路线（测试用）。"""
    from dzbrain.route.model import Waypoint, make_route
    return make_route([Waypoint.at_block(a, **kw), Waypoint.at_block(b, **kw)])


def wait_until(predicate, timeout_s: float = 2.0, interval_s: float = 0.005) -> bool:
    """等条件成立（测试里替代死 sleep）。"""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return predicate()
