# -*- coding: utf-8 -*-
"""共享状态黑盒（Blackboard）—— 任务线程之间的**高频**只读共享通道。

设计要点（对应架构设计 §3.2）：
- 值必须**不可变**（frozen dataclass / tuple / 数字 / 字符串）。写入即替换，不做原地修改。
- 每项带时间戳与来源，读取方能判断"新鲜度"（`get_fresh`），避免拿过期数据做决策。
- 一把 RLock 保护；临界区只有 dict 存取，**禁止**在持锁期间做 I/O 或调用外部回调。
- 高频数据（位姿、视觉帧）走这里；低频事件走 EventBus。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Iterable

# --------------------------------------------------------------------------- #
#  标准键名（字符串常量集中在此，避免各处拼写漂移）
# --------------------------------------------------------------------------- #
P_POSE = "pose"                       # Pose：信念位姿（米 + 地图 yaw）
P_POSE_RAW = "pose_raw"               # Pose：未加修正的原始融合位姿
P_ODOM_AGE = "odom_age"               # float：最近一帧里程计距今秒数
P_ROUTE = "route"                     # Route：当前路线（不可变）
P_ROUTE_INDEX = "route_index"         # int：当前航段索引
P_ROUTE_S = "route_s"                 # float：沿程弧长（m）
P_MISSION_PHASE = "mission_phase"     # str：IDLE/DRIVE/APPROACH/STOP/WAIT/FINISH/SAFE
P_STOP_POINT = "stop_point"           # StopPointState：当前停站点上下文
P_TARGET = "target"                   # TargetState：移动靶估计状态
P_AIM = "aim"                         # AimState：云台与开火状态
# --------------------------------------------------------------------------- #
#  ★ 共享键 `aim_red_face`（红面早退，卡 t13）—— 读写契约
# --------------------------------------------------------------------------- #
#: `P_AIM_RED`：**AimTask 写、MissionTask 读**（唯一消费者）。
#:
#: 背景（物理事实）：靶板一面蓝一面红，蓝面朝向每局随机但**恒定**；只打蓝、打红扣分。
#: 两个射击停站分居板面两侧 ⇒ 若本侧只见红面，等再久也不会变成蓝面，早退去对侧更划算。
#:
#: 取值语义（读侧必须严格按此实现）：
#:   · 键不存在 / `None` → **no-op**：Mission 行为与未加本特性时**逐位一致**（正常停站）；
#:   · `{"confirmed": False, "run_s": float, "cls": int, "threshold_s": float, "since": float}`
#:     → 只是观测（累计锁定红秒数），不得据此早退；
#:   · `{"confirmed": True, ...}` → 锁定红**连续累计** ≥ `aim.red_early_leave_s`
#:     ⇒ Mission 在射击停站内提前离站（`ACQUIRE`/`FIRE` 两段）。
#:   写侧纪律：**离开停站（`P_STOP_POINT.phase` 不再是 ACQUIRE/FIRE）或类别不再是红**
#:   时立即把 `run_s` 归零、`confirmed` 置 False —— 否则第二个停站一进去就被误判早退。
P_AIM_RED = "aim_red_face"            # dict：红面确认（AimTask 写 / MissionTask 读）
P_DRONE = "drone"                     # DroneState：无人机情报
P_FIX = "fix"                         # FixState：误差修正量（Localization 写：吸附/漂移台账）
P_FIX_EST = "fix_est"                 # FixState：**估计量**（FixTask 写：比例误差估计/漂移统计）
P_SAFETY = "safety"                   # SafetyState：安全状态
P_TELEMETRY = "telemetry"             # dict：遥测快照

# --------------------------------------------------------------------------- #
#  ★ 共享键 `pursuit_gear`（动态档选择，t36）—— 读写契约
# --------------------------------------------------------------------------- #
#: `P_PURSUIT_GEAR`：**Tracker 写、遥测/诊断读**（本轮起只作为"当前档"的观测面；
#:   实际生效路径是 `tasks/tracker.py` 把档值直接喂给 `pursuit.controller.compute_command(cruise_mps=…)`，
#:   因为驱动指令只有 Tracker 一个写者，不需要让别的模块去改速度）。
#: 取值语义（读侧必须严格按此实现，否则会破坏"关闭时逐位一致"的承诺）：
#:   · `None`（或键不存在）→ **完全 no-op**：命令速度与不开动态档时**逐位一致**；
#:   · `{"gear": "near"|"far"|"off", "cruise_mps": float, "dist_to_cross_m": float | None,
#:       "threshold_m": float, "enabled": bool, "since": float}`
#:     → 只是"当拍档位"的**观测**（供遥测/测试断言），不参与速度计算。
P_PURSUIT_GEAR = "pursuit_gear"

# --------------------------------------------------------------------------- #
#  ★ 共享键 `motion_limit`（限速/停机请求）—— 读写契约（t20 收口，双方引用此处）
# --------------------------------------------------------------------------- #
#: `P_MOTION_LIMIT`：**Localization 写**、**执行侧（pursuit 速度钳制 / 将来 Tracker）读**。
#:
#: 取值语义（读侧必须严格按此实现，否则会破坏"无漂移时逐位一致"的承诺）：
#:   · `None`（或键不存在）→ **完全 no-op**：读侧一行行为都不该因此改变 ——
#:     这是为了 t11 的"零注入大扫描"与现版本**逐位一致**；
#:   · `{"speed_mps": float, "reason": str, "drift_run": int, "anchor_refused": int,
#:       "distance_to_boundary_m": float | None, "since": float}`
#:     → 把线速度**钳到 `speed_mps` 以内**：只降不升；倒车/负速度**按绝对值钳**（保号）；
#:       `speed_mps = 0.0` 是"请求停车"，读侧必须真的给 0（**不受 `pursuit.min_speed_mps`
#:       蠕动下限兜底**，否则请求会被下限顶回去）；`reason` 等其余字段仅供日志/遥测。
#:   本键**不取代 Safety**：`P_SAFETY` 仍是唯一的强制停车者，这里只是"提前一脚刹车"。
#:
#: 为什么键名常量放在这里：写侧（`tasks/localization.py`）与读侧（`pursuit/controller.py`）
#: 各写一份字符串就是"拼写漂移"的温床；两侧都引用本常量，并由
#: `tests/test_motion_limit.py` 断言 `localization.P_MOTION_LIMIT == P_MOTION_LIMIT`。
P_MOTION_LIMIT = "motion_limit"


@dataclass(frozen=True)
class Entry:
    value: Any
    source: str
    ts: float


class Blackboard:
    """线程安全的键值共享区。"""

    def __init__(self) -> None:
        self._data: dict[str, Entry] = {}
        self._lock = threading.RLock()
        self.writes = 0

    # ---------------------------- 写 ---------------------------- #
    def set(self, key: str, value: Any, source: str = "", ts: float | None = None) -> None:
        entry = Entry(value=value, source=source, ts=time.time() if ts is None else ts)
        with self._lock:
            self._data[key] = entry
            self.writes += 1

    def update(self, source: str = "", **items: Any) -> None:
        now = time.time()
        with self._lock:
            for k, v in items.items():
                self._data[k] = Entry(value=v, source=source, ts=now)
                self.writes += 1

    # ---------------------------- 读 ---------------------------- #
    def get(self, key: str, default: Any = None) -> Any:
        with self._lock:
            entry = self._data.get(key)
        return default if entry is None else entry.value

    def entry(self, key: str) -> Entry | None:
        with self._lock:
            return self._data.get(key)

    def age(self, key: str) -> float | None:
        """该键距今秒数；不存在返回 None。"""
        with self._lock:
            entry = self._data.get(key)
        return None if entry is None else time.time() - entry.ts

    def get_fresh(self, key: str, max_age_s: float, default: Any = None) -> Any:
        """只在数据新鲜时返回值，否则返回 default（防止用过期的位姿/目标发指令）。"""
        with self._lock:
            entry = self._data.get(key)
        if entry is None or (time.time() - entry.ts) > max_age_s:
            return default
        return entry.value

    def is_fresh(self, key: str, max_age_s: float) -> bool:
        age = self.age(key)
        return age is not None and age <= max_age_s

    def snapshot(self, keys: Iterable[str] | None = None) -> dict[str, Any]:
        """一致快照（单次加锁取全部），供任务每拍开头读取。"""
        with self._lock:
            if keys is None:
                return {k: e.value for k, e in self._data.items()}
            return {k: self._data[k].value for k in keys if k in self._data}

    def describe(self) -> dict[str, dict]:
        """调试用：每个键的值摘要 + 年龄 + 来源。"""
        now = time.time()
        with self._lock:
            items = list(self._data.items())
        return {
            k: {"value": repr(e.value)[:120], "age_s": round(now - e.ts, 3), "source": e.source}
            for k, e in items
        }
