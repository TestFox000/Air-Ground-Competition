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
P_POSE_UPDATED = "pose_updated_at"    # float：视觉/里程计更新时间
P_ODOM_AGE = "odom_age"               # float：最近一帧里程计距今秒数
P_ROUTE = "route"                     # Route：当前路线（不可变）
P_ROUTE_INDEX = "route_index"         # int：当前航段索引
P_ROUTE_S = "route_s"                 # float：沿程弧长（m）
P_MISSION_PHASE = "mission_phase"     # str：IDLE/DRIVE/APPROACH/STOP/WAIT/FINISH/SAFE
P_STOP_POINT = "stop_point"           # StopPointState：当前停站点上下文
P_TARGET = "target"                   # TargetState：移动靶估计状态
P_AIM = "aim"                         # AimState：云台与开火状态
P_DRONE = "drone"                     # DroneState：无人机情报
P_FIX = "fix"                         # FixState：误差修正量（Localization 写：吸附/漂移台账）
P_FIX_EST = "fix_est"                 # FixState：**估计量**（FixTask 写：比例误差估计/漂移统计）
P_SAFETY = "safety"                   # SafetyState：安全状态
P_TELEMETRY = "telemetry"             # dict：遥测快照
P_CONFIG = "config"                   # dict：配置快照（只读）


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
