"""core/blackboard.py — 线程安全共享状态（黑盒）。

多线程环境下（V2 的 LocalizationTask / TrackerTask / MissionTask 等线程间）
通过 Blackboard 共享状态，替代 V1 的 self.* 平铺字段。

实现：写时替换 + RLock 保护 + 新鲜度守卫。
与 V1 相同语义：V1 中状态字段通过 `self.xxx` 共享，这里提供线程安全的 get/set。
"""

import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set, Tuple


@dataclass
class Entry:
    """黑盒中的一个条目。"""
    value: Any
    source: str
    ts: float  # time.monotonic() timestamp


# ── 键名常量 ────────────────────────────────────────────────
# 保持与 V1 字段命名一致的字符串键

# 定位
P_POSE_X = 'current_x'
P_POSE_Y = 'current_y'
P_POSE_YAW = 'current_yaw'
P_HAS_ODOM = 'has_odom'
P_SNAP_X_OFF = 'snap_x_off'
P_SNAP_Y_OFF = 'snap_y_off'
P_SNAP_YAW_OFF = 'snap_yaw_off'
P_RAW_X = 'raw_x'
P_RAW_Y = 'raw_y'
P_RAW_MAP_YAW = 'raw_map_yaw'

# 路径
P_PATH_POINTS = 'path_points'
P_CURRENT_TARGET_IDX = 'current_target_idx'
P_PATH_LOADED = 'path_loaded'
P_ARRIVED = 'arrived'
P_ARRIVE_LOGGED = 'arrive_logged'

# 路线状态
P_SHOOT_FLAGS = 'shoot_flags'
P_SHOOT_WAIT = 'shoot_wait'
P_SHOOT_WAIT_START = 'shoot_wait_start'
P_SHOOT_COUNT_AT_ARRIVAL = 'shoot_count_at_arrival'
P_SHOOT_COUNT = 'shoot_count'
P_SCAN_FLAGS = 'scan_flags'
P_SCAN_WAIT = 'scan_wait'
P_SCAN_WAIT_START = 'scan_wait_start'
P_SCAN_SEEN_SCANNING = 'scan_seen_scanning'
P_AIM_STATUS = 'aim_status'
P_CORNER_MODE = 'corner_mode'
P_CORNER_TARGET_YAW = 'corner_target_yaw'
P_CORNER_TURN_DIR = 'corner_turn_dir'
P_CORNER_PHASE = 'corner_phase'
P_CORNER_PHASE_T = 'corner_phase_t'
P_CORNER_SEG_IDX = 'corner_seg_idx'

# route index and arc-length（V2 新增，V1 用 current_target_idx 和 find_nearest_segment 隐式表示）
P_ROUTE_INDEX = 'route_index'
P_ROUTE_S = 'route_s'


@dataclass
class Entry:
    """黑盒中的一个条目。"""
    value: Any
    source: str
    ts: float  # time.monotonic() timestamp


class Blackboard:
    """线程安全共享状态容器。

    V1 中所有状态为 self.* 平铺字段，单线程直接读写。
    本类为多线程环境提供 RLock 保护 + 写来源追踪 + 新鲜度守卫。
    """

    def __init__(self):
        self._lock = threading.RLock()
        self._data: Dict[str, Entry] = {}

    # ── 写操作 ────────────────────────────────────────────

    def set(self, key: str, value: Any, source: str = 'unknown', ts: Optional[float] = None):
        """设置一个键值。

        Args:
            key: 键名（建议用 P_* 常量）
            value: 值（应不可变：int/float/str/bool/tuple/frozen dataclass）
            source: 写入者标识（如 'odom_callback', 'control_loop'）
            ts: 时间戳（默认 time.monotonic()）
        """
        if ts is None:
            ts = time.monotonic()
        with self._lock:
            self._data[key] = Entry(value=value, source=source, ts=ts)

    def update(self, source: str, **items: Any):
        """批量写（同一个 source）。"""
        ts = time.monotonic()
        with self._lock:
            for k, v in items.items():
                self._data[k] = Entry(value=v, source=source, ts=ts)

    # ── 读操作 ────────────────────────────────────────────

    def get(self, key: str, default: Any = None) -> Any:
        """获取值；键不存在返回 default。"""
        with self._lock:
            entry = self._data.get(key)
            return entry.value if entry is not None else default

    def entry(self, key: str) -> Optional[Entry]:
        """获取 Entry 结构体（值 + 来源 + 时间戳）。"""
        with self._lock:
            return self._data.get(key)

    def age(self, key: str) -> Optional[float]:
        """获取键距今秒数；键不存在返回 None。"""
        with self._lock:
            entry = self._data.get(key)
            if entry is None:
                return None
            return time.monotonic() - entry.ts

    def get_fresh(self, key: str, max_age_s: float, default: Any = None) -> Any:
        """获取值并校验新鲜度；过期返回 default。"""
        with self._lock:
            entry = self._data.get(key)
            if entry is None:
                return default
            if time.monotonic() - entry.ts > max_age_s:
                return default
            return entry.value

    def is_fresh(self, key: str, max_age_s: float) -> bool:
        """检查键是否新鲜。"""
        with self._lock:
            entry = self._data.get(key)
            if entry is None:
                return False
            return (time.monotonic() - entry.ts) <= max_age_s

    def snapshot(self, keys: Optional[Set[str]] = None) -> Dict[str, Any]:
        """取一致快照。keys=None 返回全部。"""
        with self._lock:
            if keys is None:
                return {k: e.value for k, e in self._data.items()}
            return {k: self._data[k].value for k in keys if k in self._data}

    def describe(self) -> Dict[str, Dict[str, Any]]:
        """调试用：返回全部键的摘要（值 + 来源 + 年龄）。"""
        now = time.monotonic()
        with self._lock:
            return {
                k: {'value': e.value, 'source': e.source, 'age_s': now - e.ts}
                for k, e in self._data.items()
            }

    def __contains__(self, key: str) -> bool:
        with self._lock:
            return key in self._data

    # 注意：故意不定义 __len__，empty 的 Blackboard 不应破坏 bool(bb) 语义。
    # 想检查空状态请用 len(bb._data) 或 read-only snapshot。
