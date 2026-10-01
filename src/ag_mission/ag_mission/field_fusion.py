"""多帧情报融合：把无人机连续下发的侦察结果投票成一份稳定情报。

为什么需要：
    机载端是单帧检测，受光照 / 抖动 / 同频干扰影响会有偶发误报；
    而地面车一旦按错误情报排雷或打击友方，扣分不可逆。
    因此车端不直接采信最新一帧，而是要求同一格号在滑动窗口内被反复确认。

设计：
    * 按 seq 丢弃乱序与重复帧（链路本身已有 crc32，这里只处理顺序）
    * 地雷 / 障碍物：窗口内出现次数 >= min_votes 且占比 >= min_ratio 才采纳
    * 桥洞 / 敌我面：取窗口内出现次数最多的取值（众数）
    * 窗口按时间（ttl_s）与条数（max_history）双重裁剪

本模块 **不依赖 ROS**，可在 Windows 上直接单测。
"""

from __future__ import annotations

import time
from collections import Counter, deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Tuple

from ag_core.link_protocol import FieldIntel

__all__ = ["IntelVoteConfig", "IntelVoter"]


@dataclass
class IntelVoteConfig:
    min_votes: int = 2            # 同一格号至少要被确认几帧
    min_ratio: float = 0.5        # 且在窗口内出现率不低于此值
    max_history: int = 24         # 窗口最多保留多少帧
    ttl_s: float = 20.0           # 窗口时间跨度
    max_mines: int = 8            # 场地最多 8 颗雷，超出按票数取前 N
    max_obstacles: int = 8


def _top_n(counter: Counter, n: int, min_votes: int, min_ratio: float, total: int) -> List[str]:
    if total <= 0:
        return []
    out = []
    for item, cnt in counter.most_common():
        if cnt < min_votes:
            continue
        if cnt / total < min_ratio:
            continue
        out.append(item)
        if len(out) >= n:
            break
    return sorted(out)


def _mode(counter: Counter, min_votes: int) -> Optional[str]:
    if not counter:
        return None
    item, cnt = counter.most_common(1)[0]
    return item if cnt >= min_votes else None


@dataclass
class IntelVoter:
    cfg: IntelVoteConfig = field(default_factory=IntelVoteConfig)
    _frames: Deque[Tuple[float, FieldIntel]] = field(default_factory=deque, repr=False)
    _last_seq: int = -1

    # ---------------- 输入 ----------------
    def push(self, intel: FieldIntel, now: Optional[float] = None) -> bool:
        """喂入一帧情报。返回 True 表示这帧被接受（非乱序/重复）。"""
        now = time.time() if now is None else now
        if intel.seq <= self._last_seq:
            # ⚠️ 发送端**重启**会把 seq 归零重来（机载端崩溃重启、重开 fake_intel
            #    都会发生）。不允许回退的话，车端从此**永久拒收**——链路上
            #    心跳还在跳、看着像活着，其实情报再也进不来，最难查的一种死法。
            #    重启的两种形态都要认：
            #      ① seq 回到 0/1（发送端第一帧）——哪怕只回退几帧也是重启
            #      ② 大幅回退（>8 帧）——乱序不可能差这么多
            #    小幅回退（乱序/重复）照样丢弃。
            is_restart = (intel.seq <= 1 and intel.seq < self._last_seq) or \
                         (intel.seq < self._last_seq - 8)
            if is_restart:
                self.reset()
            else:
                return False
        self._last_seq = intel.seq
        if intel.ts <= 0:
            intel.ts = now
        self._frames.append((now, intel))
        self._trim(now)
        return True

    def reset(self) -> None:
        self._frames.clear()
        self._last_seq = -1

    # ---------------- 输出 ----------------
    @property
    def frame_count(self) -> int:
        return len(self._frames)

    @property
    def stable(self) -> Optional[FieldIntel]:
        """当前窗口投票出的稳定情报；窗口为空时返回 None。"""
        if not self._frames:
            return None
        total = len(self._frames)
        mines_c: Counter = Counter()
        obst_c: Counter = Counter()
        bridge_a_c: Counter = Counter()
        bridge_b_c: Counter = Counter()
        face_c: Counter = Counter()
        target_cells_c: Counter = Counter()
        latest: Optional[FieldIntel] = None

        for _, it in self._frames:
            latest = it
            for m in it.mines:
                mines_c[m] += 1
            for o in it.obstacles:
                obst_c[o] += 1
            if it.bridge_a:
                bridge_a_c[it.bridge_a] += 1
            if it.bridge_b:
                bridge_b_c[it.bridge_b] += 1
            if it.target_face:
                face_c[it.target_face] += 1
            for c in it.target_cells:
                target_cells_c[c] += 1

        assert latest is not None
        return FieldIntel(
            mines=_top_n(mines_c, self.cfg.max_mines, self.cfg.min_votes, self.cfg.min_ratio, total),
            obstacles=_top_n(obst_c, self.cfg.max_obstacles, self.cfg.min_votes, self.cfg.min_ratio, total),
            bridge_a=_mode(bridge_a_c, self.cfg.min_votes),
            bridge_b=_mode(bridge_b_c, self.cfg.min_votes),
            target_cells=sorted(c for c, n in target_cells_c.items() if n >= self.cfg.min_votes),
            target_face=_mode(face_c, self.cfg.min_votes),
            ts=latest.ts,
            seq=latest.seq,
        )

    @property
    def is_confident(self) -> bool:
        """情报是否已足以支撑一次规划（至少有一颗雷或一处桥洞被确认）。"""
        st = self.stable
        return bool(st and (st.mines or st.bridge_a or st.bridge_b))

    # ---------------- 内部 ----------------
    def _trim(self, now: float) -> None:
        while self._frames and (now - self._frames[0][0]) > self.cfg.ttl_s:
            self._frames.popleft()
        while len(self._frames) > self.cfg.max_history:
            self._frames.popleft()
