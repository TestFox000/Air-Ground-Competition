# -*- coding: utf-8 -*-
"""路线数据模型 —— 用**不可变对象**替代旧代码的 5 个平行数组。

旧代码的教训（docs/survey_path_executor.md §8.2）：
`path_points / shoot_flags / scan_flags / reverse_flags / shoot_groups` + 每点四元数
共 6 个平行数组靠手工同步，插入一个接近段要在 5 处 `insert`，漏一个就**静默错位**。

本模块把"一个航点"收敛成一个 frozen dataclass，路线本身不可变；
需要拼接/改写时用 `RouteBuilder` 生成**新的** Route（写时复制，线程安全）。
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Iterable, Iterator, Sequence

from ..core import geometry as geo


@dataclass(frozen=True)
class Waypoint:
    """一个航点（不可变）。"""

    block: str                      # 格名，如 "K9"
    x: float                        # 米制 x（格心）
    y: float                        # 米制 y（格心）
    yaw: float                      # **到达该点时应有的朝向**（离开段方向 / 末点终朝向）
    reverse: bool = False           # R: 前缀 —— 该段倒车到位
    shoot: bool = False             # `*` 后缀 —— 射击停站
    scan: bool = False              # `:SCAN_STOP` —— 纯扫描停站
    speed_scale: float = 1.0        # 该段限速系数（1.0 = 巡航）
    dwell_s: float = 0.0            # 到位后停留秒数（如需）
    group: str | None = None        # SHOOT@ 候选链组号（同组打中一个即跳过其余）
    branch: str | None = None       # 分叉标签（如 "cross_lower" / "cross_upper"）
    token: str = ""                 # 原始 token（日志/回溯用）

    # ------------------------------------------------------------------ #
    @classmethod
    def at_block(cls, block: str, *, yaw: float | None = None, **kw) -> "Waypoint":
        x, y = geo.block_to_meters(block)
        return cls(block=block, x=x, y=y,
                   yaw=geo.HEADING_X if yaw is None else yaw, **kw)

    @property
    def is_stop_point(self) -> bool:
        return self.shoot or self.scan

    def distance_to(self, x: float, y: float) -> float:
        return ((self.x - x) ** 2 + (self.y - y) ** 2) ** 0.5

    def with_yaw(self, yaw: float) -> "Waypoint":
        return replace(self, yaw=yaw)

    def heading_to(self, other: "Waypoint") -> float:
        return geo.heading_of(other.x - self.x, other.y - self.y, fallback=self.yaw)


@dataclass(frozen=True)
class Route:
    """一条不可变路线。"""

    waypoints: tuple[Waypoint, ...]
    name: str = ""
    #: 本路线实际采用的穿洞格，如 {"lower": "K9", "upper": "K4"}
    crossing: dict = field(default_factory=dict)

    # --------------------------- 容器协议 --------------------------- #
    def __len__(self) -> int:
        return len(self.waypoints)

    def __iter__(self) -> Iterator[Waypoint]:
        return iter(self.waypoints)

    def __getitem__(self, idx):
        return self.waypoints[idx]

    def __bool__(self) -> bool:
        return len(self.waypoints) > 0

    # --------------------------- 查询 --------------------------- #
    def blocks(self) -> list[str]:
        return [w.block for w in self.waypoints]

    def tokens(self) -> list[str]:
        return [w.token or w.block for w in self.waypoints]

    @property
    def start(self) -> Waypoint | None:
        return self.waypoints[0] if self.waypoints else None

    @property
    def end(self) -> Waypoint | None:
        return self.waypoints[-1] if self.waypoints else None

    def stop_points(self) -> list[tuple[int, Waypoint]]:
        return [(i, w) for i, w in enumerate(self.waypoints) if w.is_stop_point]

    def shoot_points(self) -> list[tuple[int, Waypoint]]:
        return [(i, w) for i, w in enumerate(self.waypoints) if w.shoot]

    def reverse_segments(self) -> list[int]:
        """倒车段的**起点索引**集合（段 i = 点 i → 点 i+1）。"""
        return [i for i, w in enumerate(self.waypoints)
                if i + 1 < len(self.waypoints) and self.waypoints[i + 1].reverse]

    def path_length(self) -> float:
        total = 0.0
        for a, b in zip(self.waypoints, self.waypoints[1:]):
            total += ((b.x - a.x) ** 2 + (b.y - a.y) ** 2) ** 0.5
        return total

    def describe(self, limit: int = 40) -> str:
        head = ", ".join(self.blocks()[:limit])
        more = f" …(+{len(self) - limit})" if len(self) > limit else ""
        return (f"路线[{self.name}] {len(self)} 点 / {self.path_length():.2f} m"
                f" / 穿洞 {self.crossing}\n  {head}{more}")

    # --------------------------- 派生 --------------------------- #
    def as_list(self) -> list[Waypoint]:
        return list(self.waypoints)

    def slice(self, start: int, end: int | None = None) -> "Route":
        return Route(self.waypoints[start:end], name=self.name, crossing=dict(self.crossing))


class RouteBuilder:
    """增量构建路线（前缀 + 中段 + 后缀的拼接都走这里）。"""

    def __init__(self, name: str = "") -> None:
        self._points: list[Waypoint] = []
        self._name = name
        self._crossing: dict[str, str] = {}

    def add(self, waypoint: Waypoint) -> "RouteBuilder":
        self._points.append(waypoint)
        return self

    def add_block(self, block: str, **kw) -> "RouteBuilder":
        return self.add(Waypoint.at_block(block, **kw))

    def extend(self, waypoints: Iterable[Waypoint]) -> "RouteBuilder":
        for w in waypoints:
            self.add(w)
        return self

    def add_route(self, route: Route) -> "RouteBuilder":
        return self.extend(route.waypoints)

    def crossing(self, half: str, block: str) -> "RouteBuilder":
        self._crossing[half] = block
        return self

    def __len__(self) -> int:
        return len(self._points)

    def build(self, *, fix_final_yaw: float | None = None,
              drop_consecutive_duplicates: bool = True) -> Route:
        """收尾：填中间点朝向（= 离开段方向）、去重相邻同格点、末点朝向。"""
        pts = list(self._points)
        if drop_consecutive_duplicates:
            deduped: list[Waypoint] = []
            for w in pts:
                if deduped and deduped[-1].block == w.block and deduped[-1].reverse == w.reverse:
                    # 保留信息更丰富的那个（射击/扫描标记）
                    prev = deduped[-1]
                    if (w.shoot or w.scan) and not (prev.shoot or prev.scan):
                        deduped[-1] = replace(w, reverse=prev.reverse)
                    continue
                deduped.append(w)
            pts = deduped

        out: list[Waypoint] = []
        for i, w in enumerate(pts):
            if i + 1 < len(pts):
                yaw = pts[i].heading_to(pts[i + 1])
            else:
                yaw = fix_final_yaw if fix_final_yaw is not None else w.yaw
            out.append(replace(w, yaw=yaw))
        return Route(tuple(out), name=self._name, crossing=dict(self._crossing))


def make_route(waypoints: Sequence[Waypoint], name: str = "", crossing: dict | None = None) -> Route:
    """便捷构造：直接给定航点序列（朝向按"离开段方向"补齐）。"""
    builder = RouteBuilder(name)
    builder.extend(waypoints)
    route = builder.build()
    if crossing:
        return Route(route.waypoints, name=route.name, crossing=dict(crossing))
    return route
