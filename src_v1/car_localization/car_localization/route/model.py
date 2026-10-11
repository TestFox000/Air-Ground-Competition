"""route/model.py — Waypoint / Route 数据模型。

替代 V1 的 dict 列表 + 平行数组（path_points, shoot_flags, scan_flags）。
每点四元数 → Waypoint.yaw 统一存储；平行数组合并为 Waypoint 字段。
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from ..core.geometry import BLOCK_SIZE, COL_LETTERS, block_to_meters, yaw_to_quaternion


@dataclass(frozen=True)
class Waypoint:
    """单个航点。

    对应 V1 path_points[i] dict:
      {'x','y','z','qx','qy','qz','qw'}  +  shoot_flags[i] + scan_flags[i]

    V1 使用四元数存朝向，本类使用 yaw 弧度；通过 property 提供四元数接口保持兼容。
    """
    x: float      # 米制 x
    y: float      # 米制 y
    yaw: float    # 朝向（弧度）
    z: float = 0.0
    reverse: bool = False         # 是否倒车段（V1 无，保留）
    shoot: bool = False           # 是否射击停车点（V1 shoot_flags[i]）
    scan: bool = True             # 是否触发全向扫描（V1 scan_flags[i]；默认 True 因为路线点位一律触发)
    speed_scale: float = 1.0     # 速度系数
    dwell_s: float = 0.0         # 停站时间
    group: int = 0                # 分支组
    branch: str = ''              # 分支标识
    token: str = ''               # 原始 token
    pivot: bool = False           # 是否需要 PIVOT（V1 运行时判，模型保留标记位）

    # ── 便利构造 ──────────────────────────────────────────

    @staticmethod
    def at_block(block: str, yaw: Optional[float] = None, **kw) -> 'Waypoint':
        """从格名构造 Waypoint（自动算米制坐标）。

        Args:
            block: 格名，如 'M10', 'G8'
            yaw: 朝向弧度，默认根据 heading_dir 或 block 方向推断
            **kw: 传给 Waypoint 的其他字段
        """
        x, y = block_to_meters(block)
        if yaw is None:
            yaw = 0.0
        return Waypoint(x=x, y=y, yaw=yaw, token=block, **kw)

    @staticmethod
    def from_v1_dict(d: dict) -> 'Waypoint':
        """从 V1 格式 path_points[i] dict 构造。"""
        qz = d.get('qz', 0.0)
        qw = d.get('qw', 1.0)
        yaw = 2 * math.atan2(qz, qw)
        return Waypoint(
            x=d['x'], y=d['y'], yaw=yaw,
            z=d.get('z', 0.0),
        )

    # ── 属性 ──────────────────────────────────────────────

    @property
    def quaternion(self) -> Tuple[float, float, float, float]:
        """返回四元数 (qx, qy, qz, qw)，与 yaw_to_quaternion 一致。"""
        return yaw_to_quaternion(self.yaw)

    def to_v1_dict(self) -> dict:
        """转回 V1 dict 格式（path_points 中的一项）。"""
        qz, qw = self.quaternion[2], self.quaternion[3]
        return {
            'x': self.x, 'y': self.y, 'z': self.z,
            'qx': 0.0, 'qy': 0.0, 'qz': qz, 'qw': qw,
        }

    def is_stop_point(self) -> bool:
        """是否停站（射击或扫描）。"""
        return self.shoot or self.scan

    def distance_to(self, x: float, y: float) -> float:
        """到目标坐标的欧氏距离。"""
        return math.hypot(self.x - x, self.y - y)

    def with_yaw(self, yaw: float) -> 'Waypoint':
        """替换 yaw 生成新 Waypoint。"""
        return Waypoint(
            x=self.x, y=self.y, yaw=yaw, z=self.z,
            reverse=self.reverse, shoot=self.shoot, scan=self.scan,
            speed_scale=self.speed_scale, dwell_s=self.dwell_s,
            group=self.group, branch=self.branch, token=self.token,
            pivot=self.pivot,
        )

    def heading_to(self, other: 'Waypoint') -> float:
        """指向另一航点的方向角（弧度）。"""
        return math.atan2(other.y - self.y, other.x - self.x)


@dataclass(frozen=True)
class Route:
    """不可变路线（V1 self.path_points 列表的替代品）。"""
    waypoints: Tuple[Waypoint, ...]  # 航点序列
    name: str = ''                   # 路线名
    crossing: Dict[str, str] = field(default_factory=dict)  # {"lower": "K9", "upper": "K4"}

    def __len__(self) -> int:
        return len(self.waypoints)

    def __getitem__(self, idx: int) -> Waypoint:
        return self.waypoints[idx]

    def blocks(self) -> List[str]:
        """返回格名列表。"""
        return [wp.token for wp in self.waypoints if wp.token]

    def tokens(self) -> List[str]:
        """返回 token 列表。"""
        return [wp.token for wp in self.waypoints if wp.token]

    def stop_points(self) -> List[Tuple[int, Waypoint]]:
        """返回停站航点列表 [(索引, 航点)]。"""
        return [(i, wp) for i, wp in enumerate(self.waypoints) if wp.is_stop_point()]

    def pivot_points(self) -> List[Tuple[int, Waypoint]]:
        """返回折返点列表。"""
        return [(i, wp) for i, wp in enumerate(self.waypoints) if wp.pivot]

    def path_length(self) -> float:
        """返回折线总长。"""
        total = 0.0
        for i in range(1, len(self.waypoints)):
            total += math.hypot(
                self.waypoints[i].x - self.waypoints[i-1].x,
                self.waypoints[i].y - self.waypoints[i-1].y,
            )
        return total


class RouteBuilder:
    """路线构建器（V1 follow_route_command 中内联建 list 的独立版）。

    使用方式（模拟 V1 逻辑）：
      builder = RouteBuilder(name='route')
      builder.add(block='G8', yaw=...)
      builder.add(block='A10', shoot=True)
      route = builder.build()
    """

    def __init__(self, name: str = ''):
        self._waypoints: List[Waypoint] = []
        self._name = name

    def add(self, block: str = '', x: float = 0.0, y: float = 0.0,
            yaw: float = 0.0, shoot: bool = False, scan: bool = True,
            token: str = '', **kw) -> 'RouteBuilder':
        """添加一个航点。"""
        if block:
            x, y = block_to_meters(block)
        wp = Waypoint(x=x, y=y, yaw=yaw, shoot=shoot, scan=scan,
                      token=token or block, **kw)
        # 去重（V1 L650-654 逻辑）
        if self._waypoints:
            last = self._waypoints[-1]
            if abs(last.x - wp.x) < 1e-9 and abs(last.y - wp.y) < 1e-9:
                if shoot:
                    self._waypoints[-1] = Waypoint(
                        x=last.x, y=last.y, yaw=last.yaw, z=last.z,
                        reverse=last.reverse, shoot=True,
                        scan=last.scan, speed_scale=last.speed_scale,
                        dwell_s=last.dwell_s, group=last.group,
                        branch=last.branch, token=last.token, pivot=last.pivot,
                    )
                return self
        self._waypoints.append(wp)
        return self

    def build(self, fix_final_yaw: Optional[float] = None,
              drop_consecutive_duplicates: bool = True,
              approach_dist: float = 0.6) -> Route:
        """构建不可变 Route。

        Args:
            fix_final_yaw: 强制终点 yaw（对应 V1 final_heading 逻辑）
            drop_consecutive_duplicates: 去重（默认 True）
            approach_dist: 末端接近段长度
        """
        wps = list(self._waypoints)
        if len(wps) < 1:
            return Route(waypoints=(), name=self._name)

        # 回填中间点 yaw（方向 = 离开段方向）
        for i in range(len(wps)):
            if i + 1 < len(wps):
                # 中间点：指向下一个点
                yw = math.atan2(wps[i + 1].y - wps[i].y,
                                wps[i + 1].x - wps[i].x)
                wps[i] = wps[i].with_yaw(yw)
            elif fix_final_yaw is not None:
                # 末点：强制 yaw
                wps[i] = wps[i].with_yaw(fix_final_yaw)

        return Route(waypoints=tuple(wps), name=self._name)


def make_route(waypoints: List[Waypoint], name: str = '',
               crossing: Optional[Dict[str, str]] = None) -> Route:
    """快速构造 Route。"""
    return Route(waypoints=tuple(waypoints), name=name,
                 crossing=crossing or {})
