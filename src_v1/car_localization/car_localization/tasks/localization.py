"""tasks/localization.py — 定位任务：里程计→信念位姿 + 锚定吸附。

从 V1 path_executor.py 拆出：
  - odom_callback (L829-851)：格单位→米 + snap offset 应用
  - snap 锚定逻辑 (L283-295, L1179-1196)：到点后绑定 belief pose 到网格中心
  - has_odom 标志 (L220, L844)

设计模式：TaskThread 子类。每 tick：
  1. 读原始里程计缓存
  2. 计算信念位姿（raw + snap offset）
  3. 若有路线 → 计算投影（用于到位检测）
  4. 吸附逻辑：当 dist < arrival_threshold 时锚定
"""

import math
import time
from typing import Any, Dict, List, Optional, Tuple

from .base import TaskThread
from ..core.blackboard import (
    Blackboard,
    P_POSE_X, P_POSE_Y, P_POSE_YAW,
    P_HAS_ODOM,
    P_SNAP_X_OFF, P_SNAP_Y_OFF, P_SNAP_YAW_OFF,
    P_RAW_X, P_RAW_Y, P_RAW_MAP_YAW,
    P_PATH_POINTS, P_PATH_LOADED, P_ARRIVED,
    P_SHOOT_FLAGS, P_SCAN_FLAGS,
)
from ..core.geometry import (
    normalize_angle, block_to_meters,
    euler_from_quaternion, Pose,
)
from ..pursuit.path import Path, Projection, find_nearest_segment_v1


class LocalizationTask(TaskThread):
    """定位任务：20 Hz 更新信念位姿 + 锚定吸附。

    与 V1 odom_callback (L829-851) + 锚定逻辑 (L1179-1196) 行为一致：
      - 从原始里程计读取块坐标 → 米（BLOCK_SIZE 系数）
      - 应用 snap offset（snap_x_off/y_off/yaw_off）
      - 写黑盒：current_x/y/yaw、has_odom、raw_*

    V1 的锚定在 arrivl / shoot_stop 时修改 snap 偏移；本模块
    提供 anchor_to_point() 方法供外部（如 MissionTask）调用。
    """

    def __init__(
        self,
        name: str = 'localization',
        rate_hz: float = 20.0,
        blackboard: Optional[Blackboard] = None,
        logger: Optional[Any] = None,
        wheelbase: float = 0.157,
        arrival_threshold: float = 0.157,
        block_size: float = 0.35,
    ):
        super().__init__(name=name, rate_hz=rate_hz, logger=logger)
        if blackboard is None:
            blackboard = Blackboard()
        self._bb: Blackboard = blackboard
        self._wheelbase: float = wheelbase
        self._arrival_threshold: float = arrival_threshold
        self._block_size: float = block_size

        # 缓存原始里程计读数的接口（由外部注入）
        self._odom_raw_x: float = 0.0
        self._odom_raw_y: float = 0.0
        self._odom_raw_qx: float = 0.0
        self._odom_raw_qy: float = 0.0
        self._odom_raw_qz: float = 0.0
        self._odom_raw_qw: float = 1.0
        self._odom_has_data: bool = False

        # 上次读取的 snap offset 缓存
        self._last_snap_x: float = 0.0
        self._last_snap_y: float = 0.0
        self._last_snap_yaw: float = 0.0

    # ── 外部注入原始里程计 ──────────────────────────────────

    def feed_odom(
        self,
        raw_x_block: float,
        raw_y_block: float,
        qx: float, qy: float, qz: float, qw: float,
    ) -> None:
        """注入原始里程计（格单位坐标和四元数）。

        V1 odom_callback L840-841 使用 msg.pose.pose.position.x * BLOCK_SIZE
        但此处的 raw_x_block 是**已乘了 BLOCK_SIZE** 后的米制坐标。
        """
        self._odom_raw_x = raw_x_block
        self._odom_raw_y = raw_y_block
        self._odom_raw_qx = qx
        self._odom_raw_qy = qy
        self._odom_raw_qz = qz
        self._odom_raw_qw = qw
        self._odom_has_data = True

    # ── tick ─────────────────────────────────────────────────

    def tick(self) -> None:
        """每拍更新信念位姿。

        等价于 V1 odom_callback L829-851：
          1. 原始坐标 x0.35 → 米
          2. belief = raw + snap offset
          3. yaw 转换（raw_yaw + π/2 → map frame + snap_yaw_off）
        """
        if not self._odom_has_data:
            return  # 尚无里程计数据

        # － 原始位姿（V1 L840-841）－
        raw_x = self._odom_raw_x
        raw_y = self._odom_raw_y

        # － yaw 解析（V1 L846-849）－
        raw_yaw = euler_from_quaternion(
            self._odom_raw_qx, self._odom_raw_qy,
            self._odom_raw_qz, self._odom_raw_qw,
        )
        # V1 L849: raw_map_yaw = normalize_angle(raw_yaw + π/2)
        raw_map_yaw = normalize_angle(raw_yaw + math.pi / 2.0)

        # － snap offset（从黑盒读取，V1 L842-843, L850-851）－
        snap_x = self._bb.get(P_SNAP_X_OFF, 0.0)
        snap_y = self._bb.get(P_SNAP_Y_OFF, 0.0)
        snap_yaw = self._bb.get(P_SNAP_YAW_OFF, 0.0)

        # 缓存（供 tick 外使用）
        self._last_snap_x = snap_x
        self._last_snap_y = snap_y
        self._last_snap_yaw = snap_yaw

        # － 信念位姿（V1 L842-843, L850-851）－
        current_x = raw_x + snap_x
        current_y = raw_y + snap_y
        current_yaw = normalize_angle(raw_map_yaw + snap_yaw)

        # － 写黑盒 －
        now_t = time.monotonic()
        self._bb.update(
            'localization',
            **{
                P_RAW_X: raw_x,
                P_RAW_Y: raw_y,
                P_RAW_MAP_YAW: raw_map_yaw,
                P_HAS_ODOM: True,
                P_POSE_X: current_x,
                P_POSE_Y: current_y,
                P_POSE_YAW: current_yaw,
            }
        )

    # ── 锚定吸附 ────────────────────────────────────────────

    def anchor_to_point(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
    ) -> None:
        """锚定信念位姿到指定点（修正 snap offset）。

        等价于 V1 L1179-1196（end arrival）和 L797-809（shoot stop）：
          drift_x = current_x - target_x
          snap_x_off -= drift_x
          current_x = target_x
          （同理 yaw）

        Args:
            target_x, target_y: 目标坐标（格中心）
            target_yaw: 目标朝向（弧度）
        """
        cur_x = self._bb.get(P_POSE_X, 0.0)
        cur_y = self._bb.get(P_POSE_Y, 0.0)
        cur_yaw = self._bb.get(P_POSE_YAW, 0.0)

        drift_x = cur_x - target_x
        drift_y = cur_y - target_y
        yaw_drift = normalize_angle(cur_yaw - target_yaw)

        cur_snap_x = self._bb.get(P_SNAP_X_OFF, 0.0)
        cur_snap_y = self._bb.get(P_SNAP_Y_OFF, 0.0)
        cur_snap_yaw = self._bb.get(P_SNAP_YAW_OFF, 0.0)

        new_snap_x = cur_snap_x - drift_x
        new_snap_y = cur_snap_y - drift_y
        new_snap_yaw = normalize_angle(cur_snap_yaw - yaw_drift)

        self._bb.update(
            'localization.anchor',
            **{
                P_SNAP_X_OFF: new_snap_x,
                P_SNAP_Y_OFF: new_snap_y,
                P_SNAP_YAW_OFF: new_snap_yaw,
                P_POSE_X: target_x,
                P_POSE_Y: target_y,
                P_POSE_YAW: target_yaw,
            }
        )

    # ── 便利访问 ────────────────────────────────────────────

    def get_pose(self) -> Pose:
        """获取当前信念位姿。"""
        return Pose(
            x=self._bb.get(P_POSE_X, 0.0),
            y=self._bb.get(P_POSE_Y, 0.0),
            yaw=self._bb.get(P_POSE_YAW, 0.0),
        )

    def get_pose_dict(self) -> Dict[str, float]:
        """获取当前信念位姿（dict 格式，兼容 V1）。"""
        return {
            'x': self._bb.get(P_POSE_X, 0.0),
            'y': self._bb.get(P_POSE_Y, 0.0),
            'yaw': self._bb.get(P_POSE_YAW, 0.0),
        }

    def has_odom(self) -> bool:
        """是否有里程计数据。"""
        return self._bb.get(P_HAS_ODOM, False)
