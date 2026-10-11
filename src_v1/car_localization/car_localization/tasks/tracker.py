"""tasks/tracker.py — 循迹追踪任务：纯追踪 + PIVOT 原地掉头。

从 V1 path_executor.py control_loop (L1036-1372) 拆出**纯循迹职责**部分：
  - 航段投影 + 前视点（L1107-1118）
  - 纯追踪舵角 + 速度（L1316-1356）
  - PIVOT 原地掉头三段式（L1217-1314）
  - 末端减速和弯前减速（L1320-1348）
  - 扫描停站检测（L1120-1139）
  - 射击停站检测（L1141-1153）

不包含：射击等待、扫描等待、终点到达逻辑（这些在 MissionTask 中）。
"""

import math
import time
from typing import Any, Dict, List, Optional, Tuple

from .base import TaskThread
from ..core.blackboard import (
    Blackboard,
    P_POSE_X, P_POSE_Y, P_POSE_YAW, P_HAS_ODOM,
    P_PATH_POINTS, P_PATH_LOADED, P_ARRIVED,
    P_SHOOT_FLAGS, P_SHOOT_STOP_RADIUS,
    P_SCAN_FLAGS, P_SCAN_STOP_RADIUS, P_AIM_STATUS,
    P_CORNER_MODE, P_CORNER_TARGET_YAW, P_CORNER_TURN_DIR,
    P_CORNER_PHASE, P_CORNER_PHASE_T, P_CORNER_SEG_IDX,
    P_SHOOT_WAIT, P_SCAN_WAIT,
)
from ..core.geometry import normalize_angle
from ..pursuit.path import Path, Projection, find_nearest_segment_v1
from ..pursuit.lookahead import find_lookahead_point, LookaheadPoint
from ..pursuit.controller import compute_command, PursuitCommand, brake_command


# PIVOT 常量（与 V1 L1224-1227 一致）
SHARP_CORNER_RAD = math.radians(45.0)
CORNER_TRIGGER_M = 0.25
CORNER_ALIGN_RAD = math.radians(15.0)
CORNER_SLOW_M = 0.45

# 速度参数（V1 L239-240）
LOOKAHEAD_DIST = 0.15
LINEAR_SPEED = 0.3
WHEELBASE = 0.157
ARRIVAL_THRESHOLD = 0.157
SHOOT_STOP_RADIUS = 0.17
SCAN_STOP_RADIUS = 0.20
APPROACH_DIST = 0.6


class PivotPhase:
    """PIVOT 阶段（与 V1 corner_phase 一致：0/1/2）。"""
    IDLE = -1
    STOP = 0       # 停车+满舵
    SETTLE = 1     # 保持舵角 0.5s
    CREEP = 2      # 低速爬行直到对准


class TrackerTask(TaskThread):
    """循迹追踪任务：50Hz 纯追踪 + PIVOT 原地掉头。

    对应 V1 control_loop 的循迹部分（L1038-1118 + L1217-1372）：
      - 无路径 → 零速保活
      - 有路径 → 投影+前视点+纯追踪控制
      - 锐角弯 → PIVOT 三段式原地掉头
      - 扫描停站检测 + 射击停站检测（触发后由 MissionTask 处理）
      - 末端减速 + 弯前减速

    注：不处理射击/扫描等待（由 MissionTask 管理）。
    """

    def __init__(
        self,
        name: str = 'tracker',
        rate_hz: float = 50.0,
        blackboard: Optional[Blackboard] = None,
        logger: Optional[Any] = None,
        # 参数（与 V1 默认值一致）
        wheelbase: float = WHEELBASE,
        lookahead_dist: float = LOOKAHEAD_DIST,
        linear_speed: float = LINEAR_SPEED,
        arrival_threshold: float = ARRIVAL_THRESHOLD,
        shoot_stop_radius: float = SHOOT_STOP_RADIUS,
        scan_stop_radius: float = SCAN_STOP_RADIUS,
    ):
        super().__init__(name=name, rate_hz=rate_hz, logger=logger)
        if blackboard is None:
            blackboard = Blackboard()
        self._bb: Blackboard = blackboard
        self._wheelbase: float = wheelbase
        self._lookahead_dist: float = lookahead_dist
        self._linear_speed: float = linear_speed
        self._arrival_threshold: float = arrival_threshold
        self._shoot_stop_radius: float = shoot_stop_radius
        self._scan_stop_radius: float = scan_stop_radius

        # 内部状态
        self._path: Optional[Path] = None
        self._last_steer_deg: float = 0.0
        self._log_count: int = 0

        # 输出回调（由外部注入，如 io/ros_io.py）
        self._cmd_callback: Optional[callable] = None
        self._scan_req_callback: Optional[callable] = None

    def set_cmd_callback(self, cb: callable) -> None:
        """设置控制指令发布回调。"""
        self._cmd_callback = cb

    def set_scan_req_callback(self, cb: callable) -> None:
        """设置扫描请求发布回调。"""
        self._scan_req_callback = cb

    # ── 辅助 ─────────────────────────────────────────────────

    def _publish_cmd(self, linear_x: float, steer_deg: float) -> None:
        """发布控制指令（通过回调或直写黑盒）。"""
        if self._cmd_callback:
            self._cmd_callback(linear_x, steer_deg)
        # 记录到黑盒（MissionTask 可读）
        self._bb.update('tracker', linear_x=linear_x, steer_deg=steer_deg)

    def _build_path(self) -> Optional[Path]:
        """从黑盒 path_points 构建 Path 对象（缓存）。"""
        pts = self._bb.get(P_PATH_POINTS, [])
        if not pts or len(pts) < 2:
            self._path = None
            return None
        # 如果 path_points 变化了，重建缓存
        if self._path is None:
            self._path = Path(pts)
        return self._path

    # ── PIVOT 逻辑 ──────────────────────────────────────────

    def _check_pivot_entry(self, seg: Optional[dict]) -> None:
        """检测是否需要进入 PIVOT 模式（V1 L1229-1257）。"""
        if seg is None:
            return
        pts = self._bb.get(P_PATH_POINTS, [])
        if not pts or len(pts) < 3:
            return
        corner_mode = self._bb.get(P_CORNER_MODE, False)
        if corner_mode:
            return  # 已在 PIVOT 中
        idx = seg['idx']
        if idx + 2 >= len(pts):
            return  # 没有下一个段

        cx = self._bb.get(P_POSE_X, 0.0)
        cy = self._bb.get(P_POSE_Y, 0.0)
        vertex = pts[idx + 1]
        dvx = vertex['x'] - cx
        dvy = vertex['y'] - cy
        dist_to_vertex = math.hypot(dvx, dvy)

        p1 = pts[idx]
        p2 = pts[idx + 1]
        p3 = pts[idx + 2]
        d1x = p2['x'] - p1['x']
        d1y = p2['y'] - p1['y']
        d2x = p3['x'] - p2['x']
        d2y = p3['y'] - p2['y']
        cross = d1x * d2y - d1y * d2x
        dot = d1x * d2x + d1y * d2y
        corner_ang = abs(math.atan2(cross, dot))

        if corner_ang > SHARP_CORNER_RAD and dist_to_vertex < CORNER_TRIGGER_M:
            now_t = time.monotonic()
            target_yaw = math.atan2(d2y, d2x)
            turn_dir = 1 if cross > 0 else -1
            self._bb.update(
                'tracker.pivot_entry',
                **{
                    P_CORNER_MODE: True,
                    P_CORNER_PHASE: 0,
                    P_CORNER_PHASE_T: now_t,
                    P_CORNER_SEG_IDX: idx + 1,
                    P_CORNER_TARGET_YAW: target_yaw,
                    P_CORNER_TURN_DIR: turn_dir,
                }
            )
            self.log(
                f"[PIVOT] sharp corner at vertex #{idx+1}, "
                f"target_yaw={math.degrees(target_yaw):.0f}deg "
                f"dir={'L' if turn_dir > 0 else 'R'}"
            )

    def _run_pivot(self) -> bool:
        """执行 PIVOT 状态机。返回 True 表示 tick 结束（不需要继续执行纯追踪）。"""
        corner_mode = self._bb.get(P_CORNER_MODE, False)
        if not corner_mode:
            return False

        phase = self._bb.get(P_CORNER_PHASE, 0)
        phase_t = self._bb.get(P_CORNER_PHASE_T, 0.0)
        target_yaw = self._bb.get(P_CORNER_TARGET_YAW, 0.0)
        turn_dir = self._bb.get(P_CORNER_TURN_DIR, 0)
        seg_idx = self._bb.get(P_CORNER_SEG_IDX, 0)
        current_yaw = self._bb.get(P_POSE_YAW, 0.0)
        now_t = time.monotonic()

        yaw_err = normalize_angle(target_yaw - current_yaw)

        # Phase 0: stop + full steer (V1 L1265-1274)
        if phase == 0:
            steer = 30.0 * turn_dir
            self._publish_cmd(0.0, steer)
            self._bb.update('tracker.pivot',
                            **{P_CORNER_PHASE: 1, P_CORNER_PHASE_T: now_t})
            self.log("[PIVOT] phase 1: stop + max steer")
            return True  # tick ends

        # Phase 1: hold steer 0.5s (V1 L1276-1286)
        if phase == 1:
            if now_t - phase_t < 0.5:
                steer = 30.0 * turn_dir
                self._publish_cmd(0.0, steer)
                return True
            self._bb.update('tracker.pivot',
                            **{P_CORNER_PHASE: 2, P_CORNER_PHASE_T: now_t})
            self.log("[PIVOT] phase 2: creep forward")
            # Fall through to phase 2

        # Phase 2: creep until aligned (V1 L1288-1314)
        if abs(yaw_err) < CORNER_ALIGN_RAD:
            # Aligned -> exit pivot
            self._bb.update(
                'tracker.pivot_done',
                **{
                    P_CORNER_MODE: False,
                }
            )
            # 更新 current_target_idx 到下一个段
            self._bb.set('current_target_idx', seg_idx, 'tracker.pivot_done')
            self.log(
                f"[PIVOT] aligned (yaw_err={math.degrees(yaw_err):.1f}deg), "
                f"resume pure pursuit on seg {seg_idx}"
            )
            return False  # 退出 PIVOT，执行纯追踪

        # Still creeping
        steer = 30.0 * turn_dir
        self._publish_cmd(0.08, steer)
        # Re-arm arrival log guard
        self._bb.set('arrive_logged', False, 'tracker.pivot_creep')
        # Periodic log (V1 L1306-1313)
        self._log_count += 1
        if self._log_count % 20 == 0:
            self.log(
                f"[PIVOT] creeping yaw_err={math.degrees(yaw_err):.1f}deg "
                f"yaw={math.degrees(current_yaw):.1f}deg "
                f"target={math.degrees(target_yaw):.1f}deg"
            )
        return True  # tick ends

    # ── 停站检测 ────────────────────────────────────────────

    def _check_scan_stop(self) -> bool:
        """检测扫描停站。返回 True 表示触发停站、tick 结束（由 MissionTask 处理 wait）。"""
        scan_flags = self._bb.get(P_SCAN_FLAGS, [])
        pts = self._bb.get(P_PATH_POINTS, [])
        if not scan_flags or not pts:
            return False
        cx = self._bb.get(P_POSE_X, 0.0)
        cy = self._bb.get(P_POSE_Y, 0.0)
        aim_status = self._bb.get(P_AIM_STATUS, 0)

        for wp_idx in range(len(pts)):
            if wp_idx >= len(scan_flags) or not scan_flags[wp_idx]:
                continue
            wp = pts[wp_idx]
            d = math.hypot(wp['x'] - cx, wp['y'] - cy)
            if d <= self._scan_stop_radius:
                if aim_status == 2:
                    # Already locked, skip stop
                    if wp_idx < len(scan_flags):
                        new_flags = list(scan_flags)
                        new_flags[wp_idx] = False
                        self._bb.set(P_SCAN_FLAGS, new_flags, 'tracker.scan_skip')
                    self.log(f"scan point #{wp_idx} aim locked, skip stop")
                else:
                    # Signal to MissionTask: scan_stop_triggered at wp_idx
                    self._bb.update('tracker.scan_stop',
                                    scan_stop_triggered=wp_idx,
                                    scan_wait=True)
                    # Publish brake
                    self._publish_cmd(0.0, 0.0)
                    # Request scan
                    if self._scan_req_callback:
                        self._scan_req_callback(1)
                    self.log(f"scan stop at #{wp_idx}, request scan")
                return True  # 触发停站，tick 结束
        # 清除 scan_stop_triggered
        self._bb.set('scan_stop_triggered', -1, 'tracker.scan_check')
        return False

    def _check_shoot_stop(self) -> bool:
        """检测射击停站。返回 True 表示触发停站、tick 结束（由 MissionTask 处理 wait）。"""
        shoot_flags = self._bb.get(P_SHOOT_FLAGS, [])
        pts = self._bb.get(P_PATH_POINTS, [])
        if not shoot_flags or not pts:
            return False
        cx = self._bb.get(P_POSE_X, 0.0)
        cy = self._bb.get(P_POSE_Y, 0.0)

        for wp_idx in range(len(pts)):
            if wp_idx >= len(shoot_flags) or not shoot_flags[wp_idx]:
                continue
            wp = pts[wp_idx]
            d = math.hypot(wp['x'] - cx, wp['y'] - cy)
            if d <= self._shoot_stop_radius:
                # Signal to MissionTask
                self._bb.update('tracker.shoot_stop',
                                shoot_stop_triggered=wp_idx,
                                shoot_wait=True)
                # Publish brake
                self._publish_cmd(0.0, 0.0)
                self.log(f"shoot stop at #{wp_idx}")
                return True
        self._bb.set('shoot_stop_triggered', -1, 'tracker.shoot_check')
        return False

    # ── tick ─────────────────────────────────────────────────

    def tick(self) -> None:
        """主循迹逻辑（对应 V1 control_loop L1038-1372 的循迹部分）。"""
        # ── 没有里程计 → skip ────────────────────────────────
        if not self._bb.get(P_HAS_ODOM, False):
            return

        # ── 射击/扫描等待中 → 零速保活 ──────────────────────
        if self._bb.get(P_SHOOT_WAIT, False) or self._bb.get(P_SCAN_WAIT, False):
            self._publish_cmd(0.0, 0.0)
            return

        # ── 无路径 → 零速保活（V1 L1101-1105）───────────────
        if not self._bb.get(P_PATH_LOADED, False):
            self._publish_cmd(0.0, 0.0)
            return

        # ── 已到达终点 → 零速 ────────────────────────────────
        if self._bb.get(P_ARRIVED, False):
            self._publish_cmd(0.0, 0.0)
            return

        pts = self._bb.get(P_PATH_POINTS, [])
        if len(pts) < 2:
            self._publish_cmd(0.0, 0.0)
            return

        # ── 最近段投影（V1 L1107-1114）───────────────────────
        cx = self._bb.get(P_POSE_X, 0.0)
        cy = self._bb.get(P_POSE_Y, 0.0)
        cyaw = self._bb.get(P_POSE_YAW, 0.0)
        target_idx = self._bb.get('current_target_idx', 0)

        seg = find_nearest_segment_v1(pts, target_idx, cx, cy)
        if seg is not None:
            self._bb.set('current_target_idx', seg['idx'], 'tracker.tick')

        # ── PIVOT 入口检测（V1 L1229-1257）───────────────────
        self._check_pivot_entry(seg)

        # ── PIVOT 状态机运行（V1 L1260-1314）─────────────────
        if self._run_pivot():
            return

        # 重新获取 seg（PIVOT 可能改变了 current_target_idx）
        target_idx = self._bb.get('current_target_idx', 0)
        seg = find_nearest_segment_v1(pts, target_idx, cx, cy)

        # ── 瞄准点（V1 L1107-1114）───────────────────────────
        if seg is None:
            target_point = pts[-1] if pts else None
        else:
            target_point_la = find_lookahead_point(
                self._build_path() if self._path is None else self._path,
                # 需要从 seg 构建一个 Projection
                Projection(
                    seg_index=seg['idx'],
                    t=seg['t'],
                    x=seg['px'],
                    y=seg['py'],
                    s=0.0,  # 不需要 s
                    dist=seg['dist'],
                    seg_length=0.0,
                    seg_heading=0.0,
                    passed_end=(seg['t'] >= 1.0),
                ),
                self._lookahead_dist,
            ) if self._path is not None else pts[-1]
            if isinstance(target_point_la, LookaheadPoint):
                target_point = {'x': target_point_la.x, 'y': target_point_la.y}
            else:
                # V1 dict format (for path failure fallback)
                target_point = target_point_la if isinstance(target_point_la, dict) else pts[-1]

        if target_point is None:
            self._publish_cmd(0.0, 0.0)
            return

        # ── 扫描停站检测（V1 L1120-1139）─────────────────────
        scan_wait = self._bb.get(P_SCAN_WAIT, False)
        if not scan_wait:
            if self._check_scan_stop():
                return

        # ── 射击停站检测（V1 L1141-1153）─────────────────────
        shoot_wait = self._bb.get(P_SHOOT_WAIT, False)
        if not shoot_wait:
            if self._check_shoot_stop():
                return

        # ── 终点到达检测（V1 L1155-1215）─────────────────────
        # （由 MissionTask 完全处理，此处只检查是否已到达）
        final_point = pts[-1]
        dist_to_final = math.hypot(final_point['x'] - cx, final_point['y'] - cy)
        arrived = self._bb.get(P_ARRIVED, False)

        if dist_to_final < self._arrival_threshold or arrived:
            self._publish_cmd(0.0, 0.0)
            # V1 中的 arrive_logged 和 snap 由 MissionTask 处理
            return

        # ── 纯追踪控制（V1 L1316-1356）───────────────────────
        cmd = compute_command(
            pose_x=cx, pose_y=cy, pose_yaw=cyaw,
            target_x=target_point['x'], target_y=target_point['y'],
            wheelbase=self._wheelbase,
            gain=1.0,  # V1 gain（与 V1 L1013-1014 一致）
            speed=self._linear_speed,
            dist_to_final=dist_to_final,
            dist_to_vertex=(
                math.hypot(pts[seg['idx'] + 1]['x'] - cx,
                           pts[seg['idx'] + 1]['y'] - cy)
                if seg and seg['idx'] + 1 < len(pts) else None
            ) if seg else None,
            corner_angle_deg=(
                abs(math.degrees(
                    math.atan2(
                        (pts[seg['idx'] + 2]['x'] - pts[seg['idx'] + 1]['x'])
                        * (pts[seg['idx'] + 1]['y'] - pts[seg['idx']]['y'])
                        - (pts[seg['idx'] + 2]['y'] - pts[seg['idx'] + 1]['y'])
                        * (pts[seg['idx'] + 1]['x'] - pts[seg['idx']]['x']),
                        (pts[seg['idx'] + 1]['x'] - pts[seg['idx']]['x'])
                        * (pts[seg['idx'] + 2]['x'] - pts[seg['idx'] + 1]['x'])
                        + (pts[seg['idx'] + 1]['y'] - pts[seg['idx']]['y'])
                        * (pts[seg['idx'] + 2]['y'] - pts[seg['idx'] + 1]['y']),
                    )
                )) if seg and seg['idx'] + 2 < len(pts) else 0.0,
            last_steer_deg=self._last_steer_deg,
            dt=1.0 / self._rate_hz,
        )

        self._last_steer_deg = cmd.steer_deg

        # ── 发布指令（V1 L1353-1356）─────────────────────────
        self._publish_cmd(cmd.linear_x, cmd.steer_deg)

        # ── 周期性日志（V1 L1364-1372）───────────────────────
        self._log_count += 1
        if self._log_count % 40 == 0:  # 每 ~0.8s
            seg_disp = str(seg['idx']) if seg else '-'
            self.log(
                f"pos=({cx:.2f}m, {cy:.2f}m) "
                f"seg={seg_disp}/{len(pts)} "
                f"dist={dist_to_final:.2f}m "
                f"v={cmd.linear_x:.2f} steer={cmd.steer_deg:.1f}deg "
                f"yaw={cyaw:.2f}rad"
            )
