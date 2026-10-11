"""tasks/mission.py — 任务管理器：航点推进、到点判定、射击/扫描停站、终点到达、兜底。

从 V1 path_executor.py control_loop 拆出**任务调度**部分：
  - shoot_wait 射击等待逻辑（V1 L1047-1072）
  - scan_wait 扫描等待逻辑（V1 L1074-1096）
  - 停站触发（enter_shoot_stop L784-827 / enter_scan_stop L749-763）
  - 终点到达 + 锚定（V1 L1155-1215）
  - 拐角绕圈兜底（V2 新增）

设计模式：TaskThread 子类 + 隐式阶段机（IDLE→DRIVE→APPROACH→STOP→FINISH）。
"""

import math
import time
from typing import Any, Dict, List, Optional, Tuple

from .base import TaskThread
from ..core.blackboard import (
    Blackboard,
    P_POSE_X, P_POSE_Y, P_POSE_YAW,
    P_PATH_POINTS, P_PATH_LOADED, P_ARRIVED, P_ARRIVE_LOGGED,
    P_SHOOT_FLAGS, P_SHOOT_WAIT, P_SHOOT_WAIT_START,
    P_SHOOT_COUNT_AT_ARRIVAL, P_SHOOT_COUNT,
    P_SCAN_FLAGS, P_SCAN_WAIT, P_SCAN_WAIT_START,
    P_SCAN_SEEN_SCANNING, P_AIM_STATUS,
    P_CORNER_MODE,
    P_SNAP_X_OFF, P_SNAP_Y_OFF, P_SNAP_YAW_OFF,
)
from ..core.geometry import normalize_angle, block_to_meters, euler_from_quaternion
from ..pursuit.path import find_nearest_segment_v1


# 默认参数（V1 L239-278 一致）
SHOOT_COUNT_TARGET = 10
SHOOT_WAIT_TIMEOUT = 150.0
SHOOT_STOP_RADIUS = 0.17
SCAN_WAIT_TIMEOUT = 8.0
SCAN_STOP_RADIUS = 0.20
ARRIVAL_THRESHOLD = 0.157

# 拐角绕圈兜底参数
STALL_TIMEOUT_S = 12.0
STALL_ROTATION_THRESHOLD_DEG = 120.0


class MissionTask(TaskThread):
    """任务管理器：航点推进、到点判定、射击/扫描停站、终点到达、兜底。

    对应 V1 control_loop 中的任务调度部分：
      - shoot_wait 射击等待（L1047-1072）
      - scan_wait 扫描等待（L1074-1096）
      - enter_shoot_stop / enter_scan_stop
      - 终点到达 + 锚定（L1155-1215）
      - 停站后恢复前进

    注：纯循迹逻辑（投影/前视/舵角/PIVOT）在 TrackerTask 中。
    """

    def __init__(
        self,
        name: str = 'mission',
        rate_hz: float = 10.0,
        blackboard: Optional[Blackboard] = None,
        logger: Optional[Any] = None,
        # 参数
        shoot_count_target: int = SHOOT_COUNT_TARGET,
        shoot_wait_timeout: float = SHOOT_WAIT_TIMEOUT,
        shoot_stop_radius: float = SHOOT_STOP_RADIUS,
        scan_wait_timeout: float = SCAN_WAIT_TIMEOUT,
        scan_stop_radius: float = SCAN_STOP_RADIUS,
        arrival_threshold: float = ARRIVAL_THRESHOLD,
        stall_timeout_s: float = STALL_TIMEOUT_S,
        stall_rotation_threshold_deg: float = STALL_ROTATION_THRESHOLD_DEG,
        mission_scan_mode: str = 'continuous',
    ):
        super().__init__(name=name, rate_hz=rate_hz, logger=logger)
        if blackboard is None:
            blackboard = Blackboard()
        self._bb: Blackboard = blackboard
        self._shoot_count_target: int = shoot_count_target
        self._shoot_wait_timeout: float = shoot_wait_timeout
        self._shoot_stop_radius: float = shoot_stop_radius
        self._scan_wait_timeout: float = scan_wait_timeout
        self._scan_stop_radius: float = scan_stop_radius
        self._arrival_threshold: float = arrival_threshold
        self._stall_timeout_s: float = stall_timeout_s
        self._stall_rotation_threshold_deg: float = stall_rotation_threshold_deg
        self._mission_scan_mode: str = mission_scan_mode

        # 输出回调
        self._stop_signal_callback: Optional[callable] = None
        self._scan_req_callback: Optional[callable] = None

        # 兜底检测状态
        self._stall_start_time: float = 0.0
        self._stall_last_position: Optional[Tuple[float, float]] = None
        self._stall_last_yaw: float = 0.0
        self._stall_accumulated_rotation: float = 0.0  # deg

        # 上一拍位姿（供到点/停滞检测）
        self._prev_pose: Optional[Tuple[float, float, float]] = None

    def set_stop_signal_callback(self, cb: callable) -> None:
        """设置停站信号发布回调。"""
        self._stop_signal_callback = cb

    def set_scan_req_callback(self, cb: callable) -> None:
        """设置扫描请求发布回调。"""
        self._scan_req_callback = cb

    # ── 射击等待（V1 L1047-1072）──────────────────────────

    def _run_shoot_wait(self) -> bool:
        """射击等待逻辑。返回 True 表示仍在等待中。"""
        shoot_wait = self._bb.get(P_SHOOT_WAIT, False)
        if not shoot_wait:
            return False

        now_t = time.monotonic()
        shoot_count = self._bb.get(P_SHOOT_COUNT, 0)
        count_at_arrival = self._bb.get(P_SHOOT_COUNT_AT_ARRIVAL, 0)
        wait_start = self._bb.get(P_SHOOT_WAIT_START, 0.0)

        done = (shoot_count - count_at_arrival >= self._shoot_count_target)
        timed_out = (now_t - wait_start > self._shoot_wait_timeout)

        if done or timed_out:
            # stop_signal=0 -> allow route to continue
            if self._stop_signal_callback:
                self._stop_signal_callback(0)
            self._bb.set(P_SHOOT_WAIT, False, 'mission.shoot_done')
            if timed_out and not done:
                self.log("Shoot wait timed out, continuing route", level='warn')
            else:
                self.log(f"{self._shoot_count_target} shots fired, resuming route")
        # else: still waiting (TrackerTask publishes zero cmd)
        return self._bb.get(P_SHOOT_WAIT, False)

    # ── 扫描等待（V1 L1074-1096）──────────────────────────

    def _run_scan_wait(self) -> bool:
        """扫描等待逻辑。返回 True 表示仍在等待中。"""
        scan_wait = self._bb.get(P_SCAN_WAIT, False)
        if not scan_wait:
            return False

        now_t = time.monotonic()
        wait_start = self._bb.get(P_SCAN_WAIT_START, 0.0)
        aim_status = self._bb.get(P_AIM_STATUS, 0)
        seen_scanning = self._bb.get(P_SCAN_SEEN_SCANNING, False)

        elapsed = now_t - wait_start

        # Track scan status
        if aim_status == 1:
            self._bb.set(P_SCAN_SEEN_SCANNING, True, 'mission.scan_seen')

        if aim_status == 2:
            # Target locked -> resume
            self._bb.set(P_SCAN_WAIT, False, 'mission.scan_locked')
            self.log("Gimbal locked target, resume tracking", level='info')
        elif seen_scanning and aim_status == 0:
            # Scanned but no target found -> resume
            self._bb.set(P_SCAN_WAIT, False, 'mission.scan_done')
            self.log("Scan complete, no target found, continue", level='info')
        elif elapsed > self._scan_wait_timeout:
            # Timeout -> resume regardless
            self._bb.set(P_SCAN_WAIT, False, 'mission.scan_timeout')
            self.log(f"Scan wait timeout ({self._scan_wait_timeout:.0f}s), "
                     f"force continue", level='warn')
        # else: still waiting

        return self._bb.get(P_SCAN_WAIT, False)

    # ── 终点到达（V1 L1155-1215）──────────────────────────

    def _check_end_arrival(self) -> bool:
        """检测终点到达并执行锚定。返回 True 表示已到达。"""
        arrived = self._bb.get(P_ARRIVED, False)
        if arrived:
            return True

        pts = self._bb.get(P_PATH_POINTS, [])
        if not pts:
            return False

        cx = self._bb.get(P_POSE_X, 0.0)
        cy = self._bb.get(P_POSE_Y, 0.0)

        final_point = pts[-1]
        dx = final_point['x'] - cx
        dy = final_point['y'] - cy
        dist_to_final = math.hypot(dx, dy)

        if dist_to_final < self._arrival_threshold:
            self._bb.set(P_ARRIVED, True, 'mission.arrival')

            # One-shot logging + snap
            arrive_logged = self._bb.get(P_ARRIVE_LOGGED, False)
            if not arrive_logged:
                self._bb.set(P_ARRIVE_LOGGED, True, 'mission.arrival')

                target_yaw = euler_from_quaternion(
                    final_point.get('qx', 0.0),
                    final_point.get('qy', 0.0),
                    final_point.get('qz', 0.0),
                    final_point.get('qw', 1.0),
                )
                drift_x = cx - final_point['x']
                drift_y = cy - final_point['y']
                yaw_drift_deg = math.degrees(
                    normalize_angle(self._bb.get(P_POSE_YAW, 0.0) - target_yaw)
                )

                # Snap (V1 L1189-1196)
                cur_snap_x = self._bb.get(P_SNAP_X_OFF, 0.0)
                cur_snap_y = self._bb.get(P_SNAP_Y_OFF, 0.0)
                cur_snap_yaw = self._bb.get(P_SNAP_YAW_OFF, 0.0)

                self._bb.update(
                    'mission.anchor',
                    **{
                        P_SNAP_X_OFF: cur_snap_x - drift_x,
                        P_SNAP_Y_OFF: cur_snap_y - drift_y,
                        P_SNAP_YAW_OFF: normalize_angle(
                            cur_snap_yaw - math.radians(yaw_drift_deg)),
                        P_POSE_X: final_point['x'],
                        P_POSE_Y: final_point['y'],
                        P_POSE_YAW: target_yaw,
                    }
                )

                self.log(
                    f"ARRIVED! dist={dist_to_final:.3f}m, "
                    f"drift=({drift_x:+.2f}m, {drift_y:+.2f}m), "
                    f"yaw_drift={yaw_drift_deg:+.1f}deg, stopped"
                )

                # stop_signal=1 -> allow shooting at end
                if self._stop_signal_callback:
                    self._stop_signal_callback(1)
                self.log("Published stop_signal=1 (enable gimbal+shooting)")

                # Scan request at end
                if self._scan_req_callback:
                    self._scan_req_callback(1)
                self.log("Scan request at end")

            return True
        return False

    # ── 拐角绕圈兜底（V2 新增，防止死循环）───────────────

    def _check_stall_and_skip(self) -> bool:
        """检测拐角绕圈停滞，兜底跳点。返回 True 表示已跳过。"""
        pts = self._bb.get(P_PATH_POINTS, [])
        if not pts:
            self._stall_start_time = 0.0
            return False

        cx = self._bb.get(P_POSE_X, 0.0)
        cy = self._bb.get(P_POSE_Y, 0.0)
        cyaw = self._bb.get(P_POSE_YAW, 0.0)
        corner_mode = self._bb.get(P_CORNER_MODE, False)

        # 在 PIVOT 中不判定兜底
        if corner_mode:
            self._stall_start_time = 0.0
            return False

        # 获取当前目标索引
        target_idx = self._bb.get('current_target_idx', 0)
        if target_idx >= len(pts):
            return False

        current_pos = (cx, cy)

        if self._stall_start_time == 0.0:
            # 首次检测
            self._stall_start_time = time.monotonic()
            self._stall_last_position = current_pos
            self._stall_last_yaw = cyaw
            self._stall_accumulated_rotation = 0.0
            return False

        # 计算位移和累计旋转
        if self._stall_last_position is not None:
            dx = cx - self._stall_last_position[0]
            dy = cy - self._stall_last_position[1]
            movement = math.hypot(dx, dy)
            yaw_change = abs(math.degrees(normalize_angle(cyaw - self._stall_last_yaw)))
            self._stall_accumulated_rotation += yaw_change
            self._stall_last_yaw = cyaw
            self._stall_last_position = current_pos

            # 如果移动了足够距离，重置计时（说明车在正常行进）
            if movement > 0.05:
                self._stall_start_time = time.monotonic()
                self._stall_accumulated_rotation = 0.0
                return False

        elapsed = time.monotonic() - self._stall_start_time

        # 判定：停滞超过阈值
        if elapsed > self._stall_timeout_s:
            # 跳点：将 current_target_idx 推进到下个段
            new_idx = min(target_idx + 1, len(pts) - 2)
            self._bb.set('current_target_idx', new_idx, 'mission.stall_skip')
            rotation_msg = (f" / accumulated rot "
                            f"{self._stall_accumulated_rotation:.0f}deg"
                            ) if self._stall_accumulated_rotation > 0 else ""
            self.log(
                f"Stall skip: waypoint #{target_idx} stalled "
                f"{elapsed:.1f}s{rotation_msg}, "
                f"skip to seg {new_idx}"
            )
            self._stall_start_time = 0.0
            self._stall_accumulated_rotation = 0.0
            return True

        return False

    # ── 停站逻辑 ─────────────────────────────────────────

    def _handle_shoot_stop_trigger(self) -> None:
        """处理射击停站触发（由 TrackerTask 或直接检测触发）。

        设置 shoot_wait = True，MissionTask._run_shoot_wait() 会管理等待逻辑。
        """
        wp_idx = self._bb.get('shoot_stop_triggered', -1)
        if wp_idx < 0:
            return

        # 清除触发标记（一次性，防止下次 tick 重触发）
        self._bb.set('shoot_stop_triggered', -1, 'mission.shoot_stop')

        # 清除射击标记防止重触发
        shoot_flags = self._bb.get(P_SHOOT_FLAGS, [])
        if shoot_flags and wp_idx < len(shoot_flags) and shoot_flags[wp_idx]:
            new_flags = list(shoot_flags)
            new_flags[wp_idx] = False
            self._bb.set(P_SHOOT_FLAGS, new_flags, 'mission.shoot_stop')

        now_t = time.monotonic()
        self._bb.set(P_SHOOT_WAIT, True, 'mission.shoot_stop')  # ★ 关键：设置等待标志
        self._bb.set(P_SHOOT_WAIT_START, now_t, 'mission.shoot_stop')
        self._bb.set(P_SHOOT_COUNT_AT_ARRIVAL,
                     self._bb.get(P_SHOOT_COUNT, 0), 'mission.shoot_stop')

        # Stop signal = 1 (allow shooting)
        if self._stop_signal_callback:
            self._stop_signal_callback(1)
        self.log(f"Shoot stop at wp#{wp_idx}: waiting for "
                 f"{self._shoot_count_target} shots")

    def _handle_scan_stop_trigger(self) -> None:
        """处理扫描停站触发。

        设置 scan_wait = True，MissionTask._run_scan_wait() 会管理等待逻辑。
        """
        wp_idx = self._bb.get('scan_stop_triggered', -1)
        if wp_idx < 0:
            return

        # 清除触发标记（一次性）
        self._bb.set('scan_stop_triggered', -1, 'mission.scan_stop')

        # 清除扫描标记防止重触发
        scan_flags = self._bb.get(P_SCAN_FLAGS, [])
        if scan_flags and wp_idx < len(scan_flags) and scan_flags[wp_idx]:
            new_flags = list(scan_flags)
            new_flags[wp_idx] = False
            self._bb.set(P_SCAN_FLAGS, new_flags, 'mission.scan_stop')

        now_t = time.monotonic()
        self._bb.set(P_SCAN_WAIT, True, 'mission.scan_stop')  # ★ 关键：设置等待标志
        self._bb.set(P_SCAN_WAIT_START, now_t, 'mission.scan_stop')
        self._bb.set(P_SCAN_SEEN_SCANNING, False, 'mission.scan_stop')
        # Scan request = 1
        if self._scan_req_callback:
            self._scan_req_callback(1)
        self.log(f"Scan stop at wp#{wp_idx}, request scan")

    # ── tick ─────────────────────────────────────────────────

    def tick(self) -> None:
        """主调度逻辑（10Hz，处理停站/终点/兜底）。
        
        与 V1 control_loop 的调度层对应：
          - 射击等待（L1047-1072）
          - 扫描等待（L1074-1096）
          - 终点到达（L1155-1215）
          - 拐角兜底（V2 新增）
          - 停站触发处理
        """
        # ── 处理射击停站触发 ──────────────────────────────
        if self._bb.get('shoot_stop_triggered', -1) >= 0:
            self._handle_shoot_stop_trigger()

        # ── 处理扫描停站触发 ──────────────────────────────
        if self._bb.get('scan_stop_triggered', -1) >= 0:
            self._handle_scan_stop_trigger()

        # ── 射击等待 ──────────────────────────────────────
        self._run_shoot_wait()

        # ── 扫描等待 ──────────────────────────────────────
        self._run_scan_wait()

        # ── 有路径时才继续 ──────────────────────────────────
        if not self._bb.get(P_PATH_LOADED, False):
            self._stall_start_time = 0.0
            return

        pts = self._bb.get(P_PATH_POINTS, [])
        if len(pts) < 2:
            self._stall_start_time = 0.0
            return

        # ── 终点到达检测 ──────────────────────────────────
        self._check_end_arrival()

        # ── 拐角绕圈兜底检测 ──────────────────────────────
        self._check_stall_and_skip()

        # ── 到达后清空停滞状态 ────────────────────────────
        if self._bb.get(P_ARRIVED, False):
            self._stall_start_time = 0.0

        # ── 保存上一拍位姿 ────────────────────────────────
        cx = self._bb.get(P_POSE_X, 0.0)
        cy = self._bb.get(P_POSE_Y, 0.0)
        cyaw = self._bb.get(P_POSE_YAW, 0.0)
        self._prev_pose = (cx, cy, cyaw)
