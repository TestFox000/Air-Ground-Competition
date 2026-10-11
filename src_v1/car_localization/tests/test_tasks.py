"""测试：tasks/ 模块 vs V1 path_executor 语义对齐。

测试策略：
  - TestTaskThread: 基类验证（启动/停止/异常隔离/频率）
  - TestLocalization: 定位任务（里程计→信念位姿，锚定）
  - TestMission: 任务管理器（射击等待、扫描等待、终点、兜底）
  - TestSafety: 安全任务（越界/停滞/冷启动）

本测试不依赖 rclpy / ROS2。所有测试 exit 0 = 全过。
"""

import math
import sys
import os
import time
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'car_localization'))

from car_localization.tasks.base import TaskThread, TaskState, TaskHealth
from car_localization.tasks.localization import LocalizationTask
from car_localization.tasks.safety import SafetyTask
from car_localization.core.blackboard import (
    Blackboard,
    P_POSE_X, P_POSE_Y, P_POSE_YAW, P_HAS_ODOM,
    P_SNAP_X_OFF, P_SNAP_Y_OFF, P_SNAP_YAW_OFF,
    P_RAW_X, P_RAW_Y, P_RAW_MAP_YAW,
    P_PATH_POINTS, P_PATH_LOADED, P_ARRIVED, P_ARRIVE_LOGGED,
    P_SHOOT_FLAGS, P_SHOOT_WAIT, P_SHOOT_WAIT_START,
    P_SHOOT_COUNT_AT_ARRIVAL, P_SHOOT_COUNT,
    P_SCAN_FLAGS, P_SCAN_WAIT, P_SCAN_WAIT_START,
    P_SCAN_SEEN_SCANNING, P_AIM_STATUS,
    P_CORNER_MODE, P_CORNER_PHASE, P_CORNER_PHASE_T,
    P_CORNER_TARGET_YAW, P_CORNER_TURN_DIR, P_CORNER_SEG_IDX,
    P_SNAP_X_OFF, P_SNAP_Y_OFF, P_SNAP_YAW_OFF,
)
from car_localization.core.geometry import (
    normalize_angle, block_to_meters,
    euler_from_quaternion, Pose,
)

# ═══════════════════════════════════════════════════════════════
# 设置
# ═══════════════════════════════════════════════════════════════

PASS = 0
FAIL = 0


def check(cond, msg):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  [OK] {msg}")
    else:
        FAIL += 1
        print(f"  [FAIL] {msg}")


def assert_almost_equal(a, b, tol=1e-9, msg=''):
    check(abs(a - b) < tol, msg or f"{a} ≈ {b}")


def make_route_pts(*blocks):
    """从格名列表构造 V1 格式 path_points。"""
    pts = []
    for b in blocks:
        x, y = block_to_meters(b.upper().rstrip('*'))
        pts.append({'x': x, 'y': y, 'z': 0.0,
                    'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})
    return pts


# ═══════════════════════════════════════════════════════════════
# 测试 1: TaskThread 基类
# ═══════════════════════════════════════════════════════════════

def test_task_thread():
    print("\n--- Test 1: TaskThread base class ---")

    # 1.1 基础启动停止
    class CounterTask(TaskThread):
        def __init__(self):
            super().__init__('counter', rate_hz=100)
            self.tick_count = 0

        def tick(self):
            self.tick_count += 1

    t = CounterTask()
    check(t._task_name == 'counter', "task name")
    check(t._rate_hz == 100, "rate")
    check(t._period == 0.01, "period")

    t.start()
    time.sleep(0.05)  # allow some ticks
    t.stop(timeout=1.0)

    check(t.tick_count >= 1, f"tick count >= 1 (got {t.tick_count})")
    check(t.health().state == TaskState.STOPPED, "state == STOPPED")
    check(t.health().ticks >= 1, f"health ticks >= 1 (got {t.health().ticks})")
    check(t.health().errors == 0, "no errors")

    # 1.2 异常隔离
    class ErrorTask(TaskThread):
        def __init__(self, fail_after=2):
            super().__init__('error_test', rate_hz=100)
            self._count = 0
            self._fail_after = fail_after

        def tick(self):
            self._count += 1
            if self._count > self._fail_after:
                raise ValueError("testing error")

    e = ErrorTask()
    e.MAX_CONSECUTIVE_ERRORS = 5  # 连续 5 次才 FAULT
    e.start()
    time.sleep(0.15)
    e.stop(timeout=1.0)

    check(e.health().errors >= 1, f"errors >= 1 (got {e.health().errors})")
    # 可能还没到 FAULT（因为 MAX_CONSECUTIVE_ERRORS=5）

    # 1.3 _stopped 是 Event 不遮蔽 Thread._stop
    check(hasattr(t, '_stopped'), "_stopped exists (Event)")
    from threading import Event as _ThreadEvent
    check(isinstance(t._stopped, _ThreadEvent), "_stopped is Event type")

    # 1.4 状态转换
    check(t.health().state == TaskState.STOPPED, "stopped state")
    check(e.health().name == 'error_test', "health name")

    # 1.5 is_faulted / is_running
    check(not e.is_faulted() or e.health().state == TaskState.FAULT,
          "is_faulted works")


# ═══════════════════════════════════════════════════════════════
# 测试 2: LocalizationTask
# ═══════════════════════════════════════════════════════════════

def test_localization():
    print("\n--- Test 2: LocalizationTask ---")

    bb = Blackboard()
    loc = LocalizationTask(blackboard=bb)

    # 2.1 初始状态无里程计
    check(not loc.has_odom(), "no odom initially")
    pose = loc.get_pose()
    check(pose.x == 0.0, "pose.x == 0")
    check(pose.y == 0.0, "pose.y == 0")

    # 2.2 注入里程计后 tick 更新信念位姿
    # V1: raw_x = msg.pose.pose.position.x * BLOCK_SIZE
    # 我们直接在米制坐标注入（等效于已乘 BLOCK_SIZE）
    # N10 中心 = (1.5 * 0.35, 0.5 * 0.35) = (0.525, 0.175)
    # 这里注入 block 单位 (1.5, 0.5) * 0.35 = (0.525, 0.175)
    loc.feed_odom(0.525, 0.175, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    loc.tick()

    check(loc.has_odom(), "has odom after tick")
    pose = loc.get_pose()
    check(abs(pose.x - 0.525) < 1e-9, f"pose.x ≈ 0.525 (got {pose.x})")
    check(abs(pose.y - 0.175) < 1e-9, f"pose.y ≈ 0.175 (got {pose.y})")
    # yaw: raw_yaw = euler_from_quaternion(0,0,0,1) = 0
    # raw_map_yaw = normalize_angle(0 + pi/2) = pi/2
    # current_yaw = pi/2 + 0 = pi/2
    check(abs(pose.yaw - math.pi / 2) < 1e-9,
          f"pose.yaw ≈ pi/2 (got {pose.yaw})")

    # 2.3 snap offset: 设置 snap 后保持
    bb.set(P_SNAP_X_OFF, 0.1, 'test')
    bb.set(P_SNAP_Y_OFF, -0.05, 'test')
    loc.feed_odom(0.525, 0.175, 0.0, 0.0, 0.0, 1.0)
    loc.tick()
    pose = loc.get_pose()
    check(abs(pose.x - 0.625) < 1e-9, f"pose.x with snap ≈ 0.625 (got {pose.x})")
    check(abs(pose.y - 0.125) < 1e-9, f"pose.y with snap ≈ 0.125 (got {pose.y})")

    # 2.4 anchor_to_point: 锚定到 N9 中心
    # N9: col N(13) = y = (13.5-13)*0.35 = 0.175, row 9 = x = (10.5-8)*0.35 = 0.875
    n9_x, n9_y = block_to_meters('N9')
    loc.anchor_to_point(n9_x, n9_y, 0.0)
    pose = loc.get_pose()
    check(abs(pose.x - n9_x) < 1e-9, f"anchor x ≈ {n9_x} (got {pose.x})")
    check(abs(pose.y - n9_y) < 1e-9, f"anchor y ≈ {n9_y} (got {pose.y})")
    check(abs(pose.yaw - 0.0) < 1e-9, f"anchor yaw ≈ 0 (got {pose.yaw})")

    # 2.5 raw_x/y 写黑盒
    loc.feed_odom(1.0, 2.0, 0.0, 0.0, 0.7071, 0.7071)
    loc.tick()
    raw_x = bb.get(P_RAW_X, -1)
    raw_y = bb.get(P_RAW_Y, -1)
    check(abs(raw_x - 1.0) < 1e-9, f"raw_x ≈ 1.0 (got {raw_x})")
    check(abs(raw_y - 2.0) < 1e-9, f"raw_y ≈ 2.0 (got {raw_y})")

    # 2.6 get_pose_dict
    pd = loc.get_pose_dict()
    check('x' in pd and 'y' in pd and 'yaw' in pd, "pose dict keys")

    loc.stop()
    print("  localization test done")


# ═══════════════════════════════════════════════════════════════
# 测试 3: MissionTask 射击等待
# ═══════════════════════════════════════════════════════════════

def test_mission_shoot_wait():
    print("\n--- Test 3: MissionTask shoot wait ---")

    bb = Blackboard()
    from car_localization.tasks.mission import MissionTask
    mission = MissionTask(blackboard=bb)

    # 3.1 初始状态：无等待
    check(not bb.get(P_SHOOT_WAIT, False), "no shoot wait initially")
    check(not bb.get(P_SCAN_WAIT, False), "no scan wait initially")

    # 3.2 模拟射击停站触发
    bb.set('shoot_stop_triggered', 2, 'test')
    mission.tick()
    check(bb.get(P_SHOOT_WAIT, False), "shoot_wait = True after trigger")
    wait_start = bb.get(P_SHOOT_WAIT_START, 0.0)
    check(wait_start > 0, "shoot_wait_start set")

    # 3.3 还差一枪
    bb.set(P_SHOOT_COUNT, 5, 'test')
    bb.set(P_SHOOT_COUNT_AT_ARRIVAL, 0, 'test')
    mission.tick()
    check(bb.get(P_SHOOT_WAIT, False), "still waiting (not enough shots)")

    # 3.4 打够了
    bb.set(P_SHOOT_COUNT, 10, 'test')
    mission.tick()
    check(not bb.get(P_SHOOT_WAIT, False), "shoot wait cleared after target")

    # 3.5 超时测试
    bb.set('shoot_stop_triggered', 3, 'test')
    mission.tick()
    check(bb.get(P_SHOOT_WAIT, False), "triggered again")

    # 设过去的时间戳 -> 超时
    import time
    old_time = time.monotonic() - 200.0
    bb.set(P_SHOOT_WAIT_START, old_time, 'test')
    bb.set(P_SHOOT_COUNT, 0, 'test')
    bb.set(P_SHOOT_COUNT_AT_ARRIVAL, 0, 'test')
    mission.tick()
    check(not bb.get(P_SHOOT_WAIT, False), "shoot wait timed out")

    # 3.6 shoot_flags 清空
    bb.set(P_SHOOT_FLAGS, [False, True, False], 'test')
    bb.set('shoot_stop_triggered', 1, 'test')
    mission.tick()
    flags = bb.get(P_SHOOT_FLAGS, [])
    check(not flags[1], f"shoot_flags[1] cleared after trigger (got {flags[1]})")

    mission.stop()
    print("  shoot wait test done")


# ═══════════════════════════════════════════════════════════════
# 测试 4: MissionTask 扫描等待
# ═══════════════════════════════════════════════════════════════

def test_mission_scan_wait():
    print("\n--- Test 4: MissionTask scan wait ---")

    bb = Blackboard()
    from car_localization.tasks.mission import MissionTask
    mission = MissionTask(blackboard=bb)

    # 4.1 触发扫描停站
    bb.set('scan_stop_triggered', 1, 'test')
    mission.tick()
    check(bb.get(P_SCAN_WAIT, False), "scan_wait = True after trigger")

    # 4.2 云台锁定 -> 退出
    bb.set(P_AIM_STATUS, 2, 'test')
    mission.tick()
    check(not bb.get(P_SCAN_WAIT, False), "scan wait cleared on aim=2")

    # 4.3 再次触发 -> IDLE 但 seen_scanning=True -> 退出
    bb.set('scan_stop_triggered', 2, 'test')
    bb.set(P_AIM_STATUS, 0, 'test')
    mission.tick()
    check(bb.get(P_SCAN_WAIT, False), "scan wait after retrigger")
    bb.set(P_AIM_STATUS, 1, 'test')
    mission.tick()
    check(bb.get(P_SCAN_WAIT, False), "still waiting (scanning)")
    bb.set(P_AIM_STATUS, 0, 'test')
    mission.tick()
    check(not bb.get(P_SCAN_WAIT, False),
          f"scan cleared after scan_seen->idle (got {bb.get(P_SCAN_WAIT)})")

    # 4.4 超时
    bb.set('scan_stop_triggered', 3, 'test')
    mission.tick()
    check(bb.get(P_SCAN_WAIT, False), "scan wait after 3rd trigger")
    old_time = time.monotonic() - 20.0
    bb.set(P_SCAN_WAIT_START, old_time, 'test')
    mission.tick()
    check(not bb.get(P_SCAN_WAIT, False), "scan wait timeout")

    mission.stop()
    print("  scan wait test done")


# ═══════════════════════════════════════════════════════════════
# 测试 5: MissionTask 终点到达
# ═══════════════════════════════════════════════════════════════

def test_mission_arrival():
    print("\n--- Test 5: MissionTask end arrival ---")

    bb = Blackboard()
    from car_localization.tasks.mission import MissionTask
    mission = MissionTask(blackboard=bb)

    # 5.1 设置路线：N10 -> N9
    pts = make_route_pts('N10', 'N9')
    bb.set(P_PATH_POINTS, pts, 'test')
    bb.set(P_PATH_LOADED, True, 'test')
    # 放在 N9 中心
    n9_x, n9_y = block_to_meters('N9')
    bb.set(P_POSE_X, n9_x, 'test')
    bb.set(P_POSE_Y, n9_y, 'test')

    # 5.2 tick -> 到达
    mission.tick()
    check(bb.get(P_ARRIVED, False), "arrived = True")
    check(bb.get(P_ARRIVE_LOGGED, False), "arrive_logged = True")

    # 5.3 snap offset 已更新
    snap_x = bb.get(P_SNAP_X_OFF, 999)
    snap_y = bb.get(P_SNAP_Y_OFF, 999)
    # 具体值取决于之前的 snap 状态，但一定是 float
    check(isinstance(snap_x, float), "snap_x_off updated")
    check(isinstance(snap_y, float), "snap_y_off updated")

    mission.stop()
    print("  arrival test done")


# ═══════════════════════════════════════════════════════════════
# 测试 6: MissionTask 兜底跳点（stall skip）
# ═══════════════════════════════════════════════════════════════

def test_mission_stall_skip():
    print("\n--- Test 6: MissionTask stall skip ---")

    bb = Blackboard()
    from car_localization.tasks.mission import MissionTask
    # 超短超时便于测试
    mission = MissionTask(blackboard=bb, stall_timeout_s=0.001)

    # 建立路线
    pts = make_route_pts('N10', 'N9', 'N8', 'N7')
    bb.set(P_PATH_POINTS, pts, 'test')
    bb.set(P_PATH_LOADED, True, 'test')
    bb.set(P_POSE_X, 0.525, 'test')
    bb.set(P_POSE_Y, 0.175, 'test')
    bb.set('current_target_idx', 0, 'test')
    bb.set(P_CORNER_MODE, False, 'test')

    # 连续多次 tick 不动
    for i in range(30):
        mission.tick()
        # 第四次 tick 之后把 _stall_start_time 推到过去
        if i == 3:
            mission._stall_start_time = time.monotonic() - 100.0

    # 检查 current_target_idx 是否被推进
    idx = bb.get('current_target_idx', -1)
    check(idx >= 1, f"stall skip advanced target_idx (got {idx})")

    # 在 PIVOT 模式下不应触发兜底
    bb.set(P_CORNER_MODE, True, 'test')
    bb.set('current_target_idx', 0, 'test')
    mission.tick()  # 不会跳
    idx2 = bb.get('current_target_idx', -1)
    check(idx2 == 0, f"no stall skip in PIVOT mode (idx={idx2})")

    mission.stop()
    print("  stall skip test done")


# ═══════════════════════════════════════════════════════════════
# 测试 7: SafetyTask 越界检查
# ═══════════════════════════════════════════════════════════════

def test_safety_bounds():
    print("\n--- Test 7: SafetyTask bounds check ---")

    bb = Blackboard()
    safety = SafetyTask(blackboard=bb)

    # 7.1 初始状态：安全（里程计未就绪）
    state = safety.get_state()
    check(state.ok or not bb.get(P_HAS_ODOM, False),
          "safety ok or no odom (cold start not triggered yet)")

    # 7.2 给出定位但正常范围内 -> 安全
    bb.set(P_HAS_ODOM, True, 'test')
    bb.set(P_POSE_X, 1.0, 'test')
    bb.set(P_POSE_Y, 1.0, 'test')
    safety.tick()
    state = safety.get_state()
    check(state.ok, "in-bounds: safety ok")

    # 7.3 越界 -> 不安全
    bb.set(P_POSE_X, -10.0, 'test')
    bb.set(P_POSE_Y, 5.0, 'test')
    safety.tick()
    state = safety.get_state()
    check(not state.ok, "out-of-bounds: safety triggered")
    check(len(state.reasons) >= 1, "has reasons")

    # 7.4 回到界内 -> 自动恢复
    bb.set(P_POSE_X, 1.0, 'test')
    bb.set(P_POSE_Y, 1.0, 'test')
    safety.tick()
    state = safety.get_state()
    check(state.ok, "back in bounds: safety restored")
    check(len(state.reasons) == 0, "reasons cleared")

    # 7.5 停滞检测（直接用极短超时 + 强制设置内部状态）
    bb.set(P_PATH_LOADED, True, 'test')
    bb.set(P_POSE_X, 2.0, 'test')
    bb.set(P_POSE_Y, 2.0, 'test')

    safety3 = SafetyTask(blackboard=bb, stall_time_threshold=0.001)
    safety3._stall_last_pos = (2.0, 2.0)
    safety3._stall_start_time = time.monotonic() - 100.0
    safety3.tick()
    state3 = safety3.get_state()
    check(not state3.ok, "stall detected")

    safety.stop()
    print("  safety test done")


# ═══════════════════════════════════════════════════════════════
# 测试 8: Localization→Mission 端到端链路
# ═══════════════════════════════════════════════════════════════

def test_loc_mission_integration():
    print("\n--- Test 8: Localization→Mission integration ---")

    bb = Blackboard()
    loc = LocalizationTask(blackboard=bb)
    from car_localization.tasks.mission import MissionTask
    mission = MissionTask(blackboard=bb)

    # 建立路线
    pts = make_route_pts('N10', 'N9', 'M9')
    bb.set(P_PATH_POINTS, pts, 'test')
    bb.set(P_PATH_LOADED, True, 'test')

    # 模拟位置：M9 中心（终点）
    m9_x, m9_y = block_to_meters('M9')
    loc.feed_odom(m9_x, m9_y, 0.0, 0.0, 0.0, 1.0)
    loc.tick()

    # Mission tick
    mission.tick()

    # 检查：M9 与终点的距离 < arrival_threshold
    final = pts[-1]
    cx = bb.get(P_POSE_X, 0.0)
    cy = bb.get(P_POSE_Y, 0.0)
    dx = final['x'] - cx
    dy = final['y'] - cy
    dist = math.hypot(dx, dy)
    check(dist < 0.157, f"at final point (dist={dist:.3f})")
    check(bb.get(P_ARRIVED, False), "arrived!")

    loc.stop()
    mission.stop()
    print("  integration test done")


# ═══════════════════════════════════════════════════════════════
# 运行
# ═══════════════════════════════════════════════════════════════

if __name__ == '__main__':
    print("=" * 50)
    print("tasks/ 模块测试")
    print("=" * 50)

    test_task_thread()
    test_localization()
    test_mission_shoot_wait()
    test_mission_scan_wait()
    test_mission_arrival()
    test_mission_stall_skip()
    test_safety_bounds()
    test_loc_mission_integration()

    print("\n" + "=" * 50)
    print(f"总计: {PASS} 通过, {FAIL} 失败")
    print("=" * 50)

    if FAIL > 0:
        sys.exit(1)
    sys.exit(0)
