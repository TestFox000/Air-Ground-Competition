"""alignment测试：新 core/ + route/ 模块 vs V1 path_executor bitwise consistent。

V1 函数直接内联（从 path_executor.py L105-190 复制），
不依赖 rclpy / ROS2 环境。
使用 math.isclose rel_tol=1e-12 比较浮点数。
"""

import math
import sys
import os

# ═══════════════════════════════════════════════════════════════
# V1 参考实现（从 path_executor.py L105-190 逐字复制）
# ═══════════════════════════════════════════════════════════════

# ---- Map constants (V1 L105-L115) ----
V1_COL_LETTERS = 'ABCDEFGHIJKLMN'
V1_MAP_COLS = 14
V1_MAP_ROWS = 11
V1_BLOCK_SIZE = 0.35
V1_RAW_YAW_TO_MAP_YAW_OFFSET = math.pi / 2


def v1_euler_from_quaternion(x, y, z, w):
    """Quaternion -> yaw angle (radians).  V1 L123-131"""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def v1_normalize_angle(angle):
    """Normalize angle to [-pi, pi].  V1 L134-142"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def v1_block_to_meters(block_name):
    """Block name -> metric coordinates.  V1 L145-170"""
    letter = block_name[0].upper()
    number = int(block_name[1:])
    col_idx = V1_COL_LETTERS.index(letter)
    row_idx = number - 1
    x = (10.5 - row_idx) * V1_BLOCK_SIZE
    y = (13.5 - col_idx) * V1_BLOCK_SIZE
    return x, y


def v1_heading_to_quaternion(heading_dir):
    """Heading letter -> quaternion.  V1 L173-190"""
    if heading_dir == 'y':
        yaw = math.pi / 2
    elif heading_dir == 'x':
        yaw = 0.0
    else:
        yaw = 0.0
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return 0.0, 0.0, qz, qw


# ═══════════════════════════════════════════════════════════════
# V2 引用
# ═══════════════════════════════════════════════════════════════
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'car_localization'))

from car_localization.core.geometry import (
    euler_from_quaternion as v2_euler_from_quaternion,
    normalize_angle as v2_normalize_angle,
    block_to_meters as v2_block_to_meters,
    heading_to_quaternion as v2_heading_to_quaternion,
    COL_LETTERS as V2_COL_LETTERS,
    MAP_COLS as V2_MAP_COLS,
    MAP_ROWS as V2_MAP_ROWS,
    BLOCK_SIZE as V2_BLOCK_SIZE,
    RAW_YAW_TO_MAP_YAW_OFFSET as V2_RAW_YAW_OFFSET,
    Pose,
    ang_diff,
    yaw_to_quaternion,
    heading_token_to_yaw,
)

from car_localization.core.config import Config, SCHEMA, _migrate_legacy
from car_localization.core.blackboard import Blackboard, Entry
from car_localization.route.model import Waypoint, Route, RouteBuilder, make_route
from car_localization.route.parser import (
    normalize_token, parse_token, _split_block_suffix,
    parse_mission, parse_reset, format_route,
    APPROACH_DIST,
)


# ── alignment工具 ───────────────────────────────────────────────

def check(name: str, v1_val, v2_val, rel_tol=1e-12):
    """比较 V1 和 V2 的值，bitwise consistent。"""
    if isinstance(v1_val, float) and isinstance(v2_val, float):
        if math.isclose(v1_val, v2_val, rel_tol=rel_tol):
            print(f"  [OK] {name}: V1={v1_val:.15f} ~ V2={v2_val:.15f}")
            return True
        else:
            print(f"  [FAIL] {name}: V1={v1_val:.15f} != V2={v2_val:.15f} (diff={abs(v1_val-v2_val):.2e})")
            return False
    elif v1_val == v2_val:
        print(f"  [OK] {name}: V1={v1_val!r} == V2={v2_val!r}")
        return True
    else:
        print(f"  [FAIL] {name}: V1={v1_val!r} != V2={v2_val!r}")
        return False


def check_tuple(name: str, v1_tup, v2_tup, rel_tol=1e-12):
    """逐元素比较 tuple。"""
    if len(v1_tup) != len(v2_tup):
        print(f"  [FAIL] {name}: length V1={len(v1_tup)} != V2={len(v2_tup)}")
        return False
    ok = True
    for i, (a, b) in enumerate(zip(v1_tup, v2_tup)):
        if isinstance(a, float) and isinstance(b, float):
            if not math.isclose(a, b, rel_tol=rel_tol):
                print(f"  [FAIL] {name}[{i}]: V1={a:.15f} != V2={b:.15f}")
                ok = False
        elif a != b:
            print(f"  [FAIL] {name}[{i}]: V1={a!r} != V2={b!r}")
            ok = False
    if ok:
        print(f"  [OK] {name}: tuple({len(v1_tup)} elems) consistent")
    return ok


# ═══════════════════════════════════════════════════════════════
# 测试 1: 常量
# ═══════════════════════════════════════════════════════════════
def test_constants():
    print("\n=== 1. 常量alignment ===")
    results = []
    results.append(check('COL_LETTERS', V1_COL_LETTERS, V2_COL_LETTERS))
    results.append(check('MAP_COLS', V1_MAP_COLS, V2_MAP_COLS))
    results.append(check('MAP_ROWS', V1_MAP_ROWS, V2_MAP_ROWS))
    results.append(check('BLOCK_SIZE', V1_BLOCK_SIZE, V2_BLOCK_SIZE))
    results.append(check('RAW_YAW_TO_MAP_YAW_OFFSET',
                         V1_RAW_YAW_TO_MAP_YAW_OFFSET, V2_RAW_YAW_OFFSET))
    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 2: euler_from_quaternion
# ═══════════════════════════════════════════════════════════════
def test_euler_from_quaternion():
    print("\n=== 2. euler_from_quaternion ===")
    tests = [
        (0.0, 0.0, 0.0, 1.0),          # yaw=0
        (0.0, 0.0, math.sin(math.pi/4), math.cos(math.pi/4)),  # yaw=π/2
        (0.0, 0.0, -0.7071, 0.7071),    # yaw≈-π/2
        (0.0, 0.0, 1.0, 0.0),           # yaw=π
        (0.0, 0.0, 0.382683, 0.92388),  # yaw=π/8 ≈ 0.392699
        (0.1, 0.0, 0.0, 0.995),         # 微扰
        (-0.1, 0.2, 0.3, 0.927),        # 随机
    ]
    results = []
    for x, y, z, w in tests:
        v1 = v1_euler_from_quaternion(x, y, z, w)
        v2 = v2_euler_from_quaternion(x, y, z, w)
        results.append(check(f'euler(({x},{y},{z},{w})', v1, v2))
    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 3: normalize_angle
# ═══════════════════════════════════════════════════════════════
def test_normalize_angle():
    print("\n=== 3. normalize_angle ===")
    tests = [
        0.0,
        1.0,
        -1.0,
        math.pi,
        -math.pi,
        3.5,
        -3.5,
        10.0,
        -10.0,
        100.0,
        0.5 * math.pi,
        -0.5 * math.pi,
        2 * math.pi,
        -2 * math.pi,
        5 * math.pi,
        1e-10,
    ]
    results = []
    for a in tests:
        v1 = v1_normalize_angle(a)
        v2 = v2_normalize_angle(a)
        results.append(check(f'normalize({a})', v1, v2))
    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 4: block_to_meters
# ═══════════════════════════════════════════════════════════════
def test_block_to_meters():
    print("\n=== 4. block_to_meters alignment ===")
    blocks = ['A1', 'N11', 'M10', 'G8', 'D2', 'A11', 'N1', 'H6',
              'a1', 'm10', 'n11', 'A10', 'B2', 'C3', 'K9', 'E4']
    results = []
    for b in blocks:
        v1 = v1_block_to_meters(b)
        v2 = v2_block_to_meters(b)
        results.append(check_tuple(f'block_to_meters({b})', v1, v2))
    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 5: heading_to_quaternion
# ═══════════════════════════════════════════════════════════════
def test_heading_to_quaternion():
    print("\n=== 5. heading_to_quaternion ===")
    dirs = ['x', 'y', 'X', 'Y', 'z', '', 'other']
    results = []
    for d in dirs:
        v1 = v1_heading_to_quaternion(d)
        v2 = v2_heading_to_quaternion(d)
        results.append(check_tuple(f'heading_to_quaternion({d!r})', v1, v2))
    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 6: 新工具函数（无 V1 直接对应，self-consistency check）
# ═══════════════════════════════════════════════════════════════
def test_utility_functions():
    print("\n=== 6. Utility functions（self-consistency check）===")
    results = []

    # ang_diff 应与 normalize_angle(a - b) 一致
    cases = [(1.0, 0.5), (0.0, math.pi), (3.0, -2.0), (-1.0, 1.0)]
    for a, b in cases:
        diff = ang_diff(a, b)
        expect = v1_normalize_angle(a - b)
        results.append(check(f'ang_diff({a},{b})', expect, diff))

    # yaw_to_quaternion → recomputed yaw should match
    yaws = [0.0, math.pi/2, math.pi, -math.pi/2, 0.78, -1.56, 2.5, -0.3]
    for y in yaws:
        q = yaw_to_quaternion(y)
        back = v2_euler_from_quaternion(*q)
        results.append(check(f'yaw_to_quaternion->yaw({y:.6f})', y, back))

    # heading_token_to_yaw
    results.append(check('heading_token_to_yaw(x)', 0.0, heading_token_to_yaw('x')))
    results.append(check('heading_token_to_yaw(y)', math.pi/2, heading_token_to_yaw('y')))
    results.append(check('heading_token_to_yaw(z)', 0.0, heading_token_to_yaw('z')))
    results.append(check('heading_token_to_yaw("")', 0.0, heading_token_to_yaw('')))

    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 7: Config
# ═══════════════════════════════════════════════════════════════
def test_config():
    print("\n=== 7. Config test ===")
    results = []

    # 默认值与 V1 硬编码值alignment
    cfg = Config(flat={})
    v_lookahead = cfg.get('pursuit.lookahead_distance')
    v_speed = cfg.get('pursuit.cruise_speed_mps')
    v_wheelbase = cfg.get('pursuit.wheelbase')
    v_arrival = cfg.get('pursuit.arrival_threshold')
    v_approach = cfg.get('pursuit.approach_dist')
    v_shots = cfg.get('mission.shots_per_stop')
    v_shoot_timeout = cfg.get('mission.shoot_timeout_s')
    v_shoot_radius = cfg.get('mission.shoot_stop_radius')
    v_scan_timeout = cfg.get('mission.scan_wait_timeout_s')
    v_scan_radius = cfg.get('mission.scan_stop_radius')
    v_scan_mode = cfg.get('mission.scan_mode')

    # V1 硬编码值（L239-279）
    v1_lookahead = 0.15
    v1_speed = 0.3
    v1_wheelbase = 0.157
    v1_arrival = 0.10 + (math.pi * 0.073 / 4.0)
    v1_approach = 0.6
    v1_shots = 10
    v1_shoot_timeout = 150.0
    v1_shoot_radius = 0.17
    v1_scan_timeout = 8.0
    v1_scan_radius = 0.20
    v1_scan_mode = 'continuous'

    results.append(check('Config.lookahead_distance', v1_lookahead, v_lookahead))
    results.append(check('Config.cruise_speed_mps', v1_speed, v_speed))
    results.append(check('Config.wheelbase', v1_wheelbase, v_wheelbase))
    results.append(check('Config.arrival_threshold', v1_arrival, v_arrival))
    results.append(check('Config.approach_dist', v1_approach, v_approach))
    results.append(check('Config.shots_per_stop', v1_shots, v_shots))
    results.append(check('Config.shoot_timeout_s', v1_shoot_timeout, v_shoot_timeout))
    results.append(check('Config.shoot_stop_radius', v1_shoot_radius, v_shoot_radius))
    results.append(check('Config.scan_wait_timeout_s', v1_scan_timeout, v_scan_timeout))
    results.append(check('Config.scan_stop_radius', v1_scan_radius, v_scan_radius))
    results.append(check('Config.scan_mode', v1_scan_mode, v_scan_mode))

    # override
    cfg2 = Config(flat={}).override('pursuit.cruise_speed_mps', 0.5)
    results.append(check('Config.override', 0.5, cfg2.get('pursuit.cruise_speed_mps')))

    # from_dict
    cfg3 = Config.from_dict({'pursuit.wheelbase': 0.160})
    results.append(check('Config.from_dict', 0.160, cfg3.get('pursuit.wheelbase')))

    # section
    sec = cfg.section('pursuit')
    results.append(check('Config.section has keys', True, len(sec) >= 5))

    # 默认值回退（缺键）
    results.append(check('Config.get default fallback',
                         v1_lookahead, cfg.get('pursuit.lookahead_distance')))

    # migrate_legacy
    results.append(check('migrate(linear_speed)',
                         'pursuit.cruise_speed_mps', _migrate_legacy('linear_speed')))
    results.append(check('migrate(lookahead_dist)',
                         'pursuit.lookahead_distance', _migrate_legacy('lookahead_dist')))
    results.append(check('migrate(wheelbase)',
                         'pursuit.wheelbase', _migrate_legacy('wheelbase')))
    results.append(check('migrate(arrival_threshold)',
                         'pursuit.arrival_threshold', _migrate_legacy('arrival_threshold')))
    results.append(check('migrate(SHOOT_COUNT_TARGET)',
                         'mission.shots_per_stop', _migrate_legacy('SHOOT_COUNT_TARGET')))
    results.append(check('migrate(unknown_key)',
                         'unknown_key', _migrate_legacy('unknown_key')))

    # require
    try:
        val = cfg.require('pursuit.wheelbase')
        results.append(check('Config.require success', 0.157, val))
    except Exception as e:
        results.append(False)
        print(f"  [FAIL] Config.require failed: {e}")

    # validate
    issues = cfg.validate(strict=False)
    results.append(check('Config.validate clean', 0, len(issues)))

    # describe
    desc = cfg.describe()
    results.append(check('Config.describe non-empty', True, len(desc) >= 10))

    # Spec.check
    spec = SCHEMA['pursuit.wheelbase']
    results.append(check('Spec.check valid', None, spec.check(0.157)))
    err = spec.check(-1.0)
    results.append(check('Spec.check invalid', True, err is not None))

    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 8: Blackboard
# ═══════════════════════════════════════════════════════════════
def test_blackboard():
    print("\n=== 8. Blackboard test ===")
    results = []
    bb = Blackboard()

    # set/get
    bb.set('current_x', 1.23, source='test')
    v = bb.get('current_x')
    results.append(check('Blackboard.set/get', 1.23, v))

    # update
    bb.update(source='test2', current_y=4.56, current_yaw=1.0)
    v = bb.get('current_y')
    results.append(check('Blackboard.update', 4.56, v))

    # default
    v = bb.get('nonexistent', 999)
    results.append(check('Blackboard.default', 999, v))

    # age
    age = bb.age('current_x')
    results.append(check('Blackboard.age exists', True, age is not None and age >= 0))

    # entry
    entry = bb.entry('current_x')
    results.append(check('Blackboard.entry value', 1.23, entry.value))
    results.append(check('Blackboard.entry source', 'test', entry.source))

    # contains
    results.append(check('Blackboard.__contains__ True', True, 'current_x' in bb))
    results.append(check('Blackboard.__contains__ False', False, 'nope' in bb))

    # snapshot
    snap = bb.snapshot({'current_x', 'current_y'})
    results.append(check('Blackboard.snapshot x', 1.23, snap['current_x']))
    results.append(check('Blackboard.snapshot y', 4.56, snap['current_y']))

    # get_fresh
    v = bb.get_fresh('current_x', max_age_s=9999)
    results.append(check('Blackboard.get_fresh', 1.23, v))

    v = bb.get_fresh('current_x', max_age_s=-1)
    results.append(check('Blackboard.get_fresh expired', None, v))

    # is_fresh
    results.append(check('Blackboard.is_fresh True', True, bb.is_fresh('current_x', 9999)))
    results.append(check('Blackboard.is_fresh False', False, bb.is_fresh('nonexistent', 9999)))

    # contains (Blackboard deliberately has no __len__, use describe instead)
    desc = bb.describe()
    results.append(check('Blackboard describe >= 2 keys', True, len(desc) >= 2))

    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 9: Waypoint / Route / RouteBuilder
# ═══════════════════════════════════════════════════════════════
def test_route_model():
    print("\n=== 9. Waypoint / Route test ===")
    results = []

    # 构造 Waypoint
    wp = Waypoint(x=1.23, y=4.56, yaw=0.78)
    results.append(check('Waypoint.x', 1.23, wp.x))
    results.append(check('Waypoint.y', 4.56, wp.y))
    results.append(check('Waypoint.yaw', 0.78, wp.yaw))

    # at_block — 坐标应与 V1 block_to_meters 一致
    test_blocks = ['M10', 'G8', 'A1', 'N11', 'D2', 'K9']
    for b in test_blocks:
        wp2 = Waypoint.at_block(b)
        v1_x, v1_y = v1_block_to_meters(b)
        results.append(check(f'Waypoint.at_block({b}).x', v1_x, wp2.x))
        results.append(check(f'Waypoint.at_block({b}).y', v1_y, wp2.y))

    # with_yaw
    wp2 = Waypoint.at_block('M10')
    wp3 = wp2.with_yaw(1.5)
    results.append(check('Waypoint.with_yaw', 1.5, wp3.yaw))
    results.append(check('Waypoint.with_yaw keeps x', wp2.x, wp3.x))
    results.append(check('Waypoint.with_yaw keeps y', wp2.y, wp3.y))

    # distance_to
    d = wp2.distance_to(0, 0)
    expected = math.hypot(wp2.x, wp2.y)
    results.append(check('Waypoint.distance_to', expected, d))

    # heading_to
    wp_a = Waypoint.at_block('A11')
    wp_n = Waypoint.at_block('N1')
    h = wp_a.heading_to(wp_n)
    expected_h = math.atan2(wp_n.y - wp_a.y, wp_n.x - wp_a.x)
    results.append(check('Waypoint.heading_to', expected_h, h))

    # quaternion property — 应与 heading_to_quaternion 一致
    qx, qy, qz, qw = wp3.quaternion
    v1_q = v1_heading_to_quaternion('x')  # yaw=0
    expected_qz = math.sin(1.5 / 2)
    expected_qw = math.cos(1.5 / 2)
    results.append(check('Waypoint.quaternion qx', 0.0, qx))
    results.append(check('Waypoint.quaternion qy', 0.0, qy))
    results.append(check('Waypoint.quaternion qz', expected_qz, qz))
    results.append(check('Waypoint.quaternion qw', expected_qw, qw))

    # to_v1_dict
    d = wp3.to_v1_dict()
    results.append(check('Waypoint.to_v1_dict x', wp3.x, d['x']))
    results.append(check('Waypoint.to_v1_dict y', wp3.y, d['y']))
    results.append(check('Waypoint.to_v1_dict qz', expected_qz, d['qz']))
    results.append(check('Waypoint.to_v1_dict qw', expected_qw, d['qw']))
    results.append(check('Waypoint.to_v1_dict qx', 0.0, d['qx']))
    results.append(check('Waypoint.to_v1_dict qy', 0.0, d['qy']))

    # is_stop_point
    wp_scan = Waypoint(x=0, y=0, yaw=0, scan=True)
    wp_shoot = Waypoint(x=0, y=0, yaw=0, shoot=True)
    wp_none = Waypoint(x=0, y=0, yaw=0, scan=False, shoot=False)
    results.append(check('Waypoint.is_stop_point (scan)', True, wp_scan.is_stop_point()))
    results.append(check('Waypoint.is_stop_point (shoot)', True, wp_shoot.is_stop_point()))
    results.append(check('Waypoint.is_stop_point (none)', False, wp_none.is_stop_point()))

    # Route
    wps = (
        Waypoint.at_block('A1'),
        Waypoint.at_block('N11'),
    )
    route = Route(waypoints=wps, name='test')
    results.append(check('Route.__len__', 2, len(route)))
    results.append(check('Route.__getitem__ x', wps[0].x, route[0].x))

    # path_length
    length = route.path_length()
    expected_len = math.hypot(wps[1].x - wps[0].x, wps[1].y - wps[0].y)
    results.append(check('Route.path_length', expected_len, length))

    # RouteBuilder
    builder = RouteBuilder(name='build_test')
    builder.add(block='G8')
    builder.add(block='A10', shoot=True)
    built = builder.build()
    results.append(check('RouteBuilder.build len', 2, len(built)))
    results.append(check('RouteBuilder.build shoot[1]', True, built.waypoints[1].shoot))

    # 去重：重复格名
    builder2 = RouteBuilder()
    builder2.add(block='G8')
    builder2.add(block='G8', shoot=True)
    built2 = builder2.build()
    results.append(check('RouteBuilder.dedup len', 1, len(built2)))
    results.append(check('RouteBuilder.dedup shoot', True, built2.waypoints[0].shoot))

    # make_route
    mr = make_route([Waypoint(x=0, y=0, yaw=0), Waypoint(x=1, y=1, yaw=0)])
    results.append(check('make_route len', 2, len(mr)))

    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 10: Parser
# ═══════════════════════════════════════════════════════════════
def test_parser():
    print("\n=== 10. Parser test ===")
    results = []

    # parse_mission 基本路由
    route1 = parse_mission(['G8', 'A10'], name='test1', current_yaw=0.0)
    results.append(check('parse_mission G8 A10 len', 2, len(route1)))

    g8_x, g8_y = v1_block_to_meters('G8')
    a10_x, a10_y = v1_block_to_meters('A10')
    results.append(check('parse_mission G8.x', g8_x, route1[0].x))
    results.append(check('parse_mission G8.y', g8_y, route1[0].y))
    results.append(check('parse_mission A10.x', a10_x, route1[1].x))
    results.append(check('parse_mission A10.y', a10_y, route1[1].y))

    # G8→A10 方向 = Y-（北偏）
    dir_yaw = math.atan2(a10_y - g8_y, a10_x - g8_x)
    results.append(check('parse_mission G8 yaw', dir_yaw, route1[0].yaw))
    results.append(check('parse_mission A10 yaw', dir_yaw, route1[1].yaw))

    # 四点路径 + 射击
    route2 = parse_mission(['G8', 'A10', 'D2*', 'E2'], name='test2',
                           current_yaw=0.0)
    results.append(check('parse_mission 4pts len', 4, len(route2)))

    d2_x, d2_y = v1_block_to_meters('D2')
    d2_found = False
    for i, wp in enumerate(route2.waypoints):
        if abs(wp.x - d2_x) < 1e-9 and abs(wp.y - d2_y) < 1e-9:
            d2_found = True
            results.append(check(f'parse_mission D2* shoot at #{i}', True, wp.shoot))
    results.append(check('parse_mission D2 found', True, d2_found))

    # 尾部航向指定
    route3 = parse_mission(['G8', 'A10', 'C2,y'], name='test3', current_yaw=0.0)
    last_wp = route3.waypoints[-1]
    results.append(check('parse_mission ,y final_yaw', math.pi/2, last_wp.yaw))

    # 单点路径
    route4 = parse_mission(['A10'], name='single', current_yaw=0.5)
    results.append(check('parse_mission single pt len', 1, len(route4)))
    # 单点保持 current_yaw
    results.append(check('parse_mission single pt yaw', 0.5, route4[0].yaw))

    # invalid token
    try:
        parse_mission(['INVALID'])
        results.append(False)
        print("  [FAIL] parse_mission invalid token 应raise exception")
    except ValueError:
        results.append(True)
        print("  [OK] parse_mission invalid token raises ValueError")

    # parse_reset
    block, yaw = parse_reset('N10,y')
    results.append(check('parse_reset block', 'N10', block))
    results.append(check('parse_reset yaw', math.pi/2, yaw))

    block2, yaw2 = parse_reset('L9,x')
    results.append(check('parse_reset L9 block', 'L9', block2))
    results.append(check('parse_reset L9 yaw', 0.0, yaw2))

    # invalid reset spec
    try:
        parse_reset('INVALID')
        results.append(False)
        print("  [FAIL] parse_reset invalid spec 应raise exception")
    except ValueError:
        results.append(True)
        print("  [OK] parse_reset invalid spec raises ValueError")

    # _split_block_suffix
    body, fh = _split_block_suffix('G8 A10 C2,y')
    results.append(check('_split_block_suffix body', 'G8 A10 C2', body))
    results.append(check('_split_block_suffix heading', 'y', fh))

    body2, fh2 = _split_block_suffix('G8 A10')
    results.append(check('_split_block_suffix no heading', 'G8 A10', body2))
    results.append(check('_split_block_suffix no heading val', None, fh2))

    try:
        _split_block_suffix('G8,w')
        results.append(False)
        print("  [FAIL] _split_block_suffix invalid heading 应raise exception")
    except ValueError:
        results.append(True)
        print("  [OK] _split_block_suffix invalid heading raises ValueError")

    # format_route
    rt_tokens = format_route(route1)
    results.append(check('format_route len', 2, len(rt_tokens)))

    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 11: Pose dataclass
# ═══════════════════════════════════════════════════════════════
def test_pose():
    print("\n=== 11. Pose test ===")
    results = []

    p = Pose(x=1.5, y=2.5, yaw=0.3)
    results.append(check('Pose.x', 1.5, p.x))
    results.append(check('Pose.y', 2.5, p.y))
    results.append(check('Pose.yaw', 0.3, p.yaw))

    # from_xy
    p2 = Pose.from_xy(3.0, 4.0)
    results.append(check('Pose.from_xy.x', 3.0, p2.x))
    results.append(check('Pose.from_xy.yaw', 0.0, p2.yaw))

    # offset_to
    dx, dy, dist = p.offset_to(3.5, 4.5)
    results.append(check('Pose.offset_to.dx', 2.0, dx))
    results.append(check('Pose.offset_to.dy', 2.0, dy))
    results.append(check('Pose.offset_to.dist', math.hypot(2, 2), dist))

    # translated
    p3 = p.translated(0.5, -0.3)
    results.append(check('Pose.translated.x', 2.0, p3.x))
    results.append(check('Pose.translated.y', 2.2, p3.y))
    results.append(check('Pose.translated.yaw', 0.3, p3.yaw))

    # rotated
    p4 = p.rotated(0.5)
    results.append(check('Pose.rotated.yaw', 0.8, p4.yaw))
    results.append(check('Pose.rotated keeps x', 1.5, p4.x))

    # block() — 从米制反查格名
    p_m10 = Pose(x=0.525, y=0.525, yaw=0)  # M10 center
    b = p_m10.block()
    results.append(check('Pose.block M10', 'M10', b))

    p_n11 = Pose(x=0.175, y=0.175, yaw=0)  # N11 中心
    b2 = p_n11.block()
    results.append(check('Pose.block N11', 'N11', b2))

    return all(results)


# ═══════════════════════════════════════════════════════════════
# 主入口
# ═══════════════════════════════════════════════════════════════
if __name__ == '__main__':
    tests = [
        ('常量', test_constants),
        ('euler_from_quaternion', test_euler_from_quaternion),
        ('normalize_angle', test_normalize_angle),
        ('block_to_meters', test_block_to_meters),
        ('heading_to_quaternion', test_heading_to_quaternion),
        ('工具函数', test_utility_functions),
        ('Config', test_config),
        ('Blackboard', test_blackboard),
        ('Waypoint/Route', test_route_model),
        ('Parser', test_parser),
        ('Pose', test_pose),
    ]
    failed_groups = []
    for name, fn in tests:
        ok = fn()
        if not ok:
            failed_groups.append(name)

    print(f"\n{'=' * 50}")
    if not failed_groups:
        print("[OK] All alignment passed! V2 output is V1 bitwise consistent。")
    else:
        print(f"[FAIL] 以下 {len(failed_groups)} 个分组存在不一致：{' '.join(failed_groups)}")
    sys.exit(0 if not failed_groups else 1)
