# -*- coding: utf-8 -*-
"""
Full Regression Test Suite - V2 Modules vs V1 path_executor bitwise consistency.

Exit:
  0 = ALL tests pass
  1 = failures detected
"""

import math
import sys
import os
import time
import subprocess

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'car_localization'))

from car_localization.core.geometry import (
    COL_LETTERS, BLOCK_SIZE, RAW_YAW_TO_MAP_YAW_OFFSET,
    euler_from_quaternion, normalize_angle, block_to_meters,
    heading_to_quaternion, Pose, ang_diff, yaw_to_quaternion,
    heading_token_to_yaw, MAP_COLS, MAP_ROWS,
)
from car_localization.core.blackboard import (
    Blackboard, P_POSE_X, P_POSE_Y, P_ARRIVED,
    P_PATH_POINTS, P_PATH_LOADED, P_CORNER_MODE,
)
from car_localization.core.config import Config, _migrate_legacy
from car_localization.route.model import Waypoint, Route, RouteBuilder, make_route
from car_localization.route.parser import (
    _split_block_suffix, parse_mission, parse_reset, format_route,
)
from car_localization.pursuit.path import (
    Path, Projection, find_nearest_segment_v1,
)
from car_localization.pursuit.lookahead import (
    LookaheadPoint, find_lookahead_point, find_lookahead_point_v1_dict,
)
from car_localization.pursuit.controller import (
    PursuitCommand, compute_command, brake_command,
    pure_pursuit_control_v1,
)
from car_localization.io.ros_io import OdomFrame, WaypointMsg, IntMsg, RosBridge

PASS = 0
FAIL = 0
FAILURES = []

# For near-zero comparisons
_EPS = 1e-15


def check_relaxed(name, v1, v2, rel_tol=1e-12):
    """Compare with floating tolerance (handles near-zero values via abs_tol)."""
    global PASS, FAIL
    ok = True
    detail = ''
    if isinstance(v1, float) and isinstance(v2, float):
        if math.isclose(v1, v2, rel_tol=rel_tol, abs_tol=1e-15) or (math.isnan(v1) and math.isnan(v2)):
            PASS += 1
            return True
        else:
            ok = False
            diff = abs(v1 - v2)
            detail = f"V1={v1:.15f} != V2={v2:.15f} (diff={diff:.2e})"
    elif v1 == v2:
        PASS += 1
        return True
    else:
        ok = False
        if isinstance(v1, bool) and isinstance(v2, bool):
            detail = f"V1={v1} != V2={v2}"
        else:
            detail = f"V1={v1!r} != V2={v2!r}"

    if not ok:
        FAIL += 1
        FAILURES.append((name, detail))
        print(f"  [FAIL] {name}: {detail}")
    return ok


def make_v1_pt(x, y):
    return {'x': x, 'y': y, 'z': 0.0,
            'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}


def make_route_points(*blocks):
    pts = []
    for b in blocks:
        x, y = block_to_meters(b.upper().rstrip('*'))
        pts.append(make_v1_pt(x, y))
    return pts


# V1 references
def v1_euler_from_quaternion(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def v1_normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def v1_block_to_meters(block_name):
    letter = block_name[0].upper()
    number = int(block_name[1:])
    col_idx = 'ABCDEFGHIJKLMN'.index(letter)
    row_idx = number - 1
    x = (10.5 - row_idx) * 0.35
    y = (13.5 - col_idx) * 0.35
    return x, y

def v1_find_nearest_segment(pts, target_idx, cx, cy):
    if not pts or len(pts) < 2:
        return None
    best = None
    for i in range(target_idx, len(pts) - 1):
        p1, p2 = pts[i], pts[i+1]
        sx, sy = p2['x'] - p1['x'], p2['y'] - p1['y']
        seg2 = sx*sx + sy*sy
        if seg2 < 1e-12:
            continue
        t = ((cx - p1['x'])*sx + (cy - p1['y'])*sy) / seg2
        t = max(0.0, min(1.0, t))
        qx, qy = p1['x'] + t*sx, p1['y'] + t*sy
        d = math.hypot(cx-qx, cy-qy)
        if best is None or d < best['dist']:
            best = {'idx': i, 't': t, 'px': qx, 'py': qy, 'dist': d}
    return best

def v1_find_lookahead_point(pts, seg, ld):
    if len(pts) < 2:
        return pts[-1] if pts else None
    def _pt(x, y):
        return {'x': x, 'y': y, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
    CA = math.radians(20.0)
    remaining = ld
    idx = seg['idx']
    p1, p2 = pts[idx], pts[idx+1]
    seg_len = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
    left = (1.0 - seg['t']) * seg_len if seg_len > 1e-9 else 0.0
    corner_ahead = False
    if idx + 2 < len(pts):
        d1x, d1y = p2['x'] - p1['x'], p2['y'] - p1['y']
        p3 = pts[idx+2]
        d2x, d2y = p3['x'] - p2['x'], p3['y'] - p2['y']
        if abs(math.atan2(d1x*d2y - d1y*d2x, d1x*d2x + d1y*d2y)) > CA:
            corner_ahead = True
    if corner_ahead and left < 0.04:
        return p2
    if seg_len > 1e-9 and left >= remaining:
        ux, uy = (p2['x'] - p1['x']) / seg_len, (p2['y'] - p1['y']) / seg_len
        return _pt(seg['px'] + ux*remaining, seg['py'] + uy*remaining)
    remaining -= left
    j = idx + 1
    while j < len(pts) - 1 and remaining > 1e-9:
        p1, p2 = pts[j], pts[j+1]
        sl = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
        if sl > 1e-9 and sl >= remaining:
            ux, uy = (p2['x'] - p1['x']) / sl, (p2['y'] - p1['y']) / sl
            return _pt(p1['x'] + ux*remaining, p1['y'] + uy*remaining)
        remaining -= sl
        j += 1
    return pts[-1]

def v1_pure_pursuit_control(target, cx, cy, cyaw, wb=0.157, spd=0.3):
    if target is None:
        return 0.0, 0.0
    dx, dy = target['x'] - cx, target['y'] - cy
    cos_yaw, sin_yaw = math.cos(-cyaw), math.sin(-cyaw)
    rel_x = dx*cos_yaw - dy*sin_yaw
    rel_y = dx*sin_yaw + dy*cos_yaw
    dist = math.sqrt(dx*dx + dy*dy)
    if dist < 0.01:
        return 0.0, 0.0
    alpha = math.atan2(rel_y, rel_x)
    steer = math.degrees(math.atan(wb * math.sin(alpha) / max(dist, 0.10)))
    steer = max(-30.0, min(30.0, steer))
    tf = abs(steer) / 30.0
    vel = max(0.1, spd * (1.0 - 0.5 * tf))
    return vel, steer


# ======================== TEST GROUPS ========================

def test_constants():
    print("--- Constants ---")
    r = True
    r &= check_relaxed('COL_LETTERS', 'ABCDEFGHIJKLMN', COL_LETTERS)
    r &= check_relaxed('MAP_COLS', 14, MAP_COLS)
    r &= check_relaxed('BLOCK_SIZE', 0.35, BLOCK_SIZE)
    r &= check_relaxed('RAW_YAW_OFFSET', math.pi/2, RAW_YAW_TO_MAP_YAW_OFFSET)
    return r


def test_block_to_meters():
    print("--- block_to_meters ---")
    r = True
    cases = [('A1', 3.675, 4.725), ('A11', 0.175, 4.725),
             ('N1', 3.675, 0.175), ('N11', 0.175, 0.175),
             ('M10', 0.525, 0.525), ('G8', 1.225, 2.625),
             ('D2', 3.325, 3.675), ('H6', 1.925, 2.275)]
    for name, ex, ey in cases:
        v1 = v1_block_to_meters(name)
        v2 = block_to_meters(name)
        r &= check_relaxed(f'{name}.x', v1[0], v2[0])
        r &= check_relaxed(f'{name}.y', v1[1], v2[1])
    for b in ['a1', 'n11']:
        r &= check_relaxed(f'case/{b}', v1_block_to_meters(b)[0], block_to_meters(b)[0])
    return r


def test_quaternion():
    print("--- Quaternion ---")
    r = True
    cases = [
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.7071067811865476, 0.7071067811865476),
        (0.0, 0.0, -0.7071067811865476, 0.7071067811865476),
        (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, -1.0, 0.0),
        (0.0, 0.0, 0.0, -1.0), (1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0),
        (-0.1, 0.2, 0.3, 0.927),
    ]
    for qx, qy, qz, qw in cases:
        v1 = v1_euler_from_quaternion(qx, qy, qz, qw)
        v2 = euler_from_quaternion(qx, qy, qz, qw)
        r &= check_relaxed(f'q({qx},{qy},{qz},{qw})', v1, v2)
        q = yaw_to_quaternion(v1)
        back = euler_from_quaternion(*q)
        r &= check_relaxed(f'rt(v1={v1:.6f})', v1, back, rel_tol=1e-9)
    return r


def test_normalize_angle():
    print("--- normalize_angle ---")
    r = True
    for a in [0.0, math.pi, -math.pi, 3.141592653589793, -3.141592653589793,
              2*math.pi, -2*math.pi, 3*math.pi, -3*math.pi,
              100*math.pi, 1e-15, -1e-15, 1.0, -1.0]:
        v1 = v1_normalize_angle(a)
        v2 = normalize_angle(a)
        r &= check_relaxed(f'a={a:.15g}', v1, v2)
        r &= check_relaxed(f'range({a:.1f})', True, -math.pi-1e-12 <= v2 <= math.pi+1e-12)
    return r


def test_pose():
    print("--- Pose ---")
    r = True
    p = Pose(x=0, y=0, yaw=0)
    r &= check_relaxed('zero.x', 0.0, p.x)
    r &= check_relaxed('zero.y', 0.0, p.y)

    p3 = Pose.from_xy(5.0, 3.0)
    r &= check_relaxed('from_xy.x', 5.0, p3.x)
    r &= check_relaxed('from_xy.yaw', 0.0, p3.yaw)

    dx, dy, dist = p.offset_to(3, 4)
    r &= check_relaxed('offset.dx', 3.0, dx)
    r &= check_relaxed('offset.dist', 5.0, dist)

    p4 = Pose(x=1, y=2, yaw=0.5).translated(-2, 3)
    r &= check_relaxed('trans.x', -1.0, p4.x)
    r &= check_relaxed('trans.y', 5.0, p4.y)

    p5 = Pose(x=1, y=2, yaw=0).rotated(math.pi/2)
    r &= check_relaxed('rot.yaw', math.pi/2, p5.yaw)

    for b in ['A1', 'N11', 'M10', 'G8', 'H6']:
        bx, by = block_to_meters(b)
        back = Pose(x=bx, y=by, yaw=0).block()
        r &= check_relaxed(f'block_rt({b})', b, back)

    # Near-boundary should round to nearest block
    near = Pose(x=0.530, y=0.530, yaw=0)
    r &= check_relaxed('near_M10', 'M10', near.block())
    return r


def test_ang_diff():
    print("--- ang_diff ---")
    r = True
    cases = [(0.0, 0.0), (1.0, 0.5), (math.pi, 0.0), (0.0, math.pi),
             (1.5, -1.5), (10.0, 10.0)]
    for a, b in cases:
        v1 = v1_normalize_angle(a - b)
        v2 = ang_diff(a, b)
        r &= check_relaxed(f'({a:.4f},{b:.4f})', v1, v2)
    import random; random.seed(42)
    for _ in range(50):
        a, b = random.uniform(-10, 10), random.uniform(-10, 10)
        r &= check_relaxed(f'rand({a:.4f},{b:.4f})', v1_normalize_angle(a-b), ang_diff(a,b))
    return r


def test_blackboard():
    print("--- Blackboard ---")
    r = True
    bb = Blackboard()
    r &= check_relaxed('empty.get', None, bb.get('nonexistent'))
    r &= check_relaxed('empty.default', 42, bb.get('nonexistent', 42))
    r &= check_relaxed('empty.contains', False, 'x' in bb)

    bb.set('k1', 100, 't')
    r &= check_relaxed('set_get', 100, bb.get('k1'))
    bb.update(source='t2', k1=200, k2=300)
    r &= check_relaxed('update.k1', 200, bb.get('k1'))

    e = bb.entry('k1')
    r &= check_relaxed('entry.value', 200, e.value)
    r &= check_relaxed('entry.source', 't2', e.source)

    r &= check_relaxed('is_fresh', True, bb.is_fresh('k1', 99999))
    r &= check_relaxed('fresh_ok', 200, bb.get_fresh('k1', 99999))
    r &= check_relaxed('fresh_expired', None, bb.get_fresh('k1', max_age_s=-1))

    snap = bb.snapshot({'k1', 'k2', 'k3'})
    r &= check_relaxed('snap.k1', 200, snap.get('k1'))
    r &= check_relaxed('snap.all', True, len(bb.snapshot()) >= 2)

    import threading
    errors = []
    def writer():
        for i in range(50):
            bb.set('tk', i, 'w')
    def reader():
        for _ in range(50):
            v = bb.get('tk')
            if v is not None and not isinstance(v, int):
                errors.append(True)
    threads = [threading.Thread(target=f) for f in [writer]*3 + [reader]*3]
    for t in threads: t.start()
    for t in threads: t.join()
    r &= check_relaxed('concurrent', 0, len(errors))

    bb.set(P_POSE_X, 1.5, 't')
    r &= check_relaxed('P_POSE_X', 1.5, bb.get(P_POSE_X))
    return r


def test_config():
    print("--- Config ---")
    r = True

    cfg = Config(flat={})
    r &= check_relaxed('wheelbase', 0.157, cfg.get('pursuit.wheelbase'))
    r &= check_relaxed('lookahead', 0.15, cfg.get('pursuit.lookahead_distance'))
    r &= check_relaxed('speed', 0.3, cfg.get('pursuit.cruise_speed_mps'))
    # require() before override() to test fresh config
    r &= check_relaxed('require_ok', 0.157, cfg.require('pursuit.wheelbase'))
    r &= check_relaxed('fallback', 42, cfg.get('missing.k', 42))

    try:
        cfg.require('missing.k')
        r = False; print("  [FAIL] require(missing) should raise")
    except Exception:
        pass

    # override returns new independent config
    cfg2 = cfg.override('pursuit.wheelbase', 0.160)
    r &= check_relaxed('override', 0.160, cfg2.get('pursuit.wheelbase'))
    r &= check_relaxed('original_unchanged', 0.157,
                       Config(flat={}).get('pursuit.wheelbase'))

    cfg3 = Config.from_dict({'pursuit.wheelbase': 0.165})
    r &= check_relaxed('from_dict', 0.165, cfg3.get('pursuit.wheelbase'))

    r &= check_relaxed('migrate_speed', 'pursuit.cruise_speed_mps',
                       _migrate_legacy('linear_speed'))
    r &= check_relaxed('migrate_unknown', 'unknown_key', _migrate_legacy('unknown_key'))

    # validate
    r &= check_relaxed('validate', 0, len(cfg.validate(strict=False)))
    sec = cfg.section('pursuit')
    r &= check_relaxed('section', True, 'wheelbase' in sec)
    return r


def test_route_model():
    print("--- Route model ---")
    r = True

    # Waypoint
    wp = Waypoint(x=1.23, y=4.56, yaw=0.78)
    r &= check_relaxed('wp.x', 1.23, wp.x)
    r &= check_relaxed('wp.y', 4.56, wp.y)

    # is_stop_point: Waypoint defaults scan=True, shoot=False
    wp_scan = Waypoint(x=0, y=0, yaw=0, scan=True, shoot=False)
    wp_shoot = Waypoint(x=0, y=0, yaw=0, scan=False, shoot=True)
    wp_both = Waypoint(x=0, y=0, yaw=0, scan=True, shoot=True)
    # Default Waypoint has scan=True (so is_stop_point returns True)
    wp_default = Waypoint(x=0, y=0, yaw=0)
    r &= check_relaxed('default.stop', True, wp_default.is_stop_point())
    r &= check_relaxed('scan.stop', True, wp_scan.is_stop_point())
    r &= check_relaxed('shoot.stop', True, wp_shoot.is_stop_point())
    r &= check_relaxed('both.stop', True, wp_both.is_stop_point())
    # Only False when both False
    wp_none = Waypoint(x=0, y=0, yaw=0, scan=False, shoot=False)
    r &= check_relaxed('none.stop', False, wp_none.is_stop_point())

    # at_block
    m10_x, m10_y = block_to_meters('M10')
    wp_m10 = Waypoint.at_block('M10')
    r &= check_relaxed('at_M10.x', m10_x, wp_m10.x)

    wp2 = wp_m10.with_yaw(1.5)
    r &= check_relaxed('with_yaw', 1.5, wp2.yaw)
    r &= check_relaxed('with_yaw.x', wp_m10.x, wp2.x)

    # quaternion
    wp_q = Waypoint(x=1, y=2, yaw=math.pi/4)
    q = wp_q.quaternion
    r &= check_relaxed('quat.qz', math.sin(math.pi/8), q[2])
    r &= check_relaxed('quat.qw', math.cos(math.pi/8), q[3])

    d_v1 = wp_q.to_v1_dict()
    r &= check_relaxed('to_v1.x', 1.0, d_v1['x'])
    r &= check_relaxed('to_v1.y', 2.0, d_v1['y'])

    # Route
    route_empty = Route(waypoints=(), name='empty')
    r &= check_relaxed('empty_len', 0, len(route_empty))

    route_single = Route(waypoints=(Waypoint(x=0,y=0,yaw=0),), name='x')
    r &= check_relaxed('single_len', 1, len(route_single))

    # RouteBuilder dedup
    b = RouteBuilder(); b.add(block='G8'); b.add(block='G8', shoot=True)
    built = b.build()
    r &= check_relaxed('dedup_len', 1, len(built))
    r &= check_relaxed('dedup_shoot', True, built.waypoints[0].shoot)

    # make_route
    mr = make_route([Waypoint(x=0,y=0,yaw=0), Waypoint(x=3,y=0,yaw=0), Waypoint(x=3,y=4,yaw=0)])
    r &= check_relaxed('make_route_len', 3, len(mr))
    r &= check_relaxed('Lshape_len', 7.0, mr.path_length())
    return r


def test_parser():
    print("--- Parser ---")
    r = True

    # Empty raises
    try:
        parse_mission([], name='empty', current_yaw=0.0)
        r = False; print("  [FAIL] empty should raise")
    except ValueError:
        pass

    # Single point
    rt = parse_mission(['A10'], name='s', current_yaw=0.5)
    r &= check_relaxed('single.len', 1, len(rt))
    r &= check_relaxed('single.yaw', 0.5, rt[0].yaw)

    # With heading
    rt2 = parse_mission(['A10,y'], name='sy', current_yaw=0.5)
    r &= check_relaxed('y_yaw', math.pi/2, rt2[0].yaw)

    # shoot(*)
    rt3 = parse_mission(['G8', 'A10*'], name='sh', current_yaw=0.0)
    r &= check_relaxed('shoot', True, rt3[1].shoot)

    # case insensitive
    rt4 = parse_mission(['g8', 'a10'], name='low', current_yaw=0.0)
    g8x, _ = block_to_meters('G8')
    r &= check_relaxed('lower.x', g8x, rt4[0].x)

    # parse_reset
    block, yaw = parse_reset('N10,x')
    r &= check_relaxed('reset_block', 'N10', block)
    r &= check_relaxed('reset_yaw', 0.0, yaw)

    # _split_block_suffix
    body, fh = _split_block_suffix('G8 A10 C2,y')
    r &= check_relaxed('suffix_body', 'G8 A10 C2', body)
    r &= check_relaxed('suffix_fh', 'y', fh)

    body2, fh2 = _split_block_suffix('G8 A10')
    r &= check_relaxed('no_suffix_body', 'G8 A10', body2)
    r &= check_relaxed('no_suffix_fh', None, fh2)

    try:
        _split_block_suffix('G8,w')
        r = False; print("  [FAIL] suffix invalid heading should raise")
    except ValueError:
        pass

    try:
        parse_mission(['INVALID'])
        r = False; print("  [FAIL] invalid token should raise")
    except ValueError:
        pass

    tokens = format_route(rt3)
    r &= check_relaxed('format_len', 2, len(tokens))
    return r


def test_path_projection():
    print("--- Path projection ---")
    r = True

    # Empty/single -> None
    r &= check_relaxed('empty', None, Path([]).project(0,0, s_hint=0.0, window_m=1.0))
    r &= check_relaxed('single', None, Path([make_v1_pt(1,2)]).project(1,2, s_hint=0.0, window_m=1.0))

    # Two-point A1->N11
    pts = make_route_points('A1', 'N11')
    path = Path(pts)

    proj = path.project(pts[0]['x'], pts[0]['y'], s_hint=0.0, window_m=0.0)
    if proj:
        r &= check_relaxed('start.idx', 0, proj.seg_index)
        r &= check_relaxed('start.t', 0.0, proj.t)
    else:
        r = False

    proj2 = path.project(pts[-1]['x'], pts[-1]['y'], s_hint=0.0, window_m=0.0)
    if proj2:
        r &= check_relaxed('end.t', 1.0, proj2.t)
    else:
        r = False

    proj3 = path.project(-5.0, -5.0, s_hint=0.0, window_m=0)  # truly far off
    if proj3:
        r &= check_relaxed('far.dist>3.0', True, proj3.dist > 3.0)
        print(f"     (far offset dist={proj3.dist:.4f})")
    else:
        r = False

    # V1 vs V2 alignment along segment (use window_m=0 for full search from hint)
    for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
        seg = path.segments[0]
        px = seg[0] + t*(seg[2]-seg[0])
        py = seg[1] + t*(seg[3]-seg[1])
        v1 = v1_find_nearest_segment(pts, 0, px, py)
        p2 = path.project(px, py, s_hint=0.0, window_m=0)
        if v1 and p2:
            r &= check_relaxed(f't={t}.idx', v1['idx'], p2.seg_index)
        else:
            r = False

    # Multi-seg G8->G1->A1
    pts2 = make_route_points('G8', 'G1', 'A1')
    path2 = Path(pts2)
    for t in [0.0, 0.5, 1.0]:
        seg = path2.segments[0]
        px = seg[0] + t*(seg[2]-seg[0])
        py = seg[1] + t*(seg[3]-seg[1])
        v1 = v1_find_nearest_segment(pts2, 0, px, py)
        p2 = path2.project(px, py, s_hint=0.0, window_m=0)
        if v1 and p2:
            r &= check_relaxed(f'2_t={t}.idx', v1['idx'], p2.seg_index)

    # V1 compat function
    v1r = find_nearest_segment_v1(pts, 0, 0, 0)
    v1o = v1_find_nearest_segment(pts, 0, 0, 0)
    if v1r and v1o:
        r &= check_relaxed('v1_fn.idx', v1o['idx'], v1r['idx'])
        r &= check_relaxed('v1_fn.dist', v1o['dist'], v1r['dist'])
    return r


def test_lookahead():
    print("--- Lookahead ---")
    r = True
    ld = 0.15

    # Empty path -> fallback (0,0)
    la = find_lookahead_point(Path([]), None, ld)
    r &= check_relaxed('empty', True, la is not None)
    if la:
        r &= check_relaxed('empty.x', 0.0, la.x)

    # Single point
    la2 = find_lookahead_point(Path([make_v1_pt(2.0, 3.0)]), None, ld)
    r &= check_relaxed('single', True, la2 is not None)
    if la2:
        r &= check_relaxed('single.x', 2.0, la2.x)
        r &= check_relaxed('single.y', 3.0, la2.y)

    # Straight line A1->N11, t=0.2
    pts = make_route_points('A1', 'N11')
    path = Path(pts)
    sg = path.segments[0]
    seg_len = sg[4]
    proj = Projection(seg_index=0, t=0.2,
        x=sg[0]+0.2*(sg[2]-sg[0]), y=sg[1]+0.2*(sg[3]-sg[1]),
        s=0.2*seg_len, dist=0.0, seg_length=seg_len, seg_heading=sg[5], passed_end=False)
    la = find_lookahead_point(path, proj, ld)
    vs = {'idx': 0, 't': 0.2, 'px': proj.x, 'py': proj.y, 'dist': 0.0}
    v1 = v1_find_lookahead_point(pts, vs, ld)
    if la and v1:
        r &= check_relaxed('straight.x', v1['x'], la.x)
        r &= check_relaxed('straight.y', v1['y'], la.y)

    # V1 dict function
    v1d = find_lookahead_point_v1_dict(pts, vs, ld)
    if v1d and v1:
        r &= check_relaxed('v1_dict.x', v1['x'], v1d['x'])

    # Corner clamp at t=0.999 on G8->G1->A1
    pts2 = make_route_points('G8', 'G1', 'A1')
    path2 = Path(pts2)
    sg0_len = path2.segments[0][4]
    proj2 = Projection(seg_index=0, t=0.999,
        x=pts2[0]['x']+0.999*(pts2[1]['x']-pts2[0]['x']),
        y=pts2[0]['y']+0.999*(pts2[1]['y']-pts2[0]['y']),
        s=0.999*sg0_len, dist=0.0, seg_length=sg0_len,
        seg_heading=path2.segments[0][5], passed_end=False)
    la2 = find_lookahead_point(path2, proj2, ld)
    vs2 = {'idx': 0, 't': 0.999, 'px': proj2.x, 'py': proj2.y, 'dist': 0.0}
    v1_2 = v1_find_lookahead_point(pts2, vs2, ld)
    if la2 and v1_2:
        r &= check_relaxed('clamp.x', v1_2['x'], la2.x)
        r &= check_relaxed('clamp.y', v1_2['y'], la2.y)
        r &= check_relaxed('clamped', True, la2.clamped_to_vertex)

    # Cross-segment (t=0.9 on A1->B2->C3, ld extends past current segment)
    pts3 = make_route_points('A1', 'B2', 'C3')
    path3 = Path(pts3)
    sg03 = path3.segments[0][4]
    proj3 = Projection(seg_index=0, t=0.9,
        x=pts3[0]['x']+0.9*(pts3[1]['x']-pts3[0]['x']),
        y=pts3[0]['y']+0.9*(pts3[1]['y']-pts3[0]['y']),
        s=0.9*sg03, dist=0.0, seg_length=sg03,
        seg_heading=path3.segments[0][5], passed_end=False)
    la3 = find_lookahead_point(path3, proj3, ld)
    vs3 = {'idx': 0, 't': 0.9, 'px': proj3.x, 'py': proj3.y, 'dist': 0.0}
    v1_3 = v1_find_lookahead_point(pts3, vs3, ld)
    if la3 and v1_3:
        r &= check_relaxed('cross.x', v1_3['x'], la3.x)
        r &= check_relaxed('cross.y', v1_3['y'], la3.y)
    return r


def test_controller():
    print("--- Controller ---")
    r = True

    targets = [
        (1.0, 0.0, 0.0, 0.0, 0.0),
        (1.0, -0.3, 0.0, 0.0, 0.0),
        (0.5, 0.5, 0.0, 0.0, 0.0),
        (1.0, 1.0, 0.0, 0.0, math.pi/4),
        (10.0, 10.0, 0.0, 0.0, 0.0),
        (0.001, 0.001, 0.0, 0.0, 0.0),
    ]
    for tx, ty, px, py, pyaw in targets:
        tgt = {'x': tx, 'y': ty, 'z': 0.0, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1}
        v1_v, v1_s = v1_pure_pursuit_control(tgt, px, py, pyaw)
        cmd = compute_command(pose_x=px, pose_y=py, pose_yaw=pyaw,
                              target_x=tx, target_y=ty, gain=0.5)
        r &= check_relaxed(f's({tx},{ty})', v1_s, cmd.steer_deg)
        r &= check_relaxed(f'v({tx},{ty})', v1_v, cmd.linear_x)

    # None target
    v1_v0, v1_s0 = v1_pure_pursuit_control(None, 0, 0, 0)
    r &= check_relaxed('none.steer', 0.0, v1_s0)
    r &= check_relaxed('none.vel', 0.0, v1_v0)

    # Brake
    brake = brake_command()
    r &= check_relaxed('brake.vel', 0.0, brake.linear_x)
    r &= check_relaxed('brake.steer', 0.0, brake.steer_deg)

    # V1 compat function
    tgt = {'x': 1.0, 'y': 0.0, 'z': 0.0, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1}
    v1_v2, v1_s2 = pure_pursuit_control_v1(tgt, 0, 0, 0)
    v1_vr, v1_sr = v1_pure_pursuit_control(tgt, 0, 0, 0)
    r &= check_relaxed('v1_fn.steer', v1_sr, v1_s2)
    r &= check_relaxed('v1_fn.vel', v1_vr, v1_v2)

    # Rate limit
    cmd = compute_command(pose_x=0, pose_y=0, pose_yaw=0,
                          target_x=10, target_y=10, gain=1.0,
                          last_steer_deg=0.0, dt=0.05, max_steer_rate_deg_s=100.0)
    r &= check_relaxed('rate_limit', True, abs(cmd.steer_deg) <= 5.0+1e-9)
    return r


def test_ros_io():
    print("--- io/ros_io ---")
    r = True

    of = OdomFrame(raw_x=1.5, raw_y=0.5, qx=0, qy=0, qz=0, qw=1, ts=12345.0)
    r &= check_relaxed('odom.x', 1.5, of.raw_x)

    wm = WaypointMsg(data='G8 A10', ts=12345.0)
    r &= check_relaxed('wp.data', 'G8 A10', wm.data)

    im = IntMsg(data=42, ts=12345.0)
    r &= check_relaxed('int', 42, im.data)

    bridge = RosBridge(node=None)
    r &= check_relaxed('created', True, bridge is not None)

    # Manually set cache
    bridge._latest_odom = of
    latest = bridge.get_latest_odom()
    r &= check_relaxed('get_odom', True, latest is not None)
    if latest:
        r &= check_relaxed('get_odom.x', 1.5, latest.raw_x)

    r &= check_relaxed('shoot_default', 0, bridge.get_shoot_count())
    r &= check_relaxed('aim_default', 0, bridge.get_aim_status())
    r &= check_relaxed('wp_default', None, bridge.get_waypoint())

    bridge.shutdown(brake=False)
    r &= check_relaxed('shutdown', True, True)
    return r


def test_e2e_routes():
    """Route->projection->lookahead->controller V1<->V2 on many routes/poses."""
    print("--- Pipeline E2E: multiple routes ---")
    r = True
    ld = 0.15

    specs = [
        ['A1', 'N11'], ['G8', 'A10', 'D2'], ['G8', 'G1', 'A1'],
        ['M10', 'L10', 'K10', 'J10', 'I10'],
        ['A1', 'B2', 'C3', 'D4', 'E5'],
        ['N11', 'N10', 'N9', 'N8', 'N7'],
    ]

    for ri, blocks in enumerate(specs):
        pts = make_route_points(*blocks)
        path = Path(pts)
        for si in range(path.num_segments):
            sg = path.segments[si]
            for t in [0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0]:
                px = sg[0] + t*(sg[2]-sg[0])
                py = sg[1] + t*(sg[3]-sg[1])

                v1_seg = v1_find_nearest_segment(pts, 0, px, py)
                if not v1_seg: continue
                v1_tgt = v1_find_lookahead_point(pts, v1_seg, ld)
                if not v1_tgt: continue
                v1_v, v1_s = v1_pure_pursuit_control(v1_tgt, px, py, 0.0)

                proj = path.project(px, py, s_hint=0.0, window_m=0.0)
                if not proj: continue
                la = find_lookahead_point(path, proj, ld)
                cmd = compute_command(pose_x=px, pose_y=py, pose_yaw=0,
                                      target_x=la.x, target_y=la.y, gain=0.5)
                label = f'r{ri}_s{si}_t{t}'
                r &= check_relaxed(f'{label}.steer', v1_s, cmd.steer_deg)
                r &= check_relaxed(f'{label}.vel', v1_v, cmd.linear_x)
    return r


def test_random_poses():
    """200 random poses on a 12-point route."""
    print("--- Pipeline: random poses ---")
    r = True
    ld = 0.15
    pts = make_route_points('M10','L10','K10','J10','I10','H10',
                            'G10','F10','F11','E11','D10','C10')
    path = Path(pts)

    import random; random.seed(42)
    count = 0
    for _ in range(200):
        px, py, pyaw = random.uniform(0,4), random.uniform(0,5), random.uniform(-math.pi, math.pi)
        v1_seg = v1_find_nearest_segment(pts, 0, px, py)
        if not v1_seg: continue
        v1_tgt = v1_find_lookahead_point(pts, v1_seg, ld)
        if not v1_tgt: continue
        v1_v, v1_s = v1_pure_pursuit_control(v1_tgt, px, py, pyaw)
        proj = path.project(px, py, s_hint=0.0, window_m=0.0)
        if not proj: continue
        la = find_lookahead_point(path, proj, ld)
        cmd = compute_command(pose_x=px, pose_y=py, pose_yaw=pyaw,
                              target_x=la.x, target_y=la.y, gain=0.5)
        r &= check_relaxed(f'#{count}.steer', v1_s, cmd.steer_deg)
        r &= check_relaxed(f'#{count}.vel', v1_v, cmd.linear_x)
        count += 1
    print(f"   ({count} poses tested)")
    return r


def test_yaw_quaternion_rt():
    print("--- yaw<->quaternion round-trip ---")
    r = True
    for y in [0.0, 0.1, -0.1, math.pi/4, -math.pi/4, math.pi/2, -math.pi/2,
              math.pi, -math.pi, 1.5, -1.5, 3.14, -3.14, 10.0, -10.0]:
        q = yaw_to_quaternion(y)
        back = euler_from_quaternion(*q)
        # Round-trip returns normalized angle in [-pi, pi]
        expected = v1_normalize_angle(y)
        r &= check_relaxed(f'yaw={y:.4f}', expected, back, rel_tol=1e-9)
    return r


def test_heading_tokens():
    print("--- heading_token_to_yaw ---")
    r = True
    # heading_token_to_yaw is case-sensitive (matches V1 heading_to_quaternion)
    r &= check_relaxed('x', 0.0, heading_token_to_yaw('x'))
    r &= check_relaxed('y', math.pi/2, heading_token_to_yaw('y'))
    r &= check_relaxed('X', 0.0, heading_token_to_yaw('X'))
    r &= check_relaxed('Y', 0.0, heading_token_to_yaw('Y'))  # 'Y' uppercase: V1 maps to 0.0
    r &= check_relaxed('z', 0.0, heading_token_to_yaw('z'))
    r &= check_relaxed('empty', 0.0, heading_token_to_yaw(''))
    return r


def test_stall_skip():
    """Mission stall skip (requires task module)."""
    print("--- Stall skip ---")
    r = True
    try:
        from car_localization.tasks.mission import MissionTask
        bb = Blackboard()
        pts = make_route_points('N10', 'N9', 'N8', 'N7')
        bb.set(P_PATH_POINTS, pts, 't'); bb.set(P_PATH_LOADED, True, 't')
        bb.set(P_POSE_X, 0.525, 't'); bb.set(P_POSE_Y, 0.175, 't')
        bb.set('current_target_idx', 0, 't'); bb.set(P_CORNER_MODE, False, 't')

        mission = MissionTask(blackboard=bb, stall_timeout_s=0.001)
        for i in range(30):
            mission.tick()
            if i == 3:
                mission._stall_start_time = time.monotonic() - 100.0
        idx = bb.get('current_target_idx', -1)
        r &= check_relaxed('skip_advances', True, idx >= 1)

        # PIVOT prevents skip
        bb2 = Blackboard()
        bb2.set(P_PATH_POINTS, pts, 't'); bb2.set(P_PATH_LOADED, True, 't')
        bb2.set(P_POSE_X, 0.525, 't'); bb2.set(P_POSE_Y, 0.175, 't')
        bb2.set('current_target_idx', 0, 't'); bb2.set(P_CORNER_MODE, True, 't')
        mission2 = MissionTask(blackboard=bb2, stall_timeout_s=0.001)
        for _ in range(30):
            mission2.tick()
            mission2._stall_start_time = time.monotonic() - 100.0
        idx2 = bb2.get('current_target_idx', -1)
        r &= check_relaxed('pivot_no_skip', True, idx2 == 0)
    except ImportError as e:
        print(f"  [SKIP] {e}")
    except Exception as e:
        r = False; print(f"  [FAIL] {e}")
    return r


def verify_test_coords():
    print("--- test_coords.py ---")
    try:
        import subprocess
        # test_coords.py is self-contained; run from its own directory
        script_dir = os.path.join(os.path.dirname(__file__), '..', 'car_localization')
        r = subprocess.run(
            [sys.executable, 'test_coords.py'],
            timeout=30,
            cwd=script_dir,
            env={**os.environ, 'PYTHONIOENCODING': 'utf-8'})
        if r.returncode == 0:
            print("  [OK] test_coords.py passed")
            return True
        else:
            print(f"  [FAIL] test_coords.py exit={r.returncode}")
            return False
    except Exception as e:
        print(f"  [FAIL] test_coords.py exception: {e}")
        return False


def run_existing_tests():
    print("--- Existing test suites ---")
    r = True

    def _run(filename, label, cwd=None):
        try:
            import subprocess
            er = subprocess.run(
                [sys.executable, filename],
                timeout=60,
                cwd=cwd or os.path.join(os.path.dirname(__file__), '..'),
                env={**os.environ,
                     'PYTHONPATH': os.path.join(os.path.dirname(__file__), '..', 'car_localization'),
                     'PYTHONIOENCODING': 'utf-8'})
            if er.returncode == 0:
                print(f"  [OK] {label} passed")
                return True
            else:
                print(f"  [FAIL] {label} exit={er.returncode}")
                return False
        except Exception as e:
            print(f"  [FAIL] {label}: {e}")
            return False

    for fn, label in [
        (os.path.join(os.path.dirname(__file__), 'test_v1_v2_align.py'), 'test_v1_v2_align.py'),
        (os.path.join(os.path.dirname(__file__), '..', 'car_localization', 'tests', 'test_pursuit.py'), 'test_pursuit.py'),
        (os.path.join(os.path.dirname(__file__), '..', 'car_localization', 'tests', 'test_tasks.py'), 'test_tasks.py'),
    ]:
        ok = _run(fn, label)
        if not ok:
            r = False

    return r


# ======================== MAIN ========================

if __name__ == '__main__':
    print("=" * 70)
    print("FULL REGRESSION TEST SUITE")
    print("V2 Modules vs V1 path_executor  ::  rel_tol=1e-12  abs_tol=1e-15")
    print("=" * 70)

    all_ok = True

    groups = [
        ('Constants', test_constants),
        ('block_to_meters', test_block_to_meters),
        ('Quaternion', test_quaternion),
        ('normalize_angle', test_normalize_angle),
        ('Pose', test_pose),
        ('ang_diff', test_ang_diff),
        ('Blackboard', test_blackboard),
        ('Config', test_config),
        ('Route model', test_route_model),
        ('Parser', test_parser),
        ('Path projection', test_path_projection),
        ('Lookahead', test_lookahead),
        ('Controller', test_controller),
        ('io/ros_io', test_ros_io),
        ('E2E routes', test_e2e_routes),
        ('Random poses', test_random_poses),
        ('yaw<->quat RT', test_yaw_quaternion_rt),
        ('heading_tokens', test_heading_tokens),
        ('Stall skip', test_stall_skip),
    ]

    for name, fn in groups:
        try:
            if not fn():
                all_ok = False
                print(f"  >>> Group [{name}] FAILURES <<<")
        except Exception as e:
            all_ok = False
            import traceback
            print(f"  >>> Group [{name}] exception: {e}")
            traceback.print_exc()

    coords = verify_test_coords()
    existing = run_existing_tests()
    if not coords: all_ok = False
    if not existing: all_ok = False

    print("")
    print("=" * 70)
    print("REGRESSION SUMMARY")
    print("=" * 70)
    print(f"  New edge-case tests passed : {PASS}")
    print(f"  New edge-case tests failed : {FAIL}")
    print(f"  Existing test suites       : {'PASS' if existing else 'FAIL'}")
    print(f"  test_coords.py            : {'PASS' if coords else 'FAIL'}")
    print(f"  Overall                   : {'PASS' if all_ok else 'FAIL'}")
    print("=" * 70)

    if FAILURES:
        print(f"  Failures ({len(FAILURES)}):")
        for name, detail in FAILURES:
            print(f"    {name}: {detail}")

    if all_ok and PASS > 0:
        print(f"  [OK] FULL REGRESSION PASSED")
        sys.exit(0)
    else:
        print(f"  [FAIL] {FAIL} failures detected")
        sys.exit(1)
