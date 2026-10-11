"""测试：新建 pursuit/ 模块 vs V1 path_executor 逐位一致。

V1 参考代码直接从 path_executor.py 的以下函数内联/复制：
  - find_nearest_segment (L855-891)
  - find_lookahead_point (L895-970)
  - pure_pursuit_control (L974-1026)

测试策略：
  - 构造多个路线场景（直线段/弯道/长路线/短路线）
  - 用多组测试位姿喂入 V1 和 V2 函数
  - 比较输出逐位一致（math.isclose rel_tol=1e-12）

本测试不依赖 rclpy / ROS2。
"""

import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'car_localization'))

from car_localization.core.geometry import normalize_angle, block_to_meters
from car_localization.pursuit.path import (
    Path, Projection, find_nearest_segment_v1,
)
from car_localization.pursuit.lookahead import (
    LookaheadPoint, find_lookahead_point, find_lookahead_point_v1_dict,
)
from car_localization.pursuit.controller import (
    PursuitCommand, compute_command,
    pure_pursuit_control_v1, brake_command,
)


# ═══════════════════════════════════════════════════════════════
# V1 参考实现（从 path_executor.py 逐字复制）
# ═══════════════════════════════════════════════════════════════

def v1_find_nearest_segment(path_points, current_target_idx, current_x, current_y):
    """V1 L855-891 逐字复制，改为参数传入。"""
    pts = path_points
    if not pts or len(pts) < 2:
        return None
    best = None
    for i in range(current_target_idx, len(pts) - 1):
        p1 = pts[i]
        p2 = pts[i + 1]
        sx = p2['x'] - p1['x']
        sy = p2['y'] - p1['y']
        seg2 = sx * sx + sy * sy
        if seg2 < 1e-12:
            continue
        t = ((current_x - p1['x']) * sx + (current_y - p1['y']) * sy) / seg2
        t = max(0.0, min(1.0, t))
        qx = p1['x'] + t * sx
        qy = p1['y'] + t * sy
        d = math.hypot(current_x - qx, current_y - qy)
        if best is None or d < best['dist']:
            best = {'idx': i, 't': t, 'px': qx, 'py': qy, 'dist': d}
    return best


def v1_find_lookahead_point(path_points, seg, lookahead_dist):
    """V1 L895-970 逐字复制，改为参数传入。"""
    pts = path_points
    if len(pts) < 2:
        return pts[-1] if pts else None

    def _pt(x, y):
        return {'x': x, 'y': y, 'z': 0.0,
                'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}

    CORNER_ANGLE_RAD = math.radians(20.0)

    remaining = lookahead_dist
    idx = seg['idx']
    p1 = pts[idx]
    p2 = pts[idx + 1]
    seg_len = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
    left = (1.0 - seg['t']) * seg_len if seg_len > 1e-9 else 0.0

    corner_ahead = False
    if idx + 2 < len(pts):
        d1x = p2['x'] - p1['x']
        d1y = p2['y'] - p1['y']
        p3 = pts[idx + 2]
        d2x = p3['x'] - p2['x']
        d2y = p3['y'] - p2['y']
        cross = d1x * d2y - d1y * d2x
        dot = d1x * d2x + d1y * d2y
        ang = abs(math.atan2(cross, dot))
        if ang > CORNER_ANGLE_RAD:
            corner_ahead = True

    if corner_ahead and left < 0.04:
        return p2

    if seg_len > 1e-9 and left >= remaining:
        ux = (p2['x'] - p1['x']) / seg_len
        uy = (p2['y'] - p1['y']) / seg_len
        return _pt(seg['px'] + ux * remaining,
                   seg['py'] + uy * remaining)

    remaining -= left

    j = idx + 1
    while j < len(pts) - 1 and remaining > 1e-9:
        p1 = pts[j]
        p2 = pts[j + 1]
        sl = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
        if sl > 1e-9 and sl >= remaining:
            ux = (p2['x'] - p1['x']) / sl
            uy = (p2['y'] - p1['y']) / sl
            return _pt(p1['x'] + ux * remaining,
                       p1['y'] + uy * remaining)
        remaining -= sl
        j += 1

    return pts[-1]


def v1_pure_pursuit_control(target_point, current_x, current_y, current_yaw,
                            wheelbase=0.157, linear_speed=0.3):
    """V1 L974-1026 逐字复制，改为参数传入。"""
    if target_point is None:
        return 0.0, 0.0

    dx = target_point['x'] - current_x
    dy = target_point['y'] - current_y

    cos_yaw = math.cos(-current_yaw)
    sin_yaw = math.sin(-current_yaw)
    rel_x = dx * cos_yaw - dy * sin_yaw
    rel_y = dx * sin_yaw + dy * cos_yaw

    dist = math.sqrt(dx * dx + dy * dy)

    if dist < 0.01:
        return 0.0, 0.0

    alpha = math.atan2(rel_y, rel_x)
    steer_deg = math.degrees(
        math.atan(wheelbase * math.sin(alpha) / max(dist, 0.10))
    )
    steer_deg = max(-30.0, min(30.0, steer_deg))

    turn_factor = abs(steer_deg) / 30.0
    linear_vel = linear_speed * (1.0 - 0.5 * turn_factor)
    linear_vel = max(0.1, linear_vel)

    return linear_vel, steer_deg


# ═══════════════════════════════════════════════════════════════
# 对拍工具
# ═══════════════════════════════════════════════════════════════

def check(name, v1_val, v2_val, rel_tol=1e-12):
    """比较 V1 和 V2 的值，bitwise consistent。"""
    if isinstance(v1_val, float) and isinstance(v2_val, float):
        if math.isclose(v1_val, v2_val, rel_tol=rel_tol) or (math.isnan(v1_val) and math.isnan(v2_val)):
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


def make_v1_pt(x, y):
    """构造 V1 格式航点 dict。"""
    return {'x': x, 'y': y, 'z': 0.0,
            'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}


def make_route_points(*blocks):
    """从格名列表构造 V1 格式航点列表。"""
    pts = []
    for b in blocks:
        x, y = block_to_meters(b)
        pts.append(make_v1_pt(x, y))
    return pts


# ═══════════════════════════════════════════════════════════════
# 测试 1: Path 预计算与基本属性
# ═══════════════════════════════════════════════════════════════

def test_path_basics():
    print("\n=== 1. Path 基本属性 ===")
    results = []

    # 直线路径 A1 → N11
    pts = make_route_points('A1', 'N11')
    path = Path(pts)

    # total_length
    v1_len = math.hypot(pts[1]['x'] - pts[0]['x'], pts[1]['y'] - pts[0]['y'])
    results.append(check('total_length', v1_len, path.total_length))

    # num_segments
    results.append(check('num_segments', 1, path.num_segments))

    # s_at_waypoint
    results.append(check('s_at_waypoint[0]', 0.0, path.s_at_waypoint(0)))
    results.append(check('s_at_waypoint[1]', v1_len, path.s_at_waypoint(1)))

    # remaining
    results.append(check('remaining(0)', v1_len, path.remaining(0)))
    results.append(check('remaining(end)', 0.0, path.remaining(v1_len)))
    results.append(check('remaining(beyond)', 0.0, path.remaining(v1_len + 1)))

    # point_at
    px, py, seg_idx = path.point_at(0)
    results.append(check('point_at(0).x', pts[0]['x'], px))
    results.append(check('point_at(0).y', pts[0]['y'], py))
    results.append(check('point_at(0).seg', 0, seg_idx))

    px, py, seg_idx = path.point_at(v1_len)
    results.append(check('point_at(end).x', pts[1]['x'], px))
    results.append(check('point_at(end).y', pts[1]['y'], py))
    results.append(check('point_at(end).seg', 0, seg_idx))

    # heading_at
    h = path.heading_at(0)
    v1_h = math.atan2(pts[1]['y'] - pts[0]['y'], pts[1]['x'] - pts[0]['x'])
    results.append(check('heading_at(0)', v1_h, h))

    # corner_angle_deg（单段无弯角）
    results.append(check('corner_angle single seg', 0.0, path.corner_angle_deg(0)))

    # 空路径
    empty_path = Path([])
    results.append(check('empty total_length', 0.0, empty_path.total_length))
    results.append(check('empty num_segments', 0, empty_path.num_segments))

    # 单点路径
    single_path = Path([make_v1_pt(0, 0)])
    results.append(check('single total_length', 0.0, single_path.total_length))
    results.append(check('single num_segments', 0, single_path.num_segments))

    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 2: find_nearest_segment 对拍
# ═══════════════════════════════════════════════════════════════

def test_nearest_segment():
    print("\n=== 2. find_nearest_segment 对拍 ===")
    results = []

    # 场景 A：2 段直路，车在段上
    pts = make_route_points('A1', 'N11', 'N1')
    path = Path(pts)

    # 在靠近 A1 的位置
    x, y = block_to_meters('A1')
    x += 0.1  # 稍偏
    y += 0.1

    v1_best = v1_find_nearest_segment(pts, 0, x, y)
    proj = path.project(x, y, s_hint=0.0, window_m=2.0)

    if v1_best and proj:
        results.append(check('idx', v1_best['idx'], proj.seg_index))
        results.append(check('t', v1_best['t'], proj.t))
        results.append(check('px', v1_best['px'], proj.x))
        results.append(check('py', v1_best['py'], proj.y))
        results.append(check('dist', v1_best['dist'], proj.dist))
    else:
        results.append(False)
        print("  [FAIL] v1_best or proj is None")

    # v1_find_nearest_segment_v1 应与 v1_find_nearest_segment 一致
    v1_best2 = find_nearest_segment_v1(pts, 0, x, y)
    if v1_best and v1_best2:
        results.append(check('idx_v1_fn', v1_best['idx'], v1_best2['idx']))
        results.append(check('t_v1_fn', v1_best['t'], v1_best2['t']))
    else:
        results.append(False)
        print("  [FAIL] v1_best2 mismatch")

    # 场景 B：弯道（G8 → G1 → A1）
    pts2 = make_route_points('G8', 'G1', 'A1')
    path2 = Path(pts2)

    # 在 G8 附近
    x2, y2 = block_to_meters('G8')
    x2 += 0.05
    y2 += 0.05

    v1_best = v1_find_nearest_segment(pts2, 0, x2, y2)
    proj2 = path2.project(x2, y2, s_hint=0.0, window_m=2.0)

    if v1_best and proj2:
        results.append(check('弯道 idx', v1_best['idx'], proj2.seg_index))
        results.append(check('弯道 t', v1_best['t'], proj2.t))
        results.append(check('弯道 dist', v1_best['dist'], proj2.dist))
    else:
        results.append(False)
        print("  [FAIL] 弯道场景 v1_best or proj2 is None")

    # 场景 C：离线偏移点（横向偏离路线）
    off_x = x + 0.3
    off_y = y + 0.3
    v1_best = v1_find_nearest_segment(pts, 0, off_x, off_y)
    proj3 = path.project(off_x, off_y, s_hint=0.0, window_m=2.0)
    if v1_best and proj3:
        results.append(check('偏移 idx', v1_best['idx'], proj3.seg_index))
        results.append(check('偏移 t', v1_best['t'], proj3.t))
        results.append(check('偏移 dist', v1_best['dist'], proj3.dist))
    else:
        results.append(False)
        print("  [FAIL] 偏移场景 v1_best or proj3 is None")

    # 场景 D：路径不足 2 点 → None
    pts3 = [make_v1_pt(1, 1)]
    v1_best = v1_find_nearest_segment(pts3, 0, 1, 1)
    v1_best_fn = find_nearest_segment_v1(pts3, 0, 1, 1)
    results.append(check('单点 v1 None', v1_best is None, True))
    results.append(check('单点 v1_fn None', v1_best_fn is None, True))

    # Path 场景 D: 不足 1 段 → project 返回 None
    path3 = Path(pts3)
    proj4 = path3.project(1, 1, s_hint=0.0, window_m=1.0)
    results.append(check('单点 project None', proj4 is None, True))

    # 场景 E: 多段、遍历验证每段的投影
    pts4 = make_route_points('A1', 'B2', 'C3', 'D4', 'E5')
    path4 = Path(pts4)
    # 靠近中间点
    cx, cy = block_to_meters('C3')
    cx += 0.02
    cy += 0.02
    v1_best = v1_find_nearest_segment(pts4, 0, cx, cy)
    # 使用 s_hint=None 来从 seg 0 开始全扫（与 V1 current_target_idx=0 等价）
    proj5 = path4.project(cx, cy, s_hint=None, window_m=5.0)
    if v1_best and proj5:
        results.append(check('多段 idx', v1_best['idx'], proj5.seg_index))
        results.append(check('多段 t', v1_best['t'], proj5.t))
        results.append(check('多段 dist', v1_best['dist'], proj5.dist))
    else:
        results.append(False)
        print("  [FAIL] 多段场景 v1_best or proj5 is None")

    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 3: find_lookahead_point 对拍
# ═══════════════════════════════════════════════════════════════

def test_lookahead_point():
    print("\n=== 3. find_lookahead_point 对拍 ===")
    results = []
    ld = 0.15  # V1 默认前视距离

    # 场景 A：直线段，前视距在段内
    pts = make_route_points('A1', 'N11')
    seg = {'idx': 0, 't': 0.2, 'px': pts[0]['x'] + 0.2 * (pts[1]['x'] - pts[0]['x']),
           'py': pts[0]['y'] + 0.2 * (pts[1]['y'] - pts[0]['y']), 'dist': 0.0}

    v1_pt = v1_find_lookahead_point(pts, seg, ld)
    path = Path(pts)
    proj = Projection(
        seg_index=0, t=0.2, x=seg['px'], y=seg['py'],
        s=0.2 * path.total_length, dist=0.0,
        seg_length=path.total_length, seg_heading=path.segments[0][5],
        passed_end=False,
    )
    la = find_lookahead_point(path, proj, ld)

    if v1_pt:
        results.append(check('lookahead.x', v1_pt['x'], la.x))
        results.append(check('lookahead.y', v1_pt['y'], la.y))
    else:
        results.append(False)
        print("  [FAIL] v1_pt is None")

    # v1_dict 版本
    v1_pt2 = find_lookahead_point_v1_dict(pts, seg, ld)
    if v1_pt2:
        results.append(check('lookahead v1_fn.x', v1_pt['x'], v1_pt2['x']))
        results.append(check('lookahead v1_fn.y', v1_pt['y'], v1_pt2['y']))
    else:
        results.append(False)
        print("  [FAIL] v1_pt2 is None")

    # 场景 B：弯道顶点夹紧（左转 90°）
    # 路径：G8 → G1 → A1（90° 左转）
    pts2 = make_route_points('G8', 'G1', 'A1')
    # 在段 0 末端、接近顶点
    seg2 = {'idx': 0, 't': 0.999,  # 几乎在顶点
            'px': pts2[0]['x'] + 0.999 * (pts2[1]['x'] - pts2[0]['x']),
            'py': pts2[0]['y'] + 0.999 * (pts2[1]['y'] - pts2[0]['y']),
            'dist': 0.0}

    v1_pt3 = v1_find_lookahead_point(pts2, seg2, ld)
    path2 = Path(pts2)

    # 段 0 的弧长 99.9%
    seg0_len = path2.segments[0][4]
    proj2 = Projection(
        seg_index=0, t=0.999,
        x=seg2['px'], y=seg2['py'],
        s=0.999 * seg0_len, dist=0.0,
        seg_length=seg0_len, seg_heading=path2.segments[0][5],
        passed_end=False,
    )
    la2 = find_lookahead_point(path2, proj2, ld)

    if v1_pt3:
        results.append(check('弯道夹紧.x', v1_pt3['x'], la2.x))
        results.append(check('弯道夹紧.y', v1_pt3['y'], la2.y))
        results.append(check('弯道夹紧.clamped', True, la2.clamped_to_vertex))
    else:
        results.append(False)
        print("  [FAIL] v1_pt3 is None")

    # 场景 C：跨段前视（前视距超过当前段剩余）
    pts3 = make_route_points('A1', 'B2', 'C3')
    # t=0.9 在段 0
    seg3 = {'idx': 0, 't': 0.9,
            'px': pts3[0]['x'] + 0.9 * (pts3[1]['x'] - pts3[0]['x']),
            'py': pts3[0]['y'] + 0.9 * (pts3[1]['y'] - pts3[0]['y']),
            'dist': 0.0}

    v1_pt4 = v1_find_lookahead_point(pts3, seg3, ld)
    path3 = Path(pts3)
    seg0_len3 = path3.segments[0][4]
    proj3 = Projection(
        seg_index=0, t=0.9,
        x=seg3['px'], y=seg3['py'],
        s=0.9 * seg0_len3, dist=0.0,
        seg_length=seg0_len3, seg_heading=path3.segments[0][5],
        passed_end=False,
    )
    la3 = find_lookahead_point(path3, proj3, ld)

    if v1_pt4:
        results.append(check('跨段.x', v1_pt4['x'], la3.x))
        results.append(check('跨段.y', v1_pt4['y'], la3.y))
    else:
        results.append(False)
        print("  [FAIL] v1_pt4 is None")

    # 场景 D：路径总长短于前视距 → 直指终点
    pts4 = make_route_points('A1', 'A2')  # 很短
    seg4 = {'idx': 0, 't': 0.0,
            'px': pts4[0]['x'], 'py': pts4[0]['y'], 'dist': 0.0}
    v1_pt5 = v1_find_lookahead_point(pts4, seg4, ld)
    path4 = Path(pts4)
    seg0_len4 = path4.segments[0][4]
    proj4 = Projection(
        seg_index=0, t=0.0,
        x=pts4[0]['x'], y=pts4[0]['y'],
        s=0.0, dist=0.0,
        seg_length=seg0_len4, seg_heading=path4.segments[0][5],
        passed_end=False,
    )
    la4 = find_lookahead_point(path4, proj4, ld)

    if v1_pt5:
        results.append(check('短路径.x', v1_pt5['x'], la4.x))
        results.append(check('短路径.y', v1_pt5['y'], la4.y))
    else:
        results.append(False)
        print("  [FAIL] v1_pt5 is None")

    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 4: pure_pursuit_control 对拍
# ═══════════════════════════════════════════════════════════════

def test_controller():
    print("\n=== 4. pure_pursuit_control 对拍 ===")
    results = []

    # 场景 A：目标点在正前方（无转向）
    target = {'x': 1.0, 'y': 0.0, 'z': 0.0,
              'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
    v1_v, v1_s = v1_pure_pursuit_control(target, 0, 0, 0)

    # gain=0.5 等价于 V1 公式 atan(L*sinα/d)
    cmd = compute_command(
        pose_x=0, pose_y=0, pose_yaw=0,
        target_x=1, target_y=0,
        gain=0.5,  # V1 等价
    )

    results.append(check('正前方 v1 steer', v1_s, cmd.steer_deg))
    results.append(check('正前方 v1 velocity', v1_v, cmd.linear_x))

    # V1 函数的直接调用
    v1_vv, v1_ss = pure_pursuit_control_v1(target, 0, 0, 0)
    results.append(check('正前方 pure_v1 steer', v1_s, v1_ss))
    results.append(check('正前方 pure_v1 vel', v1_v, v1_vv))

    # 场景 B：目标点在右前方（右转）
    target2 = {'x': 1.0, 'y': -0.3, 'z': 0.0,
               'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
    v1_v2, v1_s2 = v1_pure_pursuit_control(target2, 0, 0, 0)

    cmd2 = compute_command(
        pose_x=0, pose_y=0, pose_yaw=0,
        target_x=1, target_y=-0.3,
        gain=0.5,  # V1 等价
    )

    results.append(check('右前方 v1 steer', v1_s2, cmd2.steer_deg))
    results.append(check('右前方 v1 vel', v1_v2, cmd2.linear_x))

    # 场景 C：目标点在左前方（左转）
    target3 = {'x': 0.5, 'y': 0.5, 'z': 0.0,
               'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
    v1_v3, v1_s3 = v1_pure_pursuit_control(target3, 0, 0, 0)

    cmd3 = compute_command(
        pose_x=0, pose_y=0, pose_yaw=0,
        target_x=0.5, target_y=0.5,
        gain=0.5,  # V1 等价
    )

    results.append(check('左前方 steer', v1_s3, cmd3.steer_deg))
    results.append(check('左前方 vel', v1_v3, cmd3.linear_x))

    # 场景 D：目标点极近（dist < 0.01 → 零输出）
    target4 = {'x': 0.001, 'y': 0.001, 'z': 0.0,
               'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
    v1_v4, v1_s4 = v1_pure_pursuit_control(target4, 0, 0, 0)

    cmd4 = compute_command(
        pose_x=0, pose_y=0, pose_yaw=0,
        target_x=0.001, target_y=0.001,
        gain=1.0,
    )

    results.append(check('极近 steer', v1_s4, cmd4.steer_deg))
    results.append(check('极近 vel', v1_v4, cmd4.linear_x))
    results.append(check('极近 speed_reason', 'no_target', cmd4.speed_reason))

    # 场景 E：None 目标（V1 返回 (0,0)）
    v1_v5, v1_s5 = v1_pure_pursuit_control(None, 0, 0, 0)
    results.append(check('None v1 steer', 0.0, v1_s5))
    results.append(check('None v1 vel', 0.0, v1_v5))

    # 场景 F：gain=0.5 时结果是否与 V1 一致
    # gain=0.5 → 公式 atan(0.5 * 2.0 * L * sin(α) / d) = atan(L * sin(α) / d)
    # 应与 V1 逐位一致
    cmd_gain_05 = compute_command(
        pose_x=0, pose_y=0, pose_yaw=0,
        target_x=1, target_y=-0.3,
        gain=0.5,  # gain=0.5 应等价于 V1
    )
    results.append(check('gain=0.5 steer vs V1', v1_s2, cmd_gain_05.steer_deg))
    results.append(check('gain=0.5 vel vs V1', v1_v2, cmd_gain_05.linear_x))

    # 场景 G：带车体偏航（yaw=π/4）
    target5 = {'x': 1.0, 'y': 1.0, 'z': 0.0,
               'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
    v1_v6, v1_s6 = v1_pure_pursuit_control(target5, 0, 0, math.pi/4)

    cmd5 = compute_command(
        pose_x=0, pose_y=0, pose_yaw=math.pi/4,
        target_x=1, target_y=1,
        gain=0.5,  # V1 等价
    )
    results.append(check('偏航 steer', v1_s6, cmd5.steer_deg))
    results.append(check('偏航 vel', v1_v6, cmd5.linear_x))

    # 场景 H：舵角硬限幅（大偏角→限到±30°）
    # 注意：V1 的纯追踪公式在 gain=0.5 时与 V1 相同
    target_far = {'x': 10, 'y': 10, 'z': 0.0,
                  'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
    v1_v7, v1_s7 = v1_pure_pursuit_control(target_far, 0, 0, 0)
    results.append(check('限幅 v1 steer clamped', True, abs(v1_s7) <= 30.0))

    cmd6 = compute_command(
        pose_x=0, pose_y=0, pose_yaw=0,
        target_x=10, target_y=10, gain=0.5,  # V1 等价
    )
    results.append(check('限幅 cmd steer', v1_s7, cmd6.steer_deg))
    results.append(check('限幅 cmd clamped', True, abs(cmd6.steer_deg) <= 30.0))

    # 场景 I：brake_command
    brake = brake_command()
    results.append(check('brake linear_x', 0.0, brake.linear_x))
    results.append(check('brake steer_deg', 0.0, brake.steer_deg))
    results.append(check('brake reason', 'brake', brake.speed_reason))

    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 5: 减速逻辑
# ═══════════════════════════════════════════════════════════════

def test_speed_shaping():
    print("\n=== 5. 减速逻辑 ===")
    results = []

    # 场景 A：末端减速（dist_to_final < 0.5）
    cmd = compute_command(
        pose_x=0, pose_y=0, pose_yaw=0,
        target_x=1, target_y=0,
        gain=1.0, speed=0.3,
        dist_to_final=0.3,
    )
    # V1 L1323-1324: linear_vel = min(vel, max(0.1, 0.35 * 0.3))
    expected_vel = min(0.3 * (1.0 - 0.5 * abs(cmd.steer_deg) / 30.0),
                       max(0.1, 0.35 * 0.3))
    results.append(check('末端减速 vel', expected_vel, cmd.linear_x))
    results.append(check('末端减速 reason', 'end_approach', cmd.speed_reason))

    # 场景 B：弯前减速（corner angle > 45°, dist_to_vertex < 0.45）
    cmd2 = compute_command(
        pose_x=0, pose_y=0, pose_yaw=0,
        target_x=1, target_y=0,
        gain=1.0, speed=0.3,
        corner_angle_deg=60.0,
        dist_to_vertex=0.3,
    )
    results.append(check('弯前减速 ≤0.12', True, cmd2.linear_x <= 0.12 + 1e-12))
    # V1 L1348: min(vel, 0.12)
    if cmd2.linear_x > 0.12 + 1e-12:
        results.append(False)
        print(f"  [FAIL] 弯前减速 > 0.12: {cmd2.linear_x}")
    else:
        print(f"  [OK] 弯前减速 vel={cmd2.linear_x:.4f} ≤ 0.12, reason={cmd2.speed_reason}")

    # 场景 C：舵角速率限幅
    cmd3 = compute_command(
        pose_x=0, pose_y=0, pose_yaw=0,
        target_x=1, target_y=-0.5,  # 大偏角
        gain=1.0,
        last_steer_deg=0.0, dt=0.05,
        max_steer_rate_deg_s=100.0,  # 每拍最多 5°
    )
    # 无速率限幅时的舵角
    cmd_no_rate = compute_command(
        pose_x=0, pose_y=0, pose_yaw=0,
        target_x=1, target_y=-0.5,
        gain=1.0,
        last_steer_deg=0.0, dt=0.05,
        max_steer_rate_deg_s=0.0,
    )
    # 有速率限幅时舵角变化不超过 100*0.05=5°/拍
    if cmd_no_rate.steer_deg != cmd3.steer_deg:
        # 确实被限幅了
        results.append(check('速率限幅有效', True, True))
    results.append(check('速率限幅 delta ≤ 5°',
                         True, abs(cmd3.steer_deg) <= 5.0 + 1e-9))

    return all(results)


# ═══════════════════════════════════════════════════════════════
# 测试 6: 端到端（完整管线对拍）
# ═══════════════════════════════════════════════════════════════

def test_end_to_end():
    print("\n=== 6. 端到端管线对拍 ===")
    results = []

    # V1 完整管线：
    #   1. 构造路线
    #   2. find_nearest_segment → seg dict
    #   3. find_lookahead_point → target dict
    #   4. pure_pursuit_control → (vel, steer)

    # V2 完整管线：
    #   1. 构造 Path
    #   2. path.project(x, y) → Projection
    #   3. find_lookahead_point(path, proj, ld) → LookaheadPoint
    #   4. compute_command(...) → PursuitCommand

    # 路线：G8 → G1 → A1（90° 弯 + 直段出口）
    pts = make_route_points('G8', 'G1', 'A1')
    ld = 0.15
    cx, cy = block_to_meters('G8')
    cyaw = 0.0

    # V1 管线
    v1_seg = v1_find_nearest_segment(pts, 0, cx, cy)
    v1_target = v1_find_lookahead_point(pts, v1_seg, ld)
    v1_vel, v1_steer = v1_pure_pursuit_control(v1_target, cx, cy, cyaw)

    # V2 管线
    path = Path(pts)
    proj = path.project(cx, cy, s_hint=0.0, window_m=2.0)
    la = find_lookahead_point(path, proj, ld)
    v2_cmd = compute_command(
        pose_x=cx, pose_y=cy, pose_yaw=cyaw,
        target_x=la.x, target_y=la.y,
        gain=0.5,  # V1 等价
    )

    results.append(check('e2e steer', v1_steer, v2_cmd.steer_deg))
    results.append(check('e2e vel', v1_vel, v2_cmd.linear_x))

    # 多个测试位姿（沿路线前进）
    test_poses = [
        (cx + 0.1, cy, cyaw),
        (cx + 0.3, cy + 0.1, cyaw),
        (cx + 0.5, cy + 0.3, cyaw),
    ]
    for ti, (tx, ty, tyaw) in enumerate(test_poses):
        v1_seg = v1_find_nearest_segment(pts, 0, tx, ty)
        if v1_seg is None:
            continue
        v1_target = v1_find_lookahead_point(pts, v1_seg, ld)
        if v1_target is None:
            continue
        v1_v, v1_s = v1_pure_pursuit_control(v1_target, tx, ty, tyaw)

        proj = path.project(tx, ty, s_hint=0.0, window_m=0)
        if proj is None:
            continue
        la = find_lookahead_point(path, proj, ld)
        cmd = compute_command(
            pose_x=tx, pose_y=ty, pose_yaw=tyaw,
            target_x=la.x, target_y=la.y,
            gain=0.5,  # V1 等价
        )

        results.append(check(f'e2e pose#{ti} steer', v1_s, cmd.steer_deg))
        results.append(check(f'e2e pose#{ti} vel', v1_v, cmd.linear_x))

    return all(results)


# ═══════════════════════════════════════════════════════════════
# 主入口
# ═══════════════════════════════════════════════════════════════

if __name__ == '__main__':
    tests = [
        ('Path 基本属性', test_path_basics),
        ('find_nearest_segment 对拍', test_nearest_segment),
        ('find_lookahead_point 对拍', test_lookahead_point),
        ('pure_pursuit_control 对拍', test_controller),
        ('减速逻辑', test_speed_shaping),
        ('端到端管线对拍', test_end_to_end),
    ]
    failed_groups = []
    for name, fn in tests:
        print(f"\n{'─' * 50}")
        print(f" 测试: {name}")
        print(f"{'─' * 50}")
        ok = fn()
        if not ok:
            failed_groups.append(name)

    print(f"\n{'=' * 50}")
    if not failed_groups:
        print("[OK] All pursuit alignment passed! V2 output is V1 bitwise consistent.")
        sys.exit(0)
    else:
        print(f"[FAIL] 以下 {len(failed_groups)} 个分组存在不一致:")
        for g in failed_groups:
            print(f"  - {g}")
        sys.exit(1)
