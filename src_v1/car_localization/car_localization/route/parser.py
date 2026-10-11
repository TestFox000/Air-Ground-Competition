"""route/parser.py — Token 解析纯函数。

从 path_executor.py V1 L607-732（follow_route_command）拆出：
  - token 规范化、合法性校验
  - 块名解析、去重、航向计算
  - 纯函数（无 self 访问），输入 str → 输出 Route

语义与 V1 完全相同：block_to_meters, 尾随 *, ,x/,y 解析, 接近段插入, 四元数回填
"""

import math
import re
from typing import Dict, List, Optional, Tuple

from ..core.geometry import (
    BLOCK_SIZE, COL_LETTERS, block_to_meters, normalize_angle,
)
from .model import Route, RouteBuilder, Waypoint


# ── 常量 ──────────────────────────────────────────────────

REVERSE_PREFIX = 'R:'
PIVOT_SUFFIX = ':PIVOT'
APPROACH_DIST = 0.6  # V1 self.approach_dist 的默认值


# ── Token 解析 ─────────────────────────────────────────────

def normalize_token(token: str) -> str:
    """规范化 token：去除首尾空格 + 大写 + 标准化 'R:' 前缀和 ':PIVOT' 后缀。"""
    t = token.strip().upper()
    # 保持 *、R:、:PIVOT 后缀不变
    return t


def parse_token(token: str) -> dict:
    """解析单个 route token → 命名 dict。

    Args:
        token: 如 'G8', 'D2*', 'R:D5', 'C3:PIVOT'

    Returns:
        dict with keys: block, reverse, shoot, scan, heading, group, pivot
    """
    t = token.strip().upper()
    reverse = False
    shoot = False
    pivot = False

    # 去掉倒车前缀 R:
    if t.startswith(REVERSE_PREFIX):
        reverse = True
        t = t[len(REVERSE_PREFIX):]

    # 去掉折返后缀 :PIVOT
    if t.endswith(PIVOT_SUFFIX):
        pivot = True
        t = t[:-len(PIVOT_SUFFIX)]

    # 去掉射击标记 *
    if t.endswith('*'):
        shoot = True
        t = t[:-1]

    block = t

    return {
        'block': block,
        'reverse': reverse,
        'shoot': shoot,
        'scan': True,       # 路线点位一律触发一次全向扫描（V1 L658）
        'heading': None,    # 不从 token 推断，由调用方决定
        'group': 0,
        'pivot': pivot,
    }


def _split_block_suffix(data: str) -> Tuple[str, Optional[str]]:
    """从命令体中分离尾部 ,x/,y 航向指定。

    对应 V1 L618-626。

    Returns:
        (body, final_heading)  — final_heading 为 'x'/'y'/None
    """
    final_heading = None
    body = data
    if ',' in data:
        body, tail = data.rsplit(',', 1)
        final_heading = tail.strip().lower()
        if final_heading not in ('x', 'y'):
            raise ValueError(f"Invalid heading: '{tail}'. Use 'x' or 'y'")
    return body, final_heading


def _dedup_waypoints(wps: List[Waypoint], shoot: bool) -> bool:
    """去重逻辑（V1 L650-654）。

    如果 wps[-1] 与当前坐标相同，标记射击标志并返回 True。
    """
    if len(wps) >= 1:
        last = wps[-1]
        if abs(last.x - wps[-1].x) < 1e-9 and abs(last.y - wps[-1].y) < 1e-9:
            return True
    return False


def _compute_yaw_and_approach(
    points: List[dict],
    final_heading: Optional[str],
    current_yaw: float,
    approach_dist: float = 0.6,
) -> Tuple[float, Optional[dict], Optional[dict]]:
    """计算最后点航向 + 决定是否插入接近段。

    对应 V1 L664-698。

    Returns:
        (final_yaw, approach_point, end_point)
        approach_point 为 None 时表示不插入接近段。
    """
    # 最后点航向
    if len(points) >= 2:
        last_yaw = math.atan2(points[-1]['y'] - points[-2]['y'],
                              points[-1]['x'] - points[-2]['x'])
    else:
        last_yaw = current_yaw

    if final_heading == 'y':
        final_yaw = math.pi / 2
    elif final_heading == 'x':
        final_yaw = 0.0
    else:
        final_yaw = last_yaw

    approach_point = None
    if final_heading is not None \
            and abs(math.cos(last_yaw - final_yaw)) < 0.999:
        axis_x, axis_y = (1.0, 0.0) if final_heading == 'x' else (0.0, 1.0)
        tx, ty = points[-1]['x'], points[-1]['y']
        app_x = max(0.05, tx - axis_x * approach_dist)
        app_y = max(0.05, ty - axis_y * approach_dist)
        if abs(app_x - tx) > 1e-9 or abs(app_y - ty) > 1e-9:
            approach_point = {
                'x': app_x, 'y': app_y, 'z': 0.0,
                'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0,
            }

    return final_yaw, approach_point, points[-1]


def _backfill_quaternions(points: List[dict], final_yaw: float):
    """回填各点四元数。

    对应 V1 L704-711。
    中间点 = 指向下一个点的方向；末点 = final_yaw。
    """
    for i, p in enumerate(points):
        if i + 1 < len(points):
            yw = math.atan2(points[i + 1]['y'] - p['y'],
                            points[i + 1]['x'] - p['x'])
        else:
            yw = final_yaw
        p['qz'] = math.sin(yw / 2.0)
        p['qw'] = math.cos(yw / 2.0)


def parse_mission(tokens: List[str],
                  shoot_planner: Optional[str] = None,
                  name: str = '',
                  current_yaw: float = 0.0,
                  approach_dist: float = 0.6) -> Route:
    """解析 V1 风格路线 token 列表为 Route。

    纯函数版本对应 V1 L607-732。

    Args:
        tokens: V1 token 列表，如 ['G8', 'A10', 'D2*', 'E2,y']
                如果提供的是空格分隔的字符串，会自动拆分
        shoot_planner: 保留参数（V2 中可选择动态射击规划器）
        name: 路线名
        current_yaw: 当前位置 yaw（用于单点路径）
        approach_dist: 末端接近段长度

    Returns:
        Route 对象
    """
    # 字符串 → token 列表
    if isinstance(tokens, str):
        tokens = tokens.split()

    # 解析尾部 ,x/,y
    # 合并为一个字符串再解析
    raw_data = ' '.join(tokens)
    body, final_heading = _split_block_suffix(raw_data)
    token_list = body.split()

    # 构建点列表（V1 L632-658 逻辑）
    points = []        # waypoint list (metric)，每个点 dict
    shoot_flags = []   # parallel shoot flag
    scan_flags = []    # parallel scan flag

    for tok in token_list:
        t = tok.strip()
        shoot = t.endswith('*')
        b = (t[:-1] if shoot else t).upper()
        # 合法性校验（V1 L640-646）
        if (len(b) < 2 or b[0] not in COL_LETTERS
                or not b[1:].isdigit()):
            raise ValueError(
                f"Invalid waypoint '{tok}'. Use blocks like 'G8' or "
                "'D2*' ('*' = mid-route shoot stop)")
        px, py = block_to_meters(b.upper())
        # 去重
        if points and abs(points[-1]['x'] - px) < 1e-9 \
                and abs(points[-1]['y'] - py) < 1e-9:
            if shoot:
                shoot_flags[-1] = True
            continue
        points.append({'x': px, 'y': py, 'z': 0.0,
                       'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})
        shoot_flags.append(shoot)
        scan_flags.append(True)

    if not points:
        raise ValueError(f"No valid waypoints in tokens: {tokens}")

    # 航向 + 接近段（V1 L664-698）
    final_yaw, approach_point, _ = _compute_yaw_and_approach(
        points, final_heading, current_yaw, approach_dist)

    # 插入接近段（V1 L691-698）
    if approach_point is not None:
        points.insert(len(points) - 1, approach_point)
        shoot_flags.insert(len(shoot_flags) - 1, False)
        scan_flags.insert(len(scan_flags) - 1, False)

    # 回填四元数（V1 L704-711）
    _backfill_quaternions(points, final_yaw)

    # 转为 Route（保持 V1 语义）
    builder = RouteBuilder(name=name)
    for i, p in enumerate(points):
        qz = p.get('qz', 0.0)
        qw = p.get('qw', 1.0)
        yaw = 2 * math.atan2(qz, qw) if abs(qw) > 1e-12 or abs(qz) > 1e-12 else 0.0
        builder.add(
            x=p['x'], y=p['y'], yaw=yaw,
            shoot=shoot_flags[i] if i < len(shoot_flags) else False,
            scan=scan_flags[i] if i < len(scan_flags) else True,
            token='',
        )

    return builder.build(fix_final_yaw=final_yaw)



def parse_reset(spec: str) -> Tuple[str, float]:
    """解析 reset: 命令 → (格名, yaw)。

    对应 V1 L485-516 reset: 分支。
    'N10,y' → ('N10', π/2)
    'L9,x'  → ('L9', 0.0)
    """
    parts = spec.split(',')
    if len(parts) != 2:
        raise ValueError(f"Invalid reset spec: '{spec}'. Use 'Block,heading' like 'N10,y'")
    block = parts[0].strip().upper()
    heading = parts[1].strip().lower()
    yaw = math.pi / 2 if heading == 'y' else 0.0
    return block, yaw


def format_route(route: Route) -> List[str]:
    """Route → token 列表（回写用）。"""
    tokens = []
    for wp in route.waypoints:
        t = wp.token if wp.token else f"({wp.x:.3f},{wp.y:.3f})"
        if wp.shoot:
            t += '*'
        if wp.pivot:
            t += ':PIVOT'
        if wp.reverse:
            t = 'R:' + t
        tokens.append(t)
    return tokens


def detect_crossing(route: Route) -> Dict[str, str]:
    """检测交叉区域（预留接口）。"""
    return {}
