"""core/geometry.py — 坐标几何纯函数 + 常量 + Pose dataclass.

从 path_executor.py V1 L105-190 拆出：
  - 全局常量: COL_LETTERS, MAP_COLS, MAP_ROWS, BLOCK_SIZE, RAW_YAW_TO_MAP_YAW_OFFSET
  - 数学工具: euler_from_quaternion, normalize_angle, block_to_meters, heading_to_quaternion
  - Pose frozen dataclass (V1 current_x/current_y/current_yaw 三位一体)

语义与 V1 完全相同，输出与 V1 path_executor 逐位一致。
"""

import math
from dataclasses import dataclass
from typing import Tuple

# ── 地图常量 ────────────────────────────────────────────────
# V1 L105-115 完全一致
COL_LETTERS = 'ABCDEFGHIJKLMN'   # 14 列字母
MAP_COLS = 14                     # 列数
MAP_ROWS = 11                     # 行数
BLOCK_SIZE = 0.35                 # 格边长（米）

# serial_reader 发布 /odom/localization 时使用"格单位"；原始航向 (X+) → 地图航向 (Y+) 偏移 π/2
RAW_YAW_TO_MAP_YAW_OFFSET = math.pi / 2


# ── 纯函数 ──────────────────────────────────────────────────

def euler_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """四元数 → yaw 角（弧度）。  V1 L123-131 逐字一致。"""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def normalize_angle(angle: float) -> float:
    """角度归一化到 [-π, π]。  V1 L134-142 逐字一致。"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def block_to_meters(block_name: str) -> Tuple[float, float]:
    """格名 → 米制坐标 (x, y)。  V1 L145-170 逐字一致。

    例: M10 → (0.525, 0.175); A1 → (3.675, 4.725); N11 → (0.175, 0.175)
    """
    letter = block_name[0].upper()
    number = int(block_name[1:])
    col_idx = COL_LETTERS.index(letter)
    row_idx = number - 1
    x = (10.5 - row_idx) * BLOCK_SIZE
    y = (13.5 - col_idx) * BLOCK_SIZE
    return x, y


def heading_to_quaternion(heading_dir: str) -> Tuple[float, float, float, float]:
    """朝向字母 → 四元数 (qx, qy, qz, qw)。  V1 L173-190 逐字一致。

    heading_dir: 'x' = 朝 X+, 'y' = 朝 Y+; 其他 → 回退到 X+。
    """
    if heading_dir == 'y':
        yaw = math.pi / 2
    elif heading_dir == 'x':
        yaw = 0.0
    else:
        yaw = 0.0
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return 0.0, 0.0, qz, qw


def ang_diff(a: float, b: float) -> float:
    """两弧度之差的最小角差，等价于 normalize_angle(a - b)。"""
    return normalize_angle(a - b)


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """四元数 → yaw 弧度。euler_from_quaternion 的别名。"""
    return euler_from_quaternion(qx, qy, qz, qw)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """yaw 弧度 → 四元数 (0, 0, qz, qw)。heading_to_quaternion 的通用化。"""
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return 0.0, 0.0, qz, qw


def heading_token_to_yaw(token: str) -> float:
    """朝向字母 → yaw 弧度。'x'=>0.0, 'y'=>π/2, 其他=>0.0。"""
    if token == 'y':
        return math.pi / 2
    return 0.0


# ── Pose dataclass ──────────────────────────────────────────

@dataclass(frozen=True)
class Pose:
    """信念位姿不可变容器。对应 V1 self.current_x/current_y/current_yaw 三字段。"""
    x: float      # 米
    y: float      # 米
    yaw: float    # 弧度

    @staticmethod
    def from_xy(x: float, y: float) -> 'Pose':
        """便利构造：yaw=0。"""
        return Pose(x=x, y=y, yaw=0.0)

    def offset_to(self, tx: float, ty: float) -> Tuple[float, float, float]:
        """到目标点 (tx, ty) 的偏移量 (dx, dy, dist)。"""
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)
        return dx, dy, dist

    def translated(self, dx: float, dy: float) -> 'Pose':
        """平移生成新 Pose。"""
        return Pose(x=self.x + dx, y=self.y + dy, yaw=self.yaw)

    def rotated(self, dyaw: float) -> 'Pose':
        """旋转生成新 Pose。"""
        return Pose(x=self.x, y=self.y, yaw=self.yaw + dyaw)

    def block(self) -> str:
        """返回此 Pose 所在格名（从米制反查最近格）。"""
        # 逆运算: x = (10.5 - row_idx) * 0.35  ⇒ row_idx = 10.5 - x/0.35
        #         y = (13.5 - col_idx) * 0.35  ⇒ col_idx = 13.5 - y/0.35
        row_idx = int(round(10.5 - self.x / BLOCK_SIZE))
        col_idx = int(round(13.5 - self.y / BLOCK_SIZE))
        row_idx = max(0, min(MAP_ROWS - 1, row_idx))
        col_idx = max(0, min(MAP_COLS - 1, col_idx))
        return f"{COL_LETTERS[col_idx]}{row_idx + 1}"
