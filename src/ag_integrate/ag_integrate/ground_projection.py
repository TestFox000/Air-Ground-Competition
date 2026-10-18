"""检测框反投影：相机像素 → SLAM 系地面点 → 场地系 → 落格。

链路（2026-10-17 方案）：
    D13 检测框(像素) --相机内参--> 视线
        --相机-雷达外参--> 机体系
        --SLAM 位姿--> 局部系地面交点（相对几何精确）
        --锚定配准 T--> 场地系
        --field_frame.world_to_cell--> 格号

误差预算（外参标定的敏感项，写入承诺）：
    - 俯仰角 1° @ 3m 高 ≈ 5cm 地面误差（格 0.35m，落格预算 <0.1m）；
    - 时间同步 0.1s @ 2m/s 飞行 ≈ 0.2m → 位姿必须用同源单调钟插值；
    - 检测框底边中心 = 地面接触点（雷/靶全在地面，底边比中心稳）。

可靠性承诺：
    - 反投影与正向投影互为逆（往返误差 <1e-6 m）；
    - 视线不交地面（太平行/朝上）显式报错，不输出垃圾点；
    - 位姿插值只信任 max_gap 内的样本，越界/空缓冲/时钟回跳报错；
    - 落格失败（出界/离格心过远）返回 cell=None，绝不硬吸附。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .anchored_registration import Transform2D
from .field_frame import COLS, ROWS, X0, Y0, CELL_SIZE, CellRef, world_to_cell
from .intel_protocol import ObjectType


# ---------------------------------------------------------------- 3D 小工具（纯 Python）
def quat_to_rot(qx: float, qy: float, qz: float, qw: float):
    """四元数(x,y,z,w) → 3x3 旋转矩阵（行主序元组），自动归一化。"""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        raise ValueError("zero quaternion")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return (
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
    )


def rot_apply(r, v) -> tuple[float, float, float]:
    return (r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
            r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
            r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2])


def rot_mul(a, b):
    """a·b（先 b 后 a）。"""
    out = []
    for i in range(3):
        row = []
        for j in range(3):
            row.append(sum(a[i][k] * b[k][j] for k in range(3)))
        out.append(tuple(row))
    return tuple(out)


def quat_nlerp(q1, q2, alpha: float):
    """归一化线性插值（短弧），供位姿插值用。"""
    if q1[3] * q2[3] + q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] < 0:
        q2 = (-q2[0], -q2[1], -q2[2], -q2[3])
    q = tuple(q1[k] + alpha * (q2[k] - q1[k]) for k in range(4))
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-12:
        return q1
    return tuple(c / n for c in q)


# ---------------------------------------------------------------- 数据结构
@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass(frozen=True)
class Extrinsic:
    """相机 → 机体(雷达)系 外参：旋转四元数 + 平移（米，机体系）。"""

    qx: float
    qy: float
    qz: float
    qw: float
    tx: float
    ty: float
    tz: float


@dataclass(frozen=True)
class Pose:
    """机体 → SLAM 局部系 位姿（单调时钟 t，秒）。"""

    t: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class Detection:
    """一帧检测（D13 输出，含单调时间戳与像素框）。"""

    t: float
    kind: ObjectType
    x1: float
    y1: float
    x2: float
    y2: float
    conf: float


@dataclass(frozen=True)
class LandedDetection:
    det: Detection
    slam_x: float
    slam_y: float
    field_x: float
    field_y: float
    cell: CellRef | None


class PoseBuffer:
    """位姿环形缓冲：同源单调钟，线性位置插值 + 四元数 nlerp。

    max_gap：插值两端最大间距（秒），超过视为不可信（时间同步坑的守门员）。
    """

    def __init__(self, max_gap: float = 0.25, maxlen: int = 600):
        if max_gap <= 0:
            raise ValueError("max_gap must be positive")
        self._buf: list[Pose] = []
        self.max_gap = max_gap
        self.maxlen = maxlen

    def add(self, pose: Pose) -> None:
        if self._buf and pose.t <= self._buf[-1].t:
            raise ValueError(f"pose time not monotonic: {pose.t} <= {self._buf[-1].t}")
        self._buf.append(pose)
        if len(self._buf) > self.maxlen:
            del self._buf[:len(self._buf) - self.maxlen]

    def at(self, t: float) -> Pose:
        if not self._buf:
            raise ValueError("pose buffer empty")
        if t < self._buf[0].t or t > self._buf[-1].t:
            raise ValueError(f"t={t} outside pose buffer "
                             f"[{self._buf[0].t}, {self._buf[-1].t}]")
        lo, hi = 0, len(self._buf) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self._buf[mid].t <= t:
                lo = mid
            else:
                hi = mid
        p0, p1 = self._buf[lo], self._buf[hi]
        if p1.t - p0.t > self.max_gap:
            raise ValueError(f"pose gap {p1.t - p0.t:.3f}s > max_gap {self.max_gap}s")
        if p1.t == p0.t:
            return p0
        a = (t - p0.t) / (p1.t - p0.t)
        q = quat_nlerp((p0.qx, p0.qy, p0.qz, p0.qw),
                       (p1.qx, p1.qy, p1.qz, p1.qw), a)
        return Pose(t,
                    p0.x + a * (p1.x - p0.x),
                    p0.y + a * (p1.y - p0.y),
                    p0.z + a * (p1.z - p0.z),
                    q[0], q[1], q[2], q[3])


# ---------------------------------------------------------------- 投影
def project_ray_ground(u: float, v: float, cam: CameraModel, ext: Extrinsic,
                       pose: Pose, ground_z: float = 0.0) -> tuple[float, float]:
    """像素 (u, v) → SLAM 系地面交点 (x, y)。

    相机系约定：x 右、y 下、z 沿光轴（OpenCV/针孔标准）。
    """
    d_cam = ((u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1.0)
    r_bs = quat_to_rot(pose.qx, pose.qy, pose.qz, pose.qw)
    r_cb = quat_to_rot(ext.qx, ext.qy, ext.qz, ext.qw)
    r_cs = rot_mul(r_bs, r_cb)
    o = (pose.x + rot_apply(r_bs, (ext.tx, ext.ty, ext.tz))[0],
         pose.y + rot_apply(r_bs, (ext.tx, ext.ty, ext.tz))[1],
         pose.z + rot_apply(r_bs, (ext.tx, ext.ty, ext.tz))[2])
    d = rot_apply(r_cs, d_cam)
    if d[2] > -1e-3:
        raise ValueError("ray parallel to ground or pointing up (pitch too shallow)")
    s = (ground_z - o[2]) / d[2]
    if s <= 0:
        raise ValueError("ground plane behind camera")
    return (o[0] + s * d[0], o[1] + s * d[1])


def project_detection(det: Detection, cam: CameraModel, ext: Extrinsic,
                      pose: Pose, ground_z: float = 0.0) -> tuple[float, float]:
    """检测框底边中心（地面接触点）→ SLAM 系地面点。"""
    u = (det.x1 + det.x2) / 2.0
    v = det.y2
    return project_ray_ground(u, v, cam, ext, pose, ground_z)


def land_detections(dets, cam: CameraModel, ext: Extrinsic, poses: PoseBuffer,
                    reg: Transform2D, ground_z: float = 0.0) -> list[LandedDetection]:
    """批量落格：投影 → 配准 → 吸附格号。

    单条失败（视线不交地/位姿缺失）不拖垮整批——跳过并计入返回外的
    日志由调用方处理；落格吸附失败只置 cell=None。
    """
    out: list[LandedDetection] = []
    for det in dets:
        try:
            pose = poses.at(det.t)
            sx, sy = project_detection(det, cam, ext, pose, ground_z)
        except ValueError:
            continue
        fx, fy = reg.apply(sx, sy)
        try:
            cell = world_to_cell(fx, fy)
        except ValueError:
            cell = None
        out.append(LandedDetection(det, sx, sy, fx, fy, cell))
    return out


# ---------------------------------------------------------------- 视野足迹（负证据的来源）
def ground_footprint(cam: CameraModel, ext: Extrinsic, pose: Pose,
                     ground_z: float = 0.0) -> tuple[tuple[float, float], ...] | None:
    """图像四角投到地面 → 凸四边形；任一角不交地则返回 None（足迹无效）。"""
    corners = ((0.0, 0.0), (cam.width - 1.0, 0.0),
               (cam.width - 1.0, cam.height - 1.0), (0.0, cam.height - 1.0))
    pts = []
    for u, v in corners:
        try:
            pts.append(project_ray_ground(u, v, cam, ext, pose, ground_z))
        except ValueError:
            return None
    return tuple(pts)


def footprint_cells(quad) -> list[CellRef]:
    """格心落在凸四边形内的所有格（负证据的作用范围）。"""
    n = len(quad)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    xs = [p[0] for p in quad]
    ys = [p[1] for p in quad]
    col_lo = max(0, int((min(xs) - X0) / CELL_SIZE) - 1)
    col_hi = min(COLS - 1, int((max(xs) - X0) / CELL_SIZE) + 1)
    row_lo = max(0, int((ROWS - 1) - (max(ys) - Y0) / CELL_SIZE) - 1)
    row_hi = min(ROWS - 1, int((ROWS - 1) - (min(ys) - Y0) / CELL_SIZE) + 1)
    cells: list[CellRef] = []
    for col in range(col_lo, col_hi + 1):
        for row in range(row_lo, row_hi + 1):
            cx = X0 + col * CELL_SIZE
            cy = Y0 + (ROWS - 1 - row) * CELL_SIZE
            signs = [cross(quad[k], quad[(k + 1) % n], (cx, cy)) for k in range(n)]
            if all(s >= -1e-9 for s in signs) or all(s <= 1e-9 for s in signs):
                cells.append(CellRef(col, row))
    return cells
