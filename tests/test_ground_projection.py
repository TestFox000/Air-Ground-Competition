"""ground_projection 反投影测试。

锁定的可靠性承诺：
    - 正反投影互逆（任意位姿/外参下往返误差 <1e-6 m）；
    - 底边中心 = 地面接触点，落格端到端正确；
    - 视线不交地（太平行/朝上）显式报错；
    - 位姿插值单调钟纪律 + max_gap 守门 + 越界/回跳报错；
    - 落格失败置 cell=None，绝不硬吸附。
"""

import math

import pytest

from ag_integrate.anchored_registration import Transform2D
from ag_integrate.field_frame import CellRef, cell_to_world
from ag_integrate.ground_projection import (
    CameraModel,
    Detection,
    Extrinsic,
    Pose,
    PoseBuffer,
    footprint_cells,
    ground_footprint,
    land_detections,
    project_ray_ground,
    quat_to_rot,
    rot_apply,
    rot_mul,
)
from ag_integrate.intel_protocol import ObjectType

CAM = CameraModel(fx=400.0, fy=400.0, cx=320.0, cy=256.0, width=640, height=512)
# 下视安装：相机系 → 机体系 = 绕 x 轴 180°（y、z 取反），相机光轴沿机体系 -z 朝下
NADIR_EXT = Extrinsic(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
IDENT_TF = Transform2D(1.0, 0.0, 0.0, 0.0, 0.0, 3, 3, True)


def _nadir_pose(x=1.0, y=2.0, z=3.0):
    return Pose(0.0, x, y, z, 0.0, 0.0, 0.0, 1.0)


def test_nadir_projection():
    """正下视：像素偏移 × 高度 / 焦距 = 地面偏移（几何自检）。"""
    pose = _nadir_pose(1.0, 2.0, 3.0)
    gx, gy = project_ray_ground(320.0, 256.0, CAM, NADIR_EXT, pose)
    assert (gx, gy) == (1.0, 2.0)
    # 像素 (400, 256)：dx=0.2 → 地面 x 偏移 0.2*3=0.6
    gx, gy = project_ray_ground(400.0, 256.0, CAM, NADIR_EXT, pose)
    assert abs(gx - 1.6) < 1e-9 and abs(gy - 2.0) < 1e-9
    # 像素 (320, 296)：dy=0.1 → 机体系 -y → 地面 y 偏移 -0.3
    gx, gy = project_ray_ground(320.0, 296.0, CAM, NADIR_EXT, pose)
    assert abs(gx - 1.0) < 1e-9 and abs(gy - 1.7) < 1e-9


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def test_roundtrip_arbitrary_orientation():
    """斜视位姿（yaw 30° + pitch 15°）+ 下视外参带安装扰动：往返 <1e-6。"""
    q_yaw = (0.0, 0.0, math.sin(math.radians(15)), math.cos(math.radians(15)))
    q_pitch = (0.0, math.sin(math.radians(7.5)), 0.0, math.cos(math.radians(7.5)))
    q = _quat_mul(q_yaw, q_pitch)
    pose = Pose(0.0, 0.5, 0.5, 3.0, *q)
    # 下视安装（绕 x 180°）叠加小安装角扰动 + 平移偏置
    q_ext = _quat_mul((1.0, 0.0, 0.0, 0.0), (0.02, 0.03, -0.01, 0.999))
    ext = Extrinsic(*q_ext, 0.01, -0.02, 0.03)
    for (u, v) in [(320.0, 256.0), (200.0, 300.0), (450.0, 180.0)]:
        gx, gy = project_ray_ground(u, v, CAM, ext, pose)
        # 正向重建：地面点 → 像素
        r_cs = rot_mul(quat_to_rot(*q), quat_to_rot(ext.qx, ext.qy, ext.qz, ext.qw))
        oc = rot_apply(quat_to_rot(*q), (ext.tx, ext.ty, ext.tz))
        o = (pose.x + oc[0], pose.y + oc[1], pose.z + oc[2])
        d = (gx - o[0], gy - o[1], 0.0 - o[2])
        n = math.sqrt(sum(c * c for c in d))
        d = tuple(c / n for c in d)
        d_cam = rot_apply(tuple(zip(*r_cs)), d)   # R⁻¹ = Rᵀ
        assert d_cam[2] > 0
        u2 = CAM.cx + CAM.fx * d_cam[0] / d_cam[2]
        v2 = CAM.cy + CAM.fy * d_cam[1] / d_cam[2]
        assert abs(u2 - u) < 1e-6 and abs(v2 - v) < 1e-6


def test_ray_parallel_guard():
    """相机平视（光轴水平）→ 视线不交地面 → 报错。"""
    pose = Pose(0.0, 1.0, 2.0, 1.5, 0.0, 0.0, 0.0, 1.0)
    ext = Extrinsic(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)   # 光轴沿机体系 x（水平）
    with pytest.raises(ValueError, match="parallel|ground"):
        project_ray_ground(320.0, 256.0, CAM, ext, pose)


def test_pose_buffer_interpolation():
    buf = PoseBuffer(max_gap=0.25)
    buf.add(Pose(10.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0))
    buf.add(Pose(10.1, 1.4, 2.0, 3.0, 0.0, 0.0, 0.7071, 0.7071))  # yaw 90°
    mid = buf.at(10.05)
    assert abs(mid.x - 1.2) < 1e-9
    # nlerp 中点 ≈ 45°（sin/cos 分量各半）
    assert abs(mid.qz - 0.3827) < 1e-3 and abs(mid.qw - 0.9239) < 1e-3


def test_pose_buffer_guards():
    buf = PoseBuffer(max_gap=0.25)
    with pytest.raises(ValueError, match="empty"):
        buf.at(1.0)
    buf.add(Pose(10.0, 0, 0, 3, 0, 0, 0, 1))
    with pytest.raises(ValueError, match="outside"):
        buf.at(9.9)
    with pytest.raises(ValueError, match="monotonic"):
        buf.add(Pose(10.0, 0, 0, 3, 0, 0, 0, 1))
    buf2 = PoseBuffer(max_gap=0.1)
    buf2.add(Pose(10.0, 0, 0, 3, 0, 0, 0, 1))
    buf2.add(Pose(10.3, 0, 0, 3, 0, 0, 0, 1))
    with pytest.raises(ValueError, match="gap"):
        buf2.at(10.15)


def test_land_detection_end_to_end():
    """检测框底边中心落 F5 格心 → 落格 = F5。"""
    fx, fy = cell_to_world(CellRef(5, 4))            # F5 格心 (1.815, 2.395)
    pose = _nadir_pose(fx, fy, 3.0)
    buf = PoseBuffer()
    buf.add(Pose(0.0, fx, fy, 3.0, 0, 0, 0, 1))
    det = Detection(0.0, ObjectType.PAPER, 300.0, 200.0, 340.0, 256.0, 0.9)
    landed = land_detections([det], CAM, NADIR_EXT, buf, IDENT_TF)
    assert len(landed) == 1
    ld = landed[0]
    assert abs(ld.field_x - fx) < 1e-9 and abs(ld.field_y - fy) < 1e-9
    assert ld.cell == CellRef(5, 4)


def test_land_out_of_field_returns_none():
    pose = _nadir_pose(50.0, 50.0, 3.0)              # 场外
    buf = PoseBuffer()
    buf.add(Pose(0.0, 50.0, 50.0, 3.0, 0, 0, 0, 1))
    det = Detection(0.0, ObjectType.PAPER, 300.0, 200.0, 340.0, 256.0, 0.9)
    landed = land_detections([det], CAM, NADIR_EXT, buf, IDENT_TF)
    assert len(landed) == 1 and landed[0].cell is None


def test_land_skips_bad_pose():
    """位姿缺失（时间戳越界）的单条检测被跳过，不拖垮整批。"""
    buf = PoseBuffer()
    buf.add(Pose(5.0, 1.0, 2.0, 3.0, 0, 0, 0, 1))
    good = Detection(5.0, ObjectType.PAPER, 300.0, 200.0, 340.0, 256.0, 0.9)
    bad = Detection(99.0, ObjectType.PAPER, 300.0, 200.0, 340.0, 256.0, 0.9)
    landed = land_detections([good, bad], CAM, NADIR_EXT, buf, IDENT_TF)
    assert len(landed) == 1 and landed[0].det is good


def test_footprint_cells_nadir():
    """正下视 3m：足迹覆盖以机下为中心的一片区，含正下方格。"""
    pose = _nadir_pose(1.815, 2.395, 3.0)            # F5 格心上方
    quad = ground_footprint(CAM, NADIR_EXT, pose)
    assert quad is not None
    cells = footprint_cells(quad)
    assert CellRef(5, 4) in cells                    # 正下方必在
    assert len(cells) > 10                           # 半径 2m 级视野


def test_footprint_invalid_when_shallow():
    """视场一角不交地（掠射）→ 足迹无效 → None（负证据不施加）。"""
    # 机体俯仰 80°：下视相机变成近水平掠射，图像上缘射线朝上
    pitch = math.radians(80)
    pose = Pose(0.0, 1.0, 1.0, 2.0, 0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))
    assert ground_footprint(CAM, NADIR_EXT, pose) is None
    # 完全水平必无效
    horiz = Pose(0.0, 1.0, 1.0, 2.0, 0.0, 0.0, 1.0, 0.0)   # 绕 z 90° 不改变俯仰
    ext_h = Extrinsic(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    assert ground_footprint(CAM, ext_h, horiz) is None
