#!/usr/bin/env python3
"""数值验证 px4_odom_convert 的坐标系链路是否自洽（位置与姿态是否用同一个旋转）。

复刻自（均在本仓库镜像中）：
  fc-audit/src-mirror/px4_odom_convert/src/px4_odom_convert.cpp:121-179
  fc-audit/src-mirror/px4_ros_com/include/px4_ros_com/frame_transforms.h:173,180,316-349
  fc-audit/src-mirror/px4_ros_com/src/lib/frame_transforms.cpp:157-198
"""
import numpy as np

np.set_printoptions(precision=6, suppress=True)


# ---------- 基础旋转工具 ----------
def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def quat_from_euler(roll, pitch, yaw):
    """frame_transforms.cpp:58-64 —— YPR is ZYX axes: Rz(yaw)*Ry(pitch)*Rx(roll)"""
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)


def quat_mul(q1, q2):
    """Hamilton 乘积，q = [w, x, y, z]"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def R_to_quat(R):
    """Eigen::Quaterniond(R) 的等价实现"""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return np.array([w, x, y, z])


def quat_to_R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# ---------- frame_transforms.h 里的两个静态量 ----------
# frame_transforms.h:173  NED_ENU_Q = quaternion_from_euler(M_PI, 0.0, M_PI_2)
# frame_transforms.h:180  AIRCRAFT_BASELINK_Q = quaternion_from_euler(M_PI, 0.0, 0.0)
R_NED_ENU = quat_from_euler(np.pi, 0.0, np.pi / 2)      # 旋转矩阵形式
R_FLU_FRD = quat_from_euler(np.pi, 0.0, 0.0)            # 旋转矩阵形式
Q_NED_ENU = R_to_quat(R_NED_ENU)                        # 四元数形式 [w,x,y,z]
Q_AIRCRAFT_BASELINK = R_to_quat(R_FLU_FRD)

# ---------- px4_odom_convert.cpp:20-23 写死的 SLAM->ENU ----------
R_ENU_SLAM = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
T_ENU_SLAM = np.zeros(3)

# ---------- 反射式实现（frame_transforms.cpp:179-198） ----------
M_REFLECT = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def enu_to_ned_local_frame(v):
    return M_REFLECT @ v


# ---------- 被测链路 ----------
def code_chain(p_slam, R_slam_flu, z_offset=0.0):
    # position
    p_slam_adj = np.array([p_slam[0], p_slam[1], p_slam[2] - z_offset])
    p_enu = R_ENU_SLAM @ p_slam_adj + T_ENU_SLAM
    p_ned = enu_to_ned_local_frame(p_enu)
    # orientation
    q_slam = R_to_quat(R_slam_flu)
    R_enu_flu = R_ENU_SLAM @ quat_to_R(q_slam)
    q_enu = R_to_quat(R_enu_flu)
    q_px4 = quat_mul(quat_mul(Q_NED_ENU, q_enu), Q_AIRCRAFT_BASELINK)
    return p_ned, quat_to_R(q_px4)


def main():
    print("=" * 78)
    print("[1] 反射实现 vs 四元数实现是否同一个旋转")
    print("=" * 78)
    print("NED_ENU_Q(四元数) 的旋转矩阵:\n", R_NED_ENU)
    print("enu_to_ned_local_frame 的矩阵:\n", M_REFLECT)
    same = np.allclose(R_NED_ENU, M_REFLECT, atol=1e-12)
    print("一致:", same)
    print("det(NED_ENU_Q) =", np.linalg.det(R_NED_ENU),
          " det(反射实现) =", np.linalg.det(M_REFLECT))

    print()
    print("=" * 78)
    print("[2] 位置与姿态是否共用同一个旋转 R_total = R_NED_ENU * R_ENU_SLAM")
    print("=" * 78)
    R_total = R_NED_ENU @ R_ENU_SLAM
    print("R_total:\n", R_total)
    print("det(R_total) =", np.linalg.det(R_total), "(应=1)")
    print("R_total 正交性 ||R^T R - I|| =", np.abs(R_total.T @ R_total - np.eye(3)).max())

    rng = np.random.default_rng(20261007)
    ok_pos = ok_att = True
    worst_pos = worst_att = 0.0
    for _ in range(2000):
        # 随机 SLAM 位姿（世界系 z 朝上、体系 FLU）
        yaw, pitch, roll = rng.uniform(-np.pi, np.pi, 3)
        R_slam_flu = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        p_slam = rng.uniform(-50, 50, 3)

        p_ned, R_ned_frd = code_chain(p_slam, R_slam_flu)

        p_ref = R_total @ p_slam
        R_ref = R_total @ R_slam_flu @ R_FLU_FRD

        e_p = np.abs(p_ned - p_ref).max()
        e_a = np.abs(R_ned_frd - R_ref).max()
        worst_pos = max(worst_pos, e_p)
        worst_att = max(worst_att, e_a)
        ok_pos &= e_p < 1e-9
        ok_att &= e_a < 1e-9

    print(f"2000 组随机位姿:  位置最大误差 = {worst_pos:.3e}  姿态最大误差 = {worst_att:.3e}")
    print("位置链自洽:", ok_pos, "  姿态链自洽:", ok_att)
    print("位置与姿态使用同一个 R:", ok_pos and ok_att)

    print()
    print("=" * 78)
    print("[3] R_total 把 SLAM 轴映射到 NED 的哪个方向")
    print("=" * 78)
    names_slam = ['+x_slam', '+y_slam', '+z_slam']
    for i, nm in enumerate(names_slam):
        v = R_total @ np.eye(3)[i]
        n, e, d = v
        if abs(d) > 0.99:
            desc = "向上 (Up)" if d < 0 else "向下 (Down)"
        else:
            desc = f"北={n:+.0f} 东={e:+.0f}"
            if abs(n) > 0.99:
                desc += "  → " + ("北" if n > 0 else "南")
            elif abs(e) > 0.99:
                desc += "  → " + ("东" if e > 0 else "西")
        print(f"  {nm} -> NED({n:+.0f},{e:+.0f},{d:+.0f})   {desc}")

    print()
    print("=" * 78)
    print("[4] 结论")
    print("=" * 78)
    print("R_total 是绕垂直轴（SLAM/ENU 的 z 轴）的纯偏航旋转：")
    print("  - 垂直方向映射正确：SLAM 向上 -> NED 向下（z 取反）")
    print("  - 水平面内只是把 'map 的 +x' 定义成了 NED 的西向，属于命名约定")
    print("  - 位置与姿态用的是同一个 R_total，因此即使 R_ENU_SLAM 取值任意，")
    print("    只要它是绕 z 轴的旋转，整套控制回路仍自洽；若它不是绕 z 轴")
    print("    （含 roll/pitch 分量），位置系与姿态系就会分家 —— 那才是灾难。")

    print()
    print("=" * 78)
    print("[5] 反例：R_ENU_SLAM 若混入横滚/俯仰，会污染高度通道")
    print("=" * 78)
    print("  水平位移 1 m 时，被错算成的高度变化（cm/m）：")
    for deg in (0.0, 0.5, 1.0, 2.0, 5.0):
        bad = Rz(np.pi) @ Rx(np.deg2rad(deg))
        R_total_bad = R_NED_ENU @ bad
        p = np.array([0.0, 1.0, 0.0])  # 纯水平位移
        leak = abs((R_total_bad @ p)[2]) * 100.0
        print(f"    R_ENU_SLAM 含横滚 {deg:4.1f}° -> {leak:6.2f} cm 的高度误差 / 每米水平位移")
    print("  -> 20 m 的航线在 1° 失配下会产生约 35 cm 的高度通道污染，")
    print("     而且是与水平位置成比例的系统性误差，EKF2 无法自行辨识掉。")


if __name__ == "__main__":
    main()
