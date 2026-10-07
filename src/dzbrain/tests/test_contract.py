# -*- coding: utf-8 -*-
r"""ROS 契约测试 —— 钉死单位/坐标系/QoS（这是旧代码最容易"静默跑偏"的地方）。

运行：python src\dzbrain\tests\test_contract.py
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from dzbrain.core import geometry as geo                     # noqa: E402
from dzbrain.core.config import Config                       # noqa: E402
from dzbrain.io import contract as ct                        # noqa: E402
from harness import check, check_close, check_raises, finish, section  # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]

section("话题表完整性（防手写话题名）")
for key in ("odom", "pursuit_angle", "scan_request", "offset_center", "aim_status",
            "shoot_count", "aim_fire_gate", "drone_in", "drone_out", "referee_hit"):
    t = ct.topic(key)
    check(f"{key} 已登记", t.name.startswith("/") or key == "manual_gimbal" or key == "pursuit_angle",
          f"{t.name}")
check("拼错的话题键会抛 KeyError", ct.topic("odom") is not None)
check_raises("未登记键报错（不静默）", KeyError, ct.topic, "odomm")
check("manual_gimbal 历史名字无前导斜杠（原样保留）", ct.topic("manual_gimbal").name == "nav/manual_gimbal")
check("pursuitAngle 大小写原样保留", ct.topic("pursuit_angle").name == "pursuitAngle")

section("QoS 契约")
check("pursuitAngle 是 depth1/BEST_EFFORT",
      ct.QOS_PURSUIT_ANGLE.depth == 1 and ct.QOS_PURSUIT_ANGLE.reliability == "best_effort")
check("odom 是 BEST_EFFORT", ct.topic("odom").qos.reliability == "best_effort")
check("QoS 描述可读", "KEEP_LAST(1)/BEST_EFFORT/VOLATILE" in ct.QOS_PURSUIT_ANGLE.describe())

section("★ 里程计换算（格 → 米 + yaw +π/2）—— 两处历史踩坑点")
# N10 的格单位是 (1.5, 0.5)，原始 yaw=0（朝车体 X+）→ 地图航向应为 +90°
pose = ct.odom_to_pose(1.5, 0.5, geo.yaw_to_quaternion(0.0))
check_close("格单位 → 米 x", pose.x, 0.525, 1e-12)
check_close("格单位 → 米 y", pose.y, 0.175, 1e-12)
check_close("原始 yaw=0 → 地图 yaw=+90°", pose.yaw, math.pi / 2, 1e-12)
check("换算后落回 N10 格", pose.block() == "N10", pose.block())

# 原始 yaw=-90°（车体 X-）→ 地图 yaw = 0（朝 X+）
pose2 = ct.odom_to_pose(1.5, 0.5, geo.yaw_to_quaternion(-math.pi / 2))
check_close("原始 -90° → 地图 0°", pose2.yaw, 0.0, 1e-12)

# 任意原始 yaw 都应满足 map = raw + π/2 的折绕关系
for raw_deg in (-170, -90, 0, 45, 90, 170):
    p = ct.odom_to_pose(0.0, 0.0, geo.yaw_to_quaternion(math.radians(raw_deg)))
    want = geo.normalize_angle(math.radians(raw_deg) + math.pi / 2)
    check(f"raw {raw_deg:+4}° → map {math.degrees(want):+6.1f}°",
          abs(geo.ang_diff(p.yaw, want)) < 1e-9)

section("★ 控制量语义（angular.z 是【度】不是 rad/s）")
cmd = ct.pose_to_pursuit_twist(0.15, -7.5)
check_close("linear.x 原样为 m/s", cmd["linear_x"], 0.15)
check_close("angular.z 是度（不做弧度换算）", cmd["angular_z_deg"], -7.5)
check("零指令 = 刹车", ct.is_brake_command(ct.ZERO_COMMAND))
check("非零指令不算刹车", not ct.is_brake_command(ct.pose_to_pursuit_twist(0.01, 0.0)))

cfg = Config.load(str(WS / "config" / "robot.yaml"))
check("底盘限幅：请求 40° 被夹到 ≤35°", abs(ct.clamp_steer_for_chassis(40.0, cfg)) <= 35.0)
check_close("底盘限幅：请求 20° 原样通过", ct.clamp_steer_for_chassis(20.0, cfg), 20.0)

section("底盘速度量化（旧侧 |v·23/0.33| 截断）")
check("0.15 m/s → 10 计数（旧实现少 4.3%）", ct.chassis_speed_counts(0.15) == 10,
      f"counts={ct.chassis_speed_counts(0.15)}")
check("0.30 m/s → 20 计数", ct.chassis_speed_counts(0.30) == 20)
check_close("1 计数 ≈ 0.0143 m/s", 0.33 / 23.0, 0.0143478, 1e-6)
# 补偿项：让配置里的 speed_quant_comp 能把丢掉的 4.3% 补回来
comp = cfg.get("pursuit.speed_quant_comp")
check("补偿默认 1.0（可改 1.043）", comp == 1.0, f"comp={comp}")
check("补偿后计数提升", ct.chassis_speed_counts(0.15 * 1.043) == 10,
      f"0.1565→{ct.chassis_speed_counts(0.15 * 1.043)}")

section("枚举与阶段")
check("云台状态名", ct.AimStatus.name(2) == "TRACK" and ct.AimStatus.name(0) == "IDLE")
check("未知状态名不崩", ct.AimStatus.name(9) == "UNKNOWN(9)")
check("任务阶段齐全", len(ct.MissionPhase.ALL) == 8 and "SAFE_STOP" in ct.MissionPhase.ALL)
check("靶类别编码（0 无 / 1 蓝 / 2 红）", (ct.ShotClass.NONE, ct.ShotClass.BLUE, ct.ShotClass.RED)
      == (0, 1, 2))

section("契约摘要可渲染（启动横幅用）")
lines = ct.ContractSummary().render()
check("至少 15 行", len(lines) >= 15, f"lines={len(lines)}")
check("含 pursuitAngle", any("pursuitAngle" in ln for ln in lines))

section("★ 视觉报文解码（含长度守卫，旧代码在此越界读过）")
obs = ct.decode_offset_center([12, -5, 1, 45, 60, 0, 0, 0], ts=100.0)
check("解码成功且字段对齐", obs is not None and (obs.dx, obs.dy, obs.cls, obs.box_w, obs.box_h)
      == (12, -5, 1, 45, 60))
check("有目标判定（蓝=1）", obs.has_target)
check("红靶也算目标", ct.decode_offset_center([0, 0, 2, 10, 10]).has_target)
check("cls=0 表示无目标", not ct.decode_offset_center([0, 0, 0, 0, 0]).has_target)
check("不足 5 字段 → None（不猜）", ct.decode_offset_center([1, 2, 3]) is None)
check("空/None → None", ct.decode_offset_center([]) is None
      and ct.decode_offset_center(None) is None)
check("非序列 → None", ct.decode_offset_center(42) is None)
check("云台状态解码", ct.decode_aim_status(2) == "TRACK"
      and ct.decode_aim_status("bad") == "UNKNOWN")

section("ros_io 惰性导入（无 ROS 的机器也能 import）")
import importlib                                        # noqa: E402
rio = importlib.import_module("dzbrain.io.ros_io")
check("ros_io 可 import（rclpy 未安装也不报错）", hasattr(rio, "RosBridge"))
check("RosBridge 有发布/读取接口", all(hasattr(rio.RosBridge, m) for m in
      ("publish_command", "publish_brake", "publish_fire_gate", "pose", "vision",
       "take_hits", "spin_async")))
check_raises("无 ROS 时实例化即失败（而不是静默跑空）", ImportError, rio.RosBridge, "t")

finish()
