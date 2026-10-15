"""循迹链路离线软件在环（SIL）回归测试。

验证的是"契约 + 控制逻辑"：integrate_node（真码）-> path_executor
（reference 真码）-> dzactuator 契约 -> Ackermann 底盘 -> serial_reader
语义 -> 回到 path_executor。保真边界见 sim_car.py 文件头（理想定位、
无滑移；不做动力学参数辨识）。

四个场景：
1. test_align_then_route_arrives【主证明】：复现 68.6° 历史事故摆放，
   align 模式先闭环摆正车头，再发车 —— 应在预算内真到终点且不画圈；
2. test_no_align_circles【对照组】：同样摆放不 align 直接发车 ——
   应在同等预算内到不了终点且净转角超一圈（证明仿真能抓原 bug）；
3. test_align_timeout_fallback：不喂 0x4002 -> 15s 超时退回断言 ->
   path_executor 上线后仍发出路线；
4. test_sharp_corner_pivot：90° 拐角触发真 path_executor 的 PIVOT 三相。
"""

import math
from pathlib import Path

import pytest

# 循迹 SIL 依赖车上取回的官方源码（reference/car_src，51MB，gitignore 不入库）。
# 没有这份参考实现的副本（如新克隆）上跳过整套测试，而不是在收集期炸 FileNotFoundError。
_CAR_SRC = Path(__file__).resolve().parents[1] / "reference" / "car_src"
if not _CAR_SRC.exists():
    pytest.skip("reference/car_src 未取回（gitignore 不入库的车端官方源码）",
                allow_module_level=True)

import ros_stub  # noqa: F401  必须先于被测节点模块导入

from ag_integrate.integrate_node import IntegrateNode
from ag_integrate.intel_protocol import IntelObject, ObjectType

import sim_car
from sim_car import TrackingWorld

# ---------------------------------------------------------------- 场景常量
# 历史事故复现：断言朝向 'y'（90°），真实朝向 158.6°，偏差 δ=68.6°；
# IMU 开机朝向 θ_boot=90°（先开机、后搬车，见 sim_car 文件头机理分析）。
TRUE_YAW_DEG = 158.6
ASSERT_YAW_DEG = 90.0
BOOT_YAW_DEG = 90.0

# 情报帧内容：2 雷 + 移动蓝靶落在轨道 D8-E8 上（蓝面朝 S）-> 门控放行，
# 移动靶经 track 先验注入后停靠射击格为 D9/E9（蓝面南侧正交邻格）。
# 雷位 (K8,H9) 的选取：路线拐角全部是 0°/45°/90°/135° 中可被真码
# 处理的组合，且在 5 个随机种子下全程到达（选型扫描见 _sim_scenario_scan.py）。
MINES = ("K8", "H9")
TARGET_CELL = "D8"
TARGET_TRACK = ["D8", "E8"]
TARGET_BLUE_FACE = "S"

# 与真管线逐字节对齐的预期命令（2026-10-15 实测产出；变了说明规划契约变了）
EXPECTED_RESET = "reset:N10,y"
EXPECTED_ROUTE = ("N10 M9 L8 K8 J8 I8 H9 G9 F9 E9* "
                  "F10 G11 H11 I11 J11 K11 L11 M11 N11,x")

ROUTE_BUDGET_S = 90.0   # 阶段2 行车预算（仿真秒）；对照组用同等预算
ALIGN_BUDGET_S = 15.0   # align 超时参数默认值，收敛必须比它快

# 对齐爬行速度取 0.05m/s（节点参数，非默认 0.10）：Ackermann 不能原地转，
# 对齐期间车必然爬出一段圆弧；慢爬让 dzactuator 的 atan(CARL*w/v) 映射
# 打出更大舵角、转弯半径更小，对齐漂移 |O| 从 ~0.35m 降到 ~0.21m，
# 终点真误差才有裕度落进一格（0.35m）以内 —— 实车调参同理。


def _obj(cell: str, typ: ObjectType) -> IntelObject:
    """格号 -> 无人机世界坐标（dzuav 约定，与 test_integrate_node 相同）。"""
    letters = "ABCDEFGHIJKLMN"
    col = letters.index(cell[0])
    row = int(cell[1:]) - 1
    return IntelObject(typ, 0.065 + col * 0.35, 0.295 + (10 - row) * 0.35,
                       0.3, 0.3)


def intel_objects() -> tuple:
    return tuple(_obj(c, ObjectType.MINE) for c in MINES) + (
        _obj(TARGET_CELL, ObjectType.TARGET_BLUE),)


@pytest.fixture
def world_factory():
    """场景工厂：负责收尾（毁节点、关总线、还时钟、清参数覆盖）。"""
    created = []

    def factory(**kw) -> TrackingWorld:
        w = TrackingWorld(**kw)
        created.append(w)
        return w

    yield factory

    for w in created:
        w.close()
    ros_stub.BUS.disable()
    ros_stub.SIM.disable()
    ros_stub.clear_overrides()


def _make_integrate(world: TrackingWorld, **params) -> IntegrateNode:
    base = dict(
        udp_port=0,               # 绑临时端口，永不收真包（帧由测试直喂）
        dry_run=False,            # 真发命令上总线
        publish_reset=True,
        start_cell=world.start_cell, goal_cell="N11",
        start_heading="y", final_heading="x",
        target_track=TARGET_TRACK, target_blue_face=TARGET_BLUE_FACE,
        matrix_log=False,         # 矩阵打印走真实 monotonic 时钟，仿真里关掉省噪
        align_creep_speed=0.05,   # 见上方注释：慢爬减小对齐漂移
    )
    base.update(params)
    ros_stub.set_overrides(**base)
    node = IntegrateNode()
    world.attach_integrate(node, intel_objects())
    return node


# ---------------------------------------------------------------- 主证明
def test_align_then_route_arrives(world_factory):
    """158.6° 摆放 + align：先对齐（<15s、误差 <3°），再发车真到终点。"""
    world = world_factory(start_cell="N10", true_yaw_deg=TRUE_YAW_DEG,
                          theta_boot_deg=BOOT_YAW_DEG)
    ig = _make_integrate(world, heading_bias_source="align")
    gx, gy = world.goal_xy

    # ---- 阶段1：车端栈未起，先对齐 ----
    assert world.run_until(lambda: ig._align_state == "aligned",
                           ALIGN_BUDGET_S), (
        f"对齐未收敛：state={ig._align_state} "
        f"yaw={math.degrees(world.chassis.theta):.1f}°")
    assert world.align_done_t < ALIGN_BUDGET_S
    yaw_err = world.yaw_err_to(ASSERT_YAW_DEG)
    assert abs(yaw_err) < 3.0, f"对齐后朝向残差 {yaw_err:+.2f}° 超 3°"
    # 对齐期间 /nav/waypoint 无订阅者：闸门必须挡住路线（两阶段启动语义）
    assert ig.topic("/nav/waypoint").published == []

    # ---- 阶段2：车端栈上线 -> 闸门放行 -> 真 path_executor 循迹 ----
    world.start_car_stack()
    arrived = world.run_until(lambda: world.pe.arrived, ROUTE_BUDGET_S)

    published = [m.data for m in ig.topic("/nav/waypoint").published]
    assert published[:2] == [EXPECTED_RESET, EXPECTED_ROUTE]
    assert arrived, (
        f"预算 {ROUTE_BUDGET_S}s 内未到终点："
        f"真位姿=({world.chassis.x:.2f},{world.chassis.y:.2f}) "
        f"信念=({world.pe.current_x:.2f},{world.pe.current_y:.2f}) "
        f"距终点 {world.dist_to_goal():.2f}m")
    d_goal = world.dist_to_goal()
    assert d_goal < 0.35, f"真车距终点格中心 {d_goal:.2f}m 超 0.35m"
    # 全程未画圈：有符号累计转角远小于一圈（对照组同指标 >360°）
    assert abs(world.chassis.yaw_traveled) < math.radians(360), (
        f"净转角 {math.degrees(world.chassis.yaw_traveled):.0f}° 疑似画圈")

    print(f"\n[主证明] align 收敛 {world.align_done_t:.1f}s，"
          f"路线发出 t={world.route_pub_t:.1f}s，"
          f"到点 t={world.arrived_t:.1f}s，"
          f"终点误差 {d_goal:.2f}m，"
          f"净转角 {math.degrees(world.chassis.yaw_traveled):.0f}°")


# ---------------------------------------------------------------- 对照组
def test_no_align_circles(world_factory):
    """同样 68.6° 偏差但不 align（none）：同等预算内必须到不了终点。

    这个测试必须能区分好坏：若车照样到终点，说明仿真没复现出
    "信念航向 vs 位置积分"的双重误差（画圆机理），要回头修仿真。
    """
    world = world_factory(start_cell="N10", true_yaw_deg=TRUE_YAW_DEG,
                          theta_boot_deg=BOOT_YAW_DEG)
    ig = _make_integrate(world, heading_bias_source="none")
    gx, gy = world.goal_xy
    # 不 align：车端栈直接在线，闸门立刻放行
    world.start_car_stack()

    arrived = world.run_until(lambda: world.pe.arrived, ROUTE_BUDGET_S)

    published = [m.data for m in ig.topic("/nav/waypoint").published]
    assert published[:2] == [EXPECTED_RESET, EXPECTED_ROUTE]  # 命令照样发出
    assert not arrived, (
        "对照组竟然到终点了 —— 仿真没复现出画圆机理，检查 "
        "SerialReaderSim 的位置积分/θ_boot 语义")
    assert abs(world.chassis.yaw_traveled) > math.radians(360), (
        f"净转角 {math.degrees(world.chassis.yaw_traveled):.0f}° "
        "不足一圈，没画起来？")
    assert world.dist_to_goal() > 0.35

    print(f"\n[对照组] 90s 未收敛：净转角 "
          f"{math.degrees(world.chassis.yaw_traveled):.0f}°，"
          f"终点距离 {world.dist_to_goal():.2f}m，"
          f"里程 {world.chassis.dist_traveled:.1f}m")


# ---------------------------------------------------------------- align 超时退回
def test_align_timeout_fallback(world_factory):
    """align 但没有 0x4002 帧：15s 超时 -> failed -> 退回断言照常发车。"""
    world = world_factory(start_cell="N10", true_yaw_deg=TRUE_YAW_DEG,
                          theta_boot_deg=BOOT_YAW_DEG)
    ig = _make_integrate(world, heading_bias_source="align")
    world.drone = None  # 无人机车位姿帧缺席

    # 15s 超时 + 少量余量；期间车应保持不动（无观测 -> 零角速度）
    assert world.run_until(lambda: ig._align_state == "failed", 18.0)
    assert world.t >= ALIGN_BUDGET_S - 0.2   # 确实熬到了超时点
    assert any("朝向对齐超时" in m for _lvl, m in ig.logger.records)
    assert ig.topic("/cmd_vel").published[-1].angular.z == 0.0
    assert world.chassis.dist_traveled < 0.02  # 全程基本没挪窝
    assert ig.topic("/nav/waypoint").published == []  # 还没消费者

    # path_executor 上线后：退回断言朝向，照常发出 reset+路线
    world.start_car_stack()
    assert world.run_until(
        lambda: len(ig.topic("/nav/waypoint").published) >= 2, 5.0)
    published = [m.data for m in ig.topic("/nav/waypoint").published]
    # 无观测 yaw 时退回 start_heading 断言（'y'）
    assert published[0] == EXPECTED_RESET
    assert published[1] == EXPECTED_ROUTE

    print(f"\n[超时退回] failed@15s，车端栈上线后 t={world.route_pub_t:.1f}s "
          "发出 reset+路线")


# ---------------------------------------------------------------- 90° 拐角 PIVOT
def test_sharp_corner_pivot(world_factory):
    """起点已对齐：路线含 90° 拐角 -> 真 path_executor 的 PIVOT 三相触发。

    路线 N7 -> H7 -> H10：段1 朝 +Y（与 reset:N7,y 一致），段2 朝 -X，
    顶点 H7 转角 90° > 45° 阈值。本场景不经 integrate_node，直接由
    rig 节点往总线发 reset+路线（topic 语义与实车 send_waypoint 一致）。
    """
    world = world_factory(start_cell="N7", true_yaw_deg=90.0,
                          theta_boot_deg=90.0)
    world.start_car_stack()
    pe = world.pe
    goal_xy = world.block_to_meters("H10")

    # 等首帧定位上线后发命令（reset 要用到 odom 的 raw 值）
    assert world.run_until(lambda: pe.has_odom, 2.0)
    world.waypoint_pub.publish(ros_stub.String(data="reset:N7,y"))
    world.step()
    world.waypoint_pub.publish(ros_stub.String(data="N7 H7 H10"))

    arrived = world.run_until(lambda: pe.arrived, 60.0)

    # PIVOT 三相真被踩过（状态机标志 + 真码日志双证）
    assert world.pivot_seen, "90° 拐角没有触发 PIVOT"
    assert any("[PIVOT]" in m for _lvl, m in pe.get_logger().records)
    assert arrived, (
        f"60s 内未到 H10：真位姿=({world.chassis.x:.2f},"
        f"{world.chassis.y:.2f})")
    d = math.hypot(world.chassis.x - goal_xy[0],
                   world.chassis.y - goal_xy[1])
    assert d < 0.35, f"真车距 H10 格心 {d:.2f}m 超 0.35m"

    print(f"\n[PIVOT] 拐角转向触发，到点 t={world.arrived_t:.1f}s，"
          f"终点误差 {d:.2f}m，净转角 "
          f"{math.degrees(world.chassis.yaw_traveled):.0f}°")
