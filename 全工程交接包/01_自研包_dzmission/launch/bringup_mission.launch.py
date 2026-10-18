#!/usr/bin/env python3
"""dzmission 比赛总启动文件（端到端联调入口）

★★ 为什么需要这个文件 ★★
  规则 image10：**准备阶段只有 2 分钟**，且比赛阶段全程禁止人工干预。
  若上场要手开 4~5 个终端，2 分钟根本不够，而且漏起一个节点就是零分。
  ⇒ 必须有一个**统一入口**一次性把比赛必需的节点全部拉起。

启动内容：
  ┌─ 相机      usb_cam / dzactuator camera.launch.py   （/image_raw）
  ├─ 检测      rknn_pt det_node                        （/offset_center）
  ├─ 驱动      dzactuator dzcontrol                    （底盘/云台/激光 + 裁判板串口）
  ├─ 定位      dzmission 内部（航位推算 + IMU 偏航 + anchor）
  └─ 编排      dzmission dzmission_node                （任务状态机 + 仲裁 + 避碰 + 瞄准）

★★ 互斥关系（重要，见 doc/INTEGRATION.md）★★
  本文件**故意不启动** `dzrknn_dataprocess`，因为它与 dzmission 冲突：
    · 它也订阅 /offset_center，并在回调里**直接驱动云台**（CameMo_Conl）
      -> 与 dzmission 的 TargetAimer 抢云台
    · 它也发布 /LaserShot_Command
      -> 两个节点都能触发开火（弹量只有 10 发，且可能未对准就开火）
    · 它**没有红蓝面联锁** -> 存在打友方（扣分）的风险
  若要用 dzrknn_dataprocess 的云台逻辑，就必须把 dzmission 的
  `aim.own_gimbal` 设为 false 并接受"无红蓝保护"。

用法：
  ros2 launch dzmission bringup_mission.launch.py
  # 只联调编排层（不起硬件）：
  ros2 launch dzmission bringup_mission.launch.py use_hardware:=false

参数（现场可调，尽量少）：
  use_hardware   是否启动相机/检测/驱动（台架联调可关）
  use_nav2       是否启动 nav2（本方案用格子图上规划，通常 false）
  params_file    dzmission 参数文件
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _try_include(pkg, launch_file, **kwargs):
    """若包/文件存在则返回 IncludeLaunchDescription，否则返回 None

    这样本文件在"缺某个包"时不会整体报错 —— 台架联调时常常只装一部分。
    """
    try:
        share = get_package_share_directory(pkg)
    except Exception:
        return None
    path = os.path.join(share, 'launch', launch_file)
    if not os.path.isfile(path):
        return None
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=kwargs.get('launch_arguments', {}).items()
        if kwargs.get('launch_arguments') else None,
        condition=kwargs.get('condition'))


def generate_launch_description():
    pkg_share = get_package_share_directory('dzmission')
    default_params = os.path.join(pkg_share, 'params', 'dzmission.yaml')

    params_file = LaunchConfiguration('params_file')
    use_hardware = LaunchConfiguration('use_hardware')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_det = LaunchConfiguration('use_det')
    log_level = LaunchConfiguration('log_level')

    args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='dzmission 参数文件绝对路径'),
        DeclareLaunchArgument(
            'use_hardware', default_value='true',
            description='是否启动相机/检测/底盘驱动（台架联调可设 false）'),
        DeclareLaunchArgument(
            'use_det', default_value='true',
            description='是否启动 rknn_pt 检测节点'),
        DeclareLaunchArgument(
            'use_nav2', default_value='false',
            description='是否启动 nav2（本方案用格子图规划，通常 false）'),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='日志级别'),
    ]

    actions = list(args)

    # ---------------------------------------------------------------
    # 0. 开场提示：把"必须检查的东西"打在屏幕上
    #    规则：准备阶段 2 分钟，比赛阶段禁止人工干预。
    # ---------------------------------------------------------------
    actions.append(LogInfo(msg=[
        "\n",
        "=" * 70, "\n",
        " dzmission 比赛总启动\n",
        " 上场前必查（准备阶段 2 分钟）：\n",
        "   1) ros2 topic echo /offset_center --once\n",
        "      -> 应有 10 个整数，[8]=面(0/1/2) [9]=激光红点(0/1)\n",
        "   2) ros2 topic hz /offset_center   -> 应约 15 Hz\n",
        "   3) ros2 topic echo /pursuitAngle  -> angular.z 范围 ±35（度，非 rad/s）\n",
        "   4) ros2 topic echo /referee/ammo  -> 应为 10\n",
        "   5) 确认 **未启动** dzrknn_dataprocess（否则抢云台/抢激光）\n",
        "=" * 70, "\n",
    ]))

    # ---------------------------------------------------------------
    # 1. 相机（/image_raw）
    #    det_node 订阅 /image_raw；相机节点通常不在本工作空间（usb_cam）。
    # ---------------------------------------------------------------
    cam = _try_include(
        'dzactuator', 'camera.launch.py',
        condition=IfCondition(use_hardware))
    if cam is not None:
        actions.append(cam)
    else:
        actions.append(LogInfo(msg=[
            "[bringup] 未找到 dzactuator/camera.launch.py —— "
            "请确认相机节点（/image_raw）已由外部启动"]))

    # ---------------------------------------------------------------
    # 2. 视觉检测（rknn_pt/det_node -> /offset_center）
    #    延迟 300ms 启动，让相机先就绪
    # ---------------------------------------------------------------
    det = _try_include(
        'rknn_pt', 'det_node.launch.py',
        condition=IfCondition(use_det))
    if det is not None:
        actions.append(TimerAction(period=3.0, actions=[det]))
    else:
        actions.append(LogInfo(msg=[
            "[bringup] 未找到 rknn_pt/det_node.launch.py —— "
            "无视觉则无法排雷/打击"]))

    # ---------------------------------------------------------------
    # 3. 底盘/云台/激光/裁判板驱动（dzactuator dzcontrol）
    #    ★ 它订阅 /cmd_vel 与 /pursuitAngle —— 由 dzmission 仲裁后发布
    # ---------------------------------------------------------------
    drv = _try_include(
        'dzactuator', 'dzcontrol.launch.py',
        condition=IfCondition(use_hardware))
    if drv is not None:
        actions.append(drv)
    else:
        actions.append(LogInfo(msg=[
            "[bringup] 未找到 dzactuator/dzcontrol.launch.py —— "
            "底盘/云台将无驱动"]))

    # ---------------------------------------------------------------
    # 4. dzmission 主节点（任务编排 + 仲裁 + 避碰 + 瞄准）
    #    ★ 它是 /cmd_vel 与 /pursuitAngle 的**唯一**发布者
    # ---------------------------------------------------------------
    dzmission_node = Node(
        package='dzmission',
        executable='dzmission_node',
        name='dzmission',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            # dztrack 的速度输出必须让位（见 doc/INTEGRATION.md）
            ('dztrack_raw_cmd', 'dztrack_raw_cmd'),
        ],
    )
    actions.append(dzmission_node)

    # ---------------------------------------------------------------
    # 5. nav2（可选，默认关）
    # ---------------------------------------------------------------
    nav2 = _try_include(
        'nav2_bringup', 'navigation_launch.py',
        condition=IfCondition(use_nav2),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
        })
    if nav2 is not None:
        actions.append(nav2)

    actions.append(LogInfo(msg=[
        "[bringup] 启动顺序：相机 -> (3s) 检测 -> 驱动 -> dzmission",
        "          ★ dzmission 是 /cmd_vel 与 /pursuitAngle 的唯一发布者",
    ]))

    return LaunchDescription(actions)
