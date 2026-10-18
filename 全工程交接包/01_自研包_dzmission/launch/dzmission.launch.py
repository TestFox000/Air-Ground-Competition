#!/usr/bin/env python3
"""dzmission 任务编排层启动文件

启动内容（按需裁剪）：
  1. dzmission_node            任务状态机 + 运动仲裁 + 定位补偿 + 避碰 + 云台 + 裁判板
  2. nav2 navigation           由 use_nav2 控制（navigate_to_pose 动作服务）
  3. 话题重映射                nav2 的 cmd_vel 与 dztrack 的速度输出必须让位给本节点

关键约定：
  - 本节点是 /cmd_vel 与 /pursuitAngle 的唯一发布者
  - nav2 velocity_smoother 的 cmd_vel 输出改到 /cmd_vel_nav_raw
  - dztrack 的 vel_pub_ 输出改到 /dztrack_raw_cmd（需改一行 dztrack 源码，
    详见 doc/INTEGRATION.md）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('dzmission')
    default_params = os.path.join(pkg_share, 'params', 'dzmission.yaml')

    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_nav2 = LaunchConfiguration('use_nav2')
    log_level = LaunchConfiguration('log_level')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='dzmission 参数文件绝对路径')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='',
        description='顶层 namespace（多车时使用）')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='使用仿真时钟')

    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2', default_value='true',
        description='是否同时启动 nav2 导航栈（NAV_TO 指令依赖 navigate_to_pose 动作）')

    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='日志级别')

    # -------------------------------------------------------------------------
    # 1. dzmission 主节点
    # -------------------------------------------------------------------------
    dzmission_node = Node(
        package='dzmission',
        executable='dzmission_node',
        name='dzmission',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            # 让 dzactuator 的遥控话题与本地一致（若已加 namespace 则去掉前缀）
            ('logicontrol', 'logicontrol'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # -------------------------------------------------------------------------
    # 2. nav2 导航栈（可选）
    #    velocity_smoother 的 cmd_vel_smoothed 被重映射到 /cmd_vel_nav_raw，
    #    由 dzmission 的仲裁器决定是否真正下发到 /cmd_vel
    # -------------------------------------------------------------------------
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params = LaunchConfiguration('nav2_params_file')

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
        description='nav2 参数文件')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        condition=IfCondition(use_nav2),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        declare_params_file,
        declare_namespace,
        declare_use_sim_time,
        declare_use_nav2,
        declare_log_level,
        declare_nav2_params,
        nav2_launch,
        dzmission_node,
    ])
