#!/usr/bin/env python3
"""dzmission 离线/台架自测启动文件

不依赖真实无人机链路、不依赖 nav2：
  - cmd.transport = loopback（自动注入正方形巡游 NAV_TO 指令）
  - own_gimbal = true（本节点驱动云台与激光，便于单独验证瞄准链路）
  - referee.direct_serial = false（等待 dzjudgment，若未运行则弹药恒为 0，开火被联锁拦住）

用法：
  ros2 launch dzmission dzmission_selftest.launch.py
配合（另开终端）：
  ros2 launch dzactuator bringup.launch.py        # 底盘 + IMU + 里程计
  ros2 launch dzactuator rplidar_s3e_launch.py    # 雷达
  ros2 launch dzactuator fake_amcl.launch.py      # 无真实定位时用里程计假定位
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('dzmission')
    params_file = LaunchConfiguration('params_file')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'params', 'dzmission.yaml'),
        description='参数文件（自测时会覆盖 transport / own_gimbal）')

    node = Node(
        package='dzmission',
        executable='dzmission_node',
        name='dzmission',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                'cmd.transport': 'loopback',
                'cmd.loopback_hz': 0.1,
                'aim.own_gimbal': True,
                'referee.direct_serial': False,
                'mission.enable_autonomy': True,
                # 台中测试时降低限幅，避免意外高速
                'arbiter.max_v': 0.25,
                'arbiter.max_w': 0.8,
                'safety.warn_dist': 0.9,
                'safety.stop_dist': 0.4,
            },
        ],
    )

    return LaunchDescription([declare_params_file, node])
