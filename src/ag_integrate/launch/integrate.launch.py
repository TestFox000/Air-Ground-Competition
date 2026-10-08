"""每局一键启动：ros2 launch ag_integrate integrate.launch.py

开赛前的地图先验全部集中在 config/match_params.yaml 里改，不动代码。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ag_integrate")
    params_file = os.path.join(share, "config", "match_params.yaml")
    return LaunchDescription([
        Node(
            package="ag_integrate",
            executable="integrate_node",
            name="integrate_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
