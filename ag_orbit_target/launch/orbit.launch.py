import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = "ag_orbit_target"
    # 从安装后的 share 目录取参数文件，不再硬编码工作区绝对路径
    config = os.path.join(
        get_package_share_directory(pkg), "config", "orbit_params.yaml",
    )
    node = Node(
        package=pkg,
        executable="orbit_node",
        name="orbit_node",
        output="screen",
        parameters=[config],
        emulate_tty=True,
    )
    return LaunchDescription([node])
