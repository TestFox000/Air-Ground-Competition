import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ekf_params = DeclareLaunchArgument(
        'ekf_params',
        default_value=os.path.join(
            get_package_share_directory('robot_localization'),
            'params',
            'ekf_only.yaml'
        ),
        description='Path to EKF parameters file'
    )

    return LaunchDescription([
        ekf_params,
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                LaunchConfiguration('ekf_params'),
                {'use_sim_time': False}
            ],
            # remappings=[('odometry/filtered', 'ekf_odom')]
        ),
    ])
