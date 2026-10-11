from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 获取包路径

    # Launch 参数
    robot_color_arg = DeclareLaunchArgument(
        'robot_color',
        default_value='red',
        description='Robot color (red or blue)'
    )

    # 检测节点
    det_node = Node(
        package='det_node',
        executable='det_node',
        name='det_node',
        output='screen',
        parameters=[{
            'robot_color': LaunchConfiguration('robot_color'),
            # 注意：tracking 和 show_results 参数在当前代码中未使用，但保留以便将来扩展
            'tracking.max_prediction_time': 3.0,
            'tracking.max_lost_frames': 150,
            'show_results': True
        }]
    )
    
    return LaunchDescription([
        robot_color_arg,
        det_node,
    ])

