from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    save_path = DeclareLaunchArgument(
        'save_path',
        default_value='/home/duzhong/dzrobot/src/maps/my_map',
        description='Path (without extension) to save map'
    )

    return LaunchDescription([
        save_path,

        Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name='map_saver',
            output='screen',
            parameters=[{'save_map_timeout': 50000.0}],   # ← 改这里：加 .0
            arguments=['-f', LaunchConfiguration('save_path')]
        )
    ])
