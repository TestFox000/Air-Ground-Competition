from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='fake_amcl_node',
            executable='fake_amcl_node',
            name='fake_amcl_node',
            output='screen',
            parameters=[
                {
                    'input_topic': '/odometry/filtered',
                    'output_topic': '/amcl_pose',
                    'frame_id': 'map',
                }
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            output='screen',
            arguments=[
                '0.0','0.0','0.0','0.0','0.0','0.0','map','odom'
            ]
        ),

    ])