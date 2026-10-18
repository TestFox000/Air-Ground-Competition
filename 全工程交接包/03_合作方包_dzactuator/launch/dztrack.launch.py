import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    routemap = LaunchConfiguration('maping',default ='/home/dzrobot/dzrobot/src/naviPointPath/map')
    bdebug = LaunchConfiguration('bdebug',default = 1)
    loopfrep = LaunchConfiguration('pathplannerloopfrep',default = 1)
    second_mindis_path = LaunchConfiguration('second_mindis_path',default = 0.1)
    min_serch_distance = LaunchConfiguration('min_serch_distance',default = 0.2)
    min_diffangle= LaunchConfiguration('min_diffangle',default = 1.7)
    startpoint_flag_param = LaunchConfiguration('startpoint_flag_param',default = 1)
    erase_point_dis = LaunchConfiguration('erase_point_dis',default = 0.6)

    return LaunchDescription([
        # Node( 
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['0', '0', '1', '0', '0', '0', 'map', 'base_link']),
        Node(
            package='dztrack',
            executable='dztrack',
            name='dztrack',
            output="screen",
            emulate_tty=True,
            parameters=[
                # {"my_parameter": "earth"}
                {"saveMapping": False,  # False True
                "save_mapping_point_dis": 0.02,
                "savePath": "/home/duzhong/dzrobot/src/naviPointPath/mapping.txt",
                "saveStopPointPath": "/home/duzhong/dzrobot/src/naviPointPath/stopmapping.txt",
                "robot_base_frame":  "base_link",
                "global_frame":  "world",
                "cout_hglocation":  False,
                "pursuitSpeed":   0.8,
                "minDis_param":   0.1,
                "far_min_dis":  0.20,
                "near_min_dis":  0.20,
                "reduct_ratio":  0.6,
                "target_min_dis":  1.0,
                "obstacle_x_max_dis":  0.8,
                "obstacle_y_max_dis":  0.2,
                "is_backcar":  0,
                "angle_Kp":  1.8,
                "diffangle_Kp": 1.15,
                }],
            # remappings=[("/pursuitAngle", "/pursuitAngle_dztrack")],
        ),
        # Node(
        #     package='dztrack',
        #     executable='dztrack_path_bridge.py',
        #     name='dztrack_path_bridge',
        #     output='screen',
        # ),
    ])
