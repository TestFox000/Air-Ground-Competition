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
            package='dzcontrol',
            executable='dzcontrol',
            name='dzcontrol',
            output="screen",
            emulate_tty=True,
            parameters=[
                # {"my_parameter": "earth"}
                {"saveMapping": False,  # False True
                "charge_Battery_Percentage": 0.3,
                "discharge_Battery_Percentage": 0.4,
                "savePath": "/home/duzhong/dzrobot/src/naviPointPath/mapping.txt",
                "saveStopPointPath": "/home/duzhong/dzrobot/src/naviPointPath/stopmapping.txt",
                "backcar_roadPoints_savePath":  "/home/duzhong/dzrobot/src/naviPointPath/points/Parking_points.txt",
                "backcar_backPoints_savePath":  "/home/duzhong/dzrobot/src/naviPointPath/points/Parking_points.txt",
                "backcar_stopPoints_savePath": "/home/duzhong/dzrobot/src/naviPointPath/points/backcar_stopPoints.txt",
                "charge_points_savePath":  "/home/duzhong/dzrobot/src/naviPointPath/points/Parking_points.txt",
                "Parking_points_savePath": "/home/duzhong/dzrobot/src/naviPointPath/points/Parking_points.txt",
                "backcar_Paths_savePath": "/home/duzhong/dzrobot/src/naviPointPath/backcar_paths/",
                "forwardcar_Paths_savePath": "/home/duzhong/dzrobot/src/naviPointPath/forwardcar_paths/",
                "robot_base_frame":  "base_link",
                "robot_id_":  "",
                "global_frame":  "world",
                "carport_stopX":  999.0,
                "carport_stopY":  999.0,
                "carport_stopZ":  0.0,
                "carport_heading":  1.0,
                "enable_backcar":  True,
                "cout_hglocation":  False,
                "printf_rech_redgreenled_flag":  False,
                "pursuitSpeed":   0.8,
                "out_storage_speed":  0.5,
                "put_storage_speed":  -0.5,
                "far_min_dis":  0.20,
                "near_min_dis":  0.20,
                "min_angle":  0.3,
                "reduct_ratio":  0.6,
                "target_min_dis":  1.0,
                "minDis_param":  0.10,
                "obstacle_x_max_dis":  0.50,
                "obstacle_y_max_dis":  0.06,
                "is_backcar":  1,
                "angle_Kp":  1.8,
                "diffangle_Kp": 1.15,
                "kp_backControl_speed": 1.4,
                "kp_backControl_Angle": 1.2,
                # pathplanner
                'maping':routemap,
                'bdebug':bdebug,
                'pathplannerloopfrep':loopfrep,
                'second_mindis_path':second_mindis_path,
                'min_serch_distance':min_serch_distance,
                'min_diffangle':min_diffangle,
                'startpoint_flag_param':startpoint_flag_param,
                'erase_point_dis':erase_point_dis,
                # robotaxi
                'run_rate': 20
                }],
            remappings=[("/trafficLightArray", "/trafficLightArray"),
                ("/parkingSpaceStatusArray", "/parkingSpaceStatusArray")],
        )
    ])
