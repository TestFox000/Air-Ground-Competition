import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明地图文件路径参数
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/duzhong/dzrobot/src/maps/duzhong1110.yaml',
        description='Path to the localization map file'
    )

    # AMCL参数文件路径
    amcl_params = DeclareLaunchArgument(
        'amcl_params',
        default_value="/home/duzhong/dzrobot/src/dzactuator/params/amcl.yaml",
        description='Path to AMCL parameters file'
    )

    # EKF参数文件路径
    ekf_params = DeclareLaunchArgument(
        'ekf_params',
        default_value=os.path.join(
            get_package_share_directory('robot_localization'),
            'params',
            'ekf_only.yaml'
        ),
        description='Path to EKF parameters file'
    )


    # 声明命名空间参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('dzactuator'), 'launch', 'bringup.launch.py')
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('dzactuator'), 'launch', 'rplidar_s3e_launch.py')
        )
    )

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('dzactuator'), 'launch', 'amcl.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'amcl_params': LaunchConfiguration('amcl_params'),
            'namespace': LaunchConfiguration('namespace'),
            'ekf_params': LaunchConfiguration('ekf_params'),
        }.items()
    )


    return LaunchDescription([
        map_file_arg,
        namespace_arg,
        amcl_params,
        ekf_params,
        bringup_launch,
        lidar_launch,
        amcl_launch,
    ])