import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    serial_port_name = LaunchConfiguration('serial_port_name',default= '/dev/dzactuator')
    serial_baud_rate = LaunchConfiguration('serial_baud_rate',default= 115200)
    odom_frame_id = LaunchConfiguration('odom_frame_id',default= 'odom')
    robot_frame_id = LaunchConfiguration('robot_frame_id',default= 'base_link')
    gyro_frame_id = LaunchConfiguration('gyro_frame_id',default= 'imu_link')
    calibrate_lineSpeed = LaunchConfiguration('calibrate_lineSpeed',default= 0)
    ticksPerMeter = LaunchConfiguration('ticksPerMeter',default= 3290.0)
    ticksPer2PI = LaunchConfiguration('ticksPer2PI',default= 3236.0)

    # ✅ 声明 EKF 参数文件
    ekf_params = DeclareLaunchArgument(
        'ekf_params',
        default_value=os.path.join(
            get_package_share_directory('robot_localization'),
            'params',
            'ekf_0nly.yaml'
        ),
        description='Path to EKF parameters file'
    )

    return LaunchDescription([

        ekf_params,   # ✅ ★ 必须加入！

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ['0.0','0.0','0.08','0.0','0.0','0.0','base_link','imu_link']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ['0.15','0.0','0.07','3.14159','0.0','0.0','base_link','laser']
        ),


        Node(
            package='dzactuator',
            executable='dzactuator_node',
            name='dzactuator',
            output="screen",
            emulate_tty=True,
            parameters=[{
                'serial_port_name': serial_port_name,
                'serial_baud_rate': serial_baud_rate,
                'odom_frame_id': odom_frame_id,
                'robot_frame_id': robot_frame_id,
                'gyro_frame_id': gyro_frame_id,
                'calibrate_lineSpeed': calibrate_lineSpeed,
                'ticksPerMeter': ticksPerMeter,
                'ticksPer2PI': ticksPer2PI}],
        ),

        # ✅ EKF 融合，提供 odom → base_link TF
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[
        #         LaunchConfiguration('ekf_params'),
        #         {'use_sim_time': False}
        #     ]
        # ),

        # 雷达启动
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('rplidar_ros'),
        #             'launch/rplidar_s3e_launch.py'))
        # ),

    ])
