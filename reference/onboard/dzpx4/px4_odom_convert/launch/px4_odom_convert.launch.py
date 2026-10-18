from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)

    params = {
        'source_mode':           mode,
        'odom_topic':            LaunchConfiguration('odom_topic').perform(context),
        'localization_topic':    LaunchConfiguration('localization_topic').perform(context),
        'tf_map_frame':          LaunchConfiguration('tf_map_frame').perform(context),
        'tf_lidar_frame':        LaunchConfiguration('tf_lidar_frame').perform(context),
        'z_offset_m':            float(LaunchConfiguration('z_offset_m').perform(context)),
        'loop_rate_hz':          50.0,
        'max_position_jump_m':   0.5,
        'max_attitude_jump_rad': 0.5,
    }

    return [Node(
        package='px4_odom_convert',
        executable='px4_odom_convert',
        name='px4_odom_convert',
        parameters=[params],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='localization',
            description='Localization source: odom | localization | tf'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/Odometry',
            description='[odom mode] topic to subscribe (nav_msgs/Odometry)'),
        DeclareLaunchArgument(
            'localization_topic',
            default_value='/localization',
            description='[localization mode] topic to subscribe (nav_msgs/Odometry)'),
        DeclareLaunchArgument(
            'tf_map_frame',
            default_value='map',
            description='[tf mode] parent frame of the TF lookup'),
        DeclareLaunchArgument(
            'tf_lidar_frame',
            default_value='lidar',
            description='[tf mode] child frame of the TF lookup'),
        DeclareLaunchArgument(
            'z_offset_m',
            default_value='0.0',
            description='Z offset subtracted from lidar height to get drone height (m), applied in all modes'),
        OpaqueFunction(function=launch_setup),
    ])
