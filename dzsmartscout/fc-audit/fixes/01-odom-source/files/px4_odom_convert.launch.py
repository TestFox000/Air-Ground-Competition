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
        # ---------------------------------------------------------------------
        # 改动（补丁 01）：默认值 'localization' -> 'odom'
        #
        # 原因：/localization 由 fast_lio_localization/scripts/transform_fusion.py
        # 发布，而该脚本 import tf_transformations 失败（机上无此模块，
        # 兜底的 `from tf2_ros import transformations` 也不存在），进程启动即退出。
        # 结果：/localization 从未被发布过，本节点收不到任何输入，
        #       PX4 从来拿不到外部视觉。
        #
        # 'odom' 订阅 fastlio_mapping 发布的 /Odometry
        #       （src/laserMapping.cpp:1109），该话题是活的。
        #
        # 为什么现在两者等价：transform_fusion.py 在从未收到 /map_to_odom 时
        # 取 T_map_to_odom = np.eye(4)（该文件第 54 行），此时
        # /localization 与 /Odometry 逐位相同。所以切到 odom 在数学上
        # 不改变任何输入值，只是去掉了两个已死的 Python 节点。
        #
        # 等 tf_transformations / open3d 补齐、重定位链路真正跑起来之后，
        # 再把默认值切回 'localization' 即可启用带地图修正的重定位。
        # 临时用法（不必改文件）：在启动命令后加 mode:=odom
        # ---------------------------------------------------------------------
        DeclareLaunchArgument(
            'mode',
            default_value='odom',
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
