import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    ekf_params_arg = DeclareLaunchArgument(
        'ekf_params',
        default_value=os.path.join(
            get_package_share_directory('robot_localization'),
            'params',
            'ekf_only.yaml'
        ),
        description='Path to EKF parameters file'
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dzactuator'),
                'launch',
                'ekf.launch.py'
            )
        ),
        launch_arguments={
            'ekf_params': LaunchConfiguration('ekf_params'),
        }.items()
    )

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
    
    # 声明命名空间参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )
    
    # 主要节点组 - 使用命名空间
    robot_nodes = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            
            # 地图服务器节点 - 在命名空间下
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
                    {'frame_id': [LaunchConfiguration('namespace'), '/map']},  # 带命名空间的坐标系
                    {'yaml_filename': LaunchConfiguration('map')},
                    {'topic_name': 'map'}  # 话题名在命名空间下
                ]
            ),
            
            # AMCL定位
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[
                    LaunchConfiguration('amcl_params'),
                    {'use_sim_time': False},
                ],
                # remappings=[
                    # ('scan', [LaunchConfiguration('namespace'), '/scan']),  # 激光数据话题
                    # ('initialpose', [LaunchConfiguration('namespace'), '/initialpose'])
                    # ("/initialpose", "/initialpose"),
                # ]
            ),

            # 生命周期管理器 - 在命名空间下管理地图服务器和AMCL
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}
                ]
            ),
            
        ]
    )
    
    return LaunchDescription([
        ekf_params_arg,
        ekf_launch,
        map_file_arg,
        namespace_arg,
        amcl_params,
        robot_nodes,
    ])