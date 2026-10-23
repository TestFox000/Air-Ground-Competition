import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drone_id = "0"

    ego_share_dir = FindPackageShare("ego_planner").find("ego_planner")

    advanced_param_launch = os.path.join(
        ego_share_dir,
        "launch",
        "advanced_param.launch.py"
    )

    rviz_config = os.path.join(
        ego_share_dir,
        "config",
        "ros2_ego_rviz.rviz"
    )

    ego_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(advanced_param_launch),
        launch_arguments={
            "name_of_ego_planner_node": "drone_0_ego_planner_node",

            # 1: RViz goal
            # 2: preset waypoint mode
            "flight_type": "1",

            "drone_id": drone_id,

            # Real localization input
            "odom_world": "/localization",
            "grid_map/odom": "/localization",

            # Real point cloud input
            "grid_map/cloud": "/cloud_registered",

            # Unused for LiDAR point cloud mode
            "grid_map/pose": "/unused_pose",
            "grid_map/depth": "/unused_depth",

            # EGO outputs
            "planning/bspline": "/drone_0_planning/bspline",
            "planning/data_display": "/drone_0_planning/data_display",
            "grid_map/occupancy_inflate": "/drone_0_ego_planner_node/grid_map/occupancy_inflate",
            "optimal_list": "/drone_0_ego_planner_node/optimal_list",

            # Local map size
            "map_size_x_": "30.0",
            "map_size_y_": "30.0",
            "map_size_z_": "5.0",

            # Planning limits
            "planning_horizon": "7.0",
            "max_vel": "1.0",
            "max_acc": "1.0",
            "use_distinctive_trajs": "false",

            # Unused camera params, but advanced_param.launch.py requires them
            "cx": "320.0",
            "cy": "240.0",
            "fx": "387.0",
            "fy": "387.0",

            # Unused when flight_type = 1
            "point_num": "1",
            "point0_x": "0.0",
            "point0_y": "0.0",
            "point0_z": "1.0",
            "point1_x": "0.0",
            "point1_y": "0.0",
            "point1_z": "1.0",
            "point2_x": "0.0",
            "point2_y": "0.0",
            "point2_z": "1.0",
            "point3_x": "0.0",
            "point3_y": "0.0",
            "point3_z": "1.0",
            "point4_x": "0.0",
            "point4_y": "0.0",
            "point4_z": "1.0",

            "obj_num_set": "0",
        }.items()
    )

    traj_server_node = Node(
        package="ego_planner",
        executable="traj_server",
        output="screen",
        name="drone_0_traj_server",
        remappings=[
            ("position_cmd", "/drone_0_planning/pos_cmd"),
            ("planning/bspline", "/drone_0_planning/bspline"),
        ],
    )

    odom_visualization_node = Node(
        package="odom_visualization",
        executable="odom_visualization",
        output="screen",
        name="drone_0_odom_visualization",
        remappings=[
            ("odom", "/localization"),
            ("robot", "/drone_0_odom_visualization/robot"),
            ("optimal_list", "/drone_0_ego_planner_node/optimal_list"),
            ("path", "/drone_0_odom_visualization/path"),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        ego_planner,
        traj_server_node,
        odom_visualization_node,
        rviz_node,
    ])