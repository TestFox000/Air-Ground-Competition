import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# ============================================================================
# dzlink: MID-360 以「标准 PointCloud2」模式启动
#   与厂商 msg_MID360_launch.py 完全一致，唯一改动：xfer_format 1 -> 0
#     xfer_format = 0  -> /livox/lidar 为 sensor_msgs/msg/PointCloud2（PointXYZRTL）
#     xfer_format = 1  -> /livox/lidar 为 livox_ros_driver2/msg/CustomMsg
#   用途：让未安装 livox_ros_driver2 的跨机消费方（小车）开箱即用地吃点云。
#   回退：ros2 launch livox_ros_driver2 msg_MID360_launch.py  （即恢复 CustomMsg）
# ============================================================================
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar
publish_freq  = 10.0
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = '47MCN850037605'      # 与原 msg_MID360_launch.py 一致（本机这台雷达）

# 不用 __file__ 相对路径：本文件放在 /tmp，相对路径会失效
user_config_path = os.path.join(
    get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json')

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
    )

    # 故意不启动 rviz2（机载无显示，白耗 CPU）
    return LaunchDescription([livox_driver])
