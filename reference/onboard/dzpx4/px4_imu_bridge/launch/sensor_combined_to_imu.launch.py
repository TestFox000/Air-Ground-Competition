from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    frame_id = LaunchConfiguration('frame_id')
    convert_frd_to_flu = LaunchConfiguration('convert_frd_to_flu')
    log_conversion_time = LaunchConfiguration('log_conversion_time')
    timing_log_period_ms = LaunchConfiguration('timing_log_period_ms')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/fmu/out/sensor_combined',
            description='PX4 SensorCombined input topic'),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/fmu/out/imu_data',
            description='ROS sensor_msgs/Imu output topic'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='body',
            description='Frame id used in the published Imu header'),
        DeclareLaunchArgument(
            'convert_frd_to_flu',
            default_value='true',
            description='Convert PX4 FRD body-frame vectors to ROS FLU'),
        DeclareLaunchArgument(
            'log_conversion_time',
            default_value='true',
            description='Print conversion and publish timing statistics'),
        DeclareLaunchArgument(
            'timing_log_period_ms',
            default_value='1000',
            description='Timing statistics print period in milliseconds'),
        Node(
            package='px4_imu_bridge',
            executable='sensor_combined_to_imu',
            name='sensor_combined_to_imu',
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,
                'frame_id': frame_id,
                'convert_frd_to_flu': convert_frd_to_flu,
                'log_conversion_time': log_conversion_time,
                'timing_log_period_ms': timing_log_period_ms,
            }],
        ),
    ])
