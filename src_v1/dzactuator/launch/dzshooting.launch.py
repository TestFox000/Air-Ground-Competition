from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    生成dzrknn_dataprocess节点的launch文件
    """
    
    # 声明launch参数
    motor_kp_arg = DeclareLaunchArgument(
        'motor_kp',
        default_value='1000.0',
        description='PID控制器的比例增益'
    )
    
    motor_ki_arg = DeclareLaunchArgument(
        'motor_ki',
        default_value='1000.0',
        description='PID控制器的积分增益'
    )
    
    motor_kd_arg = DeclareLaunchArgument(
        'motor_kd',
        default_value='1000.0',
        description='PID控制器的微分增益'
    )
    
    ticks_per_meter_arg = DeclareLaunchArgument(
        'ticksPerMeter',
        default_value='1000.0',
        description='每米的编码器脉冲数'
    )
    
    pose_kp_arg = DeclareLaunchArgument(
        'pose_kp',
        default_value='1000.0',
        description='位置控制的比例系数'
    )
    
    # 创建节点
    dzrknn_dataprocess_node = Node(
        package='dzrknn_dataprocess',
        executable='dzactuator_node',
        name='dzrknn_dataprocess',
        output='screen',
        parameters=[{
            'motor_kp': LaunchConfiguration('motor_kp'),
            'motor_ki': LaunchConfiguration('motor_ki'),
            'motor_kd': LaunchConfiguration('motor_kd'),
            'ticksPerMeter': LaunchConfiguration('ticksPerMeter'),
            'pose_kp': LaunchConfiguration('pose_kp'),
        }]
    )

    dzjudgment_node = Node(
        package='dzjudgment',
        executable='dzjudgment',
        name='dzjudgment',
        output='screen',
        parameters=[
            {"mcubaudrate": 115200},
            {"mcuserialport": "/dev/dzjudgment"}, 
        ]
    )

    
    return LaunchDescription([
        motor_kp_arg,
        motor_ki_arg,
        motor_kd_arg,
        ticks_per_meter_arg,
        pose_kp_arg,
        dzrknn_dataprocess_node,
        dzjudgment_node,
    ])

