import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                           LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """小车全模块综合启动，按依赖顺序依次拉起（日志统一输出到本终端，Ctrl+C 一次全停）：

       0s  底盘串口+TF        dzactuator   bringup.launch.py
       2s  摄像头驱动          dzactuator   camera.launch.py
       4s  定位               car_localization localization.launch.py -> /odom/localization
       5s  图像识别            det_node     det_node.launch.py      -> /offset_center
       7s  目标追寻/云台       dzrknn_dataprocess dzrknn_dataprocess.launch.py -> /CameMo_Conl
       9s  射击模块驱动        dzjudgment   judgment.launch.py       -> 订阅 LaserShot_Command
      11s  循迹移动            path_executor (python3 直跑)          -> /pursuitAngle

    注：旧避障雷达(rplidar)分支已移除 —— 本车无激光雷达(readme 装备说明)，
        旧激光导航栈(rplidar/slam_gmapping/navigation2/robot_localization)已移出工作空间。

    用法：
      ros2 launch dzactuator bringup_all.launch.py
      ros2 launch dzactuator bringup_all.launch.py track_kp:=6.0 max_shots:=10

    日志：每个节点以 [节点名-N] 前缀输出到本终端；各节点自己的 INFO/ERROR 即模块运行状态。
    """
    dzactuator_share = get_package_share_directory('dzactuator')
    det_node_share = get_package_share_directory('det_node')
    gimbal_share = get_package_share_directory('dzrknn_dataprocess')
    localization_share = get_package_share_directory('car_localization')

    # ---- 云台瞄准参数（命令行可覆盖）----
    track_kp = LaunchConfiguration('track_kp')
    track_kd = LaunchConfiguration('track_kd')
    patrol_speed_ratio = LaunchConfiguration('patrol_speed_ratio')
    max_shots = LaunchConfiguration('max_shots')
    track_deadzone_px = LaunchConfiguration('track_deadzone_px')
    ema_alpha = LaunchConfiguration('ema_alpha')
    pitch_deg_sign = LaunchConfiguration('pitch_deg_sign')

    # 0) 底盘串口 + TF（最先起，其余节点依赖 TF）
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dzactuator_share, 'launch', 'bringup.launch.py')))

    # 2) 摄像头
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dzactuator_share, 'launch', 'camera.launch.py')))

    # 4) 定位（serial_reader -> /odom/localization，循迹的前置）
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_share, 'launch', 'localization.launch.py')))

    # 5) 图像识别
    det_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(det_node_share, 'launch', 'det_node.launch.py')))

    # 6) 目标追寻 / 云台（透传瞄准参数）
    gimbal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gimbal_share, 'launch', 'dzrknn_dataprocess.launch.py')),
        launch_arguments={
            'track_kp': track_kp,
            'track_kd': track_kd,
            'patrol_speed_ratio': patrol_speed_ratio,
            'max_shots': max_shots,
            'track_deadzone_px': track_deadzone_px,
            'ema_alpha': ema_alpha,
            'pitch_deg_sign': pitch_deg_sign,
        }.items())

    # 7) 射击模块驱动（订阅 LaserShot_Command，经串口 /dev/dzjudgment 到 ESP32 实际开火）
    judgment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dzactuator_share, 'launch', 'judgment.launch.py')))

    # 8) 循迹移动（path_executor 是 python3 直跑，没有 console_scripts 入口，用 ExecuteProcess）
    path_executor = ExecuteProcess(
        cmd=['python3',
             '/home/duzhong/dzrobot/src/car_localization/car_localization/path_executor.py'],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('track_kp', default_value='8.0',
                              description='PD 比例增益'),
        DeclareLaunchArgument('track_kd', default_value='3.0',
                              description='PD 微分增益(阻尼)'),
        DeclareLaunchArgument('patrol_speed_ratio', default_value='0.75',
                              description='扫描速度 = 云台最大转速 x 该比例'),
        DeclareLaunchArgument('max_shots', default_value='10',
                              description='全程弹药上限(硬上限 10)'),
        DeclareLaunchArgument('track_deadzone_px', default_value='12',
                              description='跟踪死区(像素)'),
        DeclareLaunchArgument('ema_alpha', default_value='0.3',
                              description='视觉低通滤波系数'),
        DeclareLaunchArgument('pitch_deg_sign', default_value='1.0',
                              description='竖直角度方向(反了改 -1.0)'),

        # 依次延迟启动，给前一个模块留初始化时间
        bringup,
        TimerAction(period=2.0, actions=[camera]),
        TimerAction(period=4.0, actions=[localization]),
        TimerAction(period=5.0, actions=[det_node]),
        TimerAction(period=7.0, actions=[gimbal]),
        TimerAction(period=9.0, actions=[judgment]),
        TimerAction(period=11.0, actions=[path_executor]),
        TimerAction(period=13.0, actions=[
            LogInfo(msg='===== 全部模块已启动完毕，可开始发送路线 =====')]),
    ])
