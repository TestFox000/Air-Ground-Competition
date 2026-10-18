from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    生成 dzrknn_dataprocess 节点的 launch 文件
    """

    # ============ 电机/云台基础参数 ============
    motor_kp_arg = DeclareLaunchArgument(
        'motor_kp', default_value='20.0',
        description='云台速度控制比例增益'
    )
    motor_ki_arg = DeclareLaunchArgument(
        'motor_ki', default_value='0.0',
        description='云台速度控制积分增益（当前仅用 P，默认 0）'
    )
    motor_kd_arg = DeclareLaunchArgument(
        'motor_kd', default_value='0.0',
        description='云台速度控制微分增益（当前仅用 P，默认 0）'
    )
    pose_kp_arg = DeclareLaunchArgument(
        'pose_kp', default_value='1000.0',
        description='位置控制比例系数'
    )

    # ============ 扫描水平参数 ============
    scan_yaw_dirs_arg = DeclareLaunchArgument(
        'scan_yaw_dirs', default_value='8',
        description='水平扫描点数'
    )
    scan_yaw_min_deg_arg = DeclareLaunchArgument(
        'scan_yaw_min_deg', default_value='-90.0',
        description='水平扫描最小角(度)'
    )
    scan_yaw_max_deg_arg = DeclareLaunchArgument(
        'scan_yaw_max_deg', default_value='90.0',
        description='水平扫描最大角(度)'
    )

    # ============ 扫描竖直参数 ============
    # 竖直层角度列表，ROS2 参数不支持直接在 DeclareLaunchArgument 里传列表默认值，
    # 这里通过字符串在节点侧解析，或直接在 parameters 里写数值列表。
    scan_pitch_levels_arg = DeclareLaunchArgument(
        'scan_pitch_levels', default_value='[30.0, 0.0, -10.0]',
        description='竖直扫描层角度列表(相对水平面，上为正)'
    )

    # ============ 云台方向与刻度换算 ============
    yaw_deg_sign_arg = DeclareLaunchArgument(
        'yaw_deg_sign', default_value='1.0',
        description='水平角度正方向 -> 刻度方向(反了改 -1)'
    )
    pitch_deg_sign_arg = DeclareLaunchArgument(
        'pitch_deg_sign', default_value='1.0',
        description='竖直角度正方向 -> 刻度方向(反了改 -1)'
    )
    gimbal_center_tick_arg = DeclareLaunchArgument(
        'gimbal_center_tick', default_value='2047',
        description='云台零位刻度'
    )
    yaw_ticks_per_deg_arg = DeclareLaunchArgument(
        'yaw_ticks_per_deg', default_value='11.111',
        description='水平刻度/度'
    )
    pitch_ticks_per_deg_arg = DeclareLaunchArgument(
        'pitch_ticks_per_deg', default_value='11.111',
        description='竖直刻度/度'
    )

    # ============ 云台安全限位与速度 ============
    gimbal_tick_min_arg = DeclareLaunchArgument(
        'gimbal_tick_min', default_value='1000',
        description='云台刻度安全下限'
    )
    gimbal_tick_max_arg = DeclareLaunchArgument(
        'gimbal_tick_max', default_value='3000',
        description='云台刻度安全上限'
    )
    gimbal_max_speed_arg = DeclareLaunchArgument(
        'gimbal_max_speed', default_value='3700',
        description='云台最大转速'
    )
    scan_speed_ratio_arg = DeclareLaunchArgument(
        'scan_speed_ratio', default_value='0.3',
        description='扫描转速比例(相对最大转速)'
    )

    # ============ 扫描节奏 ============
    scan_dwell_ms_arg = DeclareLaunchArgument(
        'scan_dwell_ms', default_value='300',
        description='每点最短驻留(ms)'
    )
    scan_arrive_tol_arg = DeclareLaunchArgument(
        'scan_arrive_tol', default_value='30',
        description='扫描到位容差(刻度)'
    )
    lost_frames_to_rescan_arg = DeclareLaunchArgument(
        'lost_frames_to_rescan', default_value='60',
        description='追踪中丢帧回扫描阈值'
    )
    vision_timeout_ms_arg = DeclareLaunchArgument(
        'vision_timeout_ms', default_value='1000',
        description='视觉静默超时(ms)'
    )
    fire_tolerance_px_arg = DeclareLaunchArgument(
        'fire_tolerance_px', default_value='10',
        description='开火像素容差'
    )
    laser_duration_ms_arg = DeclareLaunchArgument(
        'laser_duration_ms', default_value='100',
        description='激光高电平持续时间(ms)'
    )
 # ===== 新增 =====
    fire_debounce_frames_arg = DeclareLaunchArgument(
        'fire_debounce_frames', default_value='3',
        description='连续命中容差的帧数阈值(防抖)'
    )
    fire_cooldown_ms_arg = DeclareLaunchArgument(
        'fire_cooldown_ms', default_value='500',
        description='两次开火最小间隔(ms，冷却)'
    )

    # ===== 追踪平滑与死区 =====
    vision_deadband_px_arg = DeclareLaunchArgument(
        'vision_deadband_px', default_value='3',
        description='视觉误差死区(px)，小于该值视为已对中'
    )
    position_deadband_ticks_arg = DeclareLaunchArgument(
        'position_deadband_ticks', default_value='5',
        description='位置误差死区(tick)，小于该值不再驱动'
    )
    vision_smooth_alpha_arg = DeclareLaunchArgument(
        'vision_smooth_alpha', default_value='0.4',
        description='视觉误差EMA系数(0~1，越小越平滑但越滞后)'
    )
    vision_step_gain_arg = DeclareLaunchArgument(
        'vision_step_gain', default_value='0.3',
        description='每帧步进增益(tick/px)'
    )
    max_step_per_frame_arg = DeclareLaunchArgument(
        'max_step_per_frame', default_value='20',
        description='每帧位置最大增量(tick)'
    )
    min_tracking_speed_arg = DeclareLaunchArgument(
        'min_tracking_speed', default_value='0',
        description='追踪最小速度(0=不强制，避免极限环)'
    )

    pitch_motor_kp_arg = DeclareLaunchArgument(
        'pitch_motor_kp', default_value='5.0',
        description='俯仰轴比例增益（固定高度靶，应远小于 yaw）'
    )
    pitch_max_speed_arg = DeclareLaunchArgument(
        'pitch_max_speed', default_value='200',
        description='俯仰最大转速'
    )
    pitch_step_gain_arg = DeclareLaunchArgument(
        'pitch_step_gain', default_value='0.1',
        description='俯仰每像素步进增益(tick/px)'
    )
    pitch_deadband_px_arg = DeclareLaunchArgument(
        'pitch_deadband_px', default_value='8',
        description='俯仰视觉死区(px)'
    )
    pitch_position_deadband_ticks_arg = DeclareLaunchArgument(
        'pitch_position_deadband_ticks', default_value='15',
        description='俯仰位置死区(tick)'
    )
    pitch_max_step_per_frame_arg = DeclareLaunchArgument(
        'pitch_max_step_per_frame', default_value='3',
        description='每帧俯仰最大增量(tick)'
    )
    pitch_scan_speed_arg = DeclareLaunchArgument(
        'pitch_scan_speed', default_value='100',
        description='俯仰扫描转速'
    )

    fast_error_threshold_px_arg = DeclareLaunchArgument(
        'fast_error_threshold_px', default_value='40',
        description='超过此像素误差进入快速追踪模式'
    )
    fast_alpha_arg = DeclareLaunchArgument(
        'fast_alpha', default_value='0.85',
        description='快速模式 EMA 系数(0~1, 越大越跟手)'
    )
    fast_step_gain_arg = DeclareLaunchArgument(
        'fast_step_gain', default_value='0.7',
        description='快速模式步进增益(tick/px)'
    )
    fast_max_step_arg = DeclareLaunchArgument(
        'fast_max_step', default_value='80',
        description='快速模式单帧最大步进(tick)'
    )
    fast_motor_kp_arg = DeclareLaunchArgument(
        'fast_motor_kp', default_value='50.0',
        description='快速模式比例增益'
    )

    # ============ 节点 ============
    # 注意：executable 必须与 CMakeLists.txt 中 add_executable(...) 的名字一致
    dzrknn_dataprocess_node = Node(
        package='dzrknn_dataprocess',
        executable='dzactuator_node',   # ← 若 CMakeLists 里名字不同，请改这里
        name='dzrknn_dataprocess',
        output='screen',
        parameters=[{
            # 电机 / 云台基础
            'motor_kp': ParameterValue(LaunchConfiguration('motor_kp'), value_type=float),
            'motor_ki': ParameterValue(LaunchConfiguration('motor_ki'), value_type=float),
            'motor_kd': ParameterValue(LaunchConfiguration('motor_kd'), value_type=float),
            'pose_kp':  ParameterValue(LaunchConfiguration('pose_kp'),  value_type=float),

            # 水平扫描
            'scan_yaw_dirs':    ParameterValue(LaunchConfiguration('scan_yaw_dirs'),    value_type=int),
            'scan_yaw_min_deg': ParameterValue(LaunchConfiguration('scan_yaw_min_deg'), value_type=float),
            'scan_yaw_max_deg': ParameterValue(LaunchConfiguration('scan_yaw_max_deg'), value_type=float),

            # 竖直扫描层（列表）
            'scan_pitch_levels': ParameterValue(
                LaunchConfiguration('scan_pitch_levels'), value_type=list
            ),

            # 方向与刻度换算
            'yaw_deg_sign':        ParameterValue(LaunchConfiguration('yaw_deg_sign'),        value_type=float),
            'pitch_deg_sign':      ParameterValue(LaunchConfiguration('pitch_deg_sign'),      value_type=float),
            'gimbal_center_tick':  ParameterValue(LaunchConfiguration('gimbal_center_tick'),  value_type=int),
            'yaw_ticks_per_deg':   ParameterValue(LaunchConfiguration('yaw_ticks_per_deg'),   value_type=float),
            'pitch_ticks_per_deg': ParameterValue(LaunchConfiguration('pitch_ticks_per_deg'), value_type=float),

            # 安全限位与速度
            'gimbal_tick_min':  ParameterValue(LaunchConfiguration('gimbal_tick_min'),  value_type=int),
            'gimbal_tick_max':  ParameterValue(LaunchConfiguration('gimbal_tick_max'),  value_type=int),
            'gimbal_max_speed': ParameterValue(LaunchConfiguration('gimbal_max_speed'), value_type=int),
            'scan_speed_ratio': ParameterValue(LaunchConfiguration('scan_speed_ratio'), value_type=float),

            # 扫描节奏与鲁棒性
            'scan_dwell_ms':          ParameterValue(LaunchConfiguration('scan_dwell_ms'),          value_type=int),
            'scan_arrive_tol':        ParameterValue(LaunchConfiguration('scan_arrive_tol'),        value_type=int),
            'lost_frames_to_rescan':  ParameterValue(LaunchConfiguration('lost_frames_to_rescan'),  value_type=int),
            'vision_timeout_ms':      ParameterValue(LaunchConfiguration('vision_timeout_ms'),      value_type=int),
            'fire_tolerance_px':      ParameterValue(LaunchConfiguration('fire_tolerance_px'),      value_type=int),
            'laser_duration_ms':      ParameterValue(LaunchConfiguration('laser_duration_ms'),      value_type=int),
            
            'fire_tolerance_px':      ParameterValue(LaunchConfiguration('fire_tolerance_px'),      value_type=int),
            'laser_duration_ms':      ParameterValue(LaunchConfiguration('laser_duration_ms'),      value_type=int),
            'fire_debounce_frames':   ParameterValue(LaunchConfiguration('fire_debounce_frames'),   value_type=int),
            'fire_cooldown_ms':       ParameterValue(LaunchConfiguration('fire_cooldown_ms'),       value_type=int),
            
            'vision_deadband_px':      ParameterValue(LaunchConfiguration('vision_deadband_px'),      value_type=int),
            'position_deadband_ticks': ParameterValue(LaunchConfiguration('position_deadband_ticks'), value_type=int),
            'vision_smooth_alpha':     ParameterValue(LaunchConfiguration('vision_smooth_alpha'),     value_type=float),
            'vision_step_gain':        ParameterValue(LaunchConfiguration('vision_step_gain'),        value_type=float),
            'max_step_per_frame':      ParameterValue(LaunchConfiguration('max_step_per_frame'),      value_type=int),
            'min_tracking_speed':      ParameterValue(LaunchConfiguration('min_tracking_speed'),      value_type=int),
            
            'pitch_motor_kp':                 ParameterValue(LaunchConfiguration('pitch_motor_kp'),                 value_type=float),
            'pitch_max_speed':                ParameterValue(LaunchConfiguration('pitch_max_speed'),                value_type=int),
            'pitch_step_gain':                ParameterValue(LaunchConfiguration('pitch_step_gain'),                value_type=float),
            'pitch_deadband_px':              ParameterValue(LaunchConfiguration('pitch_deadband_px'),              value_type=int),
            'pitch_position_deadband_ticks':  ParameterValue(LaunchConfiguration('pitch_position_deadband_ticks'),  value_type=int),
            'pitch_max_step_per_frame':       ParameterValue(LaunchConfiguration('pitch_max_step_per_frame'),       value_type=int),
            'pitch_scan_speed':               ParameterValue(LaunchConfiguration('pitch_scan_speed'),               value_type=int),
            
            'fast_error_threshold_px': ParameterValue(LaunchConfiguration('fast_error_threshold_px'), value_type=int),
            'fast_alpha':              ParameterValue(LaunchConfiguration('fast_alpha'),              value_type=float),
            'fast_step_gain':          ParameterValue(LaunchConfiguration('fast_step_gain'),          value_type=float),
            'fast_max_step':           ParameterValue(LaunchConfiguration('fast_max_step'),           value_type=int),
            'fast_motor_kp':           ParameterValue(LaunchConfiguration('fast_motor_kp'),           value_type=float),
        }]
    )

    return LaunchDescription([
        # 基础
        motor_kp_arg, motor_ki_arg, motor_kd_arg, pose_kp_arg,
        # 水平
        scan_yaw_dirs_arg, scan_yaw_min_deg_arg, scan_yaw_max_deg_arg,
        # 竖直
        scan_pitch_levels_arg,
        # 方向/刻度
        yaw_deg_sign_arg, pitch_deg_sign_arg,
        gimbal_center_tick_arg, yaw_ticks_per_deg_arg, pitch_ticks_per_deg_arg,
        # 限位/速度
        gimbal_tick_min_arg, gimbal_tick_max_arg,
        gimbal_max_speed_arg, scan_speed_ratio_arg,
        # 节奏/鲁棒性
        scan_dwell_ms_arg, scan_arrive_tol_arg,
        lost_frames_to_rescan_arg, vision_timeout_ms_arg,
        fire_tolerance_px_arg, laser_duration_ms_arg,
        # 节点
        dzrknn_dataprocess_node,
        
        fire_tolerance_px_arg, laser_duration_ms_arg,
        fire_debounce_frames_arg, fire_cooldown_ms_arg,

        # 追踪平滑与死区
        vision_deadband_px_arg, position_deadband_ticks_arg,
        vision_smooth_alpha_arg, vision_step_gain_arg,
        max_step_per_frame_arg, min_tracking_speed_arg,

        pitch_motor_kp_arg, pitch_max_speed_arg, pitch_step_gain_arg,
        pitch_deadband_px_arg, pitch_position_deadband_ticks_arg,
        pitch_max_step_per_frame_arg, pitch_scan_speed_arg,

        fast_error_threshold_px_arg, fast_alpha_arg,
        fast_step_gain_arg, fast_max_step_arg, fast_motor_kp_arg,
    ])