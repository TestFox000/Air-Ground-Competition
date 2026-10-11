from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    生成dzrknn_dataprocess节点的launch文件
    """
    
    # 声明launch参数
    # ---- 瞄准参数（命令行可覆盖，例：ros2 launch ... track_kp:=6.0）----
    ema_alpha_arg = DeclareLaunchArgument(
        'ema_alpha',
        default_value='0.3',
        description='视觉一阶低通滤波系数(0~1)，越小越平滑但延迟越大'
    )

    track_deadzone_px_arg = DeclareLaunchArgument(
        'track_deadzone_px',
        default_value='12',
        description='跟踪死区(像素)：偏差小于此值云台完全不动，消除抖动'
    )

    fire_deadzone_px_arg = DeclareLaunchArgument(
        'fire_deadzone_px',
        default_value='8',
        description='固定开火死区(像素)：未启用动态框时使用'
    )

    fire_dz_dynamic_arg = DeclareLaunchArgument(
        'fire_dz_dynamic',
        default_value='true',
        description='启用动态开火框：目标越近视差越大、框越宽'
    )

    fire_dz_far_px_arg = DeclareLaunchArgument(
        'fire_dz_far_px',
        default_value='6.0',
        description='远距离(约35cm)开火死区(像素)'
    )

    fire_dz_near_px_arg = DeclareLaunchArgument(
        'fire_dz_near_px',
        default_value='18.0',
        description='近距离(约10cm)开火死区(像素)'
    )

    box_w_far_arg = DeclareLaunchArgument(
        'box_w_far',
        default_value='30.0',
        description='远处目标框宽标定值(像素)'
    )

    box_w_near_arg = DeclareLaunchArgument(
        'box_w_near',
        default_value='120.0',
        description='近处目标框宽标定值(像素)'
    )

    fire_real_speed_max_arg = DeclareLaunchArgument(
        'fire_real_speed_max',
        default_value='20',
        description='下位机回传实际速度小于该值才算绝对静止'
    )

    fire_hold_frames_arg = DeclareLaunchArgument(
        'fire_hold_frames',
        default_value='5',
        description='连续满足开火条件多少帧才真正开火'
    )

    fire_use_image_stable_arg = DeclareLaunchArgument(
        'fire_use_image_stable',
        default_value='true',
        description='开火稳定判据：true=画面稳定(移动中也能开火)；false=云台电机绝对静止'
    )

    fire_stable_px_per_frame_arg = DeclareLaunchArgument(
        'fire_stable_px_per_frame',
        default_value='1.5',
        description='画面稳定阈值(像素/帧)：目标单帧位移小于它才算瞄住；调大更易开火，调小更准'
    )

    predict_gain_arg = DeclareLaunchArgument(
        'predict_gain',
        default_value='0.5',
        description='目标移动预判强度(帧)，0=关闭'
    )

    max_shots_arg = DeclareLaunchArgument(
        'max_shots',
        default_value='10',
        description='全程可用发数，硬上限 10，只能调小'
    )

    target_confirm_frames_arg = DeclareLaunchArgument(
        'target_confirm_frames',
        default_value='8',
        description='连续识别到目标多少帧才算"扫描确认"，之后才允许射击'
    )

    shot_cooldown_ms_arg = DeclareLaunchArgument(
        'shot_cooldown_ms',
        default_value='500',
        description='两发之间的最小间隔(ms)'
    )

    track_kp_arg = DeclareLaunchArgument(
        'track_kp',
        default_value='8.0',
        description='PD 比例增益(原 track_speed_per_px，已调低以减小振荡)'
    )

    track_kd_arg = DeclareLaunchArgument(
        'track_kd',
        default_value='3.0',
        description='PD 微分增益(阻尼)，抑制超调振荡；太小会抖、太大会迟钝'
    )

    patrol_speed_ratio_arg = DeclareLaunchArgument(
        'patrol_speed_ratio',
        default_value='0.75',
        description='巡逻(扫描)速度 = 云台最大转速 x 该比例'
    )

    aspect_min_arg = DeclareLaunchArgument(
        'aspect_min',
        default_value='0.77',
        description='目标框宽高比下限，超出范围视为长方形目标不跟踪'
    )

    aspect_max_arg = DeclareLaunchArgument(
        'aspect_max',
        default_value='1.33',
        description='目标框宽高比上限'
    )

    track_speed_max_ratio_arg = DeclareLaunchArgument(
        'track_speed_max_ratio',
        default_value='0.63',
        description='跟踪速度上限 = 巡逻速度 x 该比例'
    )

    lost_frames_to_rescan_arg = DeclareLaunchArgument(
        'lost_frames_to_rescan',
        default_value='60',
        description='跟踪中连续丢帧超过该值才放弃跟踪回到扫描'
    )

    pitch_deg_sign_arg = DeclareLaunchArgument(
        'pitch_deg_sign',
        default_value='1.0',
        description='竖直角度方向，实测反了就改成 -1.0'
    )

    scan_passes_arg = DeclareLaunchArgument(
        'scan_passes',
        default_value='2',
        description='点位全向扫描趟数：2 = 左一趟 + 右一趟；调大扫得更久更细'
    )

    # 创建节点
    dzrknn_dataprocess_node = Node(
        package='dzrknn_dataprocess',
        executable='dzactuator_node',
        name='dzrknn_dataprocess',
        output='screen',
        parameters=[{
            # ===== 瞄准扫描参数（标定时直接改这里的数字，改完重启即可，无需重编译）=====
            'scan_yaw_dirs': 8,             # 水平横扫点数
            'scan_yaw_min_deg': -90.0,      # 水平最小角(相对车头，右为正)
            'scan_yaw_max_deg': 90.0,       # 水平最大角
            # 竖直角度：不在此处传参 —— launch_ros 不接受 Python list，
            # 写 'scan_pitch_levels': [0.0] 会直接抛 "Unrecognized data type: <class 'list'>" 导致节点起不来。
            # 节点内 declare_parameter 的默认值即 {0.0}，等价于"竖直轴固定在 0°、只做水平扫描"。
            # 需要多层扫描时：改 dzrknn_dataprocess.cpp 里的默认值并重编译。
            'yaw_deg_sign': 1.0,            # 水平角度方向反了改 -1.0
            'pitch_deg_sign': LaunchConfiguration('pitch_deg_sign'),  # 竖直角度方向(反了改-1.0)
            'gimbal_center_tick': 2047,     # 零位(正前方/水平)刻度
            'yaw_ticks_per_deg': 11.378,    # 水平 刻度/度 = 4096/360
            'pitch_ticks_per_deg': 11.378,  # 竖直 刻度/度 = 4096/360
            'gimbal_yaw_tick_min': 100,     # 水平轴刻度下限(留边界余量)
            'gimbal_yaw_tick_max': 3995,    # 水平轴刻度上限
            'gimbal_pitch_tick_min': 950,   # 竖直轴刻度下限(下位机钳位)
            'gimbal_pitch_tick_max': 2400,  # 竖直轴刻度上限(下位机钳位)
            'gimbal_max_speed': 3700,       # 云台最大转速
            # ===== 速度体系（都相对于"巡逻速度"）=====
            'patrol_speed_ratio': LaunchConfiguration('patrol_speed_ratio'),   # 巡逻速度比例
            'scan_dwell_ms': 300,           # 每个点最短驻留(ms)
            'scan_passes': LaunchConfiguration('scan_passes'),  # 点位全向扫描趟数(2=左一趟+右一趟)
            'scan_arrive_tol': 30,          # 到位容差(刻度)
            'lost_frames_to_rescan': LaunchConfiguration('lost_frames_to_rescan'),  # 跟踪丢帧回扫描阈值
            'vision_timeout_ms': 1000,      # 追踪中多久收不到视觉结果就强制回扫描(ms)
            # ===== 瞄准精度与消抖 =====
            'ema_alpha': LaunchConfiguration('ema_alpha'),                         # 视觉低通滤波系数
            'track_deadzone_px': LaunchConfiguration('track_deadzone_px'),         # 跟踪死区(像素)
            'fire_deadzone_px': LaunchConfiguration('fire_deadzone_px'),           # 固定开火死区(像素)
            'fire_dz_dynamic': LaunchConfiguration('fire_dz_dynamic'),             # 启用动态开火框
            'fire_dz_far_px': LaunchConfiguration('fire_dz_far_px'),               # 远距离开火死区(像素)
            'fire_dz_near_px': LaunchConfiguration('fire_dz_near_px'),             # 近距离开火死区(像素)
            'box_w_far': LaunchConfiguration('box_w_far'),                         # 远处目标框宽标定值
            'box_w_near': LaunchConfiguration('box_w_near'),                       # 近处目标框宽标定值
            'fire_real_speed_max': LaunchConfiguration('fire_real_speed_max'),     # 实际速度静止阈值
            'track_kp': LaunchConfiguration('track_kp'),                           # PD 比例增益
            'track_kd': LaunchConfiguration('track_kd'),                           # PD 微分增益
            'track_speed_max_ratio': LaunchConfiguration('track_speed_max_ratio'), # 跟踪速度上限比例
            'fine_zone_px': 20,             # 精调档阈值(像素)
            'fine_kp_scale': 0.5,           # 精调档 Kp 缩放
            'fine_kd_scale': 2.0,           # 精调档 Kd 放大
            'fire_speed_max': 50,           # 开火时指令速度上限
            'fire_hold_frames': LaunchConfiguration('fire_hold_frames'),           # 连续满足多少帧才开火
            'fire_use_image_stable': LaunchConfiguration('fire_use_image_stable'), # 开火稳定判据(画面稳定/电机静止)
            'fire_stable_px_per_frame': LaunchConfiguration('fire_stable_px_per_frame'), # 画面稳定阈值(像素/帧)
            'predict_gain': LaunchConfiguration('predict_gain'),                   # 目标移动预判强度(帧)
            'max_shots': LaunchConfiguration('max_shots'),                         # 全程可用发数(硬上限10)
            'shot_cooldown_ms': LaunchConfiguration('shot_cooldown_ms'),           # 两发之间最小间隔(ms)
            'target_confirm_frames': LaunchConfiguration('target_confirm_frames'), # 扫描确认帧数(确认后才允许射击)
            # ===== 激光同轴标定（用连续激光笔测出后填这里）=====
            'laser_offset_x': 0.0,          # 激光落点相对画面中心的水平偏移(像素)
            'laser_offset_y': 0.0,          # 激光落点相对画面中心的竖直偏移(像素)
            # ===== 目标筛选：只跟踪长宽比近似 1:1 的目标 =====
            'aspect_min': LaunchConfiguration('aspect_min'),                       # 框宽高比下限
            'aspect_max': LaunchConfiguration('aspect_max'),                       # 框宽高比上限
        }]
    )
    
    return LaunchDescription([
        ema_alpha_arg,
        track_deadzone_px_arg,
        fire_deadzone_px_arg,
        fire_dz_dynamic_arg,
        fire_dz_far_px_arg,
        fire_dz_near_px_arg,
        box_w_far_arg,
        box_w_near_arg,
        fire_real_speed_max_arg,
        fire_hold_frames_arg,
        fire_use_image_stable_arg,
        fire_stable_px_per_frame_arg,
        predict_gain_arg,
        max_shots_arg,
        shot_cooldown_ms_arg,
        track_kp_arg,
        track_kd_arg,
        patrol_speed_ratio_arg,
        aspect_min_arg,
        aspect_max_arg,
        target_confirm_frames_arg,
        track_speed_max_ratio_arg,
        lost_frames_to_rescan_arg,
        pitch_deg_sign_arg,
        scan_passes_arg,
        dzrknn_dataprocess_node,
    ])

