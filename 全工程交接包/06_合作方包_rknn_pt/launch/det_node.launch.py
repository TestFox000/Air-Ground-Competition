"""det_node launch —— ★ 加入算力优化参数，并把可视化默认关闭

注意 package 名的坑：
  `package=` 必须与 **package.xml 的 <name>** 一致（本仓库是 `det_node`），
  而不是与源码目录名（`rknn_pt`）一致。若不一致 `ros2 launch` 直接失败
  （dzmission/doc/INTEGRATION.md 已记录该风险）。

新增参数（全部有安全默认值，不传也能跑）：
  detect_rate_hz  检测帧率。**默认 15**（原代码硬编码 30）。
                  本赛题瞄准是静态任务（停车射击），15fps 足够，
                  省一半视觉算力。动态跟踪场景可调到 30。
  infer_threads   推理线程数。**默认 3**，与 RK3588S 的 NPU 核数一致，
                  避免核 0 被两个线程争抢（原代码固定 4）。
  show_results    是否开可视化窗口。**默认 false**。
                  车载无显示器，imshow 会白烧 CPU/GPU，甚至因无 X 卡住。
                  台架调试时传 true。
  model_path      模型绝对路径。留空则按多个候选位置自动查找。
  image_topic     输入图像话题（默认 /image_raw）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_color_arg = DeclareLaunchArgument(
        'robot_color', default_value='red',
        description='Robot color (red or blue)')

    detect_rate_arg = DeclareLaunchArgument(
        'detect_rate_hz', default_value='15.0',
        description='检测帧率(Hz)。静态瞄准 15 足够；动态跟踪可调到 30')

    threads_arg = DeclareLaunchArgument(
        'infer_threads', default_value='3',
        description='推理线程数，建议与 NPU 核数(3)一致')

    show_arg = DeclareLaunchArgument(
        'show_results', default_value='false',
        description='是否开可视化窗口（车载必须 false）')

    model_arg = DeclareLaunchArgument(
        'model_path', default_value='',
        description='模型绝对路径；留空则自动查找')

    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/image_raw',
        description='输入图像话题')

    # ★ 可见面颜色判定（HSV，需现场按光照标定）
    face_min_arg = DeclareLaunchArgument(
        'face_min_ratio', default_value='0.12',
        description='红/蓝像素占比低于此值判为"不确定"（宁可不打）')
    face_bias_arg = DeclareLaunchArgument(
        'face_red_bias', default_value='1.25',
        description='一方占比需高出另一方此倍数才判定颜色')

    # ★ 激光红点检测（命中确认）
    laser_dot_arg = DeclareLaunchArgument(
        'enable_laser_dot', default_value='true',
        description='是否检测检测框内的激光红点（作为独立开火联锁）')
    laser_v_arg = DeclareLaunchArgument(
        'laser_dot_v_min', default_value='200',
        description='红点亮度下限（激光点比靶面亮，200~255）')

    det_node = Node(
        package='det_node',          # ★ 与 package.xml 的 <name> 一致
        executable='det_node',
        name='det_node',
        output='screen',
        parameters=[{
            'robot_color': LaunchConfiguration('robot_color'),
            'detect_rate_hz': LaunchConfiguration('detect_rate_hz'),
            'infer_threads': LaunchConfiguration('infer_threads'),
            'show_results': LaunchConfiguration('show_results'),
            'model_path': LaunchConfiguration('model_path'),
            'image_topic': LaunchConfiguration('image_topic'),
            'output_topic': 'offset_center',
            # ★ 可见面颜色判定（HSV，现场按光照标定）
            'face_min_ratio': LaunchConfiguration('face_min_ratio'),
            'face_red_bias': LaunchConfiguration('face_red_bias'),
            # ★ 激光红点检测
            'enable_laser_dot': LaunchConfiguration('enable_laser_dot'),
            'laser_dot_v_min': LaunchConfiguration('laser_dot_v_min'),
            # 历史参数，原文件就有，保留以免下游依赖
            'tracking.max_prediction_time': 3.0,
            'tracking.max_lost_frames': 150,
        }]
    )

    return LaunchDescription([
        robot_color_arg,
        detect_rate_arg,
        threads_arg,
        show_arg,
        model_arg,
        image_topic_arg,
        face_min_arg,
        face_bias_arg,
        laser_dot_arg,
        laser_v_arg,
        det_node,
    ])
