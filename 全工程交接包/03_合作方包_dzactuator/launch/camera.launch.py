"""camera launch —— ★ 参数化 + 算力提示

关于像素格式的权衡（**现场需要实测决定**）：

  yuyv  未压缩，USB 带宽 = 640×480×2×30fps ≈ 18.4 MB/s
        CPU：usb_cam 侧几乎不做事，det_node 侧 cv_bridge 转 bgr8
        -> 转换发生在订阅端，CPU 成本在那里

  mjpeg 相机/驱动侧 JPEG 压缩，USB 带宽 ≈ 1/10
        CPU：usb_cam 侧要软件解码 JPEG（cv::imdecode），成本在采集端

  两者**总 CPU 成本接近**（压缩/解压总要做一次），差别在：
    · USB 带宽紧张（共用 hub / 长线）-> 选 mjpeg
    · 想省 CPU 且带宽充足         -> 选 yuyv
  本赛题 640×480 单相机，带宽不是瓶颈，**默认保留 yuyv**（原配置）。
  若要试 mjpeg：把 pixel_format 改为 'mjpeg' 并用 ros2 topic hz 对比 CPU。

关于分辨率：
  格子 35cm。640×480 下画面里一格约 60~90px，识别格号足够。
  若现场发现识别不稳，优先**拉近相机/升高分辨率**而不是加模型复杂度。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dev_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='摄像头设备路径')
    w_arg = DeclareLaunchArgument(
        'image_width', default_value='640', description='图像宽度')
    h_arg = DeclareLaunchArgument(
        'image_height', default_value='480', description='图像高度')
    fps_arg = DeclareLaunchArgument(
        'framerate', default_value='30.0', description='采集帧率')
    fmt_arg = DeclareLaunchArgument(
        'pixel_format', default_value='yuyv',
        description="像素格式：'yuyv'（默认，省 CPU）或 'mjpeg'（省带宽）")
    io_arg = DeclareLaunchArgument(
        'io_method', default_value='mmap',
        description="采集方式：'mmap' / 'read' / 'userptr'")

    cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'framerate': LaunchConfiguration('framerate'),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'io_method': LaunchConfiguration('io_method'),
        }]
    )

    return LaunchDescription([
        dev_arg, w_arg, h_arg, fps_arg, fmt_arg, io_arg, cam,
    ])
