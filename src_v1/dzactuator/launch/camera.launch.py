from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',    # 摄像头设备路径
                'image_width': 640,               # 图像宽度
                'image_height': 480,              # 图像高度
                'framerate': 30.0,                # 帧率
                'pixel_format': 'yuyv',           # 像素格式，可选 'yuyv' 或 'mjpeg'
                'io_method': 'mmap'               # 采集方式，可选 'mmap'、'read' 或 'userptr'
            }]
        )
    ])
