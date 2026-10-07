from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="dzjudgment",
            executable="dzjudgment",
            name="dzjudgment",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"mcubaudrate": 115200},
                {"mcuserialport": "/dev/dzjudgment"}, 
            ]
        )
    ])