import os

from launch import LaunchDescription
from launch_ros.actions import Node

# ★★ 底盘参数统一从配置中心读（config/robot.yaml 的 /dzactuator 段）。
#
#   为什么不再用 launch 参数逐个往里传（2026-10-03 重构）：
#     ① 类型不匹配 —— LaunchConfiguration 传进 Node(parameters=[...]) 的是【字符串】，
#        而节点 declare_parameter 的是 double/int。一旦对不上，rclcpp 会静默回落到
#        节点内置默认值（ticksPerMeter 1000 / ticksPer2PI 3200），
#        表现为"launch 里明明写了 3290，里程计却像在用 1000"——这就是 C1 的根因。
#     ② 参数散落在 launch 里，和项目铁律"全车唯一改参处 = config/robot.yaml"矛盾（C2）。
#
#   改成直接喂 params-file 后：类型由 YAML 决定（不会再错配），
#   改参数只需动 robot.yaml 且不用碰 launch，start_all.sh 流程完全不变。
#   仍支持 ROBOT_CONFIG 环境变量 —— 可临时指向另一份配置做 A/B 对比。
CONFIG_FILE = os.environ.get(
    'ROBOT_CONFIG',
    os.path.expanduser('~/dzrobot/config/robot.yaml'))

# ★ 配置文件不存在时【不能直接把路径丢给 Node】—— 那会让底盘节点在启动阶段
#   抛异常直接退出(底盘是全车第一步，挂了后面全起不来)。
#   旧写法用 LaunchConfiguration 天然带着默认值，配置文件坏了也能跑；
#   这里显式做同样的兜底：文件缺失就传空参数列表，节点用自己 declare 的内置值。
if os.path.exists(CONFIG_FILE):
    PARAMS = [CONFIG_FILE]
    print('[dzactuator] 参数来源: %s' % CONFIG_FILE)
else:
    PARAMS = []
    print('[dzactuator] ⚠️ 找不到配置文件 %s，'
          '本节点将使用内置默认参数(里程计标定会退回 1000/3200)。' % CONFIG_FILE)


def generate_launch_description():
    return LaunchDescription([

        # ---- 固定外参（纯几何常量，与标定无关，故不进配置中心）----
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.08', '0.0', '0.0', '0.0',
                       'base_link', 'imu_link'],
        ),
        # base_link -> laser：预留（加装雷达时需要；当前无激光雷达，无人订阅）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.15', '0.0', '0.07', '3.14159', '0.0', '0.0',
                       'base_link', 'laser'],
        ),

        # ---- ★底盘串口节点：参数全部来自 robot.yaml 的 /dzactuator 段 ----
        Node(
            package='dzactuator',
            executable='dzactuator_node',
            name='dzactuator',
            output='screen',
            emulate_tty=True,
            parameters=PARAMS,
        ),
    ])
