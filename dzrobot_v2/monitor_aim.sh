#!/bin/bash
# ============================================================
#  自瞄/识别 实时监控
#
#     ~/dzrobot/monitor_aim.sh
#
#  每行含义：
#    [云台状态] 目标类型  偏移(x,y)  框宽x框高  靶心偏移(x,y)  关键点置信度
#
#    云台状态: IDLE空闲 / SCAN扫描中 / TRACK已锁定
#    目标类型: 无目标 / 蓝靶(cls=1) / 红靶(cls=2)
#
#  判据速查：
#    · 目标=无目标 且 状态=SCAN   -> 视野里没有靶，云台在左右扫着找（正常）
#    · 目标=蓝/红靶 且 状态=TRACK -> 识别成功，云台已锁定跟随
#
#  Ctrl+C 退出
# ============================================================
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null

python3 -u - <<'PY'
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, UInt8

STATE = {0: 'IDLE 空闲', 1: 'SCAN 扫描', 2: 'TRACK 锁定'}
CLASS = {0: '无目标', 1: '蓝靶  ', 2: '红靶  '}


class AimMonitor(Node):
    def __init__(self):
        super().__init__('aim_monitor')
        self.state = 0
        self.create_subscription(Int32MultiArray, '/offset_center', self.on_det, 10)
        self.create_subscription(UInt8, '/aim/status', self.on_state, 10)

    def on_state(self, msg):
        self.state = msg.data

    def on_det(self, msg):
        d = list(msg.data) + [0] * (8 - len(msg.data))
        cls = d[2]
        if cls not in CLASS:
            cls = 0
        print('[%-10s] 目标=%s  偏移=(%+5d,%+5d)  框=%3dx%-3d  靶心=(%+5d,%+5d)  置信=%3d%%'
              % (STATE.get(self.state, '?'), CLASS[cls],
                 d[0], d[1], d[3], d[4], d[5], d[6], d[7]))


rclpy.init()
node = AimMonitor()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
PY
