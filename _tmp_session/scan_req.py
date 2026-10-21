import time, rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
rclpy.init()
n = Node('scan_req_cli')
p = n.create_publisher(UInt8, '/nav/scan_request', 10)
for _ in range(5):
    p.publish(UInt8(data=2))
    rclpy.spin_once(n, timeout_sec=0.15)
    time.sleep(0.1)
rclpy.spin_once(n, timeout_sec=0.6)
print('OK /nav/scan_request=2 x5')
n.destroy_node()
rclpy.shutdown()
