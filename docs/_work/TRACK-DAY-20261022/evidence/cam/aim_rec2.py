import rclpy, json, sys, time, threading
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32, UInt8, Int32, UInt8MultiArray

f = open(sys.argv[1], 'w', buffering=1)
class N(Node):
    def __init__(self):
        super().__init__('aim_rec2')
        specs = [('/offset_center', Int32MultiArray), ('/aim/lead_x', Float32),
                 ('/aim/fire_ready', UInt8), ('/aim/fire_gate', UInt8),
                 ('/aim/status', Int32), ('/shoot_count', Int32),
                 ('/LaserShot_Command', UInt8), ('/HpAndHitmsg', UInt8MultiArray)]
        for name, typ in specs:
            self.create_subscription(typ, name, (lambda m, n=name: self.cb(n, m)), 50)
    def cb(self, name, m):
        d = m.data
        if hasattr(d, '__len__') and not isinstance(d, str):
            d = list(d)
        else:
            d = int(d) if isinstance(d, (int, float)) else d
        f.write(json.dumps({'t': round(time.time(), 3), 'k': name, 'v': d}) + '\n')
rclpy.init()
n = N()
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
time.sleep(float(sys.argv[2]))
f.write(json.dumps({'t': round(time.time(), 3), 'k': 'END', 'v': 0}) + '\n')
f.close()
