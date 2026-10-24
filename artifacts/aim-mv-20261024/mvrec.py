#!/usr/bin/env python3
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Int32MultiArray, UInt8, UInt8MultiArray, Float32

out_path = sys.argv[1]
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 75.0
f = open(out_path, "w", buffering=1)
t0 = time.time()

def rec(kind, arr):
    t = time.time() - t0
    if isinstance(arr, (int, float)):
        payload = f"{arr}"
    elif isinstance(arr, list):
        payload = ";".join(str(x) for x in arr)
    else:
        payload = str(arr)
    f.write(f"{t:.4f},{t0:.3f},{kind},{payload}\n")

class Rec(Node):
    def __init__(self):
        super().__init__("mvrec")
        q = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Int32MultiArray, "/offset_center", lambda m: rec("off", list(m.data)), q)
        self.create_subscription(Int32, "/aim/status", lambda m: rec("status", int(m.data)), q)
        self.create_subscription(Int32MultiArray, "/CameMo_Conl", lambda m: rec("con", list(m.data)), q)
        self.create_subscription(Int32MultiArray, "/camemo_feedback", lambda m: rec("fb", list(m.data)), q)
        self.create_subscription(UInt8, "/LaserShot_Command", lambda m: rec("shot", int(m.data)), q)
        self.create_subscription(UInt8MultiArray, "/HpAndHitmsg", lambda m: rec("hp", list(m.data)), q)
        self.create_subscription(Int32, "/shoot_count", lambda m: rec("cnt", int(m.data)), q)
        self.create_subscription(Float32, "/aim/lead_x", lambda m: rec("lead", float(m.data)), q)
        self.create_subscription(UInt8, "/aim/fire_ready", lambda m: rec("ready", int(m.data)), q)

rclpy.init()
n = Rec()
end = time.time() + dur
try:
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
except KeyboardInterrupt:
    pass
finally:
    f.flush(); f.close()
    n.destroy_node()
    rclpy.try_shutdown()
print("REC_DONE")
