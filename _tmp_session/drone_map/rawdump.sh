#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/duzhong/dzpx4/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
timeout 6 ros2 topic echo /Odometry --once 2>&1 | head -5
echo '---'
python3 - <<'PY'
import rclpy,time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
class D(Node):
    def __init__(self):
        super().__init__('rawdump')
        self.a=[]
        self.create_subscription(Odometry,'/Odometry',self.cb,qos_profile_sensor_data)
    def cb(self,m):
        s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        p=m.pose.pose.position
        self.a.append((time.time(),s,p.x,p.y,p.z))
rclpy.init(); n=D(); t0=time.time()
while time.time()-t0<3: rclpy.spin_once(n,timeout_sec=0.05)
print('n',len(n.a))
for r in n.a[:25]: print('%.3f stamp=%.3f x=%.3f y=%.3f z=%.3f'%r)
rclpy.shutdown()
PY
echo '### END'
