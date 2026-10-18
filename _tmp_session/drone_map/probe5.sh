#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/duzhong/dzpx4/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
echo '### JUMP_CHECK_SRC'
sed -n '400,450p' /home/duzhong/dzpx4/src/px4_path/src/path_record_node.cpp
echo '### SUB_SETUP'
grep -n -E 'slam_topic|create_subscription|callback_group' /home/duzhong/dzpx4/src/px4_path/src/path_record_node.cpp | head -15
echo '### IMU_STATS'
timeout 15 python3 - <<'PY'
import rclpy,time,numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
class D(Node):
    def __init__(self):
        super().__init__('imustat')
        self.a=[]; self.g=[]
        self.create_subscription(Imu,'/livox/imu',self.cb,qos_profile_sensor_data)
    def cb(self,m):
        a=m.linear_acceleration; g=m.angular_velocity
        self.a.append((a.x,a.y,a.z)); self.g.append((g.x,g.y,g.z))
rclpy.init(); n=D(); t0=time.time()
while time.time()-t0<3: rclpy.spin_once(n,timeout_sec=0.05)
a=np.array(n.a); g=np.array(n.g)
print('n',len(a))
print('accel_mean',a.mean(0).round(4),'accel_std',a.std(0).round(4))
print('gyro_mean',g.mean(0).round(4),'gyro_std',g.std(0).round(4))
print('accel_max_abs',np.abs(a).max(0).round(3),'gyro_max_abs',np.abs(g).max(0).round(3))
rclpy.shutdown()
PY
echo '### LOCALIZATION_SERIES'
timeout 8 python3 - <<'PY'
import rclpy,time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
class D(Node):
    def __init__(self):
        super().__init__('locseries')
        self.a=[]
        self.create_subscription(Odometry,'/localization',self.cb,qos_profile_sensor_data)
    def cb(self,m):
        p=m.pose.pose.position; self.a.append((time.time(),p.x,p.y,p.z))
rclpy.init(); n=D(); t0=time.time()
while time.time()-t0<4: rclpy.spin_once(n,timeout_sec=0.05)
a=n.a
print('n',len(a))
for r in a[::20][:20]: print('x=%.2f y=%.2f z=%.2f'%(r[1],r[2],r[3]))
mx=max((abs(a[i][1]-a[i-1][1])+abs(a[i][2]-a[i-1][2])) for i in range(1,len(a))) if len(a)>1 else 0
print('max_step_xy',round(mx,2))
rclpy.shutdown()
PY
echo '### END'
