import rclpy, time
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

class D(Node):
    def __init__(self):
        super().__init__('dzmap_diag')
        self.o=[]; self.l=[]
        self.create_subscription(Odometry,'/Odometry',self.ocb,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/localization',self.lcb,qos_profile_sensor_data)
    def ocb(self,m):
        p=m.pose.pose.position; v=m.twist.twist.linear
        self.o.append((time.time(),p.x,p.y,p.z,v.x,v.y,v.z))
    def lcb(self,m):
        p=m.pose.pose.position
        self.l.append((time.time(),p.x,p.y,p.z))

def main():
    rclpy.init(); n=D(); t0=time.time()
    while time.time()-t0<12: rclpy.spin_once(n, timeout_sec=0.1)
    o=np.array(n.o); l=np.array(n.l)
    print('odom_n',len(o),'loc_n',len(l))
    if len(o)>2:
        dt=np.diff(o[:,0]); d=np.linalg.norm(np.diff(o[:,1:4],axis=0),axis=1)
        v=np.divide(d,dt,out=np.zeros_like(d),where=dt>1e-6)
        print('odom_dist_m',round(float(np.linalg.norm(np.diff(o[:,1:3],axis=0),axis=1).sum()),2))
        print('odom_net_m',round(float(np.linalg.norm(o[-1,1:3]-o[0,1:3])),2))
        print('speed_p50_p95_max',[round(float(np.percentile(v,q)),3) for q in (50,95,100)])
        print('twist_abs_p50_p95',round(float(np.percentile(np.abs(o[:,4]),50)),3),round(float(np.percentile(np.abs(o[:,4]),95)),3))
        print('odom_first',o[0,1:4].round(3),'odom_last',o[-1,1:4].round(3))
    if len(l)>2:
        d=np.linalg.norm(np.diff(l[:,1:4],axis=0),axis=1)
        print('loc_max_jump_m',round(float(d.max()),2),'loc_jumps_gt5',int((d>5).sum()))
        print('loc_first',l[0,1:4].round(3),'loc_last',l[-1,1:4].round(3))
        print('loc_unique_1m',len(set(map(tuple,np.round(l[:,1:3],0)))))
    rclpy.shutdown()
main()
