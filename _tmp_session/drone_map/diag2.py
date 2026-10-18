import rclpy, time
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

def yaw_of(q):
    return np.arctan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]*q[1]+q[2]*q[2]))

class D(Node):
    def __init__(self):
        super().__init__('dzmap_diag2')
        self.od=[]; self.lc=[]
        self.create_subscription(Odometry,'/Odometry',self.ocb,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/localization',self.lcb,qos_profile_sensor_data)
    def ocb(self,m):
        s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        p=m.pose.pose.position; q=m.pose.pose.orientation
        self.od.append((s,p.x,p.y,p.z,q.x,q.y,q.z,q.w))
    def lcb(self,m):
        s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        p=m.pose.pose.position; q=m.pose.pose.orientation
        self.lc.append((s,p.x,p.y,p.z,q.x,q.y,q.z,q.w))

def summarize(name, a):
    a=np.array(a)
    if len(a)<3:
        print(name,'n<3'); return
    t=a[:,0]; dt=np.diff(t); d=np.linalg.norm(np.diff(a[:,1:4],axis=0),axis=1)
    v=np.divide(d,dt,out=np.zeros_like(d),where=dt>1e-6)
    yaw=np.unwrap(np.array([yaw_of(q) for q in a[:,4:8]]))
    print(name,'n',len(a),'span_s',round(float(t[-1]-t[0]),2))
    print('  path_m',round(float(d.sum()),2),'net_m',round(float(np.linalg.norm(a[-1,1:3]-a[0,1:3])),2))
    print('  speed_p50_p90_p99_max',[round(float(np.percentile(v,q)),2) for q in (50,90,99,100)])
    print('  x_range',round(float(a[:,1].min()),2),round(float(a[:,1].max()),2),'y_range',round(float(a[:,2].min()),2),round(float(a[:,2].max()),2),'z_range',round(float(a[:,3].min()),2),round(float(a[:,3].max()),2))
    print('  yaw_deg_range',round(float(np.degrees(yaw.min())),1),round(float(np.degrees(yaw.max())),1),'yaw_total_deg',round(float(np.degrees(yaw[-1]-yaw[0])),1))

def main():
    rclpy.init(); n=D(); t0=time.time()
    while time.time()-t0<15: rclpy.spin_once(n, timeout_sec=0.1)
    od=np.array(n.od); lc=np.array(n.lc)
    np.savez('/tmp/dzmap/diag2.npz', odom=od, loc=lc)
    summarize('ODOM', n.od)
    summarize('LOC', n.lc)
    rclpy.shutdown()
main()
