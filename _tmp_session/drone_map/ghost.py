import rclpy, time
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

def cols(msg):
    n=msg.width*msg.height
    b=np.frombuffer(msg.data,dtype=np.uint8).reshape(n,msg.point_step)
    out={}
    for name in ('x','y','z'):
        f=next(f for f in msg.fields if f.name==name)
        out[name]=b[:,f.offset:f.offset+4].copy().view('<f4').ravel()
    return out

class D(Node):
    def __init__(self):
        super().__init__('ghosttest')
        self.peaks=[]
        self.map=[]
        self.create_subscription(PointCloud2,'/cloud_registered',self.cb,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/map_to_odom',self.mcb,qos_profile_sensor_data)
    def mcb(self,m):
        p=m.pose.pose.position; self.map.append((time.time(),p.x,p.y,p.z))
    def cb(self,msg):
        if len(self.peaks)>=25: return
        c=cols(msg); x=c['x']; y=c['y']
        m=(x>-2)&(x<2)&(y>7.2)&(y<9.8)
        if m.sum()<80: return
        h,e=np.histogram(y[m],bins=np.arange(7.2,9.8,0.02))
        if h.max()<20: return
        self.peaks.append(e[np.argmax(h)])

def main():
    rclpy.init(); n=D(); t0=time.time()
    while time.time()-t0<6: rclpy.spin_once(n,timeout_sec=0.05)
    pk=np.array(n.peaks)
    print('frames_used',len(pk))
    if len(pk)>2:
        print('wall_peak_mean',round(float(pk.mean()),3),'std',round(float(pk.std()),3),'min',round(float(pk.min()),3),'max',round(float(pk.max()),3))
    mm=np.array(n.map)
    print('map_to_odom_n',len(mm))
    if len(mm)>2:
        d=np.linalg.norm(np.diff(mm[:,1:4],axis=0),axis=1)
        print('m2o_max_step',round(float(d.max()),2),'m2o_jumps_gt1m',int((d>1).sum()),'x_range',round(float(mm[:,1].min()),2),round(float(mm[:,1].max()),2),'y_range',round(float(mm[:,2].min()),2),round(float(mm[:,2].max()),2))
    rclpy.shutdown()
main()
