#!/usr/bin/env python3
import os, sys, time, json, signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

OUT = os.environ.get('DZMAP_OUT','/tmp/dzmap')
VOX = float(os.environ.get('DZMAP_VOX','0.05'))
MAXR = float(os.environ.get('DZMAP_MAXR','40'))
DUR = float(os.environ.get('DZMAP_DUR','1200'))
SNAP = float(os.environ.get('DZMAP_SNAP','2.0'))
os.makedirs(OUT, exist_ok=True)
STOP = os.path.join(OUT,'STOP')

def get_cols(msg):
    n = msg.width*msg.height; step = msg.point_step
    if n==0 or step==0: return None
    try:
        b = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
    except Exception:
        return None
    out = {}
    for name in ('x','y','z'):
        f = next((f for f in msg.fields if f.name==name), None)
        if f is None or f.datatype != 7: return None
        col = b[:, f.offset:f.offset+4].copy()
        out[name] = col.view('<f4').ravel()
    return np.stack([out['x'],out['y'],out['z']],axis=1)

def pack(ix,iy,iz):
    off = 1<<20
    return ((ix.astype(np.int64)+off)<<42) | ((iy.astype(np.int64)+off)<<21) | (iz.astype(np.int64)+off)

class Rec(Node):
    def __init__(self):
        super().__init__('dzmap_rec')
        self.create_subscription(PointCloud2,'/cloud_registered',self.cb,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/Odometry',self.odom_cb,qos_profile_sensor_data)
        self.keys = np.zeros(0,dtype=np.int64)
        self.parts = []
        self.npts = 0
        self.path = []
        self.last_pt = 0.0
        self.frames = 0
        self.t0 = time.time()
        self.timer = self.create_timer(SNAP, self.tick)
        self.done = False
        self.get_logger().info('rec start out=%s vox=%s' % (OUT,VOX))
    def odom_cb(self,m):
        t = time.time()
        if t - self.last_pt < 0.2: return
        self.last_pt = t
        p = m.pose.pose.position
        self.path.append([round(t-self.t0,3), float(p.x), float(p.y), float(p.z)])
    def cb(self,msg):
        if self.done: return
        p = get_cols(msg)
        if p is None: return
        m = np.isfinite(p).all(axis=1)
        if not m.any(): return
        p = p[m]
        r = np.linalg.norm(p,axis=1)
        p = p[(r>0.3)&(r<MAXR)]
        if len(p)==0: return
        self.frames += 1
        idx = np.floor(p/VOX).astype(np.int64)
        k = pack(idx[:,0],idx[:,1],idx[:,2])
        ku, iu = np.unique(k, return_index=True)
        if len(self.keys):
            pos = np.searchsorted(self.keys, ku)
            clipped = np.clip(pos,0,len(self.keys)-1)
            hit = (pos < len(self.keys)) & (self.keys[clipped] == ku)
            new = ~hit
        else:
            new = np.ones(len(ku),dtype=bool)
        if new.any():
            add = p[iu[new]].astype(np.float32)
            self.parts.append(add)
            self.npts += len(add)
            self.keys = np.union1d(self.keys, ku[new])
    def tick(self):
        if self.done: return
        self.write()
        if os.path.exists(STOP):
            self.finish('stopfile')
        elif DUR>0 and time.time()-self.t0>DUR:
            self.finish('duration')
    def allpts(self):
        if not self.parts: return np.zeros((0,3),dtype=np.float32)
        return np.vstack(self.parts)
    def write(self):
        p = self.allpts()
        tmp = os.path.join(OUT,'.m.npy'); np.save(tmp,p); os.replace(tmp, os.path.join(OUT,'map_latest.npy'))
        tmp = os.path.join(OUT,'.p.npy'); np.save(tmp,np.asarray(self.path,dtype=np.float64)); os.replace(tmp, os.path.join(OUT,'path_latest.npy'))
        st = {'frames':self.frames,'points':int(len(p)),'vox':VOX,
              'elapsed':round(time.time()-self.t0,2),'path_n':len(self.path),'ts':time.time()}
        if len(p):
            st['min']=[round(float(v),3) for v in p.min(axis=0)]
            st['max']=[round(float(v),3) for v in p.max(axis=0)]
        with open(os.path.join(OUT,'.s.json'),'w') as f: f.write(json.dumps(st))
        os.replace(os.path.join(OUT,'.s.json'), os.path.join(OUT,'stats.json'))
    def finish(self,why):
        if self.done: return
        self.done=True
        try: self.write()
        except Exception: pass
        p = self.allpts()
        hdr = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
               "TYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n") % (len(p),len(p))
        with open(os.path.join(OUT,'map_final.pcd'),'wb') as f:
            f.write(hdr.encode('ascii')); f.write(np.ascontiguousarray(p,dtype='<f4').tobytes())
        with open(os.path.join(OUT,'DONE'),'w') as f: f.write(why)
        self.get_logger().info('finish %s points=%d' % (why,len(p)))

def main():
    rclpy.init()
    n = Rec()
    def h(sig,frame):
        n.finish('signal')
        try: rclpy.shutdown()
        except Exception: pass
        sys.exit(0)
    signal.signal(signal.SIGINT,h); signal.signal(signal.SIGTERM,h)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.finish('exit')
        try: n.destroy_node()
        except Exception: pass
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception: pass

main()
