import numpy as np, sys
sys.path.insert(0,'.')
from mapview import load_pcd
old=load_pcd('buliding_a_floor1.pcd')
live=np.load('map_latest.npy').astype(float)
def wall_stats(p,name,xr,yr,zr=(-0.5,2.5)):
    m=(p[:,0]>xr[0])&(p[:,0]<xr[1])&(p[:,2]>zr[0])&(p[:,2]<zr[1])
    q=p[m]
    if len(q)<100: print(name,'few',len(q)); return
    h,e=np.histogram(q[:,1],bins=np.arange(yr[0],yr[1],0.02))
    peak=e[np.argmax(h)]
    sub=q[(q[:,1]>peak-0.6)&(q[:,1]<peak+0.6)]
    print(name,'n',len(q),'y_peak',round(float(peak),2),'sigma',round(float(sub[:,1].std()),3),'p5p95',round(float(np.percentile(sub[:,1],5)),3),round(float(np.percentile(sub[:,1],95)),3))
wall_stats(old,'OLD x[0,5] y[6,10]',(0,5),(6,10))
wall_stats(old,'OLD x[-5,0] y[6,10]',(-5,0),(6,10))
wall_stats(live,'LIVE x[-2,2] y[7,10]',(-2,2),(7,10))
# floor flatness: z spread of points near floor
def floor_stats(p,name):
    zmin=np.percentile(p[:,2],5)
    q=p[(p[:,2]>zmin-0.15)&(p[:,2]<zmin+0.15)]
    print(name,'floor_z_p5',round(float(np.percentile(p[:,2],5)),3),'floor_band_n',len(q),'floor_std',round(float(q[:,2].std()),3))
floor_stats(old,'OLD')
floor_stats(live,'LIVE')
