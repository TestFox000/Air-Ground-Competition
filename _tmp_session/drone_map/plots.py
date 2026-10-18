import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
d=np.load('diag2.npz')
od=d['odom']; lc=d['loc']
def yaw(a):
    q=a[:,4:8]
    return np.unwrap(np.arctan2(2*(q[:,3]*q[:,2]+q[:,0]*q[:,1]), 1-2*(q[:,1]**2+q[:,2]**2)))
fig,axs=plt.subplots(2,2,figsize=(14,7.5),dpi=140)
for a,lab,c in ((od,'odom (FAST-LIO)','tab:blue'),(lc,'localization (map frame)','tab:red')):
    if len(a)<3: continue
    t=a[:,0]-a[0,0]
    axs[0,0].plot(t,a[:,1],c,lw=0.9,label=lab); axs[0,0].set_ylabel('x (m)')
    axs[0,1].plot(t,a[:,2],c,lw=0.9,label=lab); axs[0,1].set_ylabel('y (m)')
    axs[1,0].plot(t,a[:,3],c,lw=0.9,label=lab); axs[1,0].set_ylabel('z (m)')
    axs[1,1].plot(t,np.degrees(yaw(a)),c,lw=0.9,label=lab); axs[1,1].set_ylabel('yaw (deg)')
for ax in axs.reshape(-1):
    ax.set_xlabel('t (s)'); ax.grid(True,color='0.92',lw=0.4); ax.legend(fontsize=8)
fig.suptitle('15 s pose time series: /Odometry vs /localization',fontsize=11)
fig.tight_layout(rect=[0,0,1,0.95]); fig.savefig('series15s.png'); print('saved series15s.png')
