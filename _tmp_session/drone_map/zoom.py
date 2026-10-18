import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=np.load('map_latest.npy').astype(float)
print('n',len(p))
# wall near y=8.5, x in [-2,2]
sel=p[(p[:,0]>-2)&(p[:,0]<2)]
h,e=np.histogram(sel[:,1],bins=np.arange(6.0,11.0,0.01))
peak=e[np.argmax(h)]
print('y_peak',round(float(peak),2))
sub=sel[(sel[:,1]>peak-0.6)&(sel[:,1]<peak+0.6)]
print('thickness_sigma_m',round(float(sub[:,1].std()),3),'p5_p95',[round(float(np.percentile(sub[:,1],q)),3) for q in (5,95)])
m=(p[:,0]>-3.5)&(p[:,0]<3.5)&(p[:,1]>5.5)&(p[:,1]<11)
q=p[m]
x0,x1=-3.5,3.5; y0,y1=5.5,11; cell=0.01
nx=int((x1-x0)/cell); ny=int((y1-y0)/cell)
H,_,_=np.histogram2d(q[:,0],q[:,1],bins=[nx,ny],range=[[x0,x1],[y0,y1]],weights=q[:,2])
C,_,_=np.histogram2d(q[:,0],q[:,1],bins=[nx,ny],range=[[x0,x1],[y0,y1]])
Z=np.where(C>0,H/np.maximum(C,1),np.nan)
fig,ax=plt.subplots(figsize=(12,8),dpi=150)
im=ax.imshow(Z.T,origin='lower',extent=[x0,x1,y0,y1],aspect='equal',cmap='turbo',interpolation='nearest')
plt.colorbar(im,ax=ax,fraction=0.046,pad=0.02,label='mean Z (m)')
ax.set_title('zoom 1cm cells: x[-3.5,3.5] y[5.5,11]  N=%d'%len(q))
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.grid(True,color='0.9',lw=0.3)
fig.tight_layout(); fig.savefig('zoom_wall.png'); print('saved')
