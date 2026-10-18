import argparse, os, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def load_pcd(path):
    with open(path,'rb') as f:
        fields=sizes=types=counts=None; npts=None; data=None
        while True:
            raw=f.readline()
            if not raw: raise RuntimeError('EOF header')
            s=raw.decode('ascii','replace').strip()
            if not s or s.startswith('#'): continue
            t=s.split(); k=t[0].upper()
            if k=='FIELDS': fields=t[1:]
            elif k=='SIZE': sizes=[int(v) for v in t[1:]]
            elif k=='TYPE': types=t[1:]
            elif k=='COUNT': counts=[int(v) for v in t[1:]]
            elif k=='POINTS': npts=int(t[1])
            elif k=='DATA': data=t[1].lower(); break
        if counts is None: counts=[1]*len(fields)
        if data=='binary':
            dt=[]
            for name,sz,ty,cnt in zip(fields,sizes,types,counts):
                if ty=='F': base='<f4' if sz==4 else '<f8'
                elif ty=='I': base={1:'<i1',2:'<i2',4:'<i4',8:'<i8'}[sz]
                else: base={1:'<u1',2:'<u2',4:'<u4',8:'<u8'}[sz]
                dt.append((name,base,(cnt,)) if cnt>1 else (name,base))
            arr=np.frombuffer(f.read(), dtype=np.dtype(dt), count=npts)
            p=np.stack([arr['x'],arr['y'],arr['z']],axis=1).astype(np.float64)
        else:
            a=np.loadtxt(f, max_rows=npts)
            idx={n:i for i,n in enumerate(fields)}
            p=a[:,[idx['x'],idx['y'],idx['z']]].astype(np.float64)
    return p

def load_input(path):
    if path.lower().endswith('.npy'):
        a=np.load(path)
        if a.ndim!=2 or a.shape[0]==0: return np.zeros((0,3))
        return a.astype(np.float64)
    return load_pcd(path)

def stats(p):
    if len(p)==0: return {'n':0}
    return {'n':int(len(p)),
            'min':[round(float(v),3) for v in p.min(0)],
            'max':[round(float(v),3) for v in p.max(0)],
            'bbox':[round(float(p[:,i].max()-p[:,i].min()),3) for i in range(3)],
            'z_p05':round(float(np.percentile(p[:,2],5)),3),
            'z_p50':round(float(np.percentile(p[:,2],50)),3),
            'z_p95':round(float(np.percentile(p[:,2],95)),3)}

def render(p, prefix, title, path=None, cell=0.02):
    st=stats(p)
    if len(p)==0:
        print('EMPTY', title); return
    x,y,z=p[:,0],p[:,1],p[:,2]
    x0,x1=float(x.min()),float(x.max()); y0,y1=float(y.min()),float(y.max())
    nx=min(max(10,int(np.ceil((x1-x0)/cell))),3000); ny=min(max(10,int(np.ceil((y1-y0)/cell))),3000)
    xr=(x0,x0+nx*cell); yr=(y0,y0+ny*cell)
    H,_,_=np.histogram2d(x,y,bins=[nx,ny],range=[xr,yr],weights=z)
    C,_,_=np.histogram2d(x,y,bins=[nx,ny],range=[xr,yr])
    Z=np.where(C>0,H/np.maximum(C,1),np.nan)
    ext=[xr[0],xr[1],yr[0],yr[1]]
    fig,axs=plt.subplots(1,2,figsize=(16,7.8),dpi=140)
    ax=axs[0]
    ax.imshow(np.log10(C.T+1), origin='lower', extent=ext, aspect='equal', cmap='gray_r', interpolation='nearest')
    ax.set_title('density  log10(points+1) / %.2fm cell' % cell)
    if path is not None and len(path): ax.plot(path[:,1],path[:,2],color='tab:red',lw=0.8,alpha=0.85)
    ax=axs[1]
    im=ax.imshow(Z.T, origin='lower', extent=ext, aspect='equal', cmap='turbo', interpolation='nearest')
    plt.colorbar(im, ax=ax, fraction=0.046, pad=0.02, label='mean Z (m)')
    ax.set_title('height (mean Z per cell)')
    if path is not None and len(path): ax.plot(path[:,1],path[:,2],color='w',lw=0.8,alpha=0.8)
    for ax in axs:
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.grid(True, color='0.92', lw=0.3)
    fig.suptitle('%s   N=%d   bbox=%.1f x %.1f x %.1f m   Z[%.2f..%.2f]' % (title, st['n'], st['bbox'][0], st['bbox'][1], st['bbox'][2], st['min'][2], st['max'][2]), fontsize=11)
    fig.tight_layout(rect=[0,0,1,0.95])
    fig.savefig(prefix+'_top.png'); plt.close(fig)
    rng=np.random.default_rng(0); n=min(len(p),60000)
    q=p[rng.choice(len(p), n, replace=False)]
    fig=plt.figure(figsize=(13,9),dpi=130)
    ax=fig.add_subplot(111,projection='3d')
    ax.scatter(q[:,0],q[:,1],q[:,2],c=q[:,2],cmap='turbo',s=0.25,linewidths=0)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    try: ax.set_box_aspect((max(x1-x0,0.5), max(y1-y0,0.5), max(float(z.max()-z.min()),0.3)))
    except Exception: pass
    ax.view_init(elev=32, azim=-62)
    ax.set_title('%s   3D view   N=%d (sampled %d)' % (title, st['n'], n))
    fig.tight_layout(); fig.savefig(prefix+'_3d.png'); plt.close(fig)
    print(json.dumps(st, ensure_ascii=False))

if __name__=='__main__':
    ap=argparse.ArgumentParser()
    ap.add_argument('--input',required=True)
    ap.add_argument('--prefix',required=True)
    ap.add_argument('--title',default='')
    ap.add_argument('--path',default='')
    ap.add_argument('--cell',type=float,default=0.02)
    a=ap.parse_args()
    p=load_input(a.input)
    path=None
    if a.path and os.path.exists(a.path):
        pa=np.load(a.path)
        if pa.ndim==2 and pa.shape[0]>0: path=pa
    render(p,a.prefix,a.title or os.path.basename(a.input),path,a.cell)
