
import numpy as np, json
p = np.load(r"D:\User\Desktop\比赛存档\v1\_tmp_session\drone_map\walk1\path_latest.npy")
print("shape", p.shape)
t, x, y, z = p[:,0], p[:,1], p[:,2], p[:,3]
d = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
dt = np.diff(t)
v = d/np.maximum(dt,1e-3)
print("duration %.1fs  n=%d" % (t[-1]-t[0], len(p)))
print("x range %.1f..%.1f  y range %.1f..%.1f  z %.2f..%.2f" % (x.min(),x.max(),y.min(),y.max(),z.min(),z.max()))
p50,p90,p99 = np.percentile(d,[50,90,99])
s50,s90,s99 = np.percentile(v,[50,90,99])
print("step dist: p50 %.3f  p90 %.3f  p99 %.3f  max %.2f m" % (p50,p90,p99,d.max()))
print("speed m/s: p50 %.2f p90 %.2f p99 %.2f max %.1f" % (s50,s90,s99,v.max()))
print("steps >5m:", int((d>5).sum()), " >10m:", int((d>10).sum()), " >30m:", int((d>30).sum()))
idx = np.where(d>10)[0]
for i in idx[:15]:
    print("  t=%.2f  (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)  jump %.1fm in %.2fs" % (t[i],x[i],y[i],z[i],x[i+1],y[i+1],z[i+1],d[i],dt[i]))
for k in range(0, int(t[-1])+1, 60):
    m = (t>=k)&(t<k+60)
    if m.sum()>1:
        print("t %3d-%3ds  n=%4d  x %.1f..%.1f y %.1f..%.1f" % (k,k+60,m.sum(),x[m].min(),x[m].max(),y[m].min(),y[m].max()))
