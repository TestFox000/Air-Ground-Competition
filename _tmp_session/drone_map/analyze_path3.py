
import numpy as np
p = np.load(r"D:\User\Desktop\比赛存档\v1\_tmp_session\drone_map\walk3\path_latest.npy")
t,x,y,z = p[:,0],p[:,1],p[:,2],p[:,3]
d = np.sqrt(np.diff(x)**2+np.diff(y)**2)
print("n=%d dur=%.1fs" % (len(p), t[-1]-t[0]))
print("x %.1f..%.1f y %.1f..%.1f z %.2f..%.2f" % (x.min(),x.max(),y.min(),y.max(),z.min(),z.max()))
print("step p50 %.3f p90 %.3f p99 %.3f max %.2f" % (np.percentile(d,[50,90,99])[0],np.percentile(d,[50,90,99])[1],np.percentile(d,[50,90,99])[2],d.max()))
print("steps>1m", int((d>1).sum()), ">5m", int((d>5).sum()))
