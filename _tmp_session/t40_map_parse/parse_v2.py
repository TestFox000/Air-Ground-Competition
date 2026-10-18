
from PIL import Image
import numpy as np, json

SRC = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t40_map_parse\map_new.png"
im = Image.open(SRC).convert("RGB"); a = np.asarray(im).astype(np.int32)
H,W,_ = a.shape
R,G,B = a[:,:,0],a[:,:,1],a[:,:,2]

COLS = "ABCDEFGHIJKLMN"
COLX = [70+128*i for i in range(14)]
ROWY = [84+128*j for j in range(11)]   # j=0 -> row1

def classify(patch):
    r,g,b = patch[:,:,0],patch[:,:,1],patch[:,:,2]
    n = patch.shape[0]*patch.shape[1]
    def frac(m): return float(m.sum())/n
    red = frac((r>230)&(g<130)&(b<130))
    mine= frac((r>240)&(g>195)&(b<80))
    pale= frac((r>=220)&(r<=252)&(g>=180)&(g<=228)&(b>=75)&(b<=165))
    mag = frac((r>235)&(g<170)&(b>160)&(b-r<40))
    grn = frac((g>150)&(r<150)&(b<160)&(g-r>40))
    cand = dict(red=red,mine=mine,pale=pale,mag=mag,green=grn)
    k = max(cand,key=cand.get); v = cand[k]
    return (k if v>0.25 else "empty"), {kk:round(vv,3) for kk,vv in cand.items() if vv>0.05}

grid = {}
for j in range(11):
    for i in range(14):
        cx,cy = COLX[i],ROWY[j]
        patch = a[cy-50:cy+50, cx-50:cx+50]
        cell = COLS[i]+str(j+1)
        cls,fr = classify(patch)
        grid[cell] = (cls,fr)

groups = {}
for cell,(cls,fr) in grid.items():
    groups.setdefault(cls,[]).append(cell)
for k in groups:
    print(k, len(groups[k]), sorted(groups[k], key=lambda c:(int(c[1:]),c[0])))

# barrier line: greenish pixels between row6 and row7
y0,y1 = ROWY[5]+40, ROWY[6]-40
band = (G[y0:y1,:]>150)&(R[y0:y1,:]<160)&(B[y0:y1,:]<170)&(G[y0:y1,:]-R[y0:y1,:]>30)
ys,xs = np.nonzero(band)
if len(xs):
    print("BARRIER y band", y0,y1, "x range", xs.min(), xs.max(), "count", len(xs))
    # per-column coverage of barrier: fraction of x positions having any barrier pixel
    colhas = sorted(set(xs.tolist()))
    segs=[]; s=colhas[0]; p=colhas[0]
    for x in colhas[1:]:
        if x-p>6: segs.append((s,p)); s=x
        p=x
    segs.append((s,p))
    print("BARRIER segments", segs)
    def x2col(x): return COLS[int(round((x-70)/128))] if 0<=int(round((x-70)/128))<14 else "?"
    print("BARRIER cols", [(x2col(s),x2col(e)) for s,e in segs])
else:
    print("BARRIER none in band")
