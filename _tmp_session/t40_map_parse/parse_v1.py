
from PIL import Image
import numpy as np
from collections import Counter, deque
import json

SRC = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t40_map_parse\map_new.png"
im = Image.open(SRC).convert("RGB")
a = np.asarray(im).astype(np.int32)
h, w, _ = a.shape
print("SIZE", w, h)
print("TOPCOLORS", Counter(map(tuple, a[::7,::7].reshape(-1,3))).most_common(12))

R,G,B = a[:,:,0],a[:,:,1],a[:,:,2]
def bbox(mask,name):
    ys,xs=np.nonzero(mask)
    if len(xs)==0: print(name,"EMPTY"); return None
    bb=(int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max()))
    print(name,"count",len(xs),"bbox",bb); return bb

# candidate masks
bg   = (np.abs(R-242)<=4)&(np.abs(G-242)<=4)&(np.abs(B-242)<=4)
bbox(bg,"BG242")
red  = (R>200)&(G<110)&(B<110)
mag  = (R>200)&(G>70)&(G<190)&(B>150)
grn  = (G>100)&(R<120)&(B<140)
yel  = (R>215)&(G>150)&(B<130)
bbox(red,"RED"); bbox(mag,"MAG"); bbox(grn,"GRN"); bbox(yel,"YEL")

def ds(mask,k=4):
    hh=mask.shape[0]//k*k; ww=mask.shape[1]//k*k
    m=mask[:hh,:ww].reshape(hh//k,k,ww//k,k)
    return (m.mean(axis=(1,3))>0.6)

def comps(mask,k=4,minpx=20):
    m=ds(mask,k); H,W=m.shape; lab=np.zeros((H,W),np.int8); out=[]
    for y in range(H):
        for x in range(W):
            if m[y,x] and not lab[y,x]:
                q=deque([(y,x)]); lab[y,x]=1; ys=[];xs=[]
                while q:
                    cy,cx=q.popleft(); ys.append(cy); xs.append(cx)
                    for dy,dx in ((1,0),(-1,0),(0,1),(0,-1)):
                        ny,nx=cy+dy,cx+dx
                        if 0<=ny<H and 0<=nx<W and m[ny,nx] and not lab[ny,nx]:
                            lab[ny,nx]=1; q.append((ny,nx))
                if len(xs)>=minpx// (k*k):
                    x0,x1,y0,y1=min(xs)*k,max(xs)*k,min(ys)*k,max(ys)*k
                    reg=a[y0:y1+1,x0:x1+1].reshape(-1,3).mean(axis=0)
                    out.append(dict(cx=(x0+x1)/2, cy=(y0+y1)/2, w=x1-x0, h=y1-y0, n=len(xs), rgb=[round(v) for v in reg]))
    return out

for name,mask in (("RED",red),("MAG",mag),("GRN",grn),("YEL",yel)):
    cs=comps(mask)
    print("==",name,len(cs))
    for c in sorted(cs,key=lambda c:-c["w"]*c["h"]):
        print("  ",json.dumps(c))
