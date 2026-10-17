import hashlib
from PIL import Image
from collections import Counter

P = r'D:\User\Desktop\比赛存档\v1\_tmp_session\t39_map_official\map_official.png'
raw = open(P,'rb').read()
print('sha256:', hashlib.sha256(raw).hexdigest())
im = Image.open(P).convert('RGB')
w,h = im.size
px = im.load()
c = Counter(px[x,y] for y in range(0,h,5) for x in range(0,w,5))
print('size:', w, h)
print('top colors:', c.most_common(15))
