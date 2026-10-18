# -*- coding: utf-8 -*-
from PIL import Image
p = r"D:/User/Desktop/比赛存档/v1/_tmp_session/t28_run/snap_t28.jpg"
im = Image.open(p)
w, h = im.size
crop = im.crop((0, int(h*0.40), w, int(h*0.95)))
crop = crop.resize((crop.width*2, crop.height*2), Image.LANCZOS)
out = r"D:/User/Desktop/比赛存档/v1/_tmp_session/t28_run/snap_labels.png"
crop.save(out)
print("saved", crop.size)
