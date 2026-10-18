# -*- coding: utf-8 -*-
from PIL import Image, ImageEnhance
p = r"D:/User/Desktop/比赛存档/v1/_tmp_session/t28_run/snap_t28.jpg"
im = Image.open(p)
w, h = im.size
regions = {
  "lbl_left":  (0, int(h*0.62), int(w*0.30), int(h*0.98)),
  "lbl_mid":   (int(w*0.25), int(h*0.52), int(w*0.62), int(h*0.80)),
  "lbl_right": (int(w*0.72), int(h*0.55), int(w*1.0), int(h*0.95)),
}
for name, box in regions.items():
    c = im.crop(box)
    c = c.resize((c.width*4, c.height*4), Image.LANCZOS)
    c = ImageEnhance.Contrast(c).enhance(1.8)
    c.save(rf"D:/User/Desktop/比赛存档/v1/_tmp_session/t28_run/{name}.png")
    print(name, c.size)
