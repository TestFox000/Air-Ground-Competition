# -*- coding: utf-8 -*-
from PIL import Image, ImageEnhance
p = r"D:/User/Desktop/比赛存档/v1/_tmp_session/t28_run/snap_t28.jpg"
im = Image.open(p)
for ang in (90, 270):
    r = im.rotate(ang, expand=True)
    r = ImageEnhance.Contrast(r).enhance(1.4)
    r = r.resize((r.width*2, r.height*2), Image.LANCZOS)
    r.save(rf"D:/User/Desktop/比赛存档/v1/_tmp_session/t28_run/rot{ang}.png")
    print(ang, r.size)
