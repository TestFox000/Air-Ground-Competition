# -*- coding: utf-8 -*-
import re
p = r"D:\User\Desktop\比赛存档\视觉识别模型\无人机视觉模型\hbb_detector_d13_20261016(1).onnx"
b = open(p, "rb").read()
print("size", len(b))
runs = re.findall(rb"[\x20-\x7e]{16,}", b)
words = (b"mine", b"cell_label", b"target_blue", b"obstacle", b"arrow", b"target_red", b"car")
hits = [r for r in runs if any(w in r.lower() for w in words)]
print("ascii runs with class words:", len(hits))
for r in hits[:6]:
    print("---", r[:500].decode("ascii", "replace"))
idx = b.find(b"names")
print("first 'names' at", idx)
if idx >= 0:
    print(b[max(0, idx-80): idx+320])
