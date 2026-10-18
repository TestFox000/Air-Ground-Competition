# -*- coding: utf-8 -*-
import re
files = {
 "d13": r"D:\User\Desktop\比赛存档\视觉识别模型\无人机视觉模型\hbb_detector_d13_20261016(1).onnx",
 "cell_v4d": r"D:\User\Desktop\比赛存档\视觉识别模型\无人机视觉模型\cell_classifier_v4d_20261015.onnx",
}
for k, p in files.items():
    b = open(p, "rb").read()
    print("=====", k, "size", len(b))
    for key in (b"names", b"imgsz", b"task", b"batch"):
        i = b.find(key)
        print(" key", key.decode(), "at", i)
        if i >= 0:
            seg = b[i:i+260]
            txt = re.sub(rb"[^\x20-\x7e]", b" ", seg).decode("ascii", "replace")
            print("   ", txt[:220])
    runs = [r for r in re.findall(rb"[\x20-\x7e]{12,}", b)]
    hits = [r for r in runs if b"cell" in r.lower() or b"letter" in r.lower() or b"digit" in r.lower()]
    for r in hits[:4]:
        print("   RUN:", r[:200].decode("ascii", "replace"))
