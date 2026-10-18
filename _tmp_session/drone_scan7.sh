#!/bin/bash
python3 - <<'PY'
import re
files = [
 "/home/duzhong/dzuav_deploy/air_ground/models/release/hbb_detector_7cls_distill_20261011.onnx",
 "/home/duzhong/dzuav/models/hbb_detector_7cls_combined_20261015.onnx",
 "/home/duzhong/dzuav/models/hbb_detector_7cls_direct_20261015.onnx",
 "/home/duzhong/dzuav/models/hbb_detector_7cls_minecpv2_20261015.onnx",
 "/home/duzhong/dzuav/models/hbb_detector.onnx",
]
for p in files:
    try:
        b = open(p, "rb").read()
    except Exception as e:
        print(p, "ERR", e); continue
    i = b.find(b"names")
    print("=====", p.split("/")[-1], "size", len(b), "names_at", i)
    if i >= 0:
        seg = b[i:i+240]
        txt = re.sub(rb"[^\x20-\x7e]", b" ", seg).decode("ascii", "replace")
        print("   ", txt[:220])
PY
