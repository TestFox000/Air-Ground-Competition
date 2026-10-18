#!/bin/bash
set -o pipefail
cd ~/dzuav || exit 1
mkdir -p engines
/usr/src/tensorrt/bin/trtexec --onnx=models/cell_classifier_v4d_20261015.onnx --saveEngine=engines/cell_classifier_v4d_fp16.engine --fp16 > /tmp/cell_v4d_build.log 2>&1
echo "trtexec exit: $?"
tail -6 /tmp/cell_v4d_build.log
ls -la engines/cell_classifier_v4d_fp16.engine 2>/dev/null
