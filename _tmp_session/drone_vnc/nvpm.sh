#!/bin/bash
echo "=== nvpmodel modes"
grep -E "^\[|CPU_MAX_FREQ|GPU_MAX_FREQ|SOC_MAX_FREQ" /etc/nvpmodel.conf 2>/dev/null | head -40
echo "=== nvpmodel help"
nvpmodel -h 2>&1 | head -8
echo "=== engines"
ls -la /home/duzhong/dzuav/engines/ 2>/dev/null
echo "=== trtexec"
which trtexec; trtexec --version 2>/dev/null | head -3
echo "=== onnx models"
ls -la /home/duzhong/dzuav/models/ 2>/dev/null | head -10
