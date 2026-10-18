#!/bin/bash
echo ---SRC; find ~/dzuav/src -maxdepth 2 -type f | head -40
echo ---BUILD; ls -la ~/dzuav/build ~/dzuav/build/* 2>/dev/null | head -30
echo ---ENGINES; ls -la ~/dzuav/engines ~/dzuav/engines_d13
echo ---MODELS; ls -la ~/dzuav/models
echo ---D13LOG; head -40 ~/dzuav/../d13_engine_build.log 2>/dev/null
echo ---V4DLOG; head -20 ~/v4d_engine.log 2>/dev/null
echo ---VENV; ls ~/dzuav/.export-venv/bin 2>/dev/null | head; ~/dzuav/.export-venv/bin/python -c 'import torch; print("torch", torch.__version__)' 2>&1 | tail -1
echo ---TRTEXEC; which trtexec; trtexec --version 2>&1 | head -3
echo ---AGTEST; ls ~/ag_test 2>/dev/null | head -20; du -sh ~/ag_test 2>/dev/null
echo ---DATASETS; find ~ -maxdepth 3 -type d \( -name "*dataset*" -o -name "*data*" -o -name "*val*" \) 2>/dev/null | grep -v -E "node_modules|ros_logs|dzpx4|Livox|Micro|Fast-DDS" | head -20
echo ---IMAGES; find ~/ag_test ~/dzuav ~/Desktop -type f \( -name "*.jpg" -o -name "*.png" \) 2>/dev/null | head -20
echo ---SHA; sha256sum ~/dzuav/models/cell_classifier_v4d_20261015.onnx ~/dzuav/models/hbb_detector.onnx ~/dzuav/engines_d13/*.engine 2>/dev/null
