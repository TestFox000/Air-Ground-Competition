#!/bin/bash
echo ---HOST; hostname
echo ---HOME; ls ~
echo ---DZUAV; ls -la ~/dzuav | head -40
echo ---TRT; python3 -c 'import tensorrt as trt; print("trt", trt.__version__)' 2>&1 | tail -1
echo ---ONNX; python3 -c 'import onnxruntime as o; print("ort", o.__version__)' 2>&1 | tail -1
echo ---PYTORCH; python3 -c 'import torch; print("torch", torch.__version__, torch.cuda.is_available())' 2>&1 | tail -1
echo ---RELEASE; head -1 /etc/nv_tegra_release 2>/dev/null
echo ---MODELS; find ~/dzuav -maxdepth 4 -type f \( -name "*.onnx" -o -name "*.engine" -o -name "*.plan" -o -name "*.trt" -o -name "*.wts" -o -name "*.yaml" -o -name "*.yml" -o -name "*.json" \) 2>/dev/null | head -60
echo ---SCRIPTS; find ~/dzuav -maxdepth 3 -type f \( -name "*.py" -o -name "*.sh" \) 2>/dev/null | head -40
echo ---SIZE; du -sh ~/dzuav 2>/dev/null
echo ---IMAGES; find ~/dzuav -maxdepth 4 -type f \( -name "*.jpg" -o -name "*.png" \) 2>/dev/null | head -20
