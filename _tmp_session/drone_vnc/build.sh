#!/bin/bash
set -e
cd /tmp/dzuav_view
g++ -O2 -std=c++17 -I/home/duzhong/dzuav/include -I/usr/local/cuda/include -I/usr/include/opencv4 \
  view_1280.cpp /home/duzhong/dzuav/src/HbbDetector.cpp /home/duzhong/dzuav/src/CellClassifier.cpp /home/duzhong/dzuav/src/CellGrid.cpp \
  /usr/lib/libopencv_dnn.so /usr/lib/libopencv_highgui.so /usr/lib/libopencv_videoio.so /usr/lib/libopencv_imgcodecs.so /usr/lib/libopencv_imgproc.so /usr/lib/libopencv_core.so \
  -L/usr/local/cuda/lib64 -lcudart -lnvinfer -lnvinfer_plugin -o dzuav_view_1280
echo BUILD_OK
ls -la dzuav_view_1280
