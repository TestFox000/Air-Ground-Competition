# 13 类主链 + 机载规划层（D13 检测 → PnP 定位 → CellGrid 落格 → RoutePlanner）
cmake_minimum_required(VERSION 3.16)
project(dzuav LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)
find_path(TENSORRT_INCLUDE NvInfer.h PATHS /usr/include/aarch64-linux-gnu /usr/include)
find_library(TENSORRT_LIB nvinfer PATHS /usr/lib/aarch64-linux-gnu)
find_library(CUDART_LIB cudart PATHS /usr/local/cuda/lib64 /usr/local/cuda/targets/aarch64-linux/lib)

if(NOT TENSORRT_INCLUDE)
  message(FATAL_ERROR "NvInfer.h not found; install the JetPack TensorRT dev package (tensorrt-dev / libnvinfer-dev)")
endif()
if(NOT TENSORRT_LIB)
  message(FATAL_ERROR "libnvinfer not found; install the JetPack TensorRT runtime (libnvinfer8)")
endif()
if(NOT CUDART_LIB)
  message(FATAL_ERROR "libcudart not found; install the JetPack CUDA toolkit (cuda-cudart-dev)")
endif()

add_executable(dzuav
  src/main.cpp
  src/HbbDetector.cpp
  src/CellClassifier.cpp
  src/MapLocalizer.cpp
  src/UdpBroadcaster.cpp
  src/CellGrid.cpp
  src/RoutePlanner.cpp)

target_include_directories(dzuav PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE}
  /usr/local/cuda/include)

target_compile_options(dzuav PRIVATE -Wall -Wextra)

target_link_libraries(dzuav PRIVATE ${OpenCV_LIBS} ${TENSORRT_LIB} ${CUDART_LIB})

install(TARGETS dzuav DESTINATION lib/dzuav)
