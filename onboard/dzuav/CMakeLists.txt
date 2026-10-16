# 3.16 = JetPack 5（focal）apt 仓库的 cmake 版本；工程未用到更高版本的特性，
# 卡 3.20 会让老 JetPack 无法开箱编译。
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

# 前置依赖缺失时给出明确报错（本程序须在 Jetson 上构建：
# JetPack >= 5，自带 TensorRT 8.5+ 与 CUDA 工具链）。
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
  src/UdpBroadcaster.cpp)

target_include_directories(dzuav PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE}
  /usr/local/cuda/include)

target_compile_options(dzuav PRIVATE -Wall -Wextra)

# 架构优化：Orin Nano（Orin 系列 SoC）的 CPU 是 Cortex-A78AE——本程序绑死
# Jetson 平台本来就是事实（TensorRT/CUDA/GStreamer 管线都离不了），无需伪装可移植。
# 注意 JetPack 5 默认 GCC 9.4 还不认识 cortex-a78ae（GCC 10.1 才加入），
# 故先探测：支持则 -mcpu=cortex-a78ae，不支持退回 -mtune=cortex-a76
# （mtune 只影响指令调度、不改 ISA，GCC 8+ 都认识，安全兜底）。
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mcpu=cortex-a78ae" DZUAV_HAS_MCPU_A78AE)
if(DZUAV_HAS_MCPU_A78AE)
  target_compile_options(dzuav PRIVATE -mcpu=cortex-a78ae)
else()
  message(STATUS "compiler does not know -mcpu=cortex-a78ae (e.g. GCC 9.4); falling back to -mtune=cortex-a76")
  target_compile_options(dzuav PRIVATE -mtune=cortex-a76)
endif()

target_link_libraries(dzuav PRIVATE ${OpenCV_LIBS} ${TENSORRT_LIB} ${CUDART_LIB})

install(TARGETS dzuav DESTINATION lib/dzuav)
