cmake_minimum_required(VERSION 3.8)
project(serial VERSION 1.2.2 LANGUAGES CXX)

# =============================================================================
# serial —— 极简串口库（POSIX termios 实现）
# =============================================================================
# ★ 为什么有这个包：
#   原库 wjwwood/serial 是 ROS 1 时代的 catkin 包，**没有 ROS 2 的 apt 包**
#   （ros-humble-serial 不存在），且从源码编译需 GitHub 可达（本环境不通）。
#   dzmission / raw_transport / referee_board / cmd_link 都要
#   `#include "serial/serial.h"` 与 `serial::Serial`，
#   所以这里自建一个**接口兼容**的最小实现。
#
# ★★ 踩过的坑（重要，避免重犯）★★
#   第一版把 ament_package() 写成**条件调用**：
#       find_package(ament_cmake QUIET)
#       if(ament_cmake_FOUND)  ament_package()  endif()
#   但 package.xml 里声明了 <build_type>ament_cmake</build_type>
#     -> colcon 认为这是 ament 包，去 install/serial/share/serial/ 找 package.sh
#     -> 而 package.sh 正是 ament_package() 生成的
#     -> colcon 报：
#          Failed to find .../share/serial/package.sh
#          Check that the following packages have been built: serial
#        且 install/ 里**根本没有 serial**，dzmission 也随之编不过。
#
#   ==> 修正：**无条件** find_package(ament_cmake REQUIRED) + ament_package()。
#       package.xml 声明是 ament 包，CMakeLists 就必须真的履行 ament 包的义务。
#
# ★★ 第二个坑：导出方式 ★★
#   单纯 install(EXPORT ...) + 手写 Config 在 ament 里不够 ——
#   必须用 ament_export_targets() 把导入目标登记进 ament 索引，
#   否则下游 find_package(serial) 拿不到 serial::serial。
# =============================================================================

# ---- ament（必须，与 package.xml 的 build_type 一致）----
find_package(ament_cmake REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- 库 ----
add_library(serial STATIC src/serial.cpp)
add_library(serial::serial ALIAS serial)

target_include_directories(serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 串口实现依赖 POSIX（termios / ioctl / select）；Threads 供 <chrono>/<thread>
find_package(Threads REQUIRED)
target_link_libraries(serial PUBLIC Threads::Threads)

# ---- 安装头文件 ----
install(DIRECTORY include/
  DESTINATION include
)

# ---- 安装库 + 导出目标 ----
install(TARGETS serial
  EXPORT export_serial
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_serial HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Threads)

# ---- 自测程序（不需要真实串口也能跑）----
add_executable(serial_selftest test/serial_selftest.cpp)
target_link_libraries(serial_selftest PRIVATE serial::serial)
install(TARGETS serial_selftest
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 收尾（必须；package.sh 等 ament 环境文件在此生成）----
ament_package()

message(STATUS "serial: 库 serial::serial 已就绪；自测程序 serial_selftest")
