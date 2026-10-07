cmake_minimum_required(VERSION 3.8)
project(det_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)   # 用于从 share/ 解析模型路径
find_package(OpenCV REQUIRED)

set(RKNN_RT_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/librknnrt.so)
set(RGA_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/librga.so)

add_executable(det_node
  src/det_node.cc
  src/det/postprocess.cc
  src/det/preprocess.cc
  src/det/rkpt.cc
)

target_include_directories(det_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rga
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rknn_api
  ${CMAKE_CURRENT_SOURCE_DIR}/include/det
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(det_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  ament_index_cpp
)

target_link_libraries(det_node
  ${OpenCV_LIBS}
)

if(EXISTS "${RKNN_RT_LIB}")
  message(STATUS "Found librknnrt: ${RKNN_RT_LIB}")
  target_link_libraries(det_node "${RKNN_RT_LIB}")
else()
  message(FATAL_ERROR "librknnrt.so not found: ${RKNN_RT_LIB}")
endif()

if(EXISTS "${RGA_LIB}")
  message(STATUS "Found librga: ${RGA_LIB}")
  target_link_libraries(det_node "${RGA_LIB}")
else()
  message(WARNING "librga.so not found: ${RGA_LIB}")
endif()

set_target_properties(det_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN/.."
)

install(TARGETS det_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/param")
  install(DIRECTORY param
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/model")
  install(DIRECTORY model
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()