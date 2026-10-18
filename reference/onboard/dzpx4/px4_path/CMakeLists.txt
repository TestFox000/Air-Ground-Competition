cmake_minimum_required(VERSION 3.8)
project(px4_path)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_control REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(px4_path_core
  src/motion_trajectory.cpp
  src/path_loader.cpp
)
target_include_directories(px4_path_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(path_record_node
  src/path_record_node.cpp
  src/path_record_process.cpp
)
target_include_directories(path_record_node PRIVATE include)
ament_target_dependencies(path_record_node
  geometry_msgs
  nav_msgs
  rclcpp
)

add_executable(path_follow_node
  src/path_follow.cpp
  src/path_follow_waypoint_node.cpp
  src/path_follow_trajectory_node.cpp
)
target_include_directories(path_follow_node PRIVATE include)
ament_target_dependencies(path_follow_node
  px4_control
  px4_msgs
  rclcpp
  std_msgs
)
target_link_libraries(path_follow_node px4_path_core)

install(TARGETS
  px4_path_core
  path_record_node
  path_follow_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/path")
  install(DIRECTORY path
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(px4_path_core)

ament_package()
