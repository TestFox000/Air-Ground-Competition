cmake_minimum_required(VERSION 3.8)
project(odom_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Armadillo REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  quadrotor_msgs
  tf2_msgs
  tf2_ros
)

get_filename_component(WORKSPACE_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../"
  ABSOLUTE
)

set(POSE_UTILS_INCLUDE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../pose_utils/include"
)

set(POSE_UTILS_LIB
  "${WORKSPACE_ROOT}/install/pose_utils/lib/pose_utils/libpose_utils.so"
)

include_directories(
  include
  ${ARMADILLO_INCLUDE_DIRS}
  ${POSE_UTILS_INCLUDE_DIR}
)

add_executable(odom_visualization
  src/odom_visualization.cpp
)

ament_target_dependencies(odom_visualization
  ${dependencies}
)

target_link_libraries(odom_visualization
  ${ARMADILLO_LIBRARIES}
  ${POSE_UTILS_LIB}
)

set_target_properties(odom_visualization PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${WORKSPACE_ROOT}/install/pose_utils/lib/pose_utils"
)

install(TARGETS
  odom_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()