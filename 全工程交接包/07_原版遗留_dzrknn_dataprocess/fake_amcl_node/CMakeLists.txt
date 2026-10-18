cmake_minimum_required(VERSION 3.8)
project(fake_amcl_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(
  fake_amcl_node
  src/fake_amcl_node.cpp
)

ament_target_dependencies(
  fake_amcl_node
  rclcpp
  nav_msgs
  geometry_msgs
)

install(
  TARGETS
    fake_amcl_node
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()