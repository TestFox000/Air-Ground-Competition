cmake_minimum_required(VERSION 3.8)
project(px4_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlCommand.msg"
  "msg/GotoTarget.msg"
)

include_directories(include)

add_executable(px4_teleop_node
  src/px4_teleop_node.cpp
)
ament_target_dependencies(px4_teleop_node
  rclcpp
)

add_executable(px4_control_node
  src/px4_control_node.cpp
)
ament_target_dependencies(px4_control_node
  rclcpp
  std_msgs
  px4_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(px4_teleop_node  "${cpp_typesupport_target}")
target_link_libraries(px4_control_node "${cpp_typesupport_target}")

install(TARGETS
  px4_teleop_node
  px4_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
