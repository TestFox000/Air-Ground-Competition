cmake_minimum_required(VERSION 3.5)
project(dzactuator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial)
find_package(geometry_msgs)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread atomic system regex)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()



set(dependencies
  rclcpp
  sensor_msgs
  serial
  nav_msgs
  Boost
  tf2
  tf2_msgs
  tf2_ros
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY launch launch_auto rviz urdf params
  DESTINATION share/${PROJECT_NAME})
  

# 找头文件的路径
# include_directories(
# #   ${catkin_INCLUDE_DIRS}
#   /usr/include/
#   /opt/ros/foxy/share/
# )
include_directories(
  include
)

include_directories(
  /usr/include/
)
# include_directories("/home/duzhong/Third_package/serial-foxy/include/") 
# link_directories("/home/duzhong/Third_package/serial-foxy/build") 

add_executable(dzactuator_node src/dzactuator.cpp src/Quaternion_Solution.cpp ) 


ament_target_dependencies(dzactuator_node ${dependencies} )

install(TARGETS
  dzactuator_node
  DESTINATION lib/${PROJECT_NAME}
  # ${SERIAL_FOXY_LIBRARIES}
  # DESTINATION /home/duzhong/Third_package/serial-foxy/build
)

# target_link_libraries(dzactuator_node  libserial.so)

