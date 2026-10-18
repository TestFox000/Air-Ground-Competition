cmake_minimum_required(VERSION 3.5)
project(dztrack)

# Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(laser_geometry REQUIRED)
# find_package(nav_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
# find_package(PCL REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)


find_package(Boost REQUIRED COMPONENTS thread atomic system regex)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/
)

set(dependencies
  rclcpp
  std_msgs
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  # behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs 
  pcl_ros
  eigen3_cmake_module
  PCL
  Eigen3
  visualization_msgs
  laser_geometry
  Boost
  # nav_core
  # std_srvs
  nav2_util
)
# ament_package()
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/dztrack_path_bridge.py
  DESTINATION lib/${PROJECT_NAME})

add_executable(dztrack src/dztrack.cpp src/control.cpp)


############################3
ament_target_dependencies(dztrack  ${dependencies} )


##替代CATKIN_DEPENDS
ament_export_dependencies(${dependencies} plan2control)


install(TARGETS
  dztrack
  DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}    # 程序运行的时候调用install中的路径
)

##############

####################


ament_package()