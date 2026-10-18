cmake_minimum_required(VERSION 3.8)
project(dzmission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(serial REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  visualization_msgs
)

include_directories(include)

add_executable(dzmission_node
  src/mission_main.cpp
  src/mission_manager.cpp
  src/mission_command.cpp
  src/bridge_tracker.cpp
  src/cmd_link.cpp
  src/motion_arbiter.cpp
  src/eskf.cpp
  src/localization_compensator.cpp
  src/safety_monitor.cpp
  src/target_aimer.cpp
  src/referee_board.cpp
  src/field_grid.cpp
  src/mine_planner.cpp
  src/pivot_controller.cpp
  src/air_protocol.cpp
  src/raw_transport.cpp
  src/air_link.cpp
)

# ★ 用导入目标 serial::serial（由本工作空间的 serial 包提供）。
#   原写法 ${serial_LIBRARIES} 依赖 serial 包导出一个**普通变量**，
#   而 ament_export_targets() 导出的是**导入目标**，不保证存在该变量。
#   serial::serial 已在 check_serial.sh 第 5 节实测可用。
target_link_libraries(dzmission_node serial::serial)
ament_target_dependencies(dzmission_node ${dependencies})

# -----------------------------------------------------------------------------
# 协议层独立自检程序（★ 不依赖 ROS）
#   用于与无人机端做逐字节对撞。air_protocol.cpp 是纯逻辑，链接它不需要 ROS，
#   所以可以在任何有 g++ 的机器上直接跑：
#     g++ -std=c++17 -I include src/air_protocol.cpp \
#         test/air_protocol_selftest.cpp -o /tmp/air_self && /tmp/air_self
#   这里也注册成 CMake target，方便 colcon build 时一并编译检查语法。
# -----------------------------------------------------------------------------
add_executable(bridge_selftest
  src/bridge_tracker.cpp
  test/bridge_selftest.cpp
  )
target_include_directories(bridge_selftest PRIVATE include)

add_executable(air_protocol_selftest
  src/air_protocol.cpp
  test/air_protocol_selftest.cpp
)
target_include_directories(air_protocol_selftest PRIVATE include)

install(TARGETS
  dzmission_node
  air_protocol_selftest
  bridge_selftest
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params doc tools
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/set_affinity.sh
  test/protocol_check.py
  test/drone_sim_check.py
  test/time_source_check.py
  test/params_check.py
  test/field_grid_check.py
  test/air_protocol.py
  # ★ 无人机端离线模拟器（把地图 YAML 转成空地协议 v2 帧，UDP 发给车端）
  #   用于不接真无人机时驱动排雷/桥洞/打击等功能
  tools/drone_sim.py
  tools/fake_sensors.py
  test/topology_v5_check.py
  test/turn_check.py
  test/geometry_check.py
  test/pivot_turn_check.py
  test/pivot_controller_check.py
  test/mine_planner_check.py
  test/safety_check.py
  test/aimer_check.py
  test/aimer_interlock_check.py
  test/moving_target_check.py
  test/moving_target_beta_check.py
  test/integration_check.py
  test/cpp_callsite_check.py
  test/cpp_include_check.py
  test/ros_pkg_check.py
  test/atomic_check.py
  test/ns_check.py
  test/helper_check.py
  test/include_closure_check.py
  test/bridge_check.py
  test/doc_consistency_check.py
  test/compute_audit.py
  test/air_protocol_cpp_mirror.py
  test/opcode_check.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
