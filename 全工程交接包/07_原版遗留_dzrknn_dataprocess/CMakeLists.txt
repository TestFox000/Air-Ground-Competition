cmake_minimum_required(VERSION 3.8)
project(dzrknn_dataprocess)

# ---- C++ 标准 ----
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
)

# ---- 头文件目录 ----
include_directories(
  include
)

# ---- 可执行文件 ----
add_executable(dzactuator_node src/dzrknn_dataprocess.cpp)
ament_target_dependencies(dzactuator_node ${dependencies})

# ---- 安装 ----
install(TARGETS
  dzactuator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 如果加了 YAML 参数文件，把 config 一起安装
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ---- 测试 ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ---- 必须放在最后 ----
ament_package()