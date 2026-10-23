cmake_minimum_required(VERSION 3.8)
project(naviPointPath)

find_package(ament_cmake REQUIRED)

# 纯数据目录：安装路径/地图/前向/后向航点文件，供其他节点按包路径读取。
install(DIRECTORY forwardcar_paths backcar_paths map
        DESTINATION share/${PROJECT_NAME})

ament_package()
