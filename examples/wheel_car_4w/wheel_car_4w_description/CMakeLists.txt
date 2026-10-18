cmake_minimum_required(VERSION 3.20)
project(wheel_car_4w_description)
find_package(ament_cmake REQUIRED)
# 安装 URDF 后通过 package:// 查找，不依赖源码目录或启动时的工作目录。
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
ament_package()
