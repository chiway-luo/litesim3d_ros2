cmake_minimum_required(VERSION 3.20)
project(wheel_car_4w_description)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/config/geometry.yaml
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/extract_geometry.py
          ${CMAKE_CURRENT_SOURCE_DIR}/urdf/wheel_car_4w.urdf ${CMAKE_CURRENT_BINARY_DIR}/config/geometry.yaml
  DEPENDS scripts/extract_geometry.py urdf/wheel_car_4w.urdf)
add_custom_target(model_geometry ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/config/geometry.yaml)
# 安装 URDF 后通过 package:// 查找，不依赖源码目录或启动时的工作目录。
install(DIRECTORY urdf config DESTINATION share/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/config/geometry.yaml DESTINATION share/${PROJECT_NAME}/config)
ament_package()
