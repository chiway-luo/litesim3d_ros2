cmake_minimum_required(VERSION 3.20)
project(wheel_car_4w_sim)
find_package(ament_cmake REQUIRED)
# 纯 bringup 包：复用已有仿真内核，模型由独立 description 包提供。
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(litesim3d_interfaces REQUIRED)
  # macOS 的 CTest 启动链可能清除 DYLD_LIBRARY_PATH；在 pytest 子进程中补回
  # 本项目接口库目录。使用接口包前缀，兼容 merged / isolated 两种安装布局。
  get_filename_component(_interfaces_prefix
    "${litesim3d_interfaces_DIR}/../../.." ABSOLUTE)
  ament_add_pytest_test(wheel_car_4w_smoke test
    APPEND_LIBRARY_DIRS "${_interfaces_prefix}/lib"
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 TIMEOUT 200)
  ament_add_pytest_test(sdf_assets ../../../litesim3d_assets/test
    APPEND_LIBRARY_DIRS "${_interfaces_prefix}/lib"
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 TIMEOUT 200)
endif()
ament_package()
