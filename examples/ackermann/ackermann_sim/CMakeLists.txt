cmake_minimum_required(VERSION 3.20)
project(ackermann_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf config launch worlds DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(litesim3d_interfaces REQUIRED)
  get_filename_component(_interfaces_prefix "${litesim3d_interfaces_DIR}/../../.." ABSOLUTE)
  ament_add_pytest_test(composed_example test
    APPEND_LIBRARY_DIRS "${_interfaces_prefix}/lib"
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 TIMEOUT 240)
endif()
ament_package()
