cmake_minimum_required(VERSION 3.20)
project(litesim_installed_consumer LANGUAGES CXX)
# The exported library target must raise this to C++17 for its public headers.
set(CMAKE_CXX_STANDARD 14)
find_package(litesim3d_core REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE litesim3d_core::litesim3d_core)
