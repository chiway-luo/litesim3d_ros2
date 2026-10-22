# Included after each project() by the quality runner. Apply instrumentation to
# every project-owned compilation, including independently loaded plugin DSOs.
if(PROJECT_NAME STREQUAL "litesim3d_interfaces" OR PROJECT_NAME STREQUAL "livox_ros_driver2")
  return() # Generated bindings also load into the uninstrumented Python interpreter.
endif()
if(NOT CMAKE_CXX_COMPILER_LOADED AND NOT CMAKE_C_COMPILER_LOADED)
  return() # Resource-only packages have no compiler or executable code.
endif()
if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(FATAL_ERROR "Quality instrumentation requires Clang")
endif()
if(LITESIM_QUALITY_MODE STREQUAL "coverage")
  add_compile_options(-fprofile-instr-generate -fcoverage-mapping)
  add_link_options(-fprofile-instr-generate)
elseif(LITESIM_QUALITY_MODE STREQUAL "asan")
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=address,undefined -fno-sanitize-recover=all)
elseif(LITESIM_QUALITY_MODE STREQUAL "tsan")
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
  add_link_options(-fsanitize=thread)
else()
  message(FATAL_ERROR "Unknown LITESIM_QUALITY_MODE: ${LITESIM_QUALITY_MODE}")
endif()
