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
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Raw profiles are attributed to their exact ELF object before merging.
    # Otherwise same-name/hash mains from separate executables borrow hits.
    add_link_options("LINKER:--build-id=sha1")
    # LLVM14's %m is only a layout signature: different plugin DSOs can share
    # it and corrupt each other's raw profile within the same process. %b is
    # not supported until newer runtimes. Give each ELF target its own prefix
    # through the documented, module-local filename API (no extra counters).
    set(LITESIM_COVERAGE_HELPER "${CMAKE_CURRENT_LIST_DIR}/profile_filename.cpp")
    set_source_files_properties("${LITESIM_COVERAGE_HELPER}" PROPERTIES
      COMPILE_OPTIONS "-fno-profile-instr-generate;-fno-coverage-mapping"
      COMPILE_DEFINITIONS "LITESIM_COVERAGE_MODULE=\"${PROJECT_NAME}--$<TARGET_PROPERTY:NAME>--\"")
    function(litesim_quality_profile_targets)
      get_property(targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
      foreach(target IN LISTS targets)
        get_target_property(kind "${target}" TYPE)
        if(kind STREQUAL "EXECUTABLE" OR kind STREQUAL "SHARED_LIBRARY" OR kind STREQUAL "MODULE_LIBRARY")
          target_sources("${target}" PRIVATE "${LITESIM_COVERAGE_HELPER}")
          file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/litesim-coverage-${target}.json"
            CONTENT "{\"binary\":\"$<TARGET_FILE:${target}>\",\"filename_prefix\":\"${PROJECT_NAME}--${target}--\"}\n")
        endif()
      endforeach()
    endfunction()
    cmake_language(DEFER CALL litesim_quality_profile_targets)
  endif()
elseif(LITESIM_QUALITY_MODE STREQUAL "asan")
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=address,undefined -fno-sanitize-recover=all)
  if(PROJECT_NAME STREQUAL "litesim3d_gui")
    # Mesa's JIT can recursively lock GCC's unwinder inside malloc. Rendering
    # lives in separate GUI executables, so only these need LLVM's unwinder;
    # keep its substantial allocation overhead out of headless simulations.
    set(litesim_llvm_libdir "$ENV{LITESIM_LLVM_UNWIND_DIR}")
    if(NOT litesim_llvm_libdir)
      find_program(LITESIM_LLVM_CONFIG NAMES llvm-config-14 REQUIRED)
      execute_process(COMMAND "${LITESIM_LLVM_CONFIG}" --libdir
        OUTPUT_VARIABLE litesim_llvm_libdir OUTPUT_STRIP_TRAILING_WHITESPACE
        COMMAND_ERROR_IS_FATAL ANY)
    endif()
    if(NOT EXISTS "${litesim_llvm_libdir}/libunwind.so")
      message(FATAL_ERROR "ASan rendering requires libunwind-14-dev")
    endif()
    add_link_options(-rtlib=compiler-rt -unwindlib=libunwind "-L${litesim_llvm_libdir}")
  endif()
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    # Clang 14's ASan allocator can overlap a randomized PIE on newer kernels,
    # crashing before main. Keep this workaround confined to ASan executables;
    # plugin DSOs remain shared/PIC and normal/release builds are unchanged.
    add_link_options("$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:-no-pie>")
  endif()
elseif(LITESIM_QUALITY_MODE STREQUAL "tsan")
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
  add_link_options(-fsanitize=thread)
else()
  message(FATAL_ERROR "Unknown LITESIM_QUALITY_MODE: ${LITESIM_QUALITY_MODE}")
endif()
