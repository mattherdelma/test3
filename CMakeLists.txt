cmake_minimum_required(VERSION 3.16)
project(robot_visual_servo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Aggressive optimisation for the hot path; -march=native lets the compiler use
# the host AMD Zen ISA (AVX2/BMI etc.). Drop -march=native for portable builds.
add_compile_options(-O2 -Wall -Wextra)
if(NOT MSVC)
  add_compile_options(-march=native)
  # -Winterference-size is purely informational here (we fall back to 64 when
  # the constant is unavailable); silence it to keep the build output clean.
  add_compile_options(-Wno-interference-size)
endif()

# Header-only core library (the four highlighted deliverables live here).
add_library(rvs INTERFACE)
target_include_directories(rvs INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Integration demo executable.
add_executable(rvs_demo src/main.cpp)
target_link_libraries(rvs_demo PRIVATE rvs)

find_package(Threads REQUIRED)
target_link_libraries(rvs_demo PRIVATE Threads::Threads)

# Optional unit tests (no external framework; plain asserts).
enable_testing()
add_executable(rvs_tests tests/test_core.cpp)
target_link_libraries(rvs_tests PRIVATE rvs Threads::Threads)
# Keep assert() live even in Release — the tests rely on it.
target_compile_options(rvs_tests PRIVATE -UNDEBUG)
add_test(NAME rvs_tests COMMAND rvs_tests)
