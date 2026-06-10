cmake_minimum_required(VERSION 3.14)
project(ptz_servo_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler-specific options: MSVC (VS2019) uses /W4, others use -Wall -Wextra.
if(MSVC)
  add_compile_options(/W4 /permissive-)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
  add_compile_options(-Wall -Wextra)
endif()

add_library(ptz_control
  src/kalman_tracker.cpp
  src/adaptive_pid.cpp
  src/trajectory_planner.cpp
  src/target_selector.cpp
  src/servo_controller.cpp
)
target_include_directories(ptz_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(ptz_demo examples/demo.cpp)
target_link_libraries(ptz_demo PRIVATE ptz_control)

enable_testing()
add_executable(ptz_tests tests/test_control.cpp)
target_link_libraries(ptz_tests PRIVATE ptz_control)
add_test(NAME ptz_tests COMMAND ptz_tests)
