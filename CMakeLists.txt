cmake_minimum_required(VERSION 3.21)
project(rtpercept LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# MSVC (Visual Studio 2019, toolset v142) compatibility.
#   /MP            parallel compilation
#   /permissive-   stricter, conformant C++
#   /EHsc          standard C++ exception model
#   NOMINMAX       stop <windows.h> from defining min/max macros
#   WIN32_LEAN_AND_MEAN / _CRT_SECURE_NO_WARNINGS  reduce header bloat / noise
#   UNICODE        DXGI/D3D11 paths expect the wide-char Win32 API
# ---------------------------------------------------------------------------
if(MSVC)
  add_compile_options(/MP /permissive- /EHsc /W3 /std:c++17 /Zc:__cplusplus)
  add_compile_definitions(
      NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS UNICODE _UNICODE)
endif()

find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# Tracking library: pure C++ + Eigen, no CUDA/TensorRT. Always buildable, and
# the unit tests link only against this so they run on any platform / VS2019.
# ---------------------------------------------------------------------------
add_library(rtpercept_tracking STATIC
    src/tracking/KalmanFilter.cpp
    src/tracking/STrack.cpp
    src/tracking/LinearAssignment.cpp
    src/tracking/BYTETracker.cpp
)
target_include_directories(rtpercept_tracking PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(rtpercept_tracking PUBLIC Eigen3::Eigen)

enable_testing()

# Portable tests (run anywhere, including a plain VS2019 build with no CUDA).
add_executable(test_tracking tests/test_tracking.cpp)
target_link_libraries(test_tracking PRIVATE rtpercept_tracking)
add_test(NAME tracking COMMAND test_tracking)

add_executable(test_spsc_queue tests/test_spsc_queue.cpp)
target_include_directories(test_spsc_queue PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
find_package(Threads REQUIRED)
target_link_libraries(test_spsc_queue PRIVATE Threads::Threads)
add_test(NAME spsc_queue COMMAND test_spsc_queue)

# ---------------------------------------------------------------------------
# Full perception pipeline (CUDA + TensorRT, plus DXGI capture on Windows).
# Enabled only when the GPU toolchain is present. Toggle with -DBUILD_PIPELINE.
# ---------------------------------------------------------------------------
option(BUILD_PIPELINE "Build the CUDA/TensorRT/DXGI perception pipeline" OFF)

if(BUILD_PIPELINE)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 17)

  find_package(CUDAToolkit REQUIRED)

  find_path(TENSORRT_INCLUDE_DIR NvInfer.h
            HINTS ${TENSORRT_ROOT} ENV TENSORRT_ROOT PATH_SUFFIXES include)
  find_library(TENSORRT_LIB nvinfer
               HINTS ${TENSORRT_ROOT} ENV TENSORRT_ROOT PATH_SUFFIXES lib lib64)
  if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIB)
    message(FATAL_ERROR "TensorRT not found. Set -DTENSORRT_ROOT=/path/to/TensorRT")
  endif()

  set(PIPE_SOURCES
      src/inference/TensorRTEngine.cpp
      src/inference/YoloV8.cpp
      src/inference/kernels.cu)
  if(WIN32)
    list(APPEND PIPE_SOURCES src/capture/DxgiCapturer.cpp)
  endif()

  add_library(rtpercept_core STATIC ${PIPE_SOURCES})
  target_include_directories(rtpercept_core PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/src ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(rtpercept_core PUBLIC
      rtpercept_tracking CUDA::cudart ${TENSORRT_LIB})
  set_target_properties(rtpercept_core PROPERTIES
      CUDA_SEPARABLE_COMPILATION ON POSITION_INDEPENDENT_CODE ON
      CUDA_ARCHITECTURES 89)        # RTX 4060 Ti = Ada (sm_89)
  if(WIN32)
    target_link_libraries(rtpercept_core PUBLIC d3d11 dxgi CUDA::cuda_driver)
  endif()

  add_executable(rtpercept src/app/main.cpp)
  target_link_libraries(rtpercept PRIVATE rtpercept_core)

  # DXGI capture smoke test (Windows only).
  if(WIN32)
    add_executable(test_capture tests/test_capture.cpp)
    target_link_libraries(test_capture PRIVATE rtpercept_core)
    add_test(NAME capture COMMAND test_capture)
  endif()
endif()
