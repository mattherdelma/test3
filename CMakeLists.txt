cmake_minimum_required(VERSION 3.21)
project(rtpercept LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# Tracking library: pure C++ + Eigen, no CUDA/TensorRT. Always buildable, and
# the unit tests link only against this so they run on any platform.
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
add_executable(test_tracking tests/test_tracking.cpp)
target_link_libraries(test_tracking PRIVATE rtpercept_tracking)
add_test(NAME tracking COMMAND test_tracking)

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
endif()
