#pragma once
//
// Host-callable launchers for the CUDA pre/post-processing kernels.
// Declared here so the .cpp side does not need to be compiled by nvcc.
//
#include <cuda_runtime.h>
#include <cstdint>

namespace rtp {

// Letterbox parameters mapping the source image into the square network input.
struct LetterboxInfo {
    float scale;     // min(net_w/src_w, net_h/src_h)
    int   pad_x;     // left padding in network pixels
    int   pad_y;     // top padding in network pixels
};

// Compute letterbox geometry on the host (cheap, no device work).
LetterboxInfo computeLetterbox(int src_w, int src_h, int net_w, int net_h);

// BGRA8 (linear, `src_pitch` bytes/row) -> normalized RGB, NCHW float, padded.
// Output is `dst` of size 3*net_h*net_w floats. Pad value = 114/255.
void launchPreprocessBGRA(const void* src_bgra, int src_w, int src_h,
                          std::size_t src_pitch,
                          float* dst, int net_w, int net_h,
                          LetterboxInfo lb, cudaStream_t stream);

// One raw candidate decoded from the YOLOv8 head (network-input coordinates).
struct RawDet {
    float x1, y1, x2, y2;
    float score;
    int   cls;
};

// Decode the YOLOv8 output tensor [1, 4+num_classes, num_anchors] into RawDet[].
// Keeps candidates whose best class score >= conf_threshold using an atomic
// counter. `d_count` (device int) must be zeroed before the call.
void launchDecodeYolov8(const float* output, int num_classes, int num_anchors,
                        float conf_threshold,
                        RawDet* d_dets, int* d_count, int max_dets,
                        cudaStream_t stream);

} // namespace rtp
