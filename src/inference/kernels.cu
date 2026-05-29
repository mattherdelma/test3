#include "Kernels.h"

#include <algorithm>
#include <cmath>

namespace rtp {

LetterboxInfo computeLetterbox(int src_w, int src_h, int net_w, int net_h) {
    const float scale = std::min(static_cast<float>(net_w) / src_w,
                                 static_cast<float>(net_h) / src_h);
    const int new_w = static_cast<int>(std::round(src_w * scale));
    const int new_h = static_cast<int>(std::round(src_h * scale));
    LetterboxInfo lb;
    lb.scale = scale;
    lb.pad_x = (net_w - new_w) / 2;
    lb.pad_y = (net_h - new_h) / 2;
    return lb;
}

// ---------------------------------------------------------------------------
// Preprocess: BGRA8 -> letterboxed, normalized RGB in NCHW float layout.
// One thread per destination (network) pixel. Nearest-neighbor sampling keeps
// the kernel branch-light; swap for bilinear if accuracy demands it.
// ---------------------------------------------------------------------------
__global__ void preprocessKernel(const uchar4* __restrict__ src,
                                 int src_w, int src_h, int src_stride_px,
                                 float* __restrict__ dst,
                                 int net_w, int net_h,
                                 float scale, int pad_x, int pad_y) {
    const int dx = blockIdx.x * blockDim.x + threadIdx.x;
    const int dy = blockIdx.y * blockDim.y + threadIdx.y;
    if (dx >= net_w || dy >= net_h) return;

    const int plane = net_w * net_h;
    const int didx  = dy * net_w + dx;

    float r, g, b;
    // Map the destination pixel back into the source image.
    const int sx = static_cast<int>((dx - pad_x) / scale);
    const int sy = static_cast<int>((dy - pad_y) / scale);
    if (sx < 0 || sy < 0 || sx >= src_w || sy >= src_h) {
        r = g = b = 114.f / 255.f;            // letterbox fill
    } else {
        const uchar4 px = src[sy * src_stride_px + sx]; // BGRA
        r = px.z / 255.f;
        g = px.y / 255.f;
        b = px.x / 255.f;
    }
    // NCHW, RGB channel order.
    dst[0 * plane + didx] = r;
    dst[1 * plane + didx] = g;
    dst[2 * plane + didx] = b;
}

void launchPreprocessBGRA(const void* src_bgra, int src_w, int src_h,
                          std::size_t src_pitch,
                          float* dst, int net_w, int net_h,
                          LetterboxInfo lb, cudaStream_t stream) {
    const dim3 block(16, 16);
    const dim3 grid((net_w + block.x - 1) / block.x,
                    (net_h + block.y - 1) / block.y);
    const int src_stride_px = static_cast<int>(src_pitch / sizeof(uchar4));
    preprocessKernel<<<grid, block, 0, stream>>>(
        static_cast<const uchar4*>(src_bgra), src_w, src_h, src_stride_px,
        dst, net_w, net_h, lb.scale, lb.pad_x, lb.pad_y);
}

// ---------------------------------------------------------------------------
// Decode: YOLOv8 head [1, 4+C, A] (channel-major) -> RawDet candidates.
// One thread per anchor. Box is (cx,cy,w,h) in network-input pixels.
// ---------------------------------------------------------------------------
__global__ void decodeKernel(const float* __restrict__ out,
                             int num_classes, int num_anchors,
                             float conf, RawDet* __restrict__ dets,
                             int* __restrict__ count, int max_dets) {
    const int a = blockIdx.x * blockDim.x + threadIdx.x;
    if (a >= num_anchors) return;

    // Channel-major access: out[channel * num_anchors + anchor].
    const float cx = out[0 * num_anchors + a];
    const float cy = out[1 * num_anchors + a];
    const float w  = out[2 * num_anchors + a];
    const float h  = out[3 * num_anchors + a];

    float best = -1.f;
    int   best_c = -1;
    for (int c = 0; c < num_classes; ++c) {
        const float s = out[(4 + c) * num_anchors + a];
        if (s > best) { best = s; best_c = c; }
    }
    if (best < conf) return;

    const int slot = atomicAdd(count, 1);
    if (slot >= max_dets) return;

    RawDet d;
    d.x1 = cx - w * 0.5f;
    d.y1 = cy - h * 0.5f;
    d.x2 = cx + w * 0.5f;
    d.y2 = cy + h * 0.5f;
    d.score = best;
    d.cls   = best_c;
    dets[slot] = d;
}

void launchDecodeYolov8(const float* output, int num_classes, int num_anchors,
                        float conf_threshold,
                        RawDet* d_dets, int* d_count, int max_dets,
                        cudaStream_t stream) {
    const int block = 256;
    const int grid  = (num_anchors + block - 1) / block;
    decodeKernel<<<grid, block, 0, stream>>>(
        output, num_classes, num_anchors, conf_threshold,
        d_dets, d_count, max_dets);
}

} // namespace rtp
