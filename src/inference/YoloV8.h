#pragma once
//
// YOLOv8 detector: CUDA letterbox/normalize -> TensorRT inference -> GPU decode
// -> CPU NMS, all on a dedicated stream with pinned staging memory.
//
// Accepts a device pointer to a BGRA8 surface (the DXGI capture output), so the
// whole detect() path is zero host-copy until the tiny decoded-candidate buffer.
//
#include "TensorRTEngine.h"
#include "Kernels.h"
#include "../common/Types.h"
#include "../common/CudaUtils.h"

#include <string>
#include <vector>

namespace rtp {

struct YoloConfig {
    float conf_threshold = 0.25f;
    float nms_threshold  = 0.45f;
    int   max_dets       = 4096;   // pre-NMS candidate cap
};

class YoloV8 {
public:
    explicit YoloV8(const std::string& engine_path, YoloConfig cfg = {});

    // Detect on a captured BGRA8 device surface. Boxes are returned in the
    // coordinate system of the original (src_w x src_h) frame.
    Detections detect(const void* src_bgra, int src_w, int src_h,
                      std::size_t src_pitch);

    int netWidth()  const { return net_w_; }
    int netHeight() const { return net_h_; }
    cudaStream_t stream() const { return stream_.get(); }

private:
    std::vector<Detection> nms(std::vector<RawDet>& cands,
                               const LetterboxInfo& lb,
                               int src_w, int src_h) const;

    YoloConfig    cfg_;
    TensorRTEngine engine_;
    CudaStream     stream_;

    int net_w_{0};
    int net_h_{0};
    int num_classes_{0};
    int num_anchors_{0};

    // Persistent device buffers (allocated once, reused every frame).
    DeviceBuffer d_input_;     // NCHW float network input
    DeviceBuffer d_output_;    // raw head output
    DeviceBuffer d_dets_;      // RawDet candidate buffer
    DeviceBuffer d_count_;     // atomic candidate counter

    // Pinned host staging for the (small) decoded candidate set.
    PinnedBuffer  h_dets_;
    PinnedBuffer  h_count_;
};

} // namespace rtp
