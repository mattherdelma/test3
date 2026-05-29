#include "YoloV8.h"

#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace rtp {

YoloV8::YoloV8(const std::string& engine_path, YoloConfig cfg) : cfg_(cfg) {
    engine_.loadFromFile(engine_path);

    // Input is NCHW: [1, 3, net_h, net_w].
    const auto in = engine_.inputDims();
    if (in.nbDims != 4 || in.d[1] != 3)
        throw std::runtime_error("unexpected YOLOv8 input shape (need NCHW x3)");
    net_h_ = static_cast<int>(in.d[2]);
    net_w_ = static_cast<int>(in.d[3]);

    // Output is [1, 4+num_classes, num_anchors].
    const auto out = engine_.outputDims();
    if (out.nbDims != 3)
        throw std::runtime_error("unexpected YOLOv8 output rank (need 3)");
    num_classes_ = static_cast<int>(out.d[1]) - 4;
    num_anchors_ = static_cast<int>(out.d[2]);
    if (num_classes_ <= 0)
        throw std::runtime_error("decoded num_classes <= 0");

    // Allocate persistent buffers.
    d_input_  = makeDeviceBuffer(sizeof(float) * 3 * net_h_ * net_w_);
    d_output_ = makeDeviceBuffer(engine_.tensor(engine_.outputName()).bytes);
    d_dets_   = makeDeviceBuffer(sizeof(RawDet) * cfg_.max_dets);
    d_count_  = makeDeviceBuffer(sizeof(int));
    h_dets_   = makePinnedBuffer(sizeof(RawDet) * cfg_.max_dets);
    h_count_  = makePinnedBuffer(sizeof(int));

    // Wire the persistent device addresses into the engine once.
    engine_.setTensorAddress(engine_.inputName(),  d_input_.get());
    engine_.setTensorAddress(engine_.outputName(), d_output_.get());
}

Detections YoloV8::detect(const void* src_bgra, int src_w, int src_h,
                          std::size_t src_pitch) {
    const cudaStream_t s = stream_.get();
    const LetterboxInfo lb = computeLetterbox(src_w, src_h, net_w_, net_h_);

    // 1) Preprocess straight from the captured device surface.
    launchPreprocessBGRA(src_bgra, src_w, src_h, src_pitch,
                         static_cast<float*>(d_input_.get()),
                         net_w_, net_h_, lb, s);

    // 2) Inference on the same stream (input/output addresses already bound).
    engine_.enqueue(s);

    // 3) Decode on the GPU; reset the atomic counter first.
    CUDA_CHECK(cudaMemsetAsync(d_count_.get(), 0, sizeof(int), s));
    launchDecodeYolov8(static_cast<const float*>(d_output_.get()),
                       num_classes_, num_anchors_, cfg_.conf_threshold,
                       static_cast<RawDet*>(d_dets_.get()),
                       static_cast<int*>(d_count_.get()), cfg_.max_dets, s);

    // 4) Pull back only the candidate count + candidates (small).
    CUDA_CHECK(cudaMemcpyAsync(h_count_.get(), d_count_.get(), sizeof(int),
                               cudaMemcpyDeviceToHost, s));
    CUDA_CHECK(cudaStreamSynchronize(s));

    int n = *static_cast<int*>(h_count_.get());
    n = std::min(n, cfg_.max_dets);
    if (n <= 0) return {};

    CUDA_CHECK(cudaMemcpyAsync(h_dets_.get(), d_dets_.get(),
                               sizeof(RawDet) * n,
                               cudaMemcpyDeviceToHost, s));
    CUDA_CHECK(cudaStreamSynchronize(s));

    std::vector<RawDet> cands(static_cast<RawDet*>(h_dets_.get()),
                              static_cast<RawDet*>(h_dets_.get()) + n);
    return nms(cands, lb, src_w, src_h);
}

// Greedy per-class NMS, then map boxes back from network space to source pixels.
std::vector<Detection> YoloV8::nms(std::vector<RawDet>& cands,
                                   const LetterboxInfo& lb,
                                   int src_w, int src_h) const {
    std::vector<int> order(cands.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return cands[a].score > cands[b].score; });

    std::vector<char> removed(cands.size(), 0);
    Detections result;
    result.reserve(cands.size());

    auto toBox = [&](const RawDet& d) {
        Box b;
        b.x = d.x1; b.y = d.y1; b.w = d.x2 - d.x1; b.h = d.y2 - d.y1;
        return b;
    };

    for (std::size_t i = 0; i < order.size(); ++i) {
        const int ai = order[i];
        if (removed[ai]) continue;
        const RawDet& a = cands[ai];
        const Box abox = toBox(a);

        for (std::size_t j = i + 1; j < order.size(); ++j) {
            const int bi = order[j];
            if (removed[bi]) continue;
            const RawDet& b = cands[bi];
            if (b.cls != a.cls) continue;            // class-aware NMS
            if (iou(abox, toBox(b)) > cfg_.nms_threshold) removed[bi] = 1;
        }

        // Undo letterbox: net -> source coordinates, then clamp.
        Detection det;
        const float inv = 1.f / lb.scale;
        float x1 = (a.x1 - lb.pad_x) * inv;
        float y1 = (a.y1 - lb.pad_y) * inv;
        float x2 = (a.x2 - lb.pad_x) * inv;
        float y2 = (a.y2 - lb.pad_y) * inv;
        x1 = std::clamp(x1, 0.f, static_cast<float>(src_w - 1));
        y1 = std::clamp(y1, 0.f, static_cast<float>(src_h - 1));
        x2 = std::clamp(x2, 0.f, static_cast<float>(src_w - 1));
        y2 = std::clamp(y2, 0.f, static_cast<float>(src_h - 1));
        det.box      = {x1, y1, x2 - x1, y2 - y1};
        det.score    = a.score;
        det.class_id = a.cls;
        result.push_back(det);
    }
    return result;
}

} // namespace rtp
