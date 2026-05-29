#pragma once
//
// Shared lightweight value types used across capture / inference / tracking.
// Kept dependency-free (no OpenCV, no Eigen) so it can be included anywhere.
//
#include <cstdint>
#include <algorithm>
#include <vector>

namespace rtp {

// Axis-aligned box in pixel coordinates (top-left origin).
struct Box {
    float x{0.f};   // left
    float y{0.f};   // top
    float w{0.f};   // width
    float h{0.f};   // height

    float left()   const { return x; }
    float top()    const { return y; }
    float right()  const { return x + w; }
    float bottom() const { return y + h; }
    float area()   const { return std::max(0.f, w) * std::max(0.f, h); }

    float cx() const { return x + w * 0.5f; }
    float cy() const { return y + h * 0.5f; }
};

// Intersection-over-Union of two boxes.
inline float iou(const Box& a, const Box& b) {
    const float ix = std::max(a.left(),   b.left());
    const float iy = std::max(a.top(),    b.top());
    const float ax = std::min(a.right(),  b.right());
    const float ay = std::min(a.bottom(), b.bottom());
    const float iw = std::max(0.f, ax - ix);
    const float ih = std::max(0.f, ay - iy);
    const float inter = iw * ih;
    const float uni   = a.area() + b.area() - inter;
    return uni > 0.f ? inter / uni : 0.f;
}

// A single detection produced by the detector.
struct Detection {
    Box   box;
    float score{0.f};
    int   class_id{-1};
};

using Detections = std::vector<Detection>;

} // namespace rtp
