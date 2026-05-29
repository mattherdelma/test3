#pragma once
//
// A captured frame living in GPU memory. This is the unit that travels through
// the SPSC queue from the capture thread to the inference thread.
//
// To keep the queue cheap to move, the frame holds only a *pointer* to the
// pooled device surface plus metadata. The capture pool owns the actual memory.
//
#include <cstdint>
#include <chrono>

namespace rtp {

using Clock     = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

// Pixel layout of the captured surface. DXGI Desktop Duplication hands back
// DXGI_FORMAT_B8G8R8A8_UNORM, so BGRA8 is the default.
enum class PixelFormat : uint8_t { BGRA8 };

struct GpuFrame {
    // Device pointer to tightly-packed pixel data (row pitch == width*4).
    void*       dptr{nullptr};
    int         width{0};
    int         height{0};
    std::size_t pitch{0};          // bytes per row on device
    PixelFormat format{PixelFormat::BGRA8};
    uint64_t    frame_id{0};
    TimePoint   captured_at{};      // for end-to-end latency measurement

    // Index into the capture pool's slot array; lets the inference thread
    // release the surface back to the pool once it is done preprocessing.
    int pool_slot{-1};

    bool valid() const { return dptr != nullptr && width > 0 && height > 0; }
};

} // namespace rtp
