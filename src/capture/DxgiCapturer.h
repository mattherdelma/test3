#pragma once
//
// Low-latency desktop capture via the DXGI Desktop Duplication API (DDA).
//
// The duplication surface is acquired straight from the GPU. We register that
// D3D11 texture with CUDA (cudaGraphicsD3D11RegisterResource) and copy it,
// device-to-device, into a small ring of CUDA surfaces. No frame ever touches
// host memory — that is the "zero-copy" path the inference stage consumes.
//
// Threading model: call start() to spawn the capture thread. It pushes GpuFrame
// handles into the supplied SPSC queue. The inference thread pops them and, when
// finished, calls release(slot) to return the surface to the pool.
//
// Windows-only (D3D11 + DXGI). Guarded by _WIN32 so the rest of the tree builds
// on Linux for unit testing the inference / tracking stages.
//
#include "GpuFrame.h"
#include "../common/SPSCQueue.h"

#include <atomic>
#include <thread>
#include <array>
#include <cstdint>
#include <functional>

#if defined(_WIN32)
#include <d3d11.h>
#include <dxgi1_2.h>
#include <wrl/client.h>          // Microsoft::WRL::ComPtr (RAII for COM)
#include <cuda_runtime.h>
#include <cuda_d3d11_interop.h>
#endif

namespace rtp {

// Number of GPU surfaces in the capture pool. >= queue depth + frames in flight.
inline constexpr std::size_t kCapturePoolSize = 6;
using FrameQueue = SPSCQueue<GpuFrame, 8>;

struct CaptureStats {
    std::atomic<uint64_t> captured{0};
    std::atomic<uint64_t> dropped_full_queue{0};
    std::atomic<uint64_t> timeouts{0};
};

class DxgiCapturer {
public:
    DxgiCapturer() = default;
    ~DxgiCapturer();

    DxgiCapturer(const DxgiCapturer&) = delete;
    DxgiCapturer& operator=(const DxgiCapturer&) = delete;

    // Initialize D3D11 device, output duplication and the CUDA-interop pool.
    // `output_index` selects the monitor (0 = primary).
    void initialize(uint32_t adapter_index = 0, uint32_t output_index = 0);

    // Spawn the capture thread; frames are pushed into `out_queue`.
    void start(FrameQueue& out_queue);
    void stop();

    // Return a surface to the pool. Must be called by the consumer once the
    // frame's device data has been consumed (e.g. copied into the TRT input).
    void release(int pool_slot);

    int width()  const { return width_; }
    int height() const { return height_; }
    const CaptureStats& stats() const { return stats_; }

private:
    void captureLoop(FrameQueue& out_queue);
    bool acquireAndCopy(GpuFrame& out);     // one DDA frame -> a pool surface
    int  acquireFreeSlot();                 // wait-free pool allocation

    int width_{0};
    int height_{0};
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> frame_counter_{0};
    CaptureStats stats_;

#if defined(_WIN32)
    template <typename T> using ComPtr = Microsoft::WRL::ComPtr<T>;

    ComPtr<ID3D11Device>           device_;
    ComPtr<ID3D11DeviceContext>    context_;
    ComPtr<IDXGIOutputDuplication> duplication_;

    // One pool slot: a staging texture mapped to CUDA, plus its device pointer.
    struct PoolSlot {
        ComPtr<ID3D11Texture2D>      texture;        // GPU-side copy target
        cudaGraphicsResource_t       cuda_resource{nullptr};
        void*                        dptr{nullptr};  // mapped device pointer
        std::size_t                  pitch{0};
        std::atomic<bool>            in_use{false};
    };
    std::array<PoolSlot, kCapturePoolSize> pool_{};

    void createPool();
    void destroyPool();
#endif
};

} // namespace rtp
