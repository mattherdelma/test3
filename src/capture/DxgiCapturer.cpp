#include "DxgiCapturer.h"

#if defined(_WIN32)

#include "../common/CudaUtils.h"
#include <stdexcept>
#include <string>

using Microsoft::WRL::ComPtr;

namespace rtp {

namespace {
void throwIfFailed(HRESULT hr, const char* what) {
    if (FAILED(hr)) {
        throw std::runtime_error(std::string("DXGI/D3D11 failure: ") + what +
                                 " hr=0x" + std::to_string(static_cast<unsigned>(hr)));
    }
}
} // namespace

DxgiCapturer::~DxgiCapturer() {
    stop();
    destroyPool();
}

void DxgiCapturer::initialize(uint32_t adapter_index, uint32_t output_index) {
    // --- Enumerate the requested adapter + output --------------------------
    ComPtr<IDXGIFactory1> factory;
    throwIfFailed(CreateDXGIFactory1(__uuidof(IDXGIFactory1), &factory),
                  "CreateDXGIFactory1");

    ComPtr<IDXGIAdapter1> adapter;
    throwIfFailed(factory->EnumAdapters1(adapter_index, &adapter),
                  "EnumAdapters1");

    // --- Create the D3D11 device on that adapter ---------------------------
    const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_1,
                                        D3D_FEATURE_LEVEL_11_0};
    D3D_FEATURE_LEVEL got{};
    throwIfFailed(D3D11CreateDevice(
                      adapter.Get(), D3D_DRIVER_TYPE_UNKNOWN, nullptr,
                      D3D11_CREATE_DEVICE_BGRA_SUPPORT, levels, _countof(levels),
                      D3D11_SDK_VERSION, &device_, &got, &context_),
                  "D3D11CreateDevice");

    // --- Grab the output and start duplication -----------------------------
    ComPtr<IDXGIOutput> output;
    throwIfFailed(adapter->EnumOutputs(output_index, &output), "EnumOutputs");

    ComPtr<IDXGIOutput1> output1;
    throwIfFailed(output.As(&output1), "QueryInterface IDXGIOutput1");

    DXGI_OUTPUT_DESC desc{};
    output1->GetDesc(&desc);
    width_  = desc.DesktopCoordinates.right  - desc.DesktopCoordinates.left;
    height_ = desc.DesktopCoordinates.bottom - desc.DesktopCoordinates.top;

    throwIfFailed(output1->DuplicateOutput(device_.Get(), &duplication_),
                  "DuplicateOutput");

    // Tell CUDA which D3D11 device to interop with (must precede registration).
    cudaD3D11SetDirect3DDevice(device_.Get());

    createPool();
}

void DxgiCapturer::createPool() {
    D3D11_TEXTURE2D_DESC td{};
    td.Width            = static_cast<UINT>(width_);
    td.Height           = static_cast<UINT>(height_);
    td.MipLevels        = 1;
    td.ArraySize        = 1;
    td.Format           = DXGI_FORMAT_B8G8R8A8_UNORM;
    td.SampleDesc.Count = 1;
    td.Usage            = D3D11_USAGE_DEFAULT;
    // BIND_SHADER_RESOURCE is required for the texture to be CUDA-mappable.
    td.BindFlags        = D3D11_BIND_SHADER_RESOURCE;
    td.MiscFlags        = 0;

    const std::size_t linear_bytes =
        static_cast<std::size_t>(width_) * height_ * 4;

    for (auto& slot : pool_) {
        throwIfFailed(device_->CreateTexture2D(&td, nullptr, &slot.texture),
                      "CreateTexture2D (pool)");
        CUDA_CHECK(cudaGraphicsD3D11RegisterResource(
            &slot.cuda_resource, slot.texture.Get(),
            cudaGraphicsRegisterFlagsNone));
        // Each slot owns a tightly-packed linear BGRA buffer that the kernels
        // consume. The D3D texture is the interop staging surface.
        CUDA_CHECK(cudaMalloc(&slot.dptr, linear_bytes));
        slot.pitch  = static_cast<std::size_t>(width_) * 4;
        slot.in_use.store(false, std::memory_order_relaxed);
    }
}

void DxgiCapturer::destroyPool() {
    for (auto& slot : pool_) {
        if (slot.cuda_resource) {
            cudaGraphicsUnregisterResource(slot.cuda_resource);
            slot.cuda_resource = nullptr;
        }
        if (slot.dptr) {
            cudaFree(slot.dptr);
            slot.dptr = nullptr;
        }
        slot.texture.Reset();
    }
}

int DxgiCapturer::acquireFreeSlot() {
    for (std::size_t i = 0; i < pool_.size(); ++i) {
        bool expected = false;
        if (pool_[i].in_use.compare_exchange_strong(
                expected, true, std::memory_order_acq_rel)) {
            return static_cast<int>(i);
        }
    }
    return -1; // pool exhausted (consumer falling behind)
}

void DxgiCapturer::release(int pool_slot) {
    if (pool_slot >= 0 && pool_slot < static_cast<int>(pool_.size())) {
        pool_[pool_slot].in_use.store(false, std::memory_order_release);
    }
}

bool DxgiCapturer::acquireAndCopy(GpuFrame& out) {
    DXGI_OUTDUPL_FRAME_INFO info{};
    ComPtr<IDXGIResource> desktop_res;

    // 8 ms timeout: on a static desktop AcquireNextFrame blocks; we just retry.
    const HRESULT hr = duplication_->AcquireNextFrame(8, &info, &desktop_res);
    if (hr == DXGI_ERROR_WAIT_TIMEOUT) {
        stats_.timeouts.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    throwIfFailed(hr, "AcquireNextFrame");

    // RAII-release the DDA frame no matter how we exit this scope.
    struct FrameReleaser {
        IDXGIOutputDuplication* d;
        ~FrameReleaser() { d->ReleaseFrame(); }
    } releaser{duplication_.Get()};

    ComPtr<ID3D11Texture2D> acquired;
    throwIfFailed(desktop_res.As(&acquired), "QueryInterface desktop texture");

    const int slot = acquireFreeSlot();
    if (slot < 0) {
        return false; // drop: no free surface
    }
    PoolSlot& ps = pool_[slot];

    // GPU-side copy: desktop surface -> our CUDA-registered staging texture.
    context_->CopyResource(ps.texture.Get(), acquired.Get());

    // Map the staging texture into CUDA and pull the pixels into the linear
    // buffer. Both copies are device-to-device; nothing crosses the PCIe bus.
    CUDA_CHECK(cudaGraphicsMapResources(1, &ps.cuda_resource, 0));
    cudaArray_t array = nullptr;
    CUDA_CHECK(cudaGraphicsSubResourceGetMappedArray(&array, ps.cuda_resource, 0, 0));
    CUDA_CHECK(cudaMemcpy2DFromArray(
        ps.dptr, ps.pitch, array, 0, 0,
        static_cast<std::size_t>(width_) * 4, height_,
        cudaMemcpyDeviceToDevice));
    CUDA_CHECK(cudaGraphicsUnmapResources(1, &ps.cuda_resource, 0));

    out.dptr        = ps.dptr;
    out.width       = width_;
    out.height      = height_;
    out.pitch       = ps.pitch;
    out.format      = PixelFormat::BGRA8;
    out.frame_id    = frame_counter_.fetch_add(1, std::memory_order_relaxed);
    out.captured_at = Clock::now();
    out.pool_slot   = slot;
    return true;
}

void DxgiCapturer::start(FrameQueue& out_queue) {
    if (running_.exchange(true)) return;
    thread_ = std::thread([this, &out_queue] { captureLoop(out_queue); });
}

void DxgiCapturer::stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
}

void DxgiCapturer::captureLoop(FrameQueue& out_queue) {
    while (running_.load(std::memory_order_acquire)) {
        GpuFrame frame;
        try {
            if (!acquireAndCopy(frame)) continue;
        } catch (const std::exception&) {
            // DuplicateOutput can be lost (resolution change, mode switch);
            // a production build would re-initialize here. Skip for brevity.
            continue;
        }

        if (out_queue.push(frame)) {
            stats_.captured.fetch_add(1, std::memory_order_relaxed);
        } else {
            // Queue full: drop the freshest-but-one. Real-time beats complete.
            release(frame.pool_slot);
            stats_.dropped_full_queue.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

} // namespace rtp

#endif // _WIN32
