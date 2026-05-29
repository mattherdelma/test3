#pragma once
//
// CUDA RAII helpers and error-checking macros.
//
#include <cuda_runtime.h>
#include <stdexcept>
#include <string>
#include <memory>

namespace rtp {

#define CUDA_CHECK(call)                                                        \
    do {                                                                       \
        const cudaError_t _err = (call);                                       \
        if (_err != cudaSuccess) {                                             \
            throw ::rtp::CudaError(#call, _err, __FILE__, __LINE__);           \
        }                                                                      \
    } while (0)

class CudaError : public std::runtime_error {
public:
    CudaError(const char* expr, cudaError_t err, const char* file, int line)
        : std::runtime_error(build(expr, err, file, line)) {}

private:
    static std::string build(const char* expr, cudaError_t err,
                             const char* file, int line) {
        return std::string("CUDA error: ") + cudaGetErrorString(err) +
               " (" + cudaGetErrorName(err) + ") at " + file + ":" +
               std::to_string(line) + " -> " + expr;
    }
};

// -------- Owning RAII wrappers for CUDA resources --------

// Device memory block.
struct CudaFreeDeleter {
    void operator()(void* p) const noexcept { if (p) cudaFree(p); }
};
using DeviceBuffer = std::unique_ptr<void, CudaFreeDeleter>;

inline DeviceBuffer makeDeviceBuffer(std::size_t bytes) {
    void* ptr = nullptr;
    CUDA_CHECK(cudaMalloc(&ptr, bytes));
    return DeviceBuffer{ptr};
}

// Pinned (page-locked) host memory — enables async H2D/D2H overlap.
struct CudaHostFreeDeleter {
    void operator()(void* p) const noexcept { if (p) cudaFreeHost(p); }
};
using PinnedBuffer = std::unique_ptr<void, CudaHostFreeDeleter>;

inline PinnedBuffer makePinnedBuffer(std::size_t bytes) {
    void* ptr = nullptr;
    CUDA_CHECK(cudaHostAlloc(&ptr, bytes, cudaHostAllocDefault));
    return PinnedBuffer{ptr};
}

// CUDA stream.
class CudaStream {
public:
    CudaStream() { CUDA_CHECK(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking)); }
    ~CudaStream() { if (stream_) cudaStreamDestroy(stream_); }

    CudaStream(const CudaStream&) = delete;
    CudaStream& operator=(const CudaStream&) = delete;
    CudaStream(CudaStream&& o) noexcept : stream_(o.stream_) { o.stream_ = nullptr; }

    cudaStream_t get() const { return stream_; }
    void synchronize() const { CUDA_CHECK(cudaStreamSynchronize(stream_)); }

private:
    cudaStream_t stream_{nullptr};
};

} // namespace rtp
