#pragma once
//
// Thin RAII wrapper around a serialized TensorRT engine.
//
// Owns the runtime / engine / execution context via TensorRT's destroy-through-
// `delete` model (TRT >= 8 objects are deleted with `delete`, so unique_ptr with
// the default deleter is correct). Uses the name-based tensor I/O API
// (setTensorAddress / enqueueV3) available since TRT 8.5 and current in TRT 10.
//
#include "../common/CudaUtils.h"

#include <NvInfer.h>

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace rtp {

// Forward all TensorRT diagnostics to stderr above a severity threshold.
class TrtLogger : public nvinfer1::ILogger {
public:
    explicit TrtLogger(Severity threshold = Severity::kWARNING)
        : threshold_(threshold) {}
    void log(Severity severity, const char* msg) noexcept override;
private:
    Severity threshold_;
};

struct TensorInfo {
    std::string          name;
    nvinfer1::Dims       dims;
    nvinfer1::DataType   dtype;
    bool                 is_input;
    std::size_t          bytes;     // for the resolved (static) shape
};

class TensorRTEngine {
public:
    TensorRTEngine() = default;

    // Load a serialized .engine produced by trtexec / the builder.
    void loadFromFile(const std::string& engine_path);

    // Bind a device pointer to a named tensor for the next enqueue.
    void setTensorAddress(const std::string& name, void* device_ptr);

    // Run inference asynchronously on `stream`. All I/O addresses must be set.
    void enqueue(cudaStream_t stream);

    const std::vector<TensorInfo>& tensors() const { return tensors_; }
    const TensorInfo& tensor(const std::string& name) const;

    nvinfer1::Dims inputDims()  const { return tensors_[input_idx_].dims; }
    nvinfer1::Dims outputDims() const { return tensors_[output_idx_].dims; }
    const std::string& inputName()  const { return tensors_[input_idx_].name; }
    const std::string& outputName() const { return tensors_[output_idx_].name; }

private:
    void inspectTensors();

    TrtLogger logger_;

    // Custom deleters so unique_ptr<T> matches TRT's `delete`-based teardown.
    template <typename T>
    struct TrtDeleter { void operator()(T* p) const noexcept { delete p; } };
    template <typename T> using TrtPtr = std::unique_ptr<T, TrtDeleter<T>>;

    TrtPtr<nvinfer1::IRuntime>          runtime_;
    TrtPtr<nvinfer1::ICudaEngine>       engine_;
    TrtPtr<nvinfer1::IExecutionContext> context_;

    std::vector<TensorInfo> tensors_;
    std::unordered_map<std::string, std::size_t> name_to_idx_;
    std::size_t input_idx_{0};
    std::size_t output_idx_{0};
};

} // namespace rtp
