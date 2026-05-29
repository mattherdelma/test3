#include "TensorRTEngine.h"

#include <cstdio>
#include <fstream>
#include <numeric>
#include <stdexcept>

namespace rtp {

void TrtLogger::log(Severity severity, const char* msg) noexcept {
    if (severity <= threshold_) {
        const char* tag = "";
        switch (severity) {
            case Severity::kINTERNAL_ERROR: tag = "TRT INTERNAL"; break;
            case Severity::kERROR:          tag = "TRT ERROR";    break;
            case Severity::kWARNING:        tag = "TRT WARN";     break;
            case Severity::kINFO:           tag = "TRT INFO";     break;
            default:                        tag = "TRT VERBOSE";  break;
        }
        std::fprintf(stderr, "[%s] %s\n", tag, msg);
    }
}

namespace {
std::size_t elementSize(nvinfer1::DataType t) {
    switch (t) {
        case nvinfer1::DataType::kFLOAT: return 4;
        case nvinfer1::DataType::kHALF:  return 2;
        case nvinfer1::DataType::kINT32: return 4;
        case nvinfer1::DataType::kINT8:  return 1;
        case nvinfer1::DataType::kBOOL:  return 1;
        default:                         return 4;
    }
}

std::size_t volume(const nvinfer1::Dims& d) {
    std::size_t v = 1;
    for (int i = 0; i < d.nbDims; ++i) v *= static_cast<std::size_t>(d.d[i]);
    return v;
}
} // namespace

void TensorRTEngine::loadFromFile(const std::string& engine_path) {
    std::ifstream f(engine_path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("cannot open engine file: " + engine_path);
    const std::streamsize size = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<char> blob(static_cast<std::size_t>(size));
    if (!f.read(blob.data(), size))
        throw std::runtime_error("failed to read engine: " + engine_path);

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) throw std::runtime_error("createInferRuntime failed");

    engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
    if (!engine_) throw std::runtime_error("deserializeCudaEngine failed");

    context_.reset(engine_->createExecutionContext());
    if (!context_) throw std::runtime_error("createExecutionContext failed");

    inspectTensors();
}

void TensorRTEngine::inspectTensors() {
    const int n = engine_->getNbIOTensors();
    tensors_.reserve(static_cast<std::size_t>(n));
    bool input_set = false, output_set = false;

    for (int i = 0; i < n; ++i) {
        const char* name = engine_->getIOTensorName(i);
        TensorInfo info;
        info.name     = name;
        info.dtype    = engine_->getTensorDataType(name);
        info.dims     = engine_->getTensorShape(name);
        info.is_input = engine_->getTensorIOMode(name) ==
                        nvinfer1::TensorIOMode::kINPUT;
        info.bytes    = volume(info.dims) * elementSize(info.dtype);

        // For fixed-shape engines the shape is already concrete here. If the
        // engine were dynamic we would call setInputShape() before querying.
        const std::size_t idx = tensors_.size();
        name_to_idx_[info.name] = idx;
        if (info.is_input  && !input_set)  { input_idx_  = idx; input_set  = true; }
        if (!info.is_input && !output_set) { output_idx_ = idx; output_set = true; }
        tensors_.push_back(std::move(info));
    }
    if (!input_set || !output_set)
        throw std::runtime_error("engine missing an input or output tensor");
}

const TensorInfo& TensorRTEngine::tensor(const std::string& name) const {
    auto it = name_to_idx_.find(name);
    if (it == name_to_idx_.end())
        throw std::runtime_error("unknown tensor: " + name);
    return tensors_[it->second];
}

void TensorRTEngine::setTensorAddress(const std::string& name, void* device_ptr) {
    if (!context_->setTensorAddress(name.c_str(), device_ptr))
        throw std::runtime_error("setTensorAddress failed for " + name);
}

void TensorRTEngine::enqueue(cudaStream_t stream) {
    if (!context_->enqueueV3(stream))
        throw std::runtime_error("enqueueV3 failed");
}

} // namespace rtp
