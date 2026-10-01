#pragma once

#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dzuav
{

// 序列化 TensorRT 引擎（.engine）的公共加载器。
// 持有 runtime / engine / execution context 及一条 CUDA stream；
// 派生类在自己的构造函数里 setInputShape 并分配输入输出缓冲。
class TensorRtEngine
{
protected:
    explicit TensorRtEngine(const std::string& engine_path)
    {
        std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
        if (!file)
            throw std::runtime_error("cannot open engine: " + engine_path);
        const std::streamsize file_size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<char> data(static_cast<size_t>(file_size));
        if (!file.read(data.data(), file_size))
            throw std::runtime_error("cannot read engine: " + engine_path);

        runtime_ = nvinfer1::createInferRuntime(logger_);
        if (runtime_ == nullptr)
            throw std::runtime_error("cannot create TensorRT runtime");
        engine_ = runtime_->deserializeCudaEngine(data.data(), data.size());
        if (engine_ == nullptr)
            throw std::runtime_error("cannot deserialize engine: " + engine_path);
        context_ = engine_->createExecutionContext();
        if (context_ == nullptr)
            throw std::runtime_error("cannot create execution context");
        if (cudaStreamCreate(&stream_) != cudaSuccess)
            throw std::runtime_error("cannot create cuda stream");
    }

    virtual ~TensorRtEngine()
    {
        if (context_ != nullptr)
            context_->destroy();
        if (engine_ != nullptr)
            engine_->destroy();
        if (runtime_ != nullptr)
            runtime_->destroy();
        if (stream_ != nullptr)
            cudaStreamDestroy(stream_);
    }

    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    cudaStream_t stream_ = nullptr;

private:
    class Logger : public nvinfer1::ILogger
    {
        void log(Severity severity, const char* msg) noexcept override
        {
            if (severity <= Severity::kWARNING)
                std::cerr << "[TensorRT] " << msg << '\n';
        }
    };

    Logger logger_;
};

}  // namespace dzuav
