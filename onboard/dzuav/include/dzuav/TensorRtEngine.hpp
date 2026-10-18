#pragma once

#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dzuav/CudaCheck.hpp"

namespace dzuav
{

// 序列化 TensorRT 引擎（.engine）的公共加载器。
// 持有 runtime / engine / execution context 及一条 CUDA stream；
// 派生类在自己的构造函数里 setInputShape 并分配输入输出缓冲。
// 支持 TensorRT >= 8.5（enqueueV3/setTensorAddress/getTensorShape 需要 8.5+，
// 即 JetPack 5 自带的 TRT 版本）；8.x 与 10.x 的资源释放路径见析构函数注释。
class TensorRtEngine
{
protected:
    explicit TensorRtEngine(const std::string& engine_path)
    {
        std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
        if (!file)
            throw std::runtime_error("cannot open engine: " + engine_path);
        const std::streamsize file_size = file.tellg();
        if (file_size <= 0)
            throw std::runtime_error("cannot stat engine (empty or unreadable): " + engine_path);
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
        cudaCheck(cudaStreamCreate(&stream_));
    }

    // 独占持有 TRT/CUDA 资源，禁止拷贝（防 double-destroy/double-free）。
    TensorRtEngine(const TensorRtEngine&) = delete;
    TensorRtEngine& operator=(const TensorRtEngine&) = delete;

    virtual ~TensorRtEngine()
    {
        // TRT 8.x 这些类的析构是 protected，直接 delete 是硬编译错误，只能调
        // destroy()（8.x 已标废弃但仍可用）；TRT 10.x 起 destroy() 移除、析构公开，
        // 必须 delete。故按主版本号分流。
#if NV_TENSORRT_MAJOR >= 10
        delete context_;
        delete engine_;
        delete runtime_;
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        if (context_ != nullptr)
            context_->destroy();
        if (engine_ != nullptr)
            engine_->destroy();
        if (runtime_ != nullptr)
            runtime_->destroy();
#pragma GCC diagnostic pop
#endif
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
