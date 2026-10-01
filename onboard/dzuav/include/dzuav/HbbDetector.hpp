#pragma once

#include "dzuav/TensorRtEngine.hpp"
#include "dzuav/types.hpp"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace dzuav
{

// 基于 TensorRT 的水平框检测器（YOLOv8，输入 1280x1280，7 类）。
class HbbDetector : public TensorRtEngine
{
public:
    HbbDetector(const std::string& engine_path, float confidence, float nms);

    ~HbbDetector() override
    {
        if (input_ != nullptr)
            cudaFree(input_);
        if (output_ != nullptr)
            cudaFree(output_);
    }

    std::vector<Detection> infer(const cv::Mat& image);

private:
    float confidence_;
    float nms_;
    void* input_ = nullptr;
    void* output_ = nullptr;
};

}  // namespace dzuav
