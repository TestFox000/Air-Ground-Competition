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
        if (h_input_ != nullptr)
            cudaFreeHost(h_input_);
        if (h_output_ != nullptr)
            cudaFreeHost(h_output_);
    }

    std::vector<Detection> infer(const cv::Mat& image);

private:
    float confidence_;
    float nms_;
    void* input_ = nullptr;    // 设备侧输入
    void* output_ = nullptr;   // 设备侧输出
    float* h_input_ = nullptr;   // 页锁定主机输入（H2D 免驱动中转）
    float* h_output_ = nullptr;  // 页锁定主机输出（D2H 免驱动中转）
    cv::Mat blob_;               // 包住 h_input_ 的 CHW float 缓冲，跨帧复用
};

}  // namespace dzuav
