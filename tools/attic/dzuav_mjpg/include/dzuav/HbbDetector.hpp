#pragma once

#include "dzuav/TensorRtEngine.hpp"
#include "dzuav/types.hpp"

#include <array>
#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace dzuav
{

// 基于 TensorRT 的水平框检测器（YOLO11s，输入 1280x1280，13 类 = 17 通道输出）。
// 2026-10-16: D 方案 13 类统一模型（hbb_d13_best.pt），per-class 阈值 + paper 边缘过滤。
class HbbDetector : public TensorRtEngine
{
public:
    // 简化构造：全类统一 conf/nms，无边缘过滤。
    HbbDetector(const std::string& engine_path, float confidence, float nms);

    // 部署构造：按类别 conf/NMS + paper 边缘过滤（框中心距边 < thr 归一化则剔除，
    // letterbox 填充伪影对策；仅对 Paper 类生效）。
    HbbDetector(const std::string& engine_path,
                const std::array<float, kObjectClassCount>& conf_per_class,
                const std::array<float, kObjectClassCount>& nms_per_class,
                float edge_paper_thr = 0.F);

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

    // 独占持有 CUDA 缓冲，禁止拷贝（防 double-free）。
    HbbDetector(const HbbDetector&) = delete;
    HbbDetector& operator=(const HbbDetector&) = delete;

private:
    void allocate(const std::string& engine_path);

    std::array<float, kObjectClassCount> conf_per_class_{};
    std::array<float, kObjectClassCount> nms_per_class_{};
    float edge_paper_thr_ = 0.F;
    void* input_ = nullptr;    // 设备侧输入
    void* output_ = nullptr;   // 设备侧输出
    float* h_input_ = nullptr;   // 页锁定主机输入（H2D 免驱动中转）
    float* h_output_ = nullptr;  // 页锁定主机输出（D2H 免驱动中转）
    cv::Mat blob_;               // 包住 h_input_ 的 CHW float 缓冲，跨帧复用
};

}  // namespace dzuav
