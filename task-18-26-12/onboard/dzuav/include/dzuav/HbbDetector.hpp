#pragma once

#include "dzuav/TensorRtEngine.hpp"
#include "dzuav/types.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <string>
#include <vector>

namespace dzuav
{

// 基于 TensorRT 的水平框检测器（YOLOv8，输入 1280x1280，7 类）。
//
// 后处理策略（2026-10-15 模型侧优化）：
//   1. 按类别置信度阈值：7 类可独立设置 conf（默认全部 0.25，保持旧行为）。
//   2. 按类别 NMS IoU 阈值：7 类可独立设置（默认全部 0.45）。
//   3. mine 类边缘过滤：中心点距图像边缘 < edge_mine_thr（归一化，默认 1%）
//      的 mine 检测被剔除。实测 val-B 上 mine FP 118→79，综合精度 +0.76pp
//      （90.31% → 91.07%），净收益为正。PC 端 prelabel_onnx.py 已验证，
//      此处把同一策略部署到无人机推理端。
class HbbDetector : public TensorRtEngine
{
public:
    // 旧接口：单一置信度 + 单一 NMS，edge_mine 默认 1%（设 0 关闭）。
    HbbDetector(const std::string& engine_path, float confidence, float nms,
                float edge_mine_thr = 0.01F);

    // 新接口：按类别阈值。conf_per_class / nms_per_class 长度必须为 7。
    HbbDetector(const std::string& engine_path,
                const std::array<float, 7>& conf_per_class,
                const std::array<float, 7>& nms_per_class,
                float edge_mine_thr = 0.01F);

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
    std::array<float, 7> conf_per_class_{};
    std::array<float, 7> nms_per_class_{};
    float edge_mine_thr_;   // mine 边缘过滤阈值（归一化）；0 表示关闭
    void* input_ = nullptr;    // 设备侧输入
    void* output_ = nullptr;   // 设备侧输出
    float* h_input_ = nullptr;   // 页锁定主机输入（H2D 免驱动中转）
    float* h_output_ = nullptr;  // 页锁定主机输出（D2H 免驱动中转）
    cv::Mat blob_;               // 包住 h_input_ 的 CHW float 缓冲，跨帧复用
};

}  // namespace dzuav
