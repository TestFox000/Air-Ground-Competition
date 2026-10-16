#pragma once

#include "dzuav/TensorRtEngine.hpp"
#include "dzuav/types.hpp"

#include <opencv2/core.hpp>

#include <string>

namespace dzuav
{

// 格号分类器（96x96 MobileNetV3-Small 双头）：
// 一个头出 14 个列字母 A~N，另一个头出 11 个行号 1~11。
class CellClassifier : public TensorRtEngine
{
public:
    explicit CellClassifier(const std::string& engine_path);

    ~CellClassifier() override
    {
        for (int i = 0; i < kTtaSlots; ++i)
        {
            if (inputs_[i] != nullptr)
                cudaFree(inputs_[i]);
            if (letters_[i] != nullptr)
                cudaFree(letters_[i]);
            if (rows_[i] != nullptr)
                cudaFree(rows_[i]);
            if (h_inputs_[i] != nullptr)
                cudaFreeHost(h_inputs_[i]);
            if (h_letters_[i] != nullptr)
                cudaFreeHost(h_letters_[i]);
            if (h_rows_[i] != nullptr)
                cudaFreeHost(h_rows_[i]);
        }
    }

    CellId infer(const cv::Mat& crop);

    // TTA 角度槽位数：与 .cpp 的 kTtaAngles 一一对应（.cpp 有 static_assert 守住）。
    // 每个角度槽位一套独立缓冲，支撑单 stream 连续 enqueue（见 .cpp infer）。
    static constexpr int kTtaSlots = 5;

    // 独占持有 CUDA 缓冲，禁止拷贝（防 double-free）。
    CellClassifier(const CellClassifier&) = delete;
    CellClassifier& operator=(const CellClassifier&) = delete;

private:
    void* inputs_[kTtaSlots] = {};
    void* letters_[kTtaSlots] = {};
    void* rows_[kTtaSlots] = {};
    float* h_inputs_[kTtaSlots] = {};    // 页锁定主机输入（Orin I/O coherent，CPU 写入无额外代价）
    float* h_letters_[kTtaSlots] = {};   // 页锁定主机输出（D2H 真异步，免驱动中转）
    float* h_rows_[kTtaSlots] = {};
    // 逐角度图像处理缓冲，跨调用复用（免每 ROI 每角度重新分配）
    cv::Mat resized_;
    cv::Mat rotated_;
    cv::Mat rgb_;
};

}  // namespace dzuav
