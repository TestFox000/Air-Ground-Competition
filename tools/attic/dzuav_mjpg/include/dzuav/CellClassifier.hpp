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
        if (input_ != nullptr)
            cudaFree(input_);
        if (letters_ != nullptr)
            cudaFree(letters_);
        if (rows_ != nullptr)
            cudaFree(rows_);
        if (h_input_ != nullptr)
            cudaFreeHost(h_input_);
    }

    CellId infer(const cv::Mat& crop);

    // 独占持有 CUDA 缓冲，禁止拷贝（防 double-free）。
    CellClassifier(const CellClassifier&) = delete;
    CellClassifier& operator=(const CellClassifier&) = delete;

private:
    void* input_ = nullptr;
    void* letters_ = nullptr;
    void* rows_ = nullptr;
    float* h_input_ = nullptr;   // 页锁定主机输入（Orin I/O coherent，CPU 写入无额外代价）
};

}  // namespace dzuav
