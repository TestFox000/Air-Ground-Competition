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
    }

    CellId infer(const cv::Mat& crop);

private:
    void* input_ = nullptr;
    void* letters_ = nullptr;
    void* rows_ = nullptr;
};

}  // namespace dzuav
