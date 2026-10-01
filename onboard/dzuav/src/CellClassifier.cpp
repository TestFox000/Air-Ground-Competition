#include "dzuav/CellClassifier.hpp"

#include <algorithm>
#include <opencv2/imgproc.hpp>
namespace dzuav
{
CellClassifier::CellClassifier(const std::string& p) : TensorRtEngine(p)
{
    if (!context_->setInputShape("images", nvinfer1::Dims4{1, 3, 96, 96}))
        throw std::runtime_error("cannot set cell_classifier input shape");
    cudaMalloc(&input_, 3 * 96 * 96 * sizeof(float));
    cudaMalloc(&letters_, 14 * sizeof(float));
    cudaMalloc(&rows_, 11 * sizeof(float));
}
CellId CellClassifier::infer(const cv::Mat& crop)
{
    cv::Mat x;
    cv::resize(crop, x, {96, 96});
    cv::Mat b;
    cv::cvtColor(x, b, cv::COLOR_BGR2RGB);
    b.convertTo(b, CV_32F, 1 / 255.);
    std::vector<float> v(3 * 96 * 96);
    constexpr float mean[] = {0.485F, 0.456F, 0.406F};
    constexpr float standard_deviation[] = {0.229F, 0.224F, 0.225F};
    for (int y = 0; y < 96; y++)
        for (int x0 = 0; x0 < 96; x0++)
            for (int c = 0; c < 3; c++)
                v[c * 9216 + y * 96 + x0] =
                    (b.at<cv::Vec3f>(y, x0)[c] - mean[c]) / standard_deviation[c];
    cudaMemcpyAsync(input_, v.data(), v.size() * 4, cudaMemcpyHostToDevice, stream_);
    if (!context_->setTensorAddress("images", input_) ||
        !context_->setTensorAddress("letter_logits", letters_) ||
        !context_->setTensorAddress("row_logits", rows_) || !context_->enqueueV3(stream_))
        throw std::runtime_error("cell_classifier inference failed");
    float a[14], r[11];
    cudaMemcpyAsync(a, letters_, sizeof(a), cudaMemcpyDeviceToHost, stream_);
    cudaMemcpyAsync(r, rows_, sizeof(r), cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);
    int ai = std::max_element(a, a + 14) - a, ri = std::max_element(r, r + 11) - r;
    auto conf = [](float* x, int n, int k)
    {
        float m = x[k], s = 0;
        for (int i = 0; i < n; i++)
            s += std::exp(x[i] - m);
        return 1 / s;
    };
    return {char('A' + ai), ri + 1, std::min(conf(a, 14, ai), conf(r, 11, ri))};
}
}  // namespace dzuav
