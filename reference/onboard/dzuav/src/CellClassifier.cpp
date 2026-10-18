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
    if (cudaMallocHost(reinterpret_cast<void**>(&h_input_), 3 * 96 * 96 * sizeof(float)) != cudaSuccess)
        throw std::runtime_error("cannot allocate pinned host buffer for cell_classifier");
    // 张量地址构造期绑定一次，每个 ROI 推理只 enqueue。
    if (!context_->setTensorAddress("images", input_) ||
        !context_->setTensorAddress("letter_logits", letters_) ||
        !context_->setTensorAddress("row_logits", rows_))
        throw std::runtime_error("cannot bind cell_classifier tensor addresses");
}
namespace
{
// 小角度 TTA：对格号文字用 ±10° 投票（90° 旋转会让文字侧过来，反而降精度）。
// 每个角度独立推理后对 softmax 概率取平均，再 argmax。
constexpr float kTtaAngles[] = {-10.F, -5.F, 0.F, 5.F, 10.F};
constexpr int kTtaCount = sizeof(kTtaAngles) / sizeof(kTtaAngles[0]);
}  // namespace

CellId CellClassifier::infer(const cv::Mat& crop)
{
    cv::Mat resized;
    cv::resize(crop, resized, {96, 96});

    float letter_prob[14] = {0}, row_prob[11] = {0};
    for (float angle : kTtaAngles)
    {
        cv::Mat rotated;
        if (angle == 0.F)
            rotated = resized;
        else
        {
            cv::Mat M = cv::getRotationMatrix2D({48, 48}, angle, 1.0);
            cv::warpAffine(resized, rotated, M, {96, 96}, cv::INTER_CUBIC,
                           cv::BORDER_CONSTANT, cv::Scalar(232, 232, 232));
        }
        cv::Mat rgb;
        cv::cvtColor(rotated, rgb, cv::COLOR_BGR2RGB);
        rgb.convertTo(rgb, CV_32F, 1 / 255.);
        float* v = h_input_;
        constexpr float mean[] = {0.485F, 0.456F, 0.406F};
        constexpr float standard_deviation[] = {0.229F, 0.224F, 0.225F};
        for (int y = 0; y < 96; y++)
            for (int x0 = 0; x0 < 96; x0++)
                for (int c = 0; c < 3; c++)
                    v[c * 9216 + y * 96 + x0] =
                        (rgb.at<cv::Vec3f>(y, x0)[c] - mean[c]) / standard_deviation[c];
        cudaMemcpyAsync(input_, h_input_, 3 * 96 * 96 * sizeof(float), cudaMemcpyHostToDevice, stream_);
        if (!context_->enqueueV3(stream_))
            throw std::runtime_error("cell_classifier inference failed");
        float a[14], r[11];
        cudaMemcpyAsync(a, letters_, sizeof(a), cudaMemcpyDeviceToHost, stream_);
        cudaMemcpyAsync(r, rows_, sizeof(r), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);
        // logits -> softmax，累加到概率和
        auto softmax_accum = [](float* logits, float* prob, int n)
        {
            float m = *std::max_element(logits, logits + n), s = 0;
            for (int i = 0; i < n; i++)
                s += std::exp(logits[i] - m);
            for (int i = 0; i < n; i++)
                prob[i] += std::exp(logits[i] - m) / s;
        };
        softmax_accum(a, letter_prob, 14);
        softmax_accum(r, row_prob, 11);
    }
    for (int i = 0; i < 14; i++) letter_prob[i] /= kTtaCount;
    for (int i = 0; i < 11; i++) row_prob[i] /= kTtaCount;

    int ai = std::max_element(letter_prob, letter_prob + 14) - letter_prob;
    int ri = std::max_element(row_prob, row_prob + 11) - row_prob;
    // 置信度 = min(letter_prob, row_prob)，与原逻辑一致
    return {char('A' + ai), ri + 1, std::min(letter_prob[ai], row_prob[ri])};
}
}  // namespace dzuav
