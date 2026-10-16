#include "dzuav/CellClassifier.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
namespace dzuav
{
namespace
{
// 引擎输出形状契约校验用：Dims 打印成 "[1,14]" 便于定位换错引擎的问题。
// 模板形参兼容 TRT 8.x 的 Dims（int32）与 10.x 的 Dims64。
template <class DimsT>
std::string dimsToString(const DimsT& d)
{
    std::string s = "[";
    for (int i = 0; i < d.nbDims; ++i)
    {
        if (i > 0)
            s += ',';
        s += std::to_string(d.d[i]);
    }
    s += ']';
    return s;
}

void checkOutputShape(nvinfer1::IExecutionContext* context, const char* name, int d0, int d1)
{
    const auto dims = context->getTensorShape(name);
    if (dims.nbDims != 2 || dims.d[0] != d0 || dims.d[1] != d1)
        throw std::runtime_error(std::string("cell_classifier ") + name +
                                 " shape mismatch: expect [" + std::to_string(d0) + "," +
                                 std::to_string(d1) + "], got " + dimsToString(dims));
}
}  // namespace
CellClassifier::CellClassifier(const std::string& p) : TensorRtEngine(p)
{
    if (!context_->setInputShape("images", nvinfer1::Dims4{1, 3, 96, 96}))
        throw std::runtime_error("cannot set cell_classifier input shape");
    // 输出形状契约校验：不符即抛错并打印实际形状，防换引擎后静默读垃圾。
    checkOutputShape(context_, "letter_logits", 1, 14);
    checkOutputShape(context_, "row_logits", 1, 11);
    // 每个 TTA 角度槽位一套独立缓冲：单 stream 连续 enqueue 时互不覆盖（见 infer）。
    for (int i = 0; i < kTtaSlots; ++i)
    {
        cudaCheck(cudaMalloc(&inputs_[i], 3 * 96 * 96 * sizeof(float)));
        cudaCheck(cudaMalloc(&letters_[i], 14 * sizeof(float)));
        cudaCheck(cudaMalloc(&rows_[i], 11 * sizeof(float)));
        cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&h_inputs_[i]),
                                 3 * 96 * 96 * sizeof(float)));
        cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&h_letters_[i]), 14 * sizeof(float)));
        cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&h_rows_[i]), 11 * sizeof(float)));
    }
    // 张量地址先绑一次做契约校验；infer 里每个角度 enqueue 前会按槽位重绑
    // （setTensorAddress 是廉价 setter，enqueueV3 以调用时刻的绑定为准）。
    if (!context_->setTensorAddress("images", inputs_[0]) ||
        !context_->setTensorAddress("letter_logits", letters_[0]) ||
        !context_->setTensorAddress("row_logits", rows_[0]))
        throw std::runtime_error("cannot bind cell_classifier tensor addresses");
}
namespace
{
// 小角度 TTA：对格号文字用 ±10° 投票（90° 旋转会让文字侧过来，反而降精度）。
// 每个角度独立推理后对 softmax 概率取平均，再 argmax。
constexpr float kTtaAngles[] = {-10.F, -5.F, 0.F, 5.F, 10.F};
constexpr int kTtaCount = sizeof(kTtaAngles) / sizeof(kTtaAngles[0]);
static_assert(kTtaCount == CellClassifier::kTtaSlots, "tta angle count mismatch");
// 0° 在 kTtaAngles 中的下标（早退判定先跑它）
constexpr int kTtaZero = 2;
static_assert(kTtaAngles[kTtaZero] == 0.F, "kTtaZero must point at the 0-degree angle");
// 执行顺序：先 0°（早退判定），再其余角度。集合不变；softmax 概率平均与顺序无关。
constexpr int kTtaOrder[kTtaCount] = {kTtaZero, 0, 1, 3, 4};
// TTA 置信度早退阈值：0° 角结果的置信度 >= 此值即跳过其余 4 个角度。
// 0.8 留足余量——5 角度全投时，0° 独票已达 0.8 的样本极少被其余角度翻案；
// 拿不准就调回 >1.0（等效关闭早退，退回全量投票）。
constexpr float kTtaEarlyExitConf = .8F;
}  // namespace

CellId CellClassifier::infer(const cv::Mat& crop)
{
    cv::resize(crop, resized_, {96, 96});

    float letter_prob[14] = {0}, row_prob[11] = {0};
    int votes = 0;   // 实际计入的角度数（早退/NaN 弃权时 < kTtaCount，分母按实际计）

    // 单角度预处理 + 该槽位的 H2D/infer/D2H 连续 enqueue（不同步）：
    // 同一 stream 上操作按序执行，CPU 预处理下一角度与 GPU 跑上一角度重叠；
    // 原实现每角度一次 cudaStreamSynchronize（5 次/ROI），现早退 1 次、全量 2 次。
    const auto enqueue = [&](int slot)
    {
        const float angle = kTtaAngles[slot];
        const cv::Mat* src = &resized_;
        if (angle != 0.F)
        {
            cv::Mat M = cv::getRotationMatrix2D({48, 48}, angle, 1.0);
            cv::warpAffine(resized_, rotated_, M, {96, 96}, cv::INTER_CUBIC,
                           cv::BORDER_CONSTANT, cv::Scalar(232, 232, 232));
            src = &rotated_;
        }
        cv::cvtColor(*src, rgb_, cv::COLOR_BGR2RGB);
        rgb_.convertTo(rgb_, CV_32F, 1 / 255.);
        float* v = h_inputs_[slot];
        constexpr float mean[] = {0.485F, 0.456F, 0.406F};
        constexpr float standard_deviation[] = {0.229F, 0.224F, 0.225F};
        for (int y = 0; y < 96; y++)
            for (int x0 = 0; x0 < 96; x0++)
                for (int c = 0; c < 3; c++)
                    v[c * 9216 + y * 96 + x0] =
                        (rgb_.at<cv::Vec3f>(y, x0)[c] - mean[c]) / standard_deviation[c];
        // enqueue 前按槽位重绑张量地址（TRT 8.5：enqueueV3 以调用时刻绑定为准）
        if (!context_->setTensorAddress("images", inputs_[slot]) ||
            !context_->setTensorAddress("letter_logits", letters_[slot]) ||
            !context_->setTensorAddress("row_logits", rows_[slot]))
            throw std::runtime_error("cannot bind cell_classifier tensor addresses");
        cudaCheck(cudaMemcpyAsync(inputs_[slot], h_inputs_[slot], 3 * 96 * 96 * sizeof(float),
                                  cudaMemcpyHostToDevice, stream_));
        if (!context_->enqueueV3(stream_))
            throw std::runtime_error("cell_classifier inference failed");
        cudaCheck(cudaMemcpyAsync(h_letters_[slot], letters_[slot], 14 * sizeof(float),
                                  cudaMemcpyDeviceToHost, stream_));
        cudaCheck(cudaMemcpyAsync(h_rows_[slot], rows_[slot], 11 * sizeof(float),
                                  cudaMemcpyDeviceToHost, stream_));
    };

    // logits -> softmax，累加到概率和。含 NaN/Inf 的角度弃权（畸形 logits 会
    // 污染 softmax 分母；弃权的代价只是该角度不投票）。
    const auto softmax_accum = [&](int slot, float* letter_acc, float* row_acc) -> bool
    {
        const float* a = h_letters_[slot];
        const float* r = h_rows_[slot];
        for (int i = 0; i < 14; ++i)
            if (!std::isfinite(a[i]))
                return false;
        for (int i = 0; i < 11; ++i)
            if (!std::isfinite(r[i]))
                return false;
        const auto accum = [](const float* logits, float* prob, int n)
        {
            const float m = *std::max_element(logits, logits + n);
            float s = 0;
            for (int i = 0; i < n; i++)
                s += std::exp(logits[i] - m);
            for (int i = 0; i < n; i++)
                prob[i] += std::exp(logits[i] - m) / s;
        };
        accum(a, letter_acc, 14);
        accum(r, row_acc, 11);
        return true;
    };
    // 当前概率和对应的置信度（与返回值口径一致：min(letter, row) 各自 argmax）
    const auto confidence = [&]() -> float
    {
        const int ai = std::max_element(letter_prob, letter_prob + 14) - letter_prob;
        const int ri = std::max_element(row_prob, row_prob + 11) - row_prob;
        return std::min(letter_prob[ai], row_prob[ri]);
    };

    // 先跑 0°：置信度足够高就早退，跳过其余角度（省 4 次预处理+推理）
    enqueue(kTtaZero);
    cudaCheck(cudaStreamSynchronize(stream_));
    if (softmax_accum(kTtaZero, letter_prob, row_prob))
        ++votes;
    // votes==1 时概率和即单角度概率（早退判定与最终置信度同口径）
    if (!(votes == 1 && confidence() >= kTtaEarlyExitConf))
    {
        for (int k = 1; k < kTtaCount; ++k)
            enqueue(kTtaOrder[k]);
        cudaCheck(cudaStreamSynchronize(stream_));
        for (int k = 1; k < kTtaCount; ++k)
            if (softmax_accum(kTtaOrder[k], letter_prob, row_prob))
                ++votes;
    }
    // 分母按实际投票角度数（早退只计已跑角度；NaN 弃权不计）
    if (votes > 0)
    {
        for (int i = 0; i < 14; i++) letter_prob[i] /= votes;
        for (int i = 0; i < 11; i++) row_prob[i] /= votes;
    }
    // 全部角度弃权（极端异常）：概率全 0 -> argmax 置信度 0，调用方阈值拒收
    int ai = std::max_element(letter_prob, letter_prob + 14) - letter_prob;
    int ri = std::max_element(row_prob, row_prob + 11) - row_prob;
    // 置信度 = min(letter_prob, row_prob)，与原逻辑一致
    return {char('A' + ai), ri + 1, std::min(letter_prob[ai], row_prob[ri])};
}
}  // namespace dzuav
