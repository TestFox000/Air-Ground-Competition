#include "dzuav/CellClassifier.hpp"

#include <algorithm>
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
    cudaCheck(cudaMalloc(&input_, 3 * 96 * 96 * sizeof(float)));
    cudaCheck(cudaMalloc(&letters_, 14 * sizeof(float)));
    cudaCheck(cudaMalloc(&rows_, 11 * sizeof(float)));
    cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&h_input_), 3 * 96 * 96 * sizeof(float)));
    // 张量地址构造期绑定一次，每个 ROI 推理只 enqueue。
    if (!context_->setTensorAddress("images", input_) ||
        !context_->setTensorAddress("letter_logits", letters_) ||
        !context_->setTensorAddress("row_logits", rows_))
        throw std::runtime_error("cannot bind cell_classifier tensor addresses");
}
namespace
{
// 旋转 TTA：0/90/180/270° 四方向各推理一次，取置信度最高方向的读数。
// 依据（2026-10-17 视频侦察全天验证）：任务航线含反向航段，标签会以 180°
// 甚至任意 90° 倍数出现；训练增强仅 ±25°（dataset.py RandomAffine），不含
// 90° 倍数——必须靠四方向枚举覆盖任意机头朝向（与 Python 参考实现
// make_v4d_testpack.py 同构）。小角度倾斜由训练增强自身容忍。
constexpr int kTtaRotations = 4;
}  // namespace

CellId CellClassifier::infer(const cv::Mat& crop)
{
    cv::Mat resized;
    cv::resize(crop, resized, {96, 96});

    float best_conf = -1.F;
    int best_letter = 0, best_row = 0;
    for (int k = 0; k < kTtaRotations; ++k)
    {
        cv::Mat rotated;
        switch (k)
        {
        case 1: cv::rotate(resized, rotated, cv::ROTATE_90_CLOCKWISE); break;
        case 2: cv::rotate(resized, rotated, cv::ROTATE_180); break;
        case 3: cv::rotate(resized, rotated, cv::ROTATE_90_COUNTERCLOCKWISE); break;
        default: rotated = resized; break;
        }
        // 预处理 = BGR→RGB→/255，无 ImageNet 归一化。ONNX 实测契约
        //（2026-10-17 本地 106 真值裁剪 A/B：裸 [0,1] 全对 106/106，
        // ImageNet 归一化仅 69/106——云端 v4d 训练实际未按 dataset.py
        // 的 Normalize 口径，以实测为准，今日 recon 全天同口径验证）。
        cv::Mat rgb;
        cv::cvtColor(rotated, rgb, cv::COLOR_BGR2RGB);
        rgb.convertTo(rgb, CV_32F, 1 / 255.);
        float* v = h_input_;
        for (int y = 0; y < 96; y++)
            for (int x0 = 0; x0 < 96; x0++)
                for (int c = 0; c < 3; c++)
                    v[c * 9216 + y * 96 + x0] = rgb.at<cv::Vec3f>(y, x0)[c];
        cudaCheck(cudaMemcpyAsync(input_, h_input_, 3 * 96 * 96 * sizeof(float),
                                  cudaMemcpyHostToDevice, stream_));
        if (!context_->enqueueV3(stream_))
            throw std::runtime_error("cell_classifier inference failed");
        float a[14], r[11];
        cudaCheck(cudaMemcpyAsync(a, letters_, sizeof(a), cudaMemcpyDeviceToHost, stream_));
        cudaCheck(cudaMemcpyAsync(r, rows_, sizeof(r), cudaMemcpyDeviceToHost, stream_));
        cudaCheck(cudaStreamSynchronize(stream_));
        // logits -> softmax（单方向内不再跨角度平均：四个方向是离散的朝向
        // 假设，平均会把正确方向的概率稀释进三个错误方向）
        auto softmax = [](float* logits, int n)
        {
            float m = *std::max_element(logits, logits + n), s = 0;
            for (int i = 0; i < n; i++)
                s += std::exp(logits[i] - m);
            for (int i = 0; i < n; i++)
                logits[i] = std::exp(logits[i] - m) / s;
        };
        softmax(a, 14);
        softmax(r, 11);
        const int ai = std::max_element(a, a + 14) - a;
        const int ri = std::max_element(r, r + 11) - r;
        const float conf = std::min(a[ai], r[ri]);
        if (conf > best_conf)
        {
            best_conf = conf;
            best_letter = ai;
            best_row = ri;
        }
    }
    // 置信度 = min(letter_prob, row_prob)，与原逻辑一致
    return {char('A' + best_letter), best_row + 1, best_conf};
}
}  // namespace dzuav
