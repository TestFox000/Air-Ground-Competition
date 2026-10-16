#include "dzuav/HbbDetector.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
namespace dzuav
{
namespace
{
// 引擎输出形状契约校验用：Dims 打印成 "[1,11,33600]" 便于定位换错引擎的问题。
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
}  // namespace
HbbDetector::HbbDetector(const std::string& p, float c, float n)
    : TensorRtEngine(p), confidence_(c), nms_(n)
{
    if (!context_->setInputShape("images", nvinfer1::Dims4{1, 3, 1280, 1280}))
        throw std::runtime_error("cannot set hbb_detector input shape");
    // 输出形状契约校验：不符即抛错并打印实际形状，防换引擎后静默读垃圾。
    const auto out_dims = context_->getTensorShape("output0");
    if (out_dims.nbDims != 3 || out_dims.d[0] != 1 || out_dims.d[1] != 11 ||
        out_dims.d[2] != 33600)
        throw std::runtime_error("hbb_detector output0 shape mismatch: expect [1,11,33600], got " +
                                 dimsToString(out_dims));
    cudaCheck(cudaMalloc(&input_, 3 * 1280 * 1280 * sizeof(float)));
    cudaCheck(cudaMalloc(&output_, 11 * 33600 * sizeof(float)));
    // 页锁定主机缓冲：H2D/D2H DMA 直传，免驱动临时中转（NVIDIA TRT 性能建议）。
    cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&h_input_), 3 * 1280 * 1280 * sizeof(float)));
    cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&h_output_), 11 * 33600 * sizeof(float)));
    // h_input_ 全量预填 padding 常数 = 114/255（114 为训练侧 letterbox pad 值，
    // 见 tools/ml/prelabel_onnx.py letterbox 与原实现 copyMakeBorder 的 114；
    // 归一化口径与 blobFromImage(scalefactor=1/255, mean=0) 的 (x-0)*1/255 一致）。
    // 此后每帧只重写有效区域（见 infer），padding 区保持本常数不变。
    std::fill(h_input_, h_input_ + 3 * 1280 * 1280, 114.F * (1.F / 255.F));
    // 张量地址构造期绑定一次，每帧只 enqueue。
    if (!context_->setTensorAddress("images", input_) ||
        !context_->setTensorAddress("output0", output_))
        throw std::runtime_error("cannot bind hbb_detector tensor addresses");
}
std::vector<Detection> HbbDetector::infer(const cv::Mat& image)
{
    const float s = std::min(1280.f / image.cols, 1280.f / image.rows);
    // 一次 warpAffine 完成"等比缩放 + 贴左上 + 114 填充"（替代 resize+copyMakeBorder
    // 两次全图拷贝）。同为双线性插值，定点舍入路径与 resize 可能差 ±1 灰度——
    // 对 FP16 引擎无感，模型输入语义（尺度/填充值/排布）不变。
    const cv::Mat M = (cv::Mat_<double>(2, 3) << s, 0, 0, 0, s, 0);
    cv::warpAffine(image, letterbox_, M, {1280, 1280}, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                   cv::Scalar(114, 114, 114));
    // 有效区域尺寸（与 resize(Size(), s, s) 的取整口径一致），clamp 防御 16:9 以外画幅
    const int vw = std::min(1280, std::max(1, cvRound(image.cols * s)));
    const int vh = std::min(1280, std::max(1, cvRound(image.rows * s)));
    if (vw != norm_w_ || vh != norm_h_)
    {
        // 有效区变化（如断流降规格）：padding 区可能混入上一种尺寸的有效像素，
        // 全量重填 114/255 一次（罕见路径，代价可忽略）。
        std::fill(h_input_, h_input_ + 3 * 1280 * 1280, 114.F * (1.F / 255.F));
        norm_w_ = vw;
        norm_h_ = vh;
    }
    // 归一化只写有效区域 [0,vw)x[0,vh)：v/255 且 BGR->RGB 交换（与 blobFromImage
    // (swapRB=true, scalefactor=1/255) 逐位同口径）；padding 区保持构造期预填常数。
    constexpr float inv255 = 1.F / 255.F;
    float* plane_r = h_input_;
    float* plane_g = h_input_ + 1280 * 1280;
    float* plane_b = h_input_ + 2 * 1280 * 1280;
    for (int y = 0; y < vh; ++y)
    {
        const cv::Vec3b* px = letterbox_.ptr<cv::Vec3b>(y);
        const int row_off = y * 1280;
        for (int x = 0; x < vw; ++x)
        {
            plane_r[row_off + x] = px[x][2] * inv255;
            plane_g[row_off + x] = px[x][1] * inv255;
            plane_b[row_off + x] = px[x][0] * inv255;
        }
    }
    cudaCheck(cudaMemcpyAsync(input_, h_input_, 3 * 1280 * 1280 * sizeof(float),
                              cudaMemcpyHostToDevice, stream_));
    if (!context_->enqueueV3(stream_))
        throw std::runtime_error("hbb_detector inference failed");
    cudaCheck(cudaMemcpyAsync(h_output_, output_, 11 * 33600 * sizeof(float),
                              cudaMemcpyDeviceToHost, stream_));
    cudaCheck(cudaStreamSynchronize(stream_));
    const float* raw = h_output_;
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> classes;
    for (int i = 0; i < 33600; i++)
    {
        int cls = 0;
        float score = 0;
        for (int c = 0; c < 7; c++)
            if (raw[(4 + c) * 33600 + i] > score)
            {
                score = raw[(4 + c) * 33600 + i];
                cls = c;
            }
        if (score < confidence_)
            continue;
        // 畸形输出防御：非有限值直接丢弃（NaN 分数在上面比较中自然落选，
        // 但 +Inf 会闯过阈值；坐标非有限会在 float->int 转换处爆出未定义框）
        const float cx = raw[i], cy = raw[33600 + i], w = raw[2 * 33600 + i],
                    h = raw[3 * 33600 + i];
        if (!std::isfinite(score) || !std::isfinite(cx) || !std::isfinite(cy) ||
            !std::isfinite(w) || !std::isfinite(h))
            continue;
        const float wf = w / s, hf = h / s, xf = (cx - w * .5f) / s, yf = (cy - h * .5f) / s;
        // float->int：cvRound 四舍五入（替代 int() 截断），再 clamp 到图像范围
        // （原实现框可越出右/下边缘，下游交集运算才兜底）
        const int x = std::clamp(cvRound(xf), 0, image.cols - 1);
        const int y = std::clamp(cvRound(yf), 0, image.rows - 1);
        const int bw = std::clamp(cvRound(wf), 1, image.cols - x);
        const int bh = std::clamp(cvRound(hf), 1, image.rows - y);
        boxes.emplace_back(cv::Point(x, y), cv::Size(bw, bh));
        scores.push_back(score);
        classes.push_back(cls);
    }
    std::vector<Detection> out;
    for (int c = 0; c < 7; c++)
    {
        std::vector<cv::Rect> b;
        std::vector<float> q;
        std::vector<int> map;
        for (size_t i = 0; i < boxes.size(); i++)
            if (classes[i] == c)
            {
                b.push_back(boxes[i]);
                q.push_back(scores[i]);
                map.push_back(i);
            }
        std::vector<int> keep;
        cv::dnn::NMSBoxes(b, q, confidence_, nms_, keep);
        for (int k : keep)
        {
            int i = map[k];
            out.push_back({static_cast<ObjectClass>(classes[i]), scores[i], boxes[i]});
        }
    }
    return out;
}
}  // namespace dzuav
