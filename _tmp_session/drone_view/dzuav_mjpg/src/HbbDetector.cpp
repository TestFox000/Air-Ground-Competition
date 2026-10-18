#include "dzuav/HbbDetector.hpp"

#include <algorithm>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
namespace dzuav
{
namespace
{
// 引擎输出形状契约校验用：Dims 打印成 "[1,17,33600]" 便于定位换错引擎的问题。
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

// 输出通道数：4 框坐标 + 13 类（D 方案 13 类统一模型）。
constexpr int kOutChannels = 4 + kObjectClassCount;   // 17
constexpr int kAnchors = 33600;                        // (160²+80²+40²)，imgsz=1280
}  // namespace

HbbDetector::HbbDetector(const std::string& p, float c, float n)
    : TensorRtEngine(p)
{
    conf_per_class_.fill(c);
    nms_per_class_.fill(n);
    allocate(p);
}

HbbDetector::HbbDetector(const std::string& p,
                         const std::array<float, kObjectClassCount>& conf_per_class,
                         const std::array<float, kObjectClassCount>& nms_per_class,
                         float edge_paper_thr)
    : TensorRtEngine(p),
      conf_per_class_(conf_per_class),
      nms_per_class_(nms_per_class),
      edge_paper_thr_(edge_paper_thr)
{
    allocate(p);
}

void HbbDetector::allocate(const std::string& p)
{
    if (!context_->setInputShape("images", nvinfer1::Dims4{1, 3, 1280, 1280}))
        throw std::runtime_error("cannot set hbb_detector input shape");
    // 输出形状契约校验：不符即抛错并打印实际形状，防换引擎后静默读垃圾。
    const auto out_dims = context_->getTensorShape("output0");
    if (out_dims.nbDims != 3 || out_dims.d[0] != 1 ||
        out_dims.d[1] != kOutChannels || out_dims.d[2] != kAnchors)
        throw std::runtime_error("hbb_detector output0 shape mismatch: expect [1,17,33600], got " +
                                 dimsToString(out_dims));
    cudaCheck(cudaMalloc(&input_, 3 * 1280 * 1280 * sizeof(float)));
    cudaCheck(cudaMalloc(&output_, kOutChannels * kAnchors * sizeof(float)));
    // 页锁定主机缓冲：H2D/D2H DMA 直传，免驱动临时中转（NVIDIA TRT 性能建议）。
    cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&h_input_), 3 * 1280 * 1280 * sizeof(float)));
    cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&h_output_),
                             kOutChannels * kAnchors * sizeof(float)));
    // blob_ 包住 pinned 内存：blobFromImage 对同形状 OutputArray 不重新分配（Mat::create
    // 契约），直接写入；即使个别 OpenCV 版本内部重新分配，后续拷贝也总是从 blob_.ptr()
    // 取址，正确性不受影响（只是退回普通内存路径）。
    const int sizes[] = {1, 3, 1280, 1280};
    blob_ = cv::Mat(4, sizes, CV_32F, h_input_);
    // 张量地址构造期绑定一次，每帧只 enqueue。
    if (!context_->setTensorAddress("images", input_) ||
        !context_->setTensorAddress("output0", output_))
        throw std::runtime_error("cannot bind hbb_detector tensor addresses");
}

std::vector<Detection> HbbDetector::infer(const cv::Mat& image)
{
    cv::Mat r;
    float s = std::min(1280.f / image.cols, 1280.f / image.rows);
    cv::resize(image, r, cv::Size(), s, s);
    cv::copyMakeBorder(r, r, 0, 1280 - r.rows, 0, 1280 - r.cols, cv::BORDER_CONSTANT,
                       cv::Scalar(114, 114, 114));
    cv::dnn::blobFromImage(r, blob_, 1 / 255.f, cv::Size(1280, 1280), {}, true, false);
    cudaCheck(cudaMemcpyAsync(input_, blob_.ptr<float>(), blob_.total() * sizeof(float),
                              cudaMemcpyHostToDevice, stream_));
    if (!context_->enqueueV3(stream_))
        throw std::runtime_error("hbb_detector inference failed");
    cudaCheck(cudaMemcpyAsync(h_output_, output_, kOutChannels * kAnchors * sizeof(float),
                              cudaMemcpyDeviceToHost, stream_));
    cudaCheck(cudaStreamSynchronize(stream_));
    const float* raw = h_output_;

    // ---- 解码 + 按类别置信度过滤 ----
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> classes;
    boxes.reserve(1024);
    scores.reserve(1024);
    classes.reserve(1024);
    for (int i = 0; i < kAnchors; i++)
    {
        int cls = 0;
        float score = 0;
        for (int c = 0; c < kObjectClassCount; c++)
            if (raw[(4 + c) * kAnchors + i] > score)
            {
                score = raw[(4 + c) * kAnchors + i];
                cls = c;
            }
        if (score < conf_per_class_[cls])
            continue;
        float w = raw[2 * kAnchors + i] / s, h = raw[3 * kAnchors + i] / s,
              x = (raw[i] - raw[2 * kAnchors + i] * .5f) / s,
              y = (raw[kAnchors + i] - raw[3 * kAnchors + i] * .5f) / s;
        boxes.emplace_back(cv::Rect(cv::Point(std::max(0, int(x)), std::max(0, int(y))),
                                    cv::Size(std::max(1, int(w)), std::max(1, int(h)))));
        scores.push_back(score);
        classes.push_back(cls);
    }

    // ---- 按类别 NMS ----
    std::vector<Detection> out;
    out.reserve(64);
    for (int c = 0; c < kObjectClassCount; c++)
    {
        std::vector<cv::Rect> b;
        std::vector<float> q;
        std::vector<int> map;
        b.reserve(boxes.size() / 4);
        q.reserve(boxes.size() / 4);
        map.reserve(boxes.size() / 4);
        for (size_t i = 0; i < boxes.size(); i++)
            if (classes[i] == c)
            {
                b.push_back(boxes[i]);
                q.push_back(scores[i]);
                map.push_back(static_cast<int>(i));
            }
        if (b.empty())
            continue;
        std::vector<int> keep;
        cv::dnn::NMSBoxes(b, q, conf_per_class_[c], nms_per_class_[c], keep);
        for (int k : keep)
        {
            int i = map[k];
            out.push_back({static_cast<ObjectClass>(classes[i]), scores[i], boxes[i]});
        }
    }

    // ---- Paper 类边缘过滤（PC 端验证的 letterbox 伪影对策延续）----
    // 仅对 Paper(cls=0) 生效：框中心距图像边缘 < edge_paper_thr（归一化）则剔除。
    // 语义注记：K 列桥洞区 paper（=通行箭头）理论不应滤，但落格判定在车端，
    // 机载无格位置信息；边缘箭头占比极低，实测净收益为正（mine 时代 +0.76pp）。
    if (edge_paper_thr_ > 0.F && !out.empty())
    {
        const float ex = edge_paper_thr_ * static_cast<float>(image.cols);
        const float ey = edge_paper_thr_ * static_cast<float>(image.rows);
        std::vector<Detection> filtered;
        filtered.reserve(out.size());
        for (const auto& d : out)
        {
            if (d.type == ObjectClass::Paper)
            {
                const float cx = d.box.x + d.box.width * .5F;
                const float cy = d.box.y + d.box.height * .5F;
                if (cx < ex || cx > image.cols - ex || cy < ey || cy > image.rows - ey)
                    continue;   // 边缘 paper → 剔除
            }
            filtered.push_back(d);
        }
        out.swap(filtered);
    }
    return out;
}
}  // namespace dzuav
