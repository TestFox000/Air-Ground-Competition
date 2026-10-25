#include "dzuav/HbbDetector.hpp"

#include <algorithm>
#include <string>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
namespace dzuav
{
namespace
{
// 引擎输出形状契约校验用：Dims 打印成 "[1,K,33600]" 便于定位换错引擎的问题。
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

// 输出通道数：4 框坐标 + 类别数（types.hpp 单点派生；ft_v1 14 类 = 18）。
constexpr int kOutChannels = 4 + kObjectClassCount;   // 18（旧 d13 = 17）
constexpr int kAnchors = 33600;                        // (160²+80²+40²)，imgsz=1280
constexpr int kInputSide = 1280;

// paper 与 cell_label 的重叠判据，对齐 video_recon.LABEL_OVERLAP_IOU。
constexpr float kLabelOverlapIou = 0.30F;

float rectIoU(const cv::Rect& a, const cv::Rect& b)
{
    const cv::Rect inter = a & b;
    if (inter.area() <= 0)
        return 0.F;
    const int uni = a.area() + b.area() - inter.area();
    return uni > 0 ? static_cast<float>(inter.area()) / static_cast<float>(uni) : 0.F;
}
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
    if (!context_->setInputShape("images", nvinfer1::Dims4{1, 3, kInputSide, kInputSide}))
        throw std::runtime_error("cannot set hbb_detector input shape");
    // 输出形状契约校验：不符即抛错并打印实际形状，防换引擎后静默读垃圾。
    // kOutChannels 随 types.hpp 类别数派生：ft_v1 14 类 = 18，旧 d13 = 17——
    // 17 通道 d13 引擎配本代码会在此 fail-fast（换引擎见 BINARY_MANIFEST）。
    const auto out_dims = context_->getTensorShape("output0");
    if (out_dims.nbDims != 3 || out_dims.d[0] != 1 ||
        out_dims.d[1] != kOutChannels || out_dims.d[2] != kAnchors)
        throw std::runtime_error("hbb_detector output0 shape mismatch: expect [1," +
                                 std::to_string(kOutChannels) + "," + std::to_string(kAnchors) +
                                 "], got " + dimsToString(out_dims));
    const size_t in_bytes = 3 * static_cast<size_t>(kInputSide) * kInputSide * sizeof(float);
    cudaCheck(cudaMalloc(&input_, in_bytes));
    cudaCheck(cudaMalloc(&output_, kOutChannels * kAnchors * sizeof(float)));
    // 页锁定主机缓冲：H2D/D2H DMA 直传，免驱动临时中转（NVIDIA TRT 性能建议）。
    cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&h_input_), in_bytes));
    cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&h_output_),
                             kOutChannels * kAnchors * sizeof(float)));
    // blob_ 包住 pinned 内存：blobFromImage 对同形状 OutputArray 不重新分配（Mat::create
    // 契约），直接写入；即使个别 OpenCV 版本内部重新分配，后续拷贝也总是从 blob_.ptr()
    // 取址，正确性不受影响（只是退回普通内存路径）。
    const int sizes[] = {1, 3, kInputSide, kInputSide};
    blob_ = cv::Mat(4, sizes, CV_32F, h_input_);
    // 张量地址构造期绑定一次，每帧只 enqueue。
    if (!context_->setTensorAddress("images", input_) ||
        !context_->setTensorAddress("output0", output_))
        throw std::runtime_error("cannot bind hbb_detector tensor addresses");
}

std::vector<Detection> HbbDetector::infer(const cv::Mat& image)
{
    // letterbox 必须**居中**填充，与训练/导出同口径（Ultralytics letterbox 居中；
    // 已验证的 PC 参考实现 video_recon.D13Detector 亦居中）。
    // 原先只在右下补边（top-left 对齐）：1920x1080 输入下 s=2/3、缩放后 1280x720，
    // 画面在模型坐标系里整体上移 pad_y=280px，与所有既有指标的输入分布都不同
    // （mine R98.3%/arrow R88.9% 那组走 ultralytics predict，val-B 边缘过滤那组
    // 同样走 ultralytics，106/106 走 video_recon —— 三者全是居中）。
    // 解码侧必须减回 pad，否则框会整体偏移 pad/s 个原图像素（此处 420px）。
    //
    // 两处口径必须与参考实现一致，否则会引入恒定方向的 1px 级偏差：
    //   1) s 用 **double**：原先 `cv::resize(image, r, cv::Size(), s, s)` 内部按
    //      cvRound((double)fx * cols) 算目标尺寸，用 float 算会与其分叉。
    //   2) 取整用 **cvRound**（四舍六入五成双 / round-half-to-even），不是
    //      std::lround（half-away-from-zero）。实测本仓 OpenCV 5.0：718.5→718、
    //      719.5→720，与 Python round() 一致；video_recon.py:340 用的正是 round()
    //      （已核）。Ultralytics letterbox 推断同为 half-to-even（离线机上无
    //      ultralytics 包，未能直接核源码；以 video_recon 为准即可，它是本仓
    //      产出 106/106 那组指标的实现）。用 lround 会让本文件成为异类。
    //      ⚠️ 2026-10-20 曾真的写成 lround 并给镜像加了配套测试，突变测试全绿——
    //      因为"镜像与 C++ 一致"查不出"两个一起偏离参考"。故测试同时锁三方。
    const double s = std::min(static_cast<double>(kInputSide) / image.cols,
                              static_cast<double>(kInputSide) / image.rows);
    const int nw = std::max(1, std::min(kInputSide, cvRound(image.cols * s)));
    const int nh = std::max(1, std::min(kInputSide, cvRound(image.rows * s)));
    const int pad_x = (kInputSide - nw) / 2, pad_y = (kInputSide - nh) / 2;
    cv::Mat r;
    cv::resize(image, r, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    cv::Mat canvas(kInputSide, kInputSide, image.type(), cv::Scalar(114, 114, 114));
    r.copyTo(canvas(cv::Rect(pad_x, pad_y, nw, nh)));
    cv::dnn::blobFromImage(canvas, blob_, 1 / 255.f, cv::Size(kInputSide, kInputSide), {},
                           true, false);
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
              x = (raw[i] - raw[2 * kAnchors + i] * .5f - pad_x) / s,
              y = (raw[kAnchors + i] - raw[3 * kAnchors + i] * .5f - pad_y) / s;
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

    // ---- 标签文字误检过滤（对齐 video_recon.filter_label_overlap）----
    // 高分辨率近拍下，垫面印刷格号（"E9"/"D4"）会被 paper 类误检成雷贴纸，
    // 2026-10-17 实拍实锤：f00700 的 "E9" 两个字各检出 paper 0.58 / 0.88。
    // 这两个分数远高于任何可用 paper 阈值（部署值 0.25），**调阈值拦不住**，
    // 只能靠"paper 框与任一 cell_label 框重叠即判为标签文字"剔除。
    // 物理依据：真雷贴纸贴在格心、印刷格号在格角，两者不重叠；K 列桥洞箭头
    // 在桥顶挡板上，也不与垫面格号重叠。
    // 漏掉这一层的后果是车端把格号当雷：要么浪费弹药去打一个印在地上的字，
    // 要么把可通行格判成雷区堵死规划。
    if (out.size() > 1)
    {
        std::vector<cv::Rect> labels;
        labels.reserve(out.size());
        for (const auto& d : out)
            if (d.type == ObjectClass::CellLabel)
                labels.push_back(d.box);
        if (!labels.empty())
        {
            std::vector<Detection> kept;
            kept.reserve(out.size());
            for (const auto& d : out)
            {
                if (d.type == ObjectClass::Paper)
                {
                    float best = 0.F;
                    for (const auto& lb : labels)
                        best = std::max(best, rectIoU(d.box, lb));
                    if (best > kLabelOverlapIou)
                        continue;   // 与格号重叠 → 判为标签文字，剔除
                }
                kept.push_back(d);
            }
            out.swap(kept);
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
