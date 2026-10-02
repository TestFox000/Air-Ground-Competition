#include "dzuav/HbbDetector.hpp"

#include <algorithm>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
namespace dzuav
{
HbbDetector::HbbDetector(const std::string& p, float c, float n)
    : TensorRtEngine(p), confidence_(c), nms_(n)
{
    if (!context_->setInputShape("images", nvinfer1::Dims4{1, 3, 1280, 1280}))
        throw std::runtime_error("cannot set hbb_detector input shape");
    cudaMalloc(&input_, 3 * 1280 * 1280 * sizeof(float));
    cudaMalloc(&output_, 11 * 33600 * sizeof(float));
    // 页锁定主机缓冲：H2D/D2H DMA 直传，免驱动临时中转（NVIDIA TRT 性能建议）。
    if (cudaMallocHost(reinterpret_cast<void**>(&h_input_), 3 * 1280 * 1280 * sizeof(float)) != cudaSuccess ||
        cudaMallocHost(reinterpret_cast<void**>(&h_output_), 11 * 33600 * sizeof(float)) != cudaSuccess)
        throw std::runtime_error("cannot allocate pinned host buffers for hbb_detector");
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
    cudaMemcpyAsync(input_, blob_.ptr<float>(), blob_.total() * sizeof(float), cudaMemcpyHostToDevice,
                    stream_);
    if (!context_->enqueueV3(stream_))
        throw std::runtime_error("hbb_detector inference failed");
    cudaMemcpyAsync(h_output_, output_, 11 * 33600 * sizeof(float), cudaMemcpyDeviceToHost,
                    stream_);
    cudaStreamSynchronize(stream_);
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
        float w = raw[2 * 33600 + i] / s, h = raw[3 * 33600 + i] / s,
              x = (raw[i] - raw[2 * 33600 + i] * .5f) / s,
              y = (raw[33600 + i] - raw[3 * 33600 + i] * .5f) / s;
        boxes.emplace_back(cv::Rect(cv::Point(std::max(0, int(x)), std::max(0, int(y))),
                                    cv::Size(std::max(1, int(w)), std::max(1, int(h)))));
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
