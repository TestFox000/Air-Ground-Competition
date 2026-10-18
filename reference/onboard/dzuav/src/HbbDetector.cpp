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
}
std::vector<Detection> HbbDetector::infer(const cv::Mat& image)
{
    cv::Mat r;
    float s = std::min(1280.f / image.cols, 1280.f / image.rows);
    cv::resize(image, r, cv::Size(), s, s);
    cv::copyMakeBorder(r, r, 0, 1280 - r.rows, 0, 1280 - r.cols, cv::BORDER_CONSTANT,
                       cv::Scalar(114, 114, 114));
    cv::Mat blob = cv::dnn::blobFromImage(r, 1 / 255.f, cv::Size(1280, 1280), {}, true, false);
    cudaMemcpyAsync(input_, blob.ptr<float>(), blob.total() * sizeof(float), cudaMemcpyHostToDevice,
                    stream_);
    if (!context_->setTensorAddress("images", input_) ||
        !context_->setTensorAddress("output0", output_) || !context_->enqueueV3(stream_))
        throw std::runtime_error("hbb_detector inference failed");
    std::vector<float> raw(11 * 33600);
    cudaMemcpyAsync(raw.data(), output_, raw.size() * sizeof(float), cudaMemcpyDeviceToHost,
                    stream_);
    cudaStreamSynchronize(stream_);
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
