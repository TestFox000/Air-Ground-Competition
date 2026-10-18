#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "dzuav/CellClassifier.hpp"
#include "dzuav/HbbDetector.hpp"
#include "dzuav/MapLocalizer.hpp"

using namespace dzuav;

namespace
{
std::atomic<bool> g_running{true};

void handleSignal(int) { g_running.store(false); }

const char* className(ObjectClass type)
{
    static const char* names[] = {"paper",     "cell_label", "car",        "target_red",
                                  "target_blue", "obstacle", "bucket_red", "bucket_green",
                                  "bucket_cyan", "car_start", "car_end",    "uav_start",
                                  "uav_end"};
    return names[static_cast<int>(type)];
}

std::array<float, kObjectClassCount> kDeployConf = {
    0.25F, 0.50F, 0.25F, 0.25F, 0.25F, 0.40F, 0.40F,
    0.40F, 0.40F, 0.25F, 0.25F, 0.25F, 0.25F,
};
constexpr float kDeployNms = 0.45F;
constexpr float kEdgePaperThr = 0.005F;

std::string uvcPipeline(const std::string& device, int width, int height, int fps)
{
    std::ostringstream os;
    os << "v4l2src device=" << device
       << " ! image/jpeg,width=" << width << ",height=" << height;
    if (fps > 0)
        os << ",framerate=" << fps << "/1";
    os << " ! jpegdec ! videoconvert ! video/x-raw,format=BGR"
       << " ! appsink drop=true max-buffers=1";
    return os.str();
}

void parseSpec(const std::string& spec, int& width, int& height, int& fps)
{
    int w = 0, h = 0, f = 0;
    if (std::sscanf(spec.c_str(), "%dx%d@%d", &w, &h, &f) == 3 && w > 0 && h > 0 && f >= 0)
    {
        width = w;
        height = h;
        fps = f;
    }
}
}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    if (argc < 4)
    {
        std::cerr << "usage: viewer <uvc_device> <hbb_engine> <cell_engine> [WxH@FPS]\n";
        return 2;
    }

    const std::string device = argv[1];
    const std::string hbb_path = argv[2];
    const std::string cell_path = argv[3];
    const std::string spec = argc > 4 ? argv[4] : "1280x720@30";
    int width = 1280, height = 720, fps = 30;
    parseSpec(spec, width, height, fps);

    const std::string pipeline = uvcPipeline(device, width, height, fps);
    std::cout << "opening camera: " << pipeline << std::endl;

    cv::VideoCapture capture(pipeline, cv::CAP_GSTREAMER);
    if (!capture.isOpened())
    {
        std::cerr << "cannot open camera: " << device << std::endl;
        return 3;
    }

    std::array<float, kObjectClassCount> nms_per_class;
    nms_per_class.fill(kDeployNms);
    HbbDetector hbb(hbb_path, kDeployConf, nms_per_class, kEdgePaperThr);
    CellClassifier cell(cell_path);
    MapLocalizer map;

    cv::namedWindow("dzuav live", cv::WINDOW_NORMAL);
    cv::resizeWindow("dzuav live", 1280, 720);

    cv::Mat frame;
    uint64_t index = 0;
    double fps_est = 0.0;
    auto t_prev = std::chrono::steady_clock::now();

    while (g_running.load())
    {
        if (!capture.read(frame) || frame.empty())
        {
            usleep(33000);
            continue;
        }

        cv::Mat image = map.undistort(frame);
        const auto detections = hbb.infer(image);

        std::vector<std::pair<CellId, cv::Point2f>> landmarks;
        for (const auto& detection : detections)
        {
            if (detection.type != ObjectClass::CellLabel)
                continue;
            const cv::Rect roi = detection.box & cv::Rect(0, 0, image.cols, image.rows);
            if (roi.area() <= 4)
                continue;
            if (roi.x < 8 || roi.y < 8 ||
                roi.x + roi.width > image.cols - 8 || roi.y + roi.height > image.rows - 8)
                continue;
            const CellId id = cell.infer(image(roi));
            if (id.valid() && id.confidence > 0.45F)
                landmarks.push_back({id, {roi.x + roi.width * 0.5F, roi.y + roi.height * 0.5F}});
        }
        const bool localized = map.solve(landmarks, image.size());

        cv::Mat visual = image.clone();
        for (const auto& detection : detections)
        {
            const bool is_cell = detection.type == ObjectClass::CellLabel;
            cv::rectangle(visual, detection.box,
                          is_cell ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0), 2);
            std::string label = std::string(className(detection.type)) + " " +
                                cv::format("%.2f", detection.confidence);
            if (is_cell)
            {
                const cv::Point2f center(detection.box.x + detection.box.width * 0.5F,
                                         detection.box.y + detection.box.height * 0.5F);
                const auto it = std::find_if(
                    landmarks.begin(), landmarks.end(),
                    [&](const auto& value) { return cv::norm(value.second - center) < 2; });
                if (it != landmarks.end())
                    label += " " + it->first.text();
            }
            else if (localized)
            {
                const cv::Point2f center(detection.box.x + detection.box.width * 0.5F,
                                         detection.box.y + detection.box.height * 0.5F);
                const auto world = map.project(center);
                label += cv::format(" (%.2f,%.2f)", world.x, world.y);
            }
            cv::putText(visual, label,
                        {int(detection.box.x), std::max(18, int(detection.box.y) - 5)},
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 255), 2);
        }

        ++index;
        if (index % 15 == 0)
        {
            const auto now = std::chrono::steady_clock::now();
            fps_est = 15.0 / std::chrono::duration<double>(now - t_prev).count();
            t_prev = now;
        }
        char hud[256];
        std::snprintf(hud, sizeof(hud), "dets=%zu cells=%zu localized=%d fps=%.1f",
                      detections.size(), landmarks.size(), int(localized), fps_est);
        cv::putText(visual, hud, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 255), 2);

        cv::imshow("dzuav live", visual);
        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q')
            break;

        if (index % 30 == 0)
            std::cout << "frame " << index << " dets=" << detections.size()
                      << " cells=" << landmarks.size()
                      << " localized=" << localized << std::endl;
    }

    cv::destroyAllWindows();
    return 0;
}
