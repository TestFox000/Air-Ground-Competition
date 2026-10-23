#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>

#include "dzuav/CellClassifier.hpp"
#include "dzuav/HbbDetector.hpp"
#include "dzuav/MapLocalizer.hpp"
#include "dzuav/UdpBroadcaster.hpp"

using namespace dzuav;

namespace
{
std::atomic<bool> g_running{true};
constexpr const char* kWindowName = "dzuav preview";

void handleSignal(int)
{
    g_running.store(false);
}

const char* className(ObjectClass type)
{
    static const char* names[] = {"mine",       "car",         "arrow",   "target_red",
                                  "cell_label", "target_blue", "obstacle"};
    return names[static_cast<int>(type)];
}

// 车头朝向估计：车顶贴红色方向标记（车头一侧），标记质心相对车框中心的
// 偏移方向即车头方向。图像方向经 map.project 投到场地世界坐标后取 atan2。
// 返回 false 表示这一帧估不出来（没找到标记 / 面积太小），调用方跳过即可。
bool estimateCarYaw(const cv::Mat& image, const cv::Rect& box, MapLocalizer& map,
                    double& out_x, double& out_y, float& out_yaw)
{
    const cv::Rect roi = box & cv::Rect(0, 0, image.cols, image.rows);
    if (roi.area() <= 16)
        return false;
    cv::Mat hsv;
    cv::cvtColor(image(roi), hsv, cv::COLOR_BGR2HSV);
    // 红色横跨 H 两端（≈0-10 与 ≈170-180），两段都收。
    cv::Mat mask, mask2;
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask);
    cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask2);
    mask |= mask2;
    const cv::Moments m = cv::moments(mask, true);
    if (m.m00 < 50)   // 标记像素太少：不可靠，宁缺毋滥
        return false;
    const cv::Point2f marker(roi.x + float(m.m10 / m.m00), roi.y + float(m.m01 / m.m00));
    const cv::Point2f center(roi.x + roi.width * .5F, roi.y + roi.height * .5F);
    // 投影到世界坐标再取方向：免去图像坐标系手性/旋转的坑。
    const auto wm = map.project(marker);
    const auto wc = map.project(center);
    out_x = wc.x;
    out_y = wc.y;
    out_yaw = std::atan2(float(wm.y - wc.y), float(wm.x - wc.x));
    return true;
}

void processFrame(const cv::Mat& frame, HbbDetector& hbb, CellClassifier& cell, MapLocalizer& map,
                  UdpBroadcaster& udp, const std::filesystem::path& output,
                  bool preview = false)
{
    cv::Mat image = map.undistort(frame);
    const auto detections = hbb.infer(image);
    std::vector<std::pair<CellId, cv::Point2f>> landmarks;
    for (const auto& detection : detections)
    {
        if (detection.type != ObjectClass::CellLabel)
            continue;
        const auto roi = cv::Rect2f(detection.box) & cv::Rect2f(0, 0, image.cols, image.rows);
        if (roi.area() <= 4)
            continue;
        // 贴边半标签：框被图像边缘裁切，中心系统性偏离格心，会污染 PnP（弃用）
        if (roi.x < 8.f || roi.y < 8.f ||
            roi.x + roi.width > static_cast<float>(image.cols) - 8.f ||
            roi.y + roi.height > static_cast<float>(image.rows) - 8.f)
            continue;
        const CellId id = cell.infer(image(roi));
        if (id.valid() && id.confidence > .45F)   // TTA 后置信度更可靠，放宽阈值多用地标
            landmarks.push_back({id, {roi.x + roi.width * .5F, roi.y + roi.height * .5F}});
    }
    const bool localized = map.solve(landmarks, image.size());
    // 可视化只在预览/落盘时做（纯广播模式跳过 clone+绘制，省每帧 ~6MB 分配与绘制耗时）
    const bool annotate = !output.empty() || preview;
    std::vector<WorldObject> objects;
    cv::Mat visual;
    if (annotate)
        visual = image.clone();
    for (const auto& detection : detections)
    {
        std::string label;
        if (annotate)
        {
            cv::rectangle(visual, detection.box,
                          detection.type == ObjectClass::CellLabel ? cv::Scalar(0, 255, 255)
                                                                   : cv::Scalar(0, 255, 0),
                          2);
            label = std::string(className(detection.type)) + " " +
                    cv::format("%.2f", detection.confidence);
        }
        if (detection.type == ObjectClass::CellLabel)
        {
            if (annotate)
            {
                const auto it = std::find_if(
                    landmarks.begin(), landmarks.end(),
                    [&](const auto& value)
                    {
                        return cv::norm(value.second -
                                        cv::Point2f(detection.box.x + detection.box.width * .5F,
                                                    detection.box.y + detection.box.height * .5F)) < 2;
                    });
                if (it != landmarks.end())
                    label += " " + it->first.text();
            }
        }
        else if (localized)
        {
            cv::Point2f point{detection.box.x + detection.box.width * .5F,
                              detection.box.y + detection.box.height * .5F};
            const auto world = map.project(point);
            objects.push_back({detection.type,
                               cv::Point3f(float(world.x), float(world.y), 0.F),
                               detection.box.width, detection.box.height, detection.confidence});
            if (annotate)
                label += cv::format(" (%.3f,%.3f)", world.x, world.y);
        }
        if (annotate)
            cv::putText(visual, label, {int(detection.box.x), std::max(18, int(detection.box.y) - 5)},
                        cv::FONT_HERSHEY_SIMPLEX, .55, cv::Scalar(0, 0, 255), 2);
    }
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
    if (localized || std::getenv("DZUAV_FORCE_UDP"))
        udp.send(timestamp, objects);
    // 车朝向（0x4002）：定位有效 + 检测到车时，用红色方向标记估计车头朝向下发，
    // 车端 integrate_node（heading_bias_source=udp）据此修正 reset 断言，根除绕圈。
    bool car_pose_ok = false;
    double car_x = 0, car_y = 0;
    float car_yaw = 0;
    if (localized)
    {
        const Detection* car = nullptr;
        for (const auto& d : detections)
            if (d.type == ObjectClass::Car && (!car || d.confidence > car->confidence))
                car = &d;
        if (car)
            car_pose_ok = estimateCarYaw(image, car->box, map, car_x, car_y, car_yaw);
        if (car_pose_ok)
            udp.sendCarPose(timestamp, car_x, car_y, car_yaw);
    }
    std::cout << output.filename() << " localized=" << localized
              << " landmarks=" << landmarks.size() << " [";
    for (size_t i = 0; i < objects.size(); ++i)
    {
        const auto& object = objects[i];
        std::cout << (i ? "," : "") << "{\"class\":\"" << className(object.type)
                  << "\",\"center\":[" << object.position.x << ',' << object.position.y
                  << "],\"length\":" << object.length << ",\"width\":" << object.width << '}';
    }
    std::cout << "]";
    if (car_pose_ok)
        std::cout << " car_pose=(" << car_x << ',' << car_y << ",yaw=" << car_yaw << ')';
    std::cout << "\n";   // 不用 endl：省一次流刷新（30fps 下可观）
    if (annotate)
    {
        if (!output.empty())
            cv::imwrite(output.string(), visual);
        if (preview)
        {
            cv::imshow(kWindowName, visual);
            if (cv::waitKey(1) == 'q')
                g_running.store(false);
        }
    }
}

// MIPI CSI 相机（经 Jetson ISP / libargus）。
std::string argusPipeline(int sensor_id, int width, int height, int fps)
{
    std::ostringstream os;
    os << "nvarguscamerasrc sensor-id=" << sensor_id
       << " ! video/x-raw(memory:NVMM),width=" << width << ",height=" << height;
    if (fps > 0)
        os << ",framerate=" << fps << "/1";
    os << " ! nvvidconv ! video/x-raw,format=BGRx"
       << " ! videoconvert ! video/x-raw,format=BGR"
       << " ! appsink drop=true max-buffers=1";
    return os.str();
}

// USB UVC / V4L2 相机。device 形如 /dev/video0。
// AR0234 的 raw YUY2 在 1080p 只有 5fps，30fps 必须走 MJPEG；mjpeg=false 时用 raw。
std::string uvcPipeline(const std::string& device, int width, int height, int fps, bool mjpeg)
{
    std::ostringstream os;
    os << "v4l2src device=" << device
       << " ! " << (mjpeg ? "image/jpeg" : "video/x-raw")
       << ",width=" << width << ",height=" << height;
    if (fps > 0)
        os << ",framerate=" << fps << "/1";
    os << (mjpeg ? " ! jpegdec" : "")
       << " ! videoconvert ! video/x-raw,format=BGR"
       << " ! appsink drop=true max-buffers=1";
    return os.str();
}

// 解析 "1920x1080@30"；fps=0 表示不约束协商帧率。解析失败时保留调用方默认值。
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

int runCamera(const std::string& source, const std::string& hbb_path,
              const std::string& cell_path, uint16_t port, const std::string& spec)
{
    int width = 1920, height = 1080, fps = 30;
    parseSpec(spec, width, height, fps);

    std::string pipeline, uvc_device;
    if (source.rfind("argus:", 0) == 0)
    {
        pipeline = argusPipeline(std::atoi(source.substr(6).c_str()), width, height, fps);
    }
    else if (source.rfind("uvc:", 0) == 0)
    {
        uvc_device = source.substr(4);
        if (!uvc_device.empty() &&
            std::all_of(uvc_device.begin(), uvc_device.end(), [](char ch) { return std::isdigit(ch); }))
        {
            uvc_device = "/dev/video" + uvc_device;
        }
        pipeline = uvcPipeline(uvc_device, width, height, fps, /*mjpeg=*/true);
    }
    else
    {
        std::cerr << "unknown camera source: " << source
                  << " (expected argus:<sensor-id> or uvc:<device>)\n";
        return 2;
    }

    std::cout << "opening camera: " << pipeline << '\n';
    cv::VideoCapture capture(pipeline, cv::CAP_GSTREAMER);
    if (!capture.isOpened() && source.rfind("uvc:", 0) == 0)
    {
        std::cerr << "MJPEG pipeline failed; retrying with raw YUY2 (frame rate may be capped)\n";
        pipeline = uvcPipeline(uvc_device, width, height, fps, /*mjpeg=*/false);
        std::cout << "opening camera: " << pipeline << '\n';
        capture.open(pipeline, cv::CAP_GSTREAMER);
    }
    if (!capture.isOpened())
    {
        std::cerr << "cannot open camera; check the device node, GStreamer plugins and caps "
                     "(v4l2-ctl --list-formats-ext)\n";
        return 3;
    }

    HbbDetector hbb(hbb_path, .25F, .45F);
    CellClassifier cell(cell_path);
    MapLocalizer map;
    UdpBroadcaster udp(port);

    cv::Mat frame;
    uint64_t frame_index = 0;
    uint64_t empty_frames = 0;
    const bool preview = std::getenv("DZUAV_PREVIEW") != nullptr;
    if (preview)
    {
        cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
        cv::resizeWindow(kWindowName, 1280, 720);
        std::cout << "preview enabled (press q in the window to quit)\n";
    }
    while (g_running.load())
    {
        if (!capture.read(frame) || frame.empty())
        {
            ++empty_frames;
            if (empty_frames < 30)
                std::cerr << "empty frame\n";
            continue;
        }
        empty_frames = 0;
        processFrame(frame, hbb, cell, map, udp, {}, preview);
        ++frame_index;
    }
    if (preview)
        cv::destroyWindow(kWindowName);
    std::cout << "stopped after " << frame_index << " processed frames\n";
    return 0;
}
}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    if (argc < 4)
    {
        std::cerr
            << "usage:\n"
            << "  dzuav <image|video> <hbb_engine> <cell_engine> [udp_port] [output_dir]\n"
            << "  dzuav argus:<sensor-id> <hbb_engine> <cell_engine> [udp_port] [WxH@FPS]\n"
            << "  dzuav uvc:<device>  <hbb_engine> <cell_engine> [udp_port] [WxH@FPS]\n"
            << "examples:\n"
            << "  dzuav uvc:/dev/video0 engines/hbb_detector_fp16.engine "
               "engines/cell_classifier_fp16.engine 50000 1920x1080@30\n"
            << "  DZUAV_PREVIEW=1 dzuav uvc:/dev/video0 ...   # 相机模式开实时预览（按 q 退出）\n";
        return 2;
    }

    const std::string source(argv[1]);
    if (source.rfind("argus:", 0) == 0 || source.rfind("uvc:", 0) == 0)
    {
        const uint16_t port =
            argc > 4 ? static_cast<uint16_t>(std::stoi(argv[4])) : uint16_t(50000);
        const std::string spec = argc > 5 ? argv[5] : "1920x1080@30";
        return runCamera(source, argv[2], argv[3], port, spec);
    }

    // 图片 / 视频文件分支（离线分析）。
    const std::filesystem::path input(argv[1]);
    const std::filesystem::path output_dir = argc > 5 ? argv[5] : "";
    if (!output_dir.empty())
        std::filesystem::create_directories(output_dir);
    HbbDetector hbb(argv[2], .25F, .45F);
    CellClassifier cell(argv[3]);
    MapLocalizer map;
    UdpBroadcaster udp(argc > 4 ? std::stoi(argv[4]) : 50000);
    const auto outputFor = [&](size_t index)
    { return output_dir / (input.stem().string() + "_" + std::to_string(index) + ".jpg"); };
    cv::Mat image = cv::imread(input.string());
    if (!image.empty())
    {
        processFrame(image, hbb, cell, map, udp, outputFor(0));
        return 0;
    }
    cv::VideoCapture capture(input.string());
    if (!capture.isOpened())
        return 3;
    cv::Mat frame;
    for (size_t index = 0; capture.read(frame); ++index)
        processFrame(frame, hbb, cell, map, udp, outputFor(index));
    return 0;
}
