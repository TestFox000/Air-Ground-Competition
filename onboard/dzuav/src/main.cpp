#include <algorithm>
#include <array>
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
#include <utility>
#include <vector>

#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "dzuav/CellClassifier.hpp"
#include "dzuav/HbbDetector.hpp"
#include "dzuav/MapLocalizer.hpp"
#include "dzuav/UdpBroadcaster.hpp"

using namespace dzuav;

namespace
{
std::atomic<bool> g_running{true};

void handleSignal(int)
{
    g_running.store(false);
}

const char* className(ObjectClass type)
{
    static const char* names[] = {"paper",     "cell_label", "car",        "target_red",
                                  "target_blue", "obstacle", "bucket_red", "bucket_green",
                                  "bucket_cyan", "car_start", "car_end",    "uav_start",
                                  "uav_end"};
    return names[static_cast<int>(type)];
}

// D 方案 13 类部署参数（2026-10-16 四方对比口径 + 功能类默认值）。
// paper=0.25 为验证工作点（mine R98.3%/arrow R88.9% P100%）；cell=0.5 与
// 旧部署一致；靶标/车/起止点用 0.25 保召回；obstacle/桶 0.4。
std::array<float, kObjectClassCount> kDeployConf = {
    0.25F,  // paper
    0.50F,  // cell_label
    0.25F,  // car
    0.25F,  // target_red
    0.25F,  // target_blue
    0.40F,  // obstacle
    0.40F,  // bucket_red
    0.40F,  // bucket_green
    0.40F,  // bucket_cyan
    0.25F,  // car_start
    0.25F,  // car_end
    0.25F,  // uav_start
    0.25F,  // uav_end
};
constexpr float kDeployNms = 0.45F;
constexpr float kEdgePaperThr = 0.005F;

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
                  UdpBroadcaster& udp, const std::filesystem::path& output)
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
        // v4d 契约：正方形裁剪（边长=max(w,h)×1.3，居中）。矩形框直接 resize
        // 96×96 会把数字头拉伸变形导致系统性误读（2026-10-17 视频侦察实锤，
        // Python 参考实现 make_v4d_testpack.py V4D_PAD=1.3）。出界部分按图像
        // 边缘裁掉，与参考实现行为一致。
        const float side = std::max(roi.width, roi.height) * 1.3F;
        const cv::Point2f c(roi.x + roi.width * .5F, roi.y + roi.height * .5F);
        const cv::Rect crop(static_cast<int>(c.x - side * .5F),
                            static_cast<int>(c.y - side * .5F),
                            static_cast<int>(side), static_cast<int>(side));
        const cv::Rect sq = crop & cv::Rect(0, 0, image.cols, image.rows);
        if (sq.area() <= 4)
            continue;
        const CellId id = cell.infer(image(sq));
        if (id.valid() && id.confidence > .45F)   // TTA 后置信度更可靠，放宽阈值多用地标
            landmarks.push_back({id, {roi.x + roi.width * .5F, roi.y + roi.height * .5F}});
    }
    const bool localized = map.solve(landmarks, image.size());
    // 可视化只在需要落盘时做（相机实时模式跳过 clone+绘制，省每帧 ~6MB 分配与绘制耗时）
    const bool annotate = !output.empty();
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
            // 注意：length/width 字段填的是像素框宽/高，并非契约所写的"米"
            // （车端 Python 目前忽略这两个字段，无实害；契约待更新，字节布局不变）。
            objects.push_back({detection.type,
                               cv::Point3f(float(world.x), float(world.y), 0.F),
                               static_cast<float>(detection.box.width),
                               static_cast<float>(detection.box.height), detection.confidence});
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
    if (annotate && !cv::imwrite(output.string(), visual))
        std::cerr << "cannot write " << output.string() << '\n';
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
//
// 为什么不是一条 pipeline 走到底（2026-10-20 实机踩到）：
//   普通 UVC 相机（如 Realtek 0bda:5883）在 MJPEG 上能给 1080p@30，
//   但同分辨率的原始格式 YUY2 只有 **2fps**（USB2 带宽限制）。而 v4l2src
//   一旦被钉死 `video/x-raw, framerate=30/1` 就会 not-negotiated，
//   OpenCV 报 "Internal data stream error"、isOpened() 返回 false，
//   表现为「相机明明插着却打不开」（实测日志见下）。
//   所以按「MJPEG → 原始(不约束帧率) → 原始(指定帧率)」三级回退，
//   MJPEG 是 UVC 1080p30 的唯一可行路，后两级兜老相机/CSI 转接。
std::string uvcPipelineMjpeg(const std::string& device, int width, int height, int fps)
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

std::string uvcPipelineRaw(const std::string& device, int width, int height, int fps,
                           bool constrain_fps)
{
    std::ostringstream os;
    os << "v4l2src device=" << device
       << " ! video/x-raw,width=" << width << ",height=" << height;
    if (constrain_fps && fps > 0)
        os << ",framerate=" << fps << "/1";
    os << " ! videoconvert ! video/x-raw,format=BGR"
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

// 解析 UDP 端口：纯数字且在 1~65535 内才接受。stoi+static_cast<uint16_t>
// 会让 ">65535" 静默截断、非法输入直接抛异常终止，这里显式拒绝。
bool parsePort(const char* text, uint16_t& port)
{
    if (text == nullptr || *text == '\0')
        return false;
    for (const char* p = text; *p != '\0'; ++p)
        if (!std::isdigit(static_cast<unsigned char>(*p)))
            return false;
    const long v = std::strtol(text, nullptr, 10);
    if (v <= 0 || v > 65535)
        return false;
    port = static_cast<uint16_t>(v);
    return true;
}

int runCamera(const std::string& source, const std::string& hbb_path,
              const std::string& cell_path, uint16_t port, const std::string& spec)
{
    int width = 1920, height = 1080, fps = 30;
    parseSpec(spec, width, height, fps);

    std::vector<std::string> candidates;
    if (source.rfind("argus:", 0) == 0)
    {
        candidates.push_back(argusPipeline(std::atoi(source.substr(6).c_str()), width, height, fps));
    }
    else if (source.rfind("uvc:", 0) == 0)
    {
        std::string device = source.substr(4);
        if (!device.empty() &&
            std::all_of(device.begin(), device.end(),
                        [](char ch) { return std::isdigit(static_cast<unsigned char>(ch)); }))
        {
            device = "/dev/video" + device;
        }
        // 三级回退（见 uvcPipelineMjpeg 注释）：UVC 1080p30 通常只有 MJPEG 可行。
        candidates.push_back(uvcPipelineMjpeg(device, width, height, fps));
        candidates.push_back(uvcPipelineRaw(device, width, height, fps, false));
        candidates.push_back(uvcPipelineRaw(device, width, height, fps, true));
    }
    else
    {
        std::cerr << "unknown camera source: " << source
                  << " (expected argus:<sensor-id> or uvc:<device>)\n";
        return 2;
    }

    // 逐个尝试，用第一条真正能出帧的 pipeline（只 isOpened 不够：
    // not-negotiated 时 isOpened 也可能是 true，必须试读一帧）。
    cv::VideoCapture capture;
    std::string used;
    for (const auto& pipeline : candidates)
    {
        std::cout << "opening camera: " << pipeline << '\n';
        cv::VideoCapture trial(pipeline, cv::CAP_GSTREAMER);
        if (!trial.isOpened())
            continue;
        cv::Mat probe;
        if (!trial.read(probe) || probe.empty())
        {
            trial.release();
            continue;
        }
        capture = std::move(trial);
        used = pipeline;
        // 试读掉的那一帧不能丢：交给主循环作为首帧处理会重复计数，
        // 这里直接丢弃（仅 1 帧，不影响定位/时序）。
        break;
    }
    if (!capture.isOpened())
    {
        std::cerr << "cannot open camera: all pipelines failed. "
                     "UVC 1080p30 通常只能走 MJPEG（需 jpegdec 插件）；"
                     "查格式: gst-device-monitor-1.0 Video/Source\n";
        return 3;
    }
    std::cout << "camera opened with: " << used << '\n';

    std::array<float, kObjectClassCount> nms_per_class;
    nms_per_class.fill(kDeployNms);
    HbbDetector hbb(hbb_path, kDeployConf, nms_per_class, kEdgePaperThr);
    CellClassifier cell(cell_path);
    MapLocalizer map;
    UdpBroadcaster udp(port);

    cv::Mat frame;
    uint64_t frame_index = 0;
    uint64_t empty_frames = 0;
    unsigned frame_errors = 0;   // 连续处理异常计数（成功即清零）
    unsigned reopen_attempts = 0;   // 断流重开次数（收到好帧即清零，上限 3）
    while (g_running.load())
    {
        if (!capture.read(frame) || frame.empty())
        {
            ++empty_frames;
            if (empty_frames < 30)
                std::cerr << "empty frame\n";
            // 断流空转防护：让出 CPU（约一拍帧间隔），否则 read 立即 false 会
            // 100% 占核永不退出。
            usleep(33000);
            if (empty_frames >= 300)
            {
                // 原来一次重开失败就 return 4：整局再没有情报，车端只能瞎跑。
                // USB/CSI 掉线绝大多数一次重开就能恢复，改成有界重试 + 退避，
                // 真的死了照样在 3 次之后退出，不会无声空转。
                if (reopen_attempts < 3)
                {
                    ++reopen_attempts;
                    empty_frames = 0;
                    std::cerr << "camera stream lost; reopening pipeline (attempt "
                              << reopen_attempts << "/3)\n";
                    // 重开前必须 release：在已打开的 VideoCapture 上直接 open
                    // 一条新 GStreamer pipeline 可能失败，也会泄漏旧管道。
                    capture.release();
                    usleep(200000 * reopen_attempts);   // 200/400/600ms 退避
                    capture.open(used, cv::CAP_GSTREAMER);
                    if (!capture.isOpened())
                        std::cerr << "camera reopen failed (attempt "
                                  << reopen_attempts << "/3)\n";
                }
                else
                {
                    std::cerr << "camera stream did not recover after "
                              << reopen_attempts << " reopen attempts\n";
                    return 4;
                }
            }
            continue;
        }
        empty_frames = 0;
        reopen_attempts = 0;   // 恢复正常：下一次独立掉线重新获得 3 次重试额度
        // 单帧推理/图像处理异常不终结整个侦察进程：打印后继续；
        // 连续异常超上限（故障大概率是持久的）才非零退出。
        try
        {
            processFrame(frame, hbb, cell, map, udp, {});
        }
        catch (const std::exception& e)
        {
            ++frame_errors;
            std::cerr << "frame processing error (" << frame_errors
                      << " consecutive): " << e.what() << '\n';
            if (frame_errors >= 30)
            {
                std::cerr << "too many consecutive frame errors; aborting\n";
                return 5;
            }
            continue;
        }
        frame_errors = 0;
        ++frame_index;
    }
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
               "engines/cell_classifier_fp16.engine 50000 1920x1080@30\n";
        return 2;
    }

    const std::string source(argv[1]);
    uint16_t port = 50000;
    if (argc > 4 && !parsePort(argv[4], port))
    {
        std::cerr << "invalid udp port: " << argv[4] << " (expect 1-65535)\n";
        return 2;
    }
    if (source.rfind("argus:", 0) == 0 || source.rfind("uvc:", 0) == 0)
    {
        const std::string spec = argc > 5 ? argv[5] : "1920x1080@30";
        return runCamera(source, argv[2], argv[3], port, spec);
    }

    // 图片 / 视频文件分支（离线分析）。
    const std::filesystem::path input(argv[1]);
    const std::filesystem::path output_dir = argc > 5 ? argv[5] : "";
    if (!output_dir.empty())
        std::filesystem::create_directories(output_dir);
    std::array<float, kObjectClassCount> nms_per_class;
    nms_per_class.fill(kDeployNms);
    HbbDetector hbb(argv[2], kDeployConf, nms_per_class, kEdgePaperThr);
    CellClassifier cell(argv[3]);
    MapLocalizer map;
    UdpBroadcaster udp(port);
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
