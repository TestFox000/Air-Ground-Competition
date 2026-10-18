#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>

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

// live HUD 帧统计（2026-10-18）：runCamera 据此绘制 dets/cells/localized/fps。
struct FrameStats
{
    int dets = 0;
    int cells = 0;        // 有效格号地标数（过了置信阈值的）
    int localized = 0;
};

cv::Mat processFrame(const cv::Mat& frame, HbbDetector& hbb, CellClassifier& cell, MapLocalizer& map,
                     UdpBroadcaster& udp, const std::filesystem::path& output,
                     FrameStats* stats = nullptr, float fps_hud = 0.F)
{
    cv::Mat image = map.undistort(frame);
    const auto detections = hbb.infer(image);
    if (stats)
        stats->dets = static_cast<int>(detections.size());
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
    if (stats)
    {
        stats->cells = static_cast<int>(landmarks.size());
        stats->localized = localized ? 1 : 0;
    }
    // 可视化：离线模式（output 非空）画框落盘；live 模式（stats 非空）画框
    // + HUD 供 imshow（2026-10-18：替代误读的第三方 live 程序——其预处理
    // 不符 v4d 契约致形近混淆，我方链路 106/106 全对）。
    const bool annotate = !output.empty() || stats != nullptr;
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
    if (stats != nullptr)
    {
        // live HUD：语义与被替代的第三方 live 程序一致（dets/cells/localized/fps），
        // 顶部黑底黄字，远看清晰；fps 为上一帧 EMA（首帧 0）。
        const std::string hud = cv::format("dets=%d cells=%d localized=%d fps=%.1f",
                                           stats->dets, stats->cells, stats->localized, fps_hud);
        cv::rectangle(visual, {0, 0}, {visual.cols, 34}, {0, 0, 0}, -1);
        cv::putText(visual, hud, {10, 24}, cv::FONT_HERSHEY_SIMPLEX, .75,
                    {0, 255, 255}, 2);
    }
    if (annotate && !output.empty() && !cv::imwrite(output.string(), visual))
        std::cerr << "cannot write " << output.string() << '\n';
    return annotate ? visual : cv::Mat();
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
std::string uvcPipeline(const std::string& device, int width, int height, int fps)
{
    std::ostringstream os;
    os << "v4l2src device=" << device
       << " ! video/x-raw,width=" << width << ",height=" << height;
    if (fps > 0)
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

    std::string pipeline;
    if (source.rfind("argus:", 0) == 0)
    {
        pipeline = argusPipeline(std::atoi(source.substr(6).c_str()), width, height, fps);
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
        pipeline = uvcPipeline(device, width, height, fps);
    }
    else
    {
        std::cerr << "unknown camera source: " << source
                  << " (expected argus:<sensor-id> or uvc:<device>)\n";
        return 2;
    }

    std::cout << "opening camera: " << pipeline << '\n';
    cv::VideoCapture capture(pipeline, cv::CAP_GSTREAMER);
    if (!capture.isOpened())
    {
        std::cerr << "cannot open camera; check the device node, GStreamer plugins and caps "
                     "(v4l2-ctl --list-formats-ext)\n";
        return 3;
    }

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
    bool reopened = false;       // 断流后只重开一次，防无限重试
    // ---- live HUD 可视化（2026-10-18）----
    // 开关：DISPLAY 存在即开（板上 LXDE/VNC 场景）；DZUAV_VIEW=1 强制开，
    // DZUAV_VIEW=0 强制关（无头/远程省 clone+绘制开销，退回纯 stdout 行）。
    const char* view_env = std::getenv("DZUAV_VIEW");
    const bool show = (view_env && *view_env == '1') ||
                      ((view_env == nullptr || *view_env != '0') &&
                       [] { const char* d = std::getenv("DISPLAY"); return d && *d; }());
    FrameStats stats;
    float fps_ema = 0.F;
    auto t_prev = std::chrono::steady_clock::now();
    const char* win = "dzuav live";
    if (show)
        std::cout << "live view on (window: " << win << ")\n";
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
                if (!reopened)
                {
                    reopened = true;
                    empty_frames = 0;
                    std::cerr << "camera stream lost; reopening pipeline\n";
                    capture.open(pipeline, cv::CAP_GSTREAMER);
                    if (!capture.isOpened())
                    {
                        std::cerr << "camera reopen failed\n";
                        return 4;
                    }
                }
                else
                {
                    std::cerr << "camera stream did not recover after reopen\n";
                    return 4;
                }
            }
            continue;
        }
        empty_frames = 0;
        // 单帧推理/图像处理异常不终结整个侦察进程：打印后继续；
        // 连续异常超上限（故障大概率是持久的）才非零退出。
        try
        {
            // show：带 HUD 绘制返回 visual（imshow 用）；否则纯推理走原路径
            // （不 clone，stdout 文本行照打，orin_log_capture 依赖不变）。
            cv::Mat visual;
            if (show)
                visual = processFrame(frame, hbb, cell, map, udp, {}, &stats, fps_ema);
            else
                processFrame(frame, hbb, cell, map, udp, {});
            // fps EMA（处理耗时主导，read 空转不计）
            const auto t_now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(t_now - t_prev).count();
            t_prev = t_now;
            if (dt > 1e-3)
                fps_ema = fps_ema > 0.F ? float(fps_ema * 0.9 + (1.0 / dt) * 0.1)
                                         : float(1.0 / dt);
            if (show && !visual.empty())
            {
                cv::imshow(win, visual);
                cv::waitKey(1);   // 刷新 HighGUI 窗口事件（无按键逻辑，纯显示）
            }
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
