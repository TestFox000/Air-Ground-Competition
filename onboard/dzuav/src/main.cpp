// dzuav 机载侦察程序主入口。
//
// 帧时间戳设计决定（2026-10）：使用 steady_clock（单调时钟，进程启动起算毫秒），
// 不用 system_clock——Jetson 无 RTC 电池，开机时间与 NTP 校时跳变会攻击车端
// 红靶粘性否决 TTL（world_model 用帧时间戳差值计量：时钟前跳 = 粘性否决瞬间
// 过期 = 可能误击友方；后跳 = 否决期被拉长）。协议字段仍是 uint64 ms，车端
// 只做差值运算（intel_protocol / world_model 已核对），语义不破。
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iomanip>
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

// cell_label 候选分类上限（top-K 防御）：按置信度降序只分类前 N 个。
// 误检雪崩时每个候选都要跑 TTA（约 +6~12ms），10 个误检 ≈ +60~120ms；
// 场地同一帧真实可见格号数远小于此。
constexpr size_t kMaxCellClassify = 8;
// pose hold 最大龄期（秒）：本帧定位失败时用最近一次成功位姿顶替的上限，
// 超龄即认为缓存位姿陈旧——停发比错发安全。
constexpr double kPoseHoldMaxAgeS = 0.5;
// 帧日志节流：每 5 帧一行（环境变量 DZUAV_QUIET=1 时 stdout 全静默）
constexpr uint64_t kFrameLogInterval = 5;
// 周期健康行间隔（秒）
constexpr double kHealthIntervalS = 3.0;
// 断流处理预算：空读约 20s（600 x 33ms）后才升级重开——USB 重枚举慢，
// 原 300 帧（≈10s）预算在慢枚举场景会误判。
constexpr uint64_t kEmptyBudgetFrames = 600;
// 退避重连周期：约 2s（60 x 33ms）一轮，直到 g_running 为假
constexpr uint64_t kBackoffFrames = 60;

// 帧时间戳：steady_clock 单调时钟，进程启动起算毫秒（理由见文件头注释）。
uint64_t monotonicMs()
{
    static const auto kStart = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - kStart)
                                     .count());
}

const char* className(ObjectClass type)
{
    static const char* names[] = {"mine",       "car",         "arrow",   "target_red",
                                  "cell_label", "target_blue", "obstacle"};
    return names[static_cast<int>(type)];
}

// 每帧处理结果统计（runCamera 聚合成周期健康行；离线分支忽略）
struct FrameStats
{
    bool localized = false;    // 本帧 solve 成功（不含 pose hold）
    bool held = false;         // 本帧用缓存位姿顶替（pose hold）
    int landmarks = 0;         // 参与定位的地标数
    int objects = 0;           // 实际写入 0x4001 的目标数
    int filtered = 0;          // project 有限性/量程过滤丢弃的目标数
    bool car_pose_ok = false;  // 0x4002 已下发
    bool car_pose_skip = false;   // 定位有效但车朝向未下发（无车/退化/被过滤）
};

// 车头朝向估计：车顶贴红色方向标记（车头一侧），标记质心相对车框中心的
// 偏移方向即车头方向。图像方向经 map.project 投到场地世界坐标后取 atan2。
// 返回 false 表示这一帧估不出来（没找到标记 / 面积太小 / 退化），调用方跳过即可。
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
    if (!wm || !wc)
        return false;   // 投影点非有限/超界：朝向不可信
    // 退化防护：标记质心与车框中心的世界偏移太小（接近重合）时，
    // atan2(~0, ~0) 会得到随机朝向，而车端对 0x4002 无条件采信——宁可不发。
    if (std::hypot(wm->x - wc->x, wm->y - wc->y) < 0.05)
        return false;
    out_x = wc->x;
    out_y = wc->y;
    out_yaw = std::atan2(float(wm->y - wc->y), float(wm->x - wc->x));
    // 输出三元有限性兜底（project 已保证有限，这里防未来改动的回归）
    return std::isfinite(out_x) && std::isfinite(out_y) && std::isfinite(out_yaw);
}

void processFrame(const cv::Mat& frame, HbbDetector& hbb, CellClassifier& cell, MapLocalizer& map,
                  UdpBroadcaster& udp, const std::filesystem::path& output, FrameStats& stats,
                  bool log_line)
{
    cv::Mat image = map.undistort(frame);
    const auto detections = hbb.infer(image);
    // cell_label 候选 top-K 防御：按置信度降序，只分类前 kMaxCellClassify 个
    std::vector<const Detection*> cell_candidates;
    for (const auto& d : detections)
        if (d.type == ObjectClass::CellLabel)
            cell_candidates.push_back(&d);
    std::sort(cell_candidates.begin(), cell_candidates.end(),
              [](const Detection* a, const Detection* b) { return a->confidence > b->confidence; });
    if (cell_candidates.size() > kMaxCellClassify)
        cell_candidates.resize(kMaxCellClassify);
    std::vector<std::pair<CellId, cv::Point2f>> landmarks;
    for (const Detection* detection : cell_candidates)
    {
        const auto roi = cv::Rect2f(detection->box) & cv::Rect2f(0, 0, image.cols, image.rows);
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
    bool localized = map.solve(landmarks, image.size());
    stats.localized = localized;
    // pose hold：本帧定位失败但最近一次成功定位龄期未超时，用缓存位姿投影
    // 当前帧检测继续发送（态势不中断；hold 计数进健康行）
    if (!localized && map.hold(kPoseHoldMaxAgeS))
    {
        localized = true;
        stats.held = true;
    }
    stats.landmarks = static_cast<int>(landmarks.size());
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
            if (!world)
            {
                // 非有限/超场地量程的投影：丢弃并计数（健康行可见），不进 0x4001
                ++stats.filtered;
            }
            else
            {
                // 注意：length/width 字段填的是像素框宽/高，并非契约所写的"米"
                // （车端 Python 目前忽略这两个字段，无实害；契约待更新，字节布局不变）。
                objects.push_back({detection.type,
                                   cv::Point3f(float(world->x), float(world->y), 0.F),
                                   static_cast<float>(detection.box.width),
                                   static_cast<float>(detection.box.height), detection.confidence});
                if (annotate)
                    label += cv::format(" (%.3f,%.3f)", world->x, world->y);
            }
        }
        if (annotate)
            cv::putText(visual, label, {int(detection.box.x), std::max(18, int(detection.box.y) - 5)},
                        cv::FONT_HERSHEY_SIMPLEX, .55, cv::Scalar(0, 0, 255), 2);
    }
    stats.objects = static_cast<int>(objects.size());
    const uint64_t timestamp = monotonicMs();
    // 绝不发空 0x4001 心跳：车端整帧替换快照，空帧会清空已有记忆。hold 帧意味着
    // "定位暂时丢失、拿旧位姿顶着"，若此时连可投影目标都没有，发空帧等于把车端
    // 记忆的雷/靶抹掉——空 hold 帧停发。（正常 localized 空帧维持原行为：
    // 那是"看清楚了场上确实没东西"。）
    if ((localized && (!stats.held || !objects.empty())) || std::getenv("DZUAV_FORCE_UDP"))
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
        else
            stats.car_pose_skip = true;   // 定位有效但朝向未下发（无车/退化/被过滤）
    }
    stats.car_pose_ok = car_pose_ok;
    if (log_line)
    {
        // 帧行格式与 tools/orin_log_capture.py parse_line 严格对应：
        //   "<out> localized=<0|1> landmarks=<N> [{...}] [hold=1] [car_pose=(x,y,yaw=z)]"
        // hold=1 追加在 ] 之后：不影响 localized=/landmarks= 正则与 [] 括号深度匹配。
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
        if (stats.held)
            std::cout << " hold=1";
        if (car_pose_ok)
            std::cout << " car_pose=(" << car_x << ',' << car_y << ",yaw=" << car_yaw << ')';
        std::cout << "\n";   // 不用 endl：省一次流刷新（30fps 下可观）
    }
    if (annotate && !cv::imwrite(output.string(), visual))
        std::cerr << "cannot write " << output.string() << '\n';
}

// 周期健康行的聚合窗口（每 kHealthIntervalS 秒出一行，归零重计）
struct HealthWindow
{
    uint64_t frames = 0;       // 窗口内处理帧数（fps 分母）
    uint64_t localized = 0;    // solve 成功帧数（不含 pose hold）
    uint64_t hold = 0;         // pose hold 顶替帧数
    uint64_t landmarks = 0;    // 地标总数（求均值）
    uint64_t objects = 0;      // 入帧目标总数（求均值）
    uint64_t car_ok = 0;       // 0x4002 下发帧数
    uint64_t car_skip = 0;     // 定位有效但未下车朝向帧数
    uint64_t filtered = 0;     // project 过滤目标数
    uint64_t pnp = 0;          // solve 路径计数（pnp / homography / none）
    uint64_t homography = 0;
    uint64_t none = 0;
    uint64_t inliers = 0;      // 成功帧内点数总和（求均值）
    float reproj_med = 0.F;    // 窗口内最后一帧的重投影中位误差（px）
};

// 周期健康行：单行 JSON 对象 {"stats":{...}}，与帧行同属一行一条的结构化输出。
// 字段名刻意避开 "localized="/"landmarks=" 字样：orin_log_capture 只解析带
// "localized=N landmarks=N" 的帧行，健康行不匹配该模式，被原样镜像为 ros.log
// 事件——不破坏任何现存解析。
void printHealth(const HealthWindow& w, double seconds, uint64_t empty_total,
                 const UdpBroadcaster& udp)
{
    const double fps = seconds > 0 ? w.frames / seconds : 0.0;
    const double loc_pct = w.frames ? 100.0 * w.localized / w.frames : 0.0;
    const double lm_avg = w.frames ? double(w.landmarks) / w.frames : 0.0;
    const double objs_avg = w.frames ? double(w.objects) / w.frames : 0.0;
    const uint64_t solved = w.pnp + w.homography;
    const double inliers_avg = solved ? double(w.inliers) / solved : 0.0;
    std::ostringstream os;
    os << std::fixed << "{\"stats\":{\"fps\":" << std::setprecision(1) << fps
       << ",\"loc_pct\":" << std::setprecision(1) << loc_pct
       << ",\"path_pnp\":" << w.pnp << ",\"path_hom\":" << w.homography
       << ",\"path_none\":" << w.none
       << ",\"inliers_avg\":" << std::setprecision(1) << inliers_avg
       << ",\"reproj_med\":" << std::setprecision(2) << w.reproj_med
       << ",\"lm_avg\":" << std::setprecision(1) << lm_avg
       << ",\"objs_avg\":" << std::setprecision(1) << objs_avg
       << ",\"car_ok\":" << w.car_ok << ",\"car_skip\":" << w.car_skip
       << ",\"hold\":" << w.hold << ",\"filtered\":" << w.filtered
       << ",\"udp_drop\":" << udp.filteredCount()
       << ",\"udp_fail\":" << udp.sendFailures()
       << ",\"empty\":" << empty_total << ",\"frames\":" << w.frames << "}}";
    std::cout << os.str() << "\n";
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

// 解析 "1920x1080@30"；fps=0 表示不约束协商帧率。解析失败时保留调用方默认值并警告
// （静默回落曾让 "1920x1080@3O" 这类手滑无迹可寻）。
void parseSpec(const std::string& spec, int& width, int& height, int& fps)
{
    int w = 0, h = 0, f = 0;
    if (std::sscanf(spec.c_str(), "%dx%d@%d", &w, &h, &f) == 3 && w > 0 && h > 0 && f >= 0)
    {
        width = w;
        height = h;
        fps = f;
    }
    else
    {
        std::cerr << "cannot parse camera spec '" << spec
                  << "' (expect WxH@FPS); keeping previous defaults\n";
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

    // pipeline 由 (宽,高,帧率) 参数化生成：断流降规格重协商要用另一组参数重建
    std::function<std::string(int, int, int)> make_pipeline;
    if (source.rfind("argus:", 0) == 0)
    {
        const std::string id_text = source.substr(6);
        // atoi 对非数字输入静默返回 0（"argus:abc" -> sensor 0）：显式检查并警告
        if (id_text.empty() ||
            !std::all_of(id_text.begin(), id_text.end(),
                         [](char ch) { return std::isdigit(static_cast<unsigned char>(ch)); }))
            std::cerr << "invalid argus sensor-id '" << id_text << "'; falling back to 0\n";
        const int sensor_id = std::atoi(id_text.c_str());
        make_pipeline = [sensor_id](int w, int h, int f)
        { return argusPipeline(sensor_id, w, h, f); };
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
        make_pipeline = [device](int w, int h, int f) { return uvcPipeline(device, w, h, f); };
    }
    else
    {
        std::cerr << "unknown camera source: " << source
                  << " (expected argus:<sensor-id> or uvc:<device>)\n";
        return 2;
    }

    const std::string pipeline = make_pipeline(width, height, fps);
    // 降规格重协商管线（1280x720@15）：USB 重枚举后原高规格可能协商不上
    const std::string degraded_pipeline = make_pipeline(1280, 720, 15);

    std::cout << "opening camera: " << pipeline << '\n';
    cv::VideoCapture capture(pipeline, cv::CAP_GSTREAMER);
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

    const bool quiet = std::getenv("DZUAV_QUIET") != nullptr;   // stdout 全静默（比赛省带宽）
    cv::Mat frame;
    uint64_t frame_index = 0;
    uint64_t empty_frames = 0;
    uint64_t empty_total = 0;    // 累计空读（健康行可见）
    unsigned frame_errors = 0;   // 连续处理异常计数（成功即清零）
    bool reconnecting = false;   // 断流退避重连中
    HealthWindow health;
    auto health_start = std::chrono::steady_clock::now();
    while (g_running.load())
    {
        if (!capture.read(frame) || frame.empty())
        {
            ++empty_frames;
            ++empty_total;
            if (empty_frames < 30)
                std::cerr << "empty frame\n";
            // 断流空转防护：让出 CPU（约一拍帧间隔），否则 read 立即 false 会
            // 100% 占核永不退出。
            usleep(33000);
            // 断流恢复：预算约 20s（USB 重枚举慢）。现场 3 分钟比赛没有
            // "退出进程"这个容错选项——重开失败降级规格，仍失败退避重连到永远。
            const uint64_t budget = reconnecting ? kBackoffFrames : kEmptyBudgetFrames;
            if (empty_frames >= budget)
            {
                if (!reconnecting)
                {
                    std::cerr << "camera stream lost; reopening pipeline\n";
                    capture.open(pipeline, cv::CAP_GSTREAMER);
                    if (!capture.isOpened())
                    {
                        // 原规格重开失败：降规格 1280x720@15 重协商再试
                        std::cerr << "reopen failed; renegotiating degraded 1280x720@15\n";
                        capture.open(degraded_pipeline, cv::CAP_GSTREAMER);
                    }
                    if (capture.isOpened())
                    {
                        std::cerr << "camera reopened\n";
                    }
                    else
                    {
                        reconnecting = true;
                        std::cerr << "camera unavailable; retrying every ~2s (Ctrl+C to stop)\n";
                    }
                }
                else
                {
                    // 退避重连：每轮先原规格后降规格，任一成功即恢复
                    capture.open(pipeline, cv::CAP_GSTREAMER);
                    if (!capture.isOpened())
                        capture.open(degraded_pipeline, cv::CAP_GSTREAMER);
                    if (capture.isOpened())
                    {
                        reconnecting = false;
                        std::cerr << "camera recovered\n";
                    }
                }
                empty_frames = 0;
            }
            continue;
        }
        empty_frames = 0;
        // 单帧推理/图像处理异常不终结整个侦察进程：打印后继续；
        // 连续异常超上限（故障大概率是持久的）才非零退出。
        try
        {
            FrameStats stats;
            processFrame(frame, hbb, cell, map, udp, {}, stats,
                         !quiet && frame_index % kFrameLogInterval == 0);
            // 聚合健康窗口（字段口径见 HealthWindow / printHealth）
            ++health.frames;
            if (stats.localized)
                ++health.localized;
            if (stats.held)
                ++health.hold;
            health.landmarks += static_cast<uint64_t>(stats.landmarks);
            health.objects += static_cast<uint64_t>(stats.objects);
            health.filtered += static_cast<uint64_t>(stats.filtered);
            if (stats.car_pose_ok)
                ++health.car_ok;
            else if (stats.car_pose_skip)
                ++health.car_skip;
            const auto& solve_info = map.lastSolve();
            if (solve_info.path == 1)
                ++health.pnp;
            else if (solve_info.path == 2)
                ++health.homography;
            else
                ++health.none;
            if (solve_info.path != 0)
                health.inliers += static_cast<uint64_t>(solve_info.inliers);
            health.reproj_med = solve_info.median_px;
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
        // 周期健康行（ quiet 时只重置窗口不输出）
        const auto now = std::chrono::steady_clock::now();
        const double health_elapsed = std::chrono::duration<double>(now - health_start).count();
        if (health_elapsed >= kHealthIntervalS)
        {
            if (!quiet)
                printHealth(health, health_elapsed, empty_total, udp);
            health = HealthWindow{};
            health_start = now;
        }
    }
    std::cout << "stopped after " << frame_index << " processed frames\n";
    return 0;
}
}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    // 下游日志采集器（orin_log_capture）先退出时 stdout 成断管，默认 SIGPIPE
    // 会直接杀掉进程；忽略之——写失败仅置流错误位，侦察主循环不受影响。
    std::signal(SIGPIPE, SIG_IGN);

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
    HbbDetector hbb(argv[2], .25F, .45F);
    CellClassifier cell(argv[3]);
    MapLocalizer map;
    UdpBroadcaster udp(port);
    const auto outputFor = [&](size_t index)
    { return output_dir / (input.stem().string() + "_" + std::to_string(index) + ".jpg"); };
    cv::Mat image = cv::imread(input.string());
    if (!image.empty())
    {
        // 单帧异常不 terminate：离线模式打印后非零退出（这一帧就是全部工作）
        try
        {
            FrameStats stats;
            processFrame(image, hbb, cell, map, udp, outputFor(0), stats, true);
        }
        catch (const std::exception& e)
        {
            std::cerr << "frame processing error: " << e.what() << '\n';
            return 1;
        }
        return 0;
    }
    cv::VideoCapture capture(input.string());
    if (!capture.isOpened())
        return 3;
    cv::Mat frame;
    for (size_t index = 0; capture.read(frame); ++index)
    {
        // 单帧异常跳过继续：离线批处理不应被一帧坏数据 terminate 整批
        try
        {
            FrameStats stats;
            processFrame(frame, hbb, cell, map, udp, outputFor(index), stats, true);
        }
        catch (const std::exception& e)
        {
            std::cerr << "frame " << index << " processing error: " << e.what() << '\n';
        }
    }
    return 0;
}
