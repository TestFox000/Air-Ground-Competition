// 13 类主链 + 机载规划层（D13 检测 → PnP 定位 → CellGrid 落格 → RoutePlanner）
//
// 语义与 Python 版 ag_integrate 保持一致（算法级移植，不引入第二套血缘）：
//   * paper 在桥洞列（K 列）= 可通行通道；其余 paper = 雷
//   * car_start / car_end = 通格（路线起点/终点）
//   * target_red / target_blue = 靶（占相邻两格，规划禁入）
//   * obstacle / bucket_* = 障碍
//   * car / uav_start / uav_end / cell_label 不进矩阵（车/无人机标记与定位地标）
//
// 规划产物（给车的路线）：route_plan.json；可选 planned_path.txt（px4_path 兼容）。
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
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "dzuav/CellClassifier.hpp"
#include "dzuav/CellGrid.hpp"
#include "dzuav/HbbDetector.hpp"
#include "dzuav/MapLocalizer.hpp"
#include "dzuav/RoutePlanner.hpp"
#include "dzuav/UdpBroadcaster.hpp"

using namespace dzuav;

namespace
{
// —— 场地固定配置（官方场地：桥洞通道在 K 列，两处通道 K4/K9 所在列）——
constexpr char kBridgeCol = 'K';
// 竖放靶面板顶端离地高度（占上下相邻两格时用于求面板朝向）。
constexpr float kTargetPanelHeight = 0.7F;

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

// YOLO 类别 -> 矩阵占地物；返回 nullopt 表示不进矩阵。
// paper 的雷/通行语义依赖落格列（K 列=桥洞箭头），故由调用方传入。
std::optional<CellContent> contentOf(ObjectClass type, bool in_bridge_column)
{
    switch (type)
    {
    case ObjectClass::Paper:
        return in_bridge_column ? CellContent::Passage : CellContent::Mine;
    case ObjectClass::Obstacle:
    case ObjectClass::BucketRed:
    case ObjectClass::BucketGreen:
    case ObjectClass::BucketCyan:
        return CellContent::Obstacle;
    case ObjectClass::TargetRed:
    case ObjectClass::TargetBlue:
        return CellContent::Target;
    case ObjectClass::CarStart:
    case ObjectClass::CarEnd:
        return CellContent::Passage;
    default:   // cell_label / car / uav_start / uav_end
        return std::nullopt;
    }
}

// 车头朝向估计：车顶贴红色方向标记（车头一侧），标记质心相对车框中心的
// 偏移方向即车头方向。图像方向经 map.project 投到场地世界坐标后取 atan2。
bool estimateCarYaw(const cv::Mat& image, const cv::Rect& box, MapLocalizer& map,
                    double& out_x, double& out_y, float& out_yaw)
{
    const cv::Rect roi = box & cv::Rect(0, 0, image.cols, image.rows);
    if (roi.area() <= 16)
        return false;
    cv::Mat hsv;
    cv::cvtColor(image(roi), hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask, mask2;
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask);
    cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask2);
    mask |= mask2;
    const cv::Moments m = cv::moments(mask, true);
    if (m.m00 < 50)
        return false;
    const cv::Point2f marker(roi.x + float(m.m10 / m.m00), roi.y + float(m.m01 / m.m00));
    const cv::Point2f center(roi.x + roi.width * .5F, roi.y + roi.height * .5F);
    const auto wm = map.project(marker);
    const auto wc = map.project(center);
    out_x = wc.x;
    out_y = wc.y;
    out_yaw = std::atan2(float(wm.y - wc.y), float(wm.x - wc.x));
    return true;
}

// 矩阵输出节流：内容变化时打印/落盘，间隔 ≥ 1s。
struct MatrixOutput
{
    std::filesystem::path jsonPath;
    uint64_t lastVersion = ~0ull;
    int64_t lastPrintMs = 0;
};

void flushMatrix(const CellGrid& grid, MatrixOutput& out, int64_t nowMs)
{
    out.lastVersion = grid.version();
    out.lastPrintMs = nowMs;
    std::cout << grid.render() << '\n';
    if (!out.jsonPath.empty())
    {
        std::ofstream file(out.jsonPath);
        file << grid.toJson(nowMs) << '\n';
    }
}

void maybeFlushMatrix(const CellGrid& grid, MatrixOutput& out, int64_t nowMs)
{
    if (grid.version() != out.lastVersion &&
        (nowMs - out.lastPrintMs >= 1000 || out.lastVersion == ~0ull))
        flushMatrix(grid, out, nowMs);
}

// 路线规划输出：矩阵内容每变一次就重规划；路线文本变了才打印+落盘。
struct PlanOutput
{
    std::filesystem::path routeJsonPath;
    std::filesystem::path pathFilePath;
    uint64_t lastPlannedVersion = ~0ull;
    std::string lastRouteText;
    std::string lastError;
};

void maybePlan(const CellGrid& grid, RoutePlanner& planner, PlanOutput& out, int64_t nowMs)
{
    if (grid.version() == out.lastPlannedVersion)
        return;
    out.lastPlannedVersion = grid.version();
    const RoutePlan plan = planner.plan(grid);
    if (!plan.ok)
    {
        if (plan.error != out.lastError)
        {
            out.lastError = plan.error;
            std::cerr << "规划失败: " << plan.error << '\n';
        }
        return;
    }
    const std::string text = plan.cellsText();
    if (text == out.lastRouteText)
        return;
    out.lastRouteText = text;
    out.lastError.clear();
    std::cout << "=== 路线规划 ===\n"
              << "通: " << plan.cells.front().text() << " -> " << plan.cells.back().text()
              << "   雷: " << plan.minesVisited << "/" << plan.minesTotal << "   格数: "
              << plan.cells.size() << "   长度: "
              << (MapLocalizer::kCellSize * (plan.cells.size() - 1)) << " m\n"
              << text << '\n';
    for (const auto& warning : plan.warnings)
        std::cout << "警告: " << warning << '\n';
    if (!out.routeJsonPath.empty())
    {
        std::ofstream file(out.routeJsonPath);
        file << plan.toJson(nowMs) << '\n';
    }
    if (!out.pathFilePath.empty())
    {
        if (writePathFile(plan, out.pathFilePath.string()))
            std::cout << "px4_path 文件已写: " << out.pathFilePath << '\n';
        else
            std::cerr << "写 px4_path 文件失败: " << out.pathFilePath << '\n';
    }
}

// 官方固定起终点（现场摆位常量，默认 N10 → N5，可用环境变量覆盖）：
//   DZUAV_START_CELL=N10  DZUAV_GOAL_CELL=N5
// 视频未拍到 car_start/car_end 时仍能出路线；拍到也不会被桥洞通格顶替。
RoutePlanner makePlanner()
{
    RoutePlanner::Options options;
    const char* start = std::getenv("DZUAV_START_CELL");
    const char* goal = std::getenv("DZUAV_GOAL_CELL");
    options.startCell = (start && *start) ? start : "N10";
    options.endCell = (goal && *goal) ? goal : "N5";
    return RoutePlanner(options);
}

void updateGrid(const std::vector<Detection>& detections,
                const std::vector<std::pair<CellId, cv::Point2f>>& landmarks, MapLocalizer& map,
                CellGrid& grid)
{
    for (const auto& detection : detections)
    {
        if (detection.type == ObjectClass::CellLabel)
            continue;
        const float weight = detection.confidence * 2.F;

        if (detection.type == ObjectClass::TargetRed ||
            detection.type == ObjectClass::TargetBlue)
        {
            const cv::Point2f bottomCenter{detection.box.x + detection.box.width * .5F,
                                           float(detection.box.y + detection.box.height)};
            const cv::Point2f topCenter{detection.box.x + detection.box.width * .5F,
                                        float(detection.box.y)};
            const cv::Point2d base = map.project(bottomCenter);
            const cv::Point2d top = map.projectAtHeight(topCenter, kTargetPanelHeight);
            const bool have_height = top.x > -1e8;   // 未 PnP 时返回 -1e9
            const cv::Point2d span = top - base;
            bool marked = false;
            if (have_height && cv::norm(span) > 0.25)
            {
                // 竖放：基底格 + 沿靶面方向的相邻格。
                const cv::Point2d dir = span / cv::norm(span);
                const CellId cells[2] = {map.cellAt(base),
                                         map.cellAt(base + dir * MapLocalizer::kCellSize)};
                for (const auto& id : cells)
                    if (id.valid())
                    {
                        grid.observe(id.column - 'A', id.row - 1, CellContent::Target, weight);
                        marked = true;
                    }
            }
            if (!marked)
            {
                // 横放 / 无 PnP 高度信息：底边两端点覆盖两个格子。
                const cv::Point2f bl{float(detection.box.x),
                                     float(detection.box.y + detection.box.height)};
                const cv::Point2f br{float(detection.box.x + detection.box.width),
                                     float(detection.box.y + detection.box.height)};
                for (const auto& p : {bl, br})
                {
                    const CellId id = map.cellAt(map.project(p));
                    if (id.valid())
                        grid.observe(id.column - 'A', id.row - 1, CellContent::Target, weight);
                }
            }
            continue;
        }

        const cv::Point2f bottomCenter{detection.box.x + detection.box.width * .5F,
                                       float(detection.box.y + detection.box.height)};
        const CellId id = map.cellAt(map.project(bottomCenter));
        if (!id.valid())
            continue;
        const auto content = contentOf(detection.type, id.column == kBridgeCol);
        if (content)
            grid.observe(id.column - 'A', id.row - 1, *content, weight);
    }
    // 编号直接识别确认（供矩阵打印/调试）。
    for (const auto& landmark : landmarks)
        if (landmark.first.valid())
            grid.confirmId(landmark.first.column - 'A', landmark.first.row - 1,
                           landmark.first.confidence);
}

void processFrame(const cv::Mat& frame, HbbDetector& hbb, CellClassifier& cell, MapLocalizer& map,
                  CellGrid& grid, UdpBroadcaster& udp, MatrixOutput& matrixOut,
                  RoutePlanner& planner, PlanOutput& planOut,
                  const std::filesystem::path& output)
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
        if (roi.x < 8.f || roi.y < 8.f ||
            roi.x + roi.width > static_cast<float>(image.cols) - 8.f ||
            roi.y + roi.height > static_cast<float>(image.rows) - 8.f)
            continue;
        const CellId id = cell.infer(image(roi));
        if (id.valid() && id.confidence > .45F)
            landmarks.push_back({id, {roi.x + roi.width * .5F, roi.y + roi.height * .5F}});
    }
    const bool localized = map.solve(landmarks, image.size());
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
    if (localized)
        updateGrid(detections, landmarks, map, grid);
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
    if (localized || std::getenv("DZUAV_FORCE_UDP"))
        udp.send(timestamp, objects);
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
    std::cout << "\n";
    if (annotate && !cv::imwrite(output.string(), visual))
        std::cerr << "cannot write " << output.string() << '\n';

    maybeFlushMatrix(grid, matrixOut, timestamp);
    maybePlan(grid, planner, planOut, timestamp);
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
              const std::string& cell_path, uint16_t port, const std::string& spec,
              const std::filesystem::path& matrix_json,
              const std::filesystem::path& route_json,
              const std::filesystem::path& path_file)
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
    CellGrid grid;
    RoutePlanner planner = makePlanner();
    UdpBroadcaster udp(port);
    MatrixOutput matrixOut{matrix_json};
    PlanOutput planOut{route_json, path_file};

    cv::Mat frame;
    uint64_t frame_index = 0;
    uint64_t empty_frames = 0;
    unsigned frame_errors = 0;
    bool reopened = false;
    while (g_running.load())
    {
        if (!capture.read(frame) || frame.empty())
        {
            ++empty_frames;
            if (empty_frames < 30)
                std::cerr << "empty frame\n";
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
        try
        {
            processFrame(frame, hbb, cell, map, grid, udp, matrixOut, planner, planOut, {});
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
            << "  dzuav <image|video> <hbb_engine> <cell_engine> [udp_port] [output_dir] "
               "[matrix_json] [route_json] [path_file]\n"
            << "  dzuav argus:<sensor-id> <hbb_engine> <cell_engine> [udp_port] [WxH@FPS] "
               "[matrix_json] [route_json] [path_file]\n"
            << "  dzuav uvc:<device>  <hbb_engine> <cell_engine> [udp_port] [WxH@FPS] "
               "[matrix_json] [route_json] [path_file]\n"
            << "examples:\n"
            << "  dzuav uvc:/dev/video0 engines/hbb_detector_d13_fp16.engine "
               "engines/cell_classifier_v4d_fp16.engine 50000 1280x720@30 "
               "/tmp/cell_matrix.json /tmp/route_plan.json /tmp/planned_path.txt\n";
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
        const std::filesystem::path matrix_json = argc > 6 ? argv[6] : "";
        const std::filesystem::path route_json = argc > 7 ? argv[7] : "";
        const std::filesystem::path path_file = argc > 8 ? argv[8] : "";
        return runCamera(source, argv[2], argv[3], port, spec, matrix_json, route_json,
                         path_file);
    }

    // 图片 / 视频文件分支（离线分析）。
    const std::filesystem::path input(argv[1]);
    const std::filesystem::path output_dir = argc > 5 ? argv[5] : "";
    const std::filesystem::path matrix_json = argc > 6 ? argv[6] : "";
    const std::filesystem::path route_json = argc > 7 ? argv[7] : "";
    const std::filesystem::path path_file = argc > 8 ? argv[8] : "";
    if (!output_dir.empty())
        std::filesystem::create_directories(output_dir);

    std::array<float, kObjectClassCount> nms_per_class;
    nms_per_class.fill(kDeployNms);
    HbbDetector hbb(argv[2], kDeployConf, nms_per_class, kEdgePaperThr);
    CellClassifier cell(argv[3]);
    MapLocalizer map;
    CellGrid grid;
    RoutePlanner planner = makePlanner();
    UdpBroadcaster udp(port);
    MatrixOutput matrixOut{matrix_json};
    PlanOutput planOut{route_json, path_file};
    const auto outputFor = [&](size_t index) -> std::filesystem::path
    {
        // output_dir 为空 = 不要标注图：返回空路径，processFrame 会跳过 imwrite
        // （旧行为会往当前目录写每帧 jpg，实测在家目录堆了上千张）。
        if (output_dir.empty())
            return {};
        return output_dir / (input.stem().string() + "_" + std::to_string(index) + ".jpg");
    };

    cv::Mat image = cv::imread(input.string());
    if (!image.empty())
    {
        processFrame(image, hbb, cell, map, grid, udp, matrixOut, planner, planOut,
                     outputFor(0));
        return 0;
    }
    cv::VideoCapture capture(input.string());
    if (!capture.isOpened())
        return 3;
    // 可选抽帧（DZUAV_STRIDE，默认 1=逐帧；离线长视频提速用，实时模式不受影响）
    int stride = 1;
    if (const char* s = std::getenv("DZUAV_STRIDE"))
    {
        const int v = std::atoi(s);
        if (v > 0)
            stride = v;
    }
    cv::Mat frame;
    size_t read_index = 0, processed = 0;
    while (capture.read(frame))
    {
        if (stride > 1 && read_index % static_cast<size_t>(stride) != 0)
        {
            ++read_index;
            continue;
        }
        processFrame(frame, hbb, cell, map, grid, udp, matrixOut, planner, planOut,
                     outputFor(processed));
        ++processed;
        ++read_index;
    }
    return 0;
}
