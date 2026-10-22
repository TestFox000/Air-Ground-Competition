#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "dzuav/CellClassifier.hpp"
#include "dzuav/CellGrid.hpp"
#include "dzuav/HbbDetector.hpp"
#include "dzuav/MapLocalizer.hpp"
#include "dzuav/PassageDetector.hpp"
#include "dzuav/RoutePlanner.hpp"
#include "dzuav/UdpBroadcaster.hpp"

using namespace dzuav;

namespace
{
// —— 场地固定配置 ——
// 桥体占据第 7 行 D~N 列。桥体格不参与“通/空”绿色投票，避免桥面被误判为通格。
constexpr int kBridgeRow = 7;  // 1-based
constexpr char kBridgeColStart = 'D';
constexpr char kBridgeColEnd = 'N';
// “通”格绿色检测阈值：格内绿色像素占比 ≥ 此值记一次通票；≤ 下限记一次空票。
constexpr float kPassageGreenRatio = 0.45F;
constexpr float kEmptyGreenRatio = 0.05F;
// 竖放靶面板顶端离地高度（占上下相邻两格时用于求面板朝向）。
constexpr float kTargetPanelHeight = 0.7F;

std::atomic<bool> g_running{true};

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

// YOLO 类别 -> 矩阵占地物；返回 nullopt 表示不进矩阵（车、箭头、格号）。
std::optional<CellContent> contentOf(ObjectClass type)
{
    switch (type)
    {
    case ObjectClass::Mine:
        return CellContent::Mine;
    case ObjectClass::Obstacle:
        return CellContent::Obstacle;
    case ObjectClass::TargetRed:
    case ObjectClass::TargetBlue:
        return CellContent::Target;
    default:
        return std::nullopt;
    }
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
    std::filesystem::path pathFilePath;  // px4_path 兼容文件（FAST-LIO SLAM 帧）
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

void updateGrid(const cv::Mat& image, const std::vector<Detection>& detections,
                const std::vector<std::pair<CellId, cv::Point2f>>& landmarks, MapLocalizer& map,
                CellGrid& grid)
{
    // ① 占地物落格：雷/障取框底边中点（贴地）；靶按朝向标记占用的两个相邻格。
    for (const auto& detection : detections)
    {
        const auto content = contentOf(detection.type);
        if (!content)
            continue;
        const float weight = detection.confidence * 2.F;
        if (*content == CellContent::Target)
        {
            const cv::Point2f bottomCenter{detection.box.x + detection.box.width * .5F,
                                           float(detection.box.y + detection.box.height)};
            const cv::Point2f topCenter{detection.box.x + detection.box.width * .5F,
                                        float(detection.box.y)};
            const cv::Point2d base = map.project(bottomCenter);
            const cv::Point2d top = map.projectAtHeight(topCenter, kTargetPanelHeight);
            const cv::Point2d span = top - base;
            if (cv::norm(span) > 0.25)
            {
                // 竖放：基底格 + 沿靶面方向的相邻格。
                const cv::Point2d dir = span / cv::norm(span);
                const CellId cells[2] = {map.cellAt(base),
                                         map.cellAt(base + dir * MapLocalizer::kCellSize)};
                for (const auto& id : cells)
                    if (id.valid())
                        grid.observe(id.column - 'A', id.row - 1, CellContent::Target, weight);
            }
            else
            {
                // 横放：底边两端点覆盖两个格子。
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
        }
        else
        {
            const cv::Point2f bottomCenter{detection.box.x + detection.box.width * .5F,
                                           float(detection.box.y + detection.box.height)};
            const CellId id = map.cellAt(map.project(bottomCenter));
            if (id.valid())
                grid.observe(id.column - 'A', id.row - 1, *content, weight);
        }
    }
    // ② 通/空：格子四边形内绿色像素占比投票（跳过桥体格）。
    const cv::Mat greenMask = makeGreenMask(image);
    for (int row = 1; row <= CellGrid::kRows; ++row)
        for (char column = 'A'; column <= 'N'; ++column)
        {
            const int col = column - 'A', r = row - 1;
            if (grid.entry(col, r).onBridge)
                continue;
            const CellId id{column, row, 0};
            cv::Point2f quad[4];
            if (!map.projectCellQuad(id, quad))
                continue;
            float ratio = 0;
            if (!greenRatio(greenMask, quad, ratio))
                continue;
            if (ratio >= kPassageGreenRatio)
                grid.observe(col, r, CellContent::Passage, ratio * 1.5F);
            else if (ratio <= kEmptyGreenRatio)
                grid.observe(col, r, CellContent::Empty, .25F);
        }
    // ③ 编号直接识别确认。
    for (const auto& landmark : landmarks)
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
        const CellId id = cell.infer(image(roi));
        if (id.valid() && id.confidence > .55F)
            landmarks.push_back(
                {id, cv::Point2f(roi.x + roi.width * .5F, roi.y + roi.height * .5F)});
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
            objects.push_back({detection.type,
                               cv::Point3f(float(world.x), float(world.y), 0.F),
                               float(detection.box.width), float(detection.box.height),
                               detection.confidence});
            if (annotate)
                label += cv::format(" (%.3f,%.3f)", world.x, world.y);
        }
        if (annotate)
            cv::putText(visual, label, {int(detection.box.x), std::max(18, int(detection.box.y) - 5)},
                        cv::FONT_HERSHEY_SIMPLEX, .55, cv::Scalar(0, 0, 255), 2);
    }
    if (localized)
        updateGrid(image, detections, landmarks, map, grid);
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
    if (localized || std::getenv("DZUAV_FORCE_UDP"))
        udp.send(timestamp, objects);
    std::cout << output.filename() << " localized=" << localized
              << " landmarks=" << landmarks.size() << " [";
    for (size_t i = 0; i < objects.size(); ++i)
    {
        const auto& object = objects[i];
        std::cout << (i ? "," : "") << "{\"class\":\"" << className(object.type)
                  << "\",\"center\":[" << object.position.x << ',' << object.position.y
                  << "],\"length\":" << object.length << ",\"width\":" << object.width << '}';
    }
    std::cout << "]\n";  // 不用 endl：省一次流刷新（30fps 下可观）
    if (annotate)
        cv::imwrite(output.string(), visual);
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

CellGrid makeGrid()
{
    CellGrid grid;
    for (char column = kBridgeColStart; column <= kBridgeColEnd; ++column)
        grid.setBridge(column - 'A', kBridgeRow - 1);
    return grid;
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
        pipeline = argusPipeline(std::atoi(source.substr(6).c_str()), width, height, fps);
    else if (source.rfind("uvc:", 0) == 0)
    {
        std::string device = source.substr(4);
        if (!device.empty() &&
            std::all_of(device.begin(), device.end(), [](char ch) { return std::isdigit(ch); }))
            device = "/dev/video" + device;
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

    HbbDetector hbb(hbb_path, .25F, .45F);
    CellClassifier cell(cell_path);
    MapLocalizer map;
    CellGrid grid = makeGrid();
    UdpBroadcaster udp(port);
    MatrixOutput matrixOut{matrix_json};
    RoutePlanner planner;
    PlanOutput planOut{route_json, path_file};

    cv::Mat frame;
    uint64_t frame_index = 0;
    uint64_t empty_frames = 0;
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
        processFrame(frame, hbb, cell, map, grid, udp, matrixOut, planner, planOut, {});
        ++frame_index;
    }
    std::cout << "stopped after " << frame_index << " processed frames\n";
    const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
    flushMatrix(grid, matrixOut, nowMs);
    maybePlan(grid, planner, planOut, nowMs);
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
            << "  dzuav <image|video> <hbb_engine> <cell_engine> [udp_port] [output_dir] [matrix_json] [route_json] [path_file]\n"
            << "  dzuav argus:<sensor-id> <hbb_engine> <cell_engine> [udp_port] [WxH@FPS] [matrix_json] [route_json] [path_file]\n"
            << "  dzuav uvc:<device>  <hbb_engine> <cell_engine> [udp_port] [WxH@FPS] [matrix_json] [route_json] [path_file]\n"
            << "examples:\n"
            << "  dzuav uvc:/dev/video0 engines/hbb_detector_fp16.engine "
               "engines/cell_classifier_fp16.engine 50000 1920x1080@30 "
               "/tmp/cell_matrix.json /tmp/route_plan.json /tmp/planned_path.txt\n";
        return 2;
    }

    const std::string source(argv[1]);
    if (source.rfind("argus:", 0) == 0 || source.rfind("uvc:", 0) == 0)
    {
        const uint16_t port =
            argc > 4 ? static_cast<uint16_t>(std::stoi(argv[4])) : uint16_t(50000);
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
    HbbDetector hbb(argv[2], .25F, .45F);
    CellClassifier cell(argv[3]);
    MapLocalizer map;
    CellGrid grid = makeGrid();
    UdpBroadcaster udp(argc > 4 ? std::stoi(argv[4]) : 50000);
    MatrixOutput matrixOut{matrix_json};
    RoutePlanner planner;
    PlanOutput planOut{route_json, path_file};
    const auto outputFor = [&](size_t index)
    { return output_dir / (input.stem().string() + "_" + std::to_string(index) + ".jpg"); };
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
    cv::Mat frame;
    for (size_t index = 0; capture.read(frame); ++index)
        processFrame(frame, hbb, cell, map, grid, udp, matrixOut, planner, planOut,
                     outputFor(index));
    return 0;
}
