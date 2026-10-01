#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "dzuav/CellClassifier.hpp"
#include "dzuav/HbbDetector.hpp"
#include "dzuav/MapLocalizer.hpp"
#include "dzuav/UdpBroadcaster.hpp"

using namespace dzuav;

namespace
{
const char* className(ObjectClass type)
{
    static const char* names[] = {"mine",       "car",         "arrow",   "target_red",
                                  "cell_label", "target_blue", "obstacle"};
    return names[static_cast<int>(type)];
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
        const auto roi = detection.box & cv::Rect2f(0, 0, image.cols, image.rows);
        if (roi.area() <= 4)
            continue;
        const CellId id = cell.infer(image(roi));
        if (id.valid() && id.confidence > .55F)
            landmarks.push_back({id, {roi.x + roi.width * .5F, roi.y + roi.height * .5F}});
    }
    const bool localized = map.solve(landmarks, image.size());
    std::vector<WorldObject> objects;
    cv::Mat visual = image.clone();
    for (const auto& detection : detections)
    {
        cv::rectangle(visual, detection.box,
                      detection.type == ObjectClass::CellLabel ? cv::Scalar(0, 255, 255)
                                                               : cv::Scalar(0, 255, 0),
                      2);
        std::string label =
            std::string(className(detection.type)) + " " + cv::format("%.2f", detection.confidence);
        if (detection.type == ObjectClass::CellLabel)
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
        else if (localized)
        {
            cv::Point2f point{detection.box.x + detection.box.width * .5F,
                              detection.box.y + detection.box.height * .5F};
            const auto world = map.project(point);
            objects.push_back({detection.type, world, detection.box.width, detection.box.height,
                               detection.confidence});
            label += cv::format(" (%.3f,%.3f)", world.x, world.y);
        }
        cv::putText(visual, label, {int(detection.box.x), std::max(18, int(detection.box.y) - 5)},
                    cv::FONT_HERSHEY_SIMPLEX, .55, cv::Scalar(0, 0, 255), 2);
    }
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
    if (localized)
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
    std::cout << "]\n";
    if (!output.empty())
        cv::imwrite(output.string(), visual);
}
}  // namespace

int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::cerr
            << "usage: dzuav <image_or_video> <hbb_engine> <cell_engine> [udp_port] [output_dir]\n";
        return 2;
    }
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
}
