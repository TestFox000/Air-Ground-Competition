#include "dzuav/PassageDetector.hpp"

#include <opencv2/imgproc.hpp>

#include <vector>

namespace dzuav
{
cv::Mat makeGreenMask(const cv::Mat& bgr)
{
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(35, 60, 60), cv::Scalar(85, 255, 255), mask);
    return mask;
}

bool greenRatio(const cv::Mat& greenMask, const cv::Point2f quad[4], float& ratio)
{
    for (int i = 0; i < 4; ++i)
        if (quad[i].x < 0 || quad[i].y < 0 || quad[i].x > greenMask.cols - 1 ||
            quad[i].y > greenMask.rows - 1)
            return false;
    std::vector<cv::Point2f> corners(quad, quad + 4);
    const cv::Rect bbox = cv::boundingRect(corners);
    if (bbox.area() < 16)
        return false;
    cv::Mat local = cv::Mat::zeros(bbox.size(), CV_8U);
    std::vector<cv::Point> polygon;
    for (const auto& q : corners)
        polygon.emplace_back(int(q.x) - bbox.x, int(q.y) - bbox.y);
    cv::fillPoly(local, polygon, cv::Scalar(255));
    ratio = float(cv::mean(greenMask(bbox), local)[0]) / 255.F;
    return true;
}
}  // namespace dzuav
