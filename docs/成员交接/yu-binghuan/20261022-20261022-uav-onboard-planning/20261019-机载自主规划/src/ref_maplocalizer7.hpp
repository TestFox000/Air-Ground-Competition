#pragma once

#include "dzuav/types.hpp"

#include <opencv2/core.hpp>

#include <utility>
#include <vector>

namespace dzuav
{

// 用地面格号做 PnP 的实时自定位：
// solve() 成功后，project() 把像素点反投影到场地世界坐标（米）。
class MapLocalizer
{
public:
    static constexpr float kCellSize = 0.35F;  // 格子边长（米）
    static constexpr float kOriginX = 0.065F;  // A 列中心的世界 x
    static constexpr float kOriginY = 0.295F;  // 11 行中心的世界 y

    MapLocalizer();

    cv::Mat undistort(const cv::Mat& image) const;
    cv::Point3f world(const CellId& id) const;
    bool solve(const std::vector<std::pair<CellId, cv::Point2f>>& landmarks, cv::Size size);
    cv::Point2d project(const cv::Point2f& point) const;
    // 世界坐标 -> 格号（越界时 CellId::valid() 为 false）。
    CellId cellAt(const cv::Point2d& worldPos) const;
    // 地面格子四角（中心 ± 半格宽）投影到图像；未定位或任一角在相机后方时返回 false。
    bool projectCellQuad(const CellId& id, cv::Point2f quad[4]) const;
    // 把像素点反投影到高度 heightM 的水平面（heightM=0 即地面，用于高物体的落地位置）。
    cv::Point2d projectAtHeight(const cv::Point2f& point, float heightM) const;

private:
    cv::Mat K_;       // 相机内参（标定基准分辨率 1920x1080）
    cv::Mat dist_;    // 畸变系数
    cv::Mat rvec_;
    cv::Mat tvec_;
    cv::Mat H_;       // 地面平面 单应，世界 -> 像素
    cv::Mat Hinv_;    // 地面平面 单应的逆，像素 -> 世界
    cv::Mat Kscaled_; // solve() 时按帧分辨率缩放后的内参
    bool ready_ = false;
    // 畸变矫正缓存：cv::undistort 内部即 initUndistortRectifyMap+remap，
    // 映射只与分辨率相关——同尺寸帧只算一次，每帧只做 remap（输出逐位等价）。
    mutable cv::Mat map_x_;
    mutable cv::Mat map_y_;
    mutable cv::Size cached_size_{};
};

}  // namespace dzuav
