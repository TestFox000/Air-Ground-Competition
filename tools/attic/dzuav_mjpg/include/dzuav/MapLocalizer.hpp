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
    // 场地几何（与 Python 版 field_frame 完全一致）：0.35m 格、A 列中心 x=0.065、
    // 11 行中心 y=0.295。RoutePlanner 也复用 kCellSize。
    static constexpr float kCellSize = 0.35F;
    static constexpr float kOriginX = 0.065F;
    static constexpr float kOriginY = 0.295F;

    MapLocalizer();

    cv::Mat undistort(const cv::Mat& image) const;
    cv::Point3f world(const CellId& id) const;
    bool solve(const std::vector<std::pair<CellId, cv::Point2f>>& landmarks, cv::Size size);
    cv::Point2d project(const cv::Point2f& point) const;
    // 世界坐标 -> 格号（越界时 CellId::valid() 为 false）。
    CellId cellAt(const cv::Point2d& worldPos) const;
    // 像素点反投影到高度 heightM 的水平面（heightM=0 即地面）。未定位或
    // PnP 位姿不可用时返回 {-1e9,-1e9}（调用方据此走横放兜底）。
    cv::Point2d projectAtHeight(const cv::Point2f& point, float heightM) const;

private:
    cv::Mat K_;       // 相机内参（标定基准分辨率 1920x1080）
    cv::Mat dist_;    // 畸变系数
    cv::Mat rvec_;
    cv::Mat tvec_;
    cv::Mat H_;       // 地面平面 单应，世界 -> 像素（projectAtHeight 复用）
    cv::Mat Hinv_;    // 地面平面 单应的逆，像素 -> 世界
    cv::Mat Kscaled_; // solve() 时按帧分辨率缩放后的内参
    bool ready_ = false;   // solve() 是否已成功：仅作 project() 的防御性检查（防空调用读空 Hinv_）
    // 畸变矫正缓存：cv::undistort 内部即 initUndistortRectifyMap+remap，
    // 映射只与分辨率相关——同尺寸帧只算一次，每帧只做 remap（输出逐位等价）。
    mutable cv::Mat map_x_;
    mutable cv::Mat map_y_;
    mutable cv::Size cached_size_{};
};

}  // namespace dzuav
