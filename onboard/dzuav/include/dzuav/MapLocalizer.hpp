#pragma once

#include "dzuav/types.hpp"

#include <opencv2/core.hpp>

#include <chrono>
#include <optional>
#include <utility>
#include <vector>

namespace dzuav
{

// 用地面格号做 PnP 的实时自定位：
// solve() 成功后，project() 把像素点反投影到场地世界坐标（米）。
class MapLocalizer
{
public:
    // 最近一次 solve() 的结果画像（健康行可视化用，见 main.cpp 周期统计）
    struct SolveInfo
    {
        int path = 0;         // 定位路径：0=none（失败） 1=pnp 2=homography
        int inliers = 0;      // 最后一次尝试的内点数（失败时也保留，便于诊断）
        float median_px = 0.F;   // 最后一次尝试的全点重投影中位误差（px）
    };

    MapLocalizer();

    cv::Mat undistort(const cv::Mat& image) const;
    cv::Point3f world(const CellId& id) const;
    bool solve(const std::vector<std::pair<CellId, cv::Point2f>>& landmarks, cv::Size size);
    // 像素 -> 世界（米）。返回 nullopt 表示该点被过滤（齐次尺度退化 / 非有限 /
    // 超场地量程），调用方丢弃并计数；未 solve 成功仍抛异常（防误用，见 .cpp）。
    std::optional<cv::Point2d> project(const cv::Point2f& point) const;
    // pose hold：当前帧 solve 失败后调用——最近一次成功定位的龄期 <= max_age_s
    // 时，用缓存位姿顶替（返回 true，此后 project 走缓存位姿）；超龄返回 false。
    bool hold(double max_age_s);
    const SolveInfo& lastSolve() const { return last_solve_; }

private:
    cv::Mat K_;       // 相机内参（标定基准分辨率 1920x1080）
    cv::Mat dist_;    // 畸变系数
    cv::Mat rvec_;
    cv::Mat tvec_;
    cv::Mat Hinv_;    // 地面平面 单应的逆，像素 -> 世界
    bool ready_ = false;   // solve() 是否已成功：仅作 project() 的防御性检查（防空调用读空 Hinv_）
    SolveInfo last_solve_;
    // 最近成功定位的保持缓存（pose hold）：solve 成功时刷新；hold() 在龄期内
    // 用它顶替补 Hinv_，让当前帧检测仍能投影下发（态势不中断）。
    cv::Mat hold_Hinv_;
    std::chrono::steady_clock::time_point hold_time_;
    bool hold_valid_ = false;
    // 畸变矫正缓存：cv::undistort 内部即 initUndistortRectifyMap+remap，
    // 映射只与分辨率相关——同尺寸帧只算一次，每帧只做 remap（输出逐位等价）。
    mutable cv::Mat map_x_;
    mutable cv::Mat map_y_;
    mutable cv::Size cached_size_{};
    mutable cv::Mat undistorted_;     // remap 输出缓冲，跨帧复用（免每帧 ~6MB 分配）
    mutable bool scale_warned_ = false;   // 非基准分辨率"内参线性缩放近似"警告只打一次
};

}  // namespace dzuav
