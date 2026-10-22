#pragma once

#include <opencv2/core.hpp>

namespace dzuav
{

// “通”格为绿色填充方块。对整帧生成绿色像素掩码（HSV 绿色区间）。
cv::Mat makeGreenMask(const cv::Mat& bgr);

// quad 内绿色像素占比（0~1）。quad 必须完整落在图像内，否则返回 false（不可观测）。
bool greenRatio(const cv::Mat& greenMask, const cv::Point2f quad[4], float& ratio);

}  // namespace dzuav
