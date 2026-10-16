#include "dzuav/MapLocalizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
namespace dzuav
{
namespace
{
// 按帧分辨率缩放基准内参（基准 1920x1080）。
cv::Mat scaledIntrinsics(const cv::Mat& base, cv::Size sz)
{
    cv::Mat K = base.clone();
    K.row(0) *= double(sz.width) / 1920;
    K.row(1) *= double(sz.height) / 1080;
    K.at<double>(2, 2) = 1;
    return K;
}
}  // namespace
MapLocalizer::MapLocalizer()
{
    K_ = (cv::Mat_<double>(3, 3) << 995.153295, 0, 1010.494099, 0, 997.758431, 481.641588, 0, 0, 1);
    dist_ = (cv::Mat_<double>(1, 5) << -0.041172, -0.003300, -0.008916, 0.000707, 0);
}
cv::Mat MapLocalizer::undistort(const cv::Mat& i) const
{
    // 同分辨率只算一次映射；cv::undistort 即 initUndistortRectifyMap(unity R)+remap(线性)，
    // 此处显式缓存映射后每帧仅 remap，输出与逐帧 undistort 逐位等价。
    if (cached_size_ != i.size())
    {
        // 内参只有 1920x1080 基准标定：其他分辨率（如断流降规格到 720p）是
        // 线性缩放近似，精度打折——提醒一次即可，刷屏无意义。
        if (i.size() != cv::Size(1920, 1080) && !scale_warned_)
        {
            scale_warned_ = true;
            std::fprintf(stderr,
                         "warning: frame size %dx%d != 1920x1080; intrinsics are a "
                         "linear-scaled approximation\n",
                         i.cols, i.rows);
        }
        const cv::Mat K = scaledIntrinsics(K_, i.size());
        cv::initUndistortRectifyMap(K, dist_, cv::noArray(), K, i.size(), CV_16SC2, map_x_, map_y_);
        cached_size_ = i.size();
    }
    // 输出写进成员缓冲（跨帧复用，免每帧 ~6MB 分配）；返回的 Mat 与其共享数据，
    // 调用方须在下次 undistort 前用完（主流程单线程逐帧处理，满足）。
    cv::remap(i, undistorted_, map_x_, map_y_, cv::INTER_LINEAR);
    return undistorted_;
}
cv::Point3f MapLocalizer::world(const CellId& id) const
{
    return {0.065f + (id.column - 'A') * .35f, 0.295f + (11 - id.row) * .35f, 0};
}
bool MapLocalizer::solve(const std::vector<std::pair<CellId, cv::Point2f>>& l, cv::Size sz)
{
    ready_ = false;
    last_solve_ = {};   // path=none；各路径尝试时填 inliers/median，成功时填 path
    if (l.size() < 4)
        return false;
    std::vector<cv::Point3f> w;
    std::vector<cv::Point2f> p;
    for (auto& v : l)
        if (v.first.valid())
        {
            w.push_back(world(v.first));
            p.push_back(v.second);
        }
    if (w.size() < 4)
        return false;
    std::vector<cv::Point2f> h;
    cv::convexHull(p, h);
    if (std::abs(cv::contourArea(h)) < 0.02 * sz.area())
        return false;
    cv::Mat K = scaledIntrinsics(K_, sz);
    // ---- 主路径：PnP（依赖内参，精度高但内参偏差时会失败）----
    bool pnp_ok = false;
    {
        std::vector<int> in;
        // 诊断结论（runs/sim_match/_diag_pnp.py, frame_00170 真值对照）：
        //   * SOLVEPNP_ITERATIVE 对纯平面点集(z=0)直接无解 -> 改 P3P
        //   * 内参与实际相机不符时 P3P 会以极少数内点"假成功"，全点重投影中位
        //     误差 ~250px（正确内参 <3px），垃圾解会把目标投到场外 -> 质检门槛。
        if (cv::solvePnPRansac(w, p, K, cv::noArray(), rvec_, tvec_, false, 100, 5, .995, in,
                               cv::SOLVEPNP_P3P) &&
            in.size() >= 4)
        {
            std::vector<cv::Point2f> proj;
            cv::projectPoints(w, rvec_, tvec_, K, cv::noArray(), proj);
            std::vector<float> e2;
            e2.reserve(p.size());
            for (std::size_t k = 0; k < p.size(); ++k)
            {
                const float dx = proj[k].x - p[k].x;
                const float dy = proj[k].y - p[k].y;
                e2.push_back(std::sqrt(dx * dx + dy * dy));
            }
            std::vector<float> sorted(e2);
            std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
            const float median = sorted[sorted.size() / 2];
            last_solve_.inliers = static_cast<int>(in.size());
            last_solve_.median_px = median;
            const std::size_t need = std::max<std::size_t>(12, p.size() * 3 / 10);
            if (in.size() >= need && median <= 10.f)
            {
                cv::Mat R;
                cv::Rodrigues(rvec_, R);
                cv::Mat H(3, 3, CV_64F);
                R.col(0).copyTo(H.col(0));
                R.col(1).copyTo(H.col(1));
                tvec_.copyTo(H.col(2));
                H = K * H;
                // cv::invert 返回行列式（默认 LU），0=奇异：数值退化的单应
                // 不能用，放弃 PnP 路径让下面的 homography 兜底。
                if (cv::invert(H, Hinv_) != 0.0)
                    pnp_ok = true;
            }
        }
    }
    if (pnp_ok)
    {
        ready_ = true;
        last_solve_.path = 1;
        // 刷新 pose hold 缓存（见 hold()）。必须深拷贝：Mat 赋值只共享数据，
        // 后续 solve 里 cv::invert 写 Hinv_ 会把共享的缓存一起改掉。
        Hinv_.copyTo(hold_Hinv_);
        hold_time_ = std::chrono::steady_clock::now();
        hold_valid_ = true;
        return true;
    }
    // ---- 兜底：直接单应拟合（无需内参，格号世界坐标↔图像坐标）----
    // 场地为规则网格，航拍近似正射下 world(x,y) → pixel(u,v) 为单应变换。
    // PnP 因内参偏差失败时，直接拟合 H 仍可定位（Python lattice_ransac 验证有效）。
    std::vector<cv::Point2f> w2;
    w2.reserve(w.size());
    for (const auto& pt : w)
        w2.emplace_back(pt.x, pt.y);
    std::vector<uchar> mask;
    cv::Mat H = cv::findHomography(w2, p, cv::RANSAC, 5.0, mask);
    if (H.empty())
        return false;
    // 质检：重投影中位误差 <= 15px（单应无内参，门槛略宽于 PnP 的 10px）
    std::vector<cv::Point2f> proj;
    cv::perspectiveTransform(w2, proj, H);
    std::vector<float> e2;
    e2.reserve(p.size());
    int inlier = 0;
    for (std::size_t k = 0; k < p.size(); ++k)
    {
        const float dx = proj[k].x - p[k].x;
        const float dy = proj[k].y - p[k].y;
        const float d = std::sqrt(dx * dx + dy * dy);
        e2.push_back(d);
        if (mask[k])
            ++inlier;
    }
    std::vector<float> sorted(e2);
    std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
    const float median = sorted[sorted.size() / 2];
    last_solve_.inliers = inlier;
    last_solve_.median_px = median;
    const std::size_t need = std::max<std::size_t>(8, p.size() * 3 / 10);
    if (static_cast<std::size_t>(inlier) < need || median > 15.f)
        return false;   // 单应解也不可信 -> localized=0
    // 奇异检查同 PnP 路径（0=不可逆）
    if (cv::invert(H, Hinv_) == 0.0)
        return false;
    ready_ = true;
    last_solve_.path = 2;
    // 刷新 pose hold 缓存（见 hold()）。必须深拷贝：Mat 赋值只共享数据，
    // 后续 solve 里 cv::invert 写 Hinv_ 会把共享的缓存一起改掉。
    Hinv_.copyTo(hold_Hinv_);
    hold_time_ = std::chrono::steady_clock::now();
    hold_valid_ = true;
    return true;
}
bool MapLocalizer::hold(double max_age_s)
{
    // pose hold：当前帧 solve 失败时，用最近一次成功定位的位姿顶替，投影当前帧
    // 检测继续下发（态势不中断）。龄期超过 max_age_s 不用——陈旧位姿比没有更危险。
    if (!hold_valid_)
        return false;
    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - hold_time_)
                           .count();
    if (age > max_age_s)
        return false;
    // 深拷贝回 Hinv_（缓存本体独立持有，project 期间的后续 solve 不会改到它）
    hold_Hinv_.copyTo(Hinv_);
    ready_ = true;
    return true;
}
std::optional<cv::Point2d> MapLocalizer::project(const cv::Point2f& p) const
{
    // 防御性检查：solve() 未成功时 Hinv_ 为空，直接乘会触发 OpenCV 断言崩溃；
    // 现主流程只在 localized 后调用，此检查防未来误用（抛出比静默返回 (0,0)
    // 安全——(0,0) 恰是场地角点，会被当成合法坐标）。
    if (!ready_)
        throw std::runtime_error("MapLocalizer::project called before solve() succeeded");
    cv::Mat q = (cv::Mat_<double>(3, 1) << p.x, p.y, 1);
    q = Hinv_ * q;
    // 齐次尺度退化：|q2|~0 表示像素点被投到无穷远，除法会爆出任意值。
    const double q2 = q.at<double>(2);
    if (std::abs(q2) < 1e-12)
        return std::nullopt;
    const double x = q.at<double>(0) / q2;
    const double y = q.at<double>(1) / q2;
    // 非有限结果不可信（NaN/Inf 到车端会整帧丢弃，还会污染 atan2 等下游）
    if (!std::isfinite(x) || !std::isfinite(y))
        return std::nullopt;
    // 场地量程 sanity：场地约 4.9x4.2m，留足余量；超界即错误投影，丢弃
    // （调用方计数进健康行）。宁可少发一个目标，不发一个假目标。
    if (x < -1.0 || x > 6.0 || y < -1.0 || y > 5.0)
        return std::nullopt;
    return cv::Point2d(x, y);
}
}  // namespace dzuav
