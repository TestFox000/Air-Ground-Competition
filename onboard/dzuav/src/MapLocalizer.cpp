#include "dzuav/MapLocalizer.hpp"

#include <algorithm>
#include <cmath>
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
        const cv::Mat K = scaledIntrinsics(K_, i.size());
        cv::initUndistortRectifyMap(K, dist_, cv::noArray(), K, i.size(), CV_16SC2, map_x_, map_y_);
        cached_size_ = i.size();
    }
    cv::Mat o;
    cv::remap(i, o, map_x_, map_y_, cv::INTER_LINEAR);
    return o;
}
cv::Point3f MapLocalizer::world(const CellId& id) const
{
    return {0.065f + (id.column - 'A') * .35f, 0.295f + (11 - id.row) * .35f, 0};
}
bool MapLocalizer::solve(const std::vector<std::pair<CellId, cv::Point2f>>& l, cv::Size sz)
{
    ready_ = false;
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
    // 0.005（原 0.02，2026-10-17 live 事故修订）：live 近距视野地标可集中在
    // 画面不足 2% 区域；0.005 仍足以拦截共线/聚团退化（凸包边长需 ≥~100px）。
    if (std::abs(cv::contourArea(h)) < 0.005 * sz.area())
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
            // 内点下限 = max(4, 30%)（2026-10-17 live 事故修订）：原绝对下限
            // 12 为 88 格全景帧（30+ 地标）设定，live 近距视野地标仅 5~6 个时
            // 数学上不可能满足 → localized 恒 0（dets/cells 截图实锤）。
            // 4 = P3P 最小解算点数；垃圾解防线不变：下方全点重投影中位误差
            // ≤10px（内参错配/大面积错读时中位误差达数百 px 仍被拦截）。
            const std::size_t need = std::max<std::size_t>(4, p.size() * 3 / 10);
            if (in.size() >= need && median <= 10.f)
            {
                cv::Mat R;
                cv::Rodrigues(rvec_, R);
                cv::Mat H(3, 3, CV_64F);
                R.col(0).copyTo(H.col(0));
                R.col(1).copyTo(H.col(1));
                tvec_.copyTo(H.col(2));
                H = K * H;
                Hinv_ = H.inv();
                pnp_ok = true;
            }
        }
    }
    if (pnp_ok)
    {
        ready_ = true;
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
    // 质检：重投影中位误差 <= 15px（单应无内参，门槛略宽于 PnP 的 10px）。
    // 内点下限 = max(5, 30%)（2026-10-17 live 事故修订，理由同 PnP 路径）：
    // 原绝对下限 8 在 live 近距视野（地标 5~6 个）数学上不可满足。取 5 而
    // 非 4：任意 4 点对应都精确确定一个单应（零验证力），≥5 才含至少 1 个
    // 过约束点（本地仿真场景 C 实证：6 点随机全错读时 RANSAC 仍能找到 4
    // 内点假解，中位误差对 4/6 内点占比失效）。
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
    const std::size_t need = std::max<std::size_t>(5, p.size() * 3 / 10);
    if (static_cast<std::size_t>(inlier) < need || median > 15.f)
        return false;   // 单应解也不可信 -> localized=0
    Hinv_ = H.inv();
    ready_ = true;
    return true;
}
cv::Point2d MapLocalizer::project(const cv::Point2f& p) const
{
    // 防御性检查：solve() 未成功时 Hinv_ 为空，直接乘会触发 OpenCV 断言崩溃；
    // 现主流程只在 localized 后调用，此检查防未来误用（抛出比静默返回 (0,0)
    // 安全——(0,0) 恰是场地角点，会被当成合法坐标）。
    if (!ready_)
        throw std::runtime_error("MapLocalizer::project called before solve() succeeded");
    cv::Mat q = (cv::Mat_<double>(3, 1) << p.x, p.y, 1);
    q = Hinv_ * q;
    const double w = q.at<double>(2);
    const cv::Point2d out{q.at<double>(0) / w, q.at<double>(1) / w};
    // w 趋近 0（单应把点投到无穷远）时除法给出 inf/NaN，而 inf 会顺着
    // world 坐标一路流进 UDP 情报字段，在车端被当成"合法但极远"的目标，
    // 比丢一帧难查得多。抛出比返回坏值安全：调用点都在 processFrame 内，
    // 外层 try/catch（main.cpp）会把这一帧丢掉并计数，不会终结进程。
    if (!std::isfinite(out.x) || !std::isfinite(out.y))
        throw std::runtime_error(
            "MapLocalizer::project degenerate (w=" + std::to_string(w) + ")");
    return out;
}
}  // namespace dzuav
