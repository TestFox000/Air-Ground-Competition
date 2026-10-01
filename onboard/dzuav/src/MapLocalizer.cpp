#include "dzuav/MapLocalizer.hpp"

#include <opencv2/imgproc.hpp>
namespace dzuav
{
MapLocalizer::MapLocalizer()
{
    K_ = (cv::Mat_<double>(3, 3) << 995.153295, 0, 1010.494099, 0, 997.758431, 481.641588, 0, 0, 1);
    dist_ = (cv::Mat_<double>(1, 5) << -0.041172, -0.003300, -0.008916, 0.000707, 0);
}
cv::Mat MapLocalizer::undistort(const cv::Mat& i) const
{
    cv::Mat K = K_.clone();
    K.row(0) *= double(i.cols) / 1920;
    K.row(1) *= double(i.rows) / 1080;
    K.at<double>(2, 2) = 1;
    cv::Mat o;
    cv::undistort(i, o, K, dist_);
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
    if (std::abs(cv::contourArea(h)) < 0.02 * sz.area())
        return false;
    cv::Mat K = K_.clone();
    K.row(0) *= double(sz.width) / 1920;
    K.row(1) *= double(sz.height) / 1080;
    K.at<double>(2, 2) = 1;
    std::vector<int> in;
    if (!cv::solvePnPRansac(w, p, K, cv::noArray(), rvec_, tvec_, false, 100, 5, .995, in,
                            cv::SOLVEPNP_ITERATIVE) ||
        in.size() < 4)
        return false;
    cv::Mat R;
    cv::Rodrigues(rvec_, R);
    cv::Mat H(3, 3, CV_64F);
    R.col(0).copyTo(H.col(0));
    R.col(1).copyTo(H.col(1));
    tvec_.copyTo(H.col(2));
    H = K * H;
    Hinv_ = H.inv();
    ready_ = true;
    return true;
}
cv::Point2d MapLocalizer::project(const cv::Point2f& p) const
{
    cv::Mat q = (cv::Mat_<double>(3, 1) << p.x, p.y, 1);
    q = Hinv_ * q;
    return {q.at<double>(0) / q.at<double>(2), q.at<double>(1) / q.at<double>(2)};
}
}  // namespace dzuav
