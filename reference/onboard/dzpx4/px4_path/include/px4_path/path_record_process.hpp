#ifndef PX4_PATH__PATH_RECORD_PROCESS_HPP_
#define PX4_PATH__PATH_RECORD_PROCESS_HPP_

#include <cstddef>
#include <iosfwd>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace px4_path {

// 录制与处理模块共享的 FAST-LIO SLAM 位姿。这里不做 ENU/NED 坐标转换。
struct SlamPathPose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

// 独立封装录制路径的可选数据处理：
// 1. 在线滑动均值滤波；
// 2. 保存前中心滑动平均；
// 3. 删除平滑后过近的中间点。
//
// 基础录制、关键点判定和文件操作仍由 PathRecordNode 负责。
class PathRecordProcessor {
public:
  explicit PathRecordProcessor(rclcpp::Node &node);

  bool validate_parameters() const;
  void log_configuration() const;
  void write_metadata(std::ostream &output) const;
  void reset();
  SlamPathPose filter(const SlamPathPose &pose);
  std::vector<SlamPathPose>
  finalize(const std::vector<SlamPathPose> &poses) const;

private:
  static double quaternion_dot(const SlamPathPose &a, const SlamPathPose &b);
  static bool normalize_quaternion(SlamPathPose &pose);
  static double position_distance(const SlamPathPose &a, const SlamPathPose &b);
  static SlamPathPose average_poses(const std::vector<SlamPathPose> &poses,
                                    std::size_t begin, std::size_t end);
  static std::vector<SlamPathPose>
  smooth_path(const std::vector<SlamPathPose> &poses, std::size_t window_size);
  static std::vector<SlamPathPose>
  remove_close_points(const std::vector<SlamPathPose> &poses,
                      double min_distance_m);

  rclcpp::Logger logger_;
  int filter_window_{5};
  int smoothing_window_{3};
  double min_smoothed_point_distance_m_{0.0};
  std::vector<SlamPathPose> filter_buffer_;
};

} // namespace px4_path

#endif // PX4_PATH__PATH_RECORD_PROCESS_HPP_
