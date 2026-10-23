#ifndef PX4_PATH__PATH_RECORD_NODE_HPP_
#define PX4_PATH__PATH_RECORD_NODE_HPP_

#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "px4_path/path_record_process.hpp"

namespace px4_path {

// PathRecordNode 只负责基础航路录制流程：
// 1. 校验录制参数并在节点启动时自动开始；
// 2. 接收 FAST-LIO 原始 SLAM 位姿；
// 3. 校验位姿与定位跳变并抽取关键帧；
// 4. 安全写入 txt 和发布 RViz 路径。
// 在线滤波与保存前平滑由 PathRecordProcessor 可选提供。
class PathRecordNode : public rclcpp::Node {
public:
  PathRecordNode();
  ~PathRecordNode() override;

  PathRecordNode(const PathRecordNode &) = delete;
  PathRecordNode &operator=(const PathRecordNode &) = delete;

private:
  // -------------------------- 基础位姿检查 --------------------------------
  static bool is_finite_pose(const SlamPathPose &pose);
  static bool normalize_quaternion(SlamPathPose &pose);
  static double position_distance(const SlamPathPose &a, const SlamPathPose &b);
  static double horizontal_distance(const SlamPathPose &a,
                                    const SlamPathPose &b);
  static double vertical_distance(const SlamPathPose &a, const SlamPathPose &b);

  // -------------------------- 录制状态与文件操作 ---------------------------
  bool validate_parameters();
  bool prepare_output_path();
  void start_recording();
  void append_final_pose();
  void stop_recording(const char *reason);
  bool write_path_file(const std::vector<SlamPathPose> &poses);
  void publish_visualization(const std::vector<SlamPathPose> &poses);

  // ---------------------------- ROS 2 回调 ---------------------------------
  void on_slam_odom(const nav_msgs::msg::Odometry::SharedPtr message);

  // ------------------------------ 参数 -------------------------------------
  std::string slam_topic_;
  std::string expected_frame_id_;
  std::string visualization_topic_;
  std::string path_dir_;
  std::string output_file_;
  bool overwrite_existing_{true};
  bool enable_filter_smoothing_{true};
  double min_xy_m_{0.10};
  double min_z_m_{0.10};
  double max_neighbor_dist_m_{5.0};
  double min_record_interval_s_{0.10};
  int min_points_{2};

  // ---------------------------- ROS 2 接口 ---------------------------------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_slam_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_visualization_;

  // ----------------------------- 运行状态 ----------------------------------
  bool recording_{false};
  std::optional<rclcpp::Time> last_record_stamp_;
  std::optional<SlamPathPose> latest_raw_pose_;
  std::optional<std::filesystem::path> session_output_path_;
  std::vector<SlamPathPose> recorded_points_;
  std::unique_ptr<PathRecordProcessor> path_processor_;
  std::string source_frame_id_;
  bool source_frame_initialized_{false};
};

} // namespace px4_path

#endif // PX4_PATH__PATH_RECORD_NODE_HPP_
