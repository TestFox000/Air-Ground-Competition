#include "px4_path/path_record_node.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <system_error>

namespace px4_path {

bool PathRecordNode::is_finite_pose(const SlamPathPose &pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.z) && std::isfinite(pose.qx) &&
         std::isfinite(pose.qy) && std::isfinite(pose.qz) &&
         std::isfinite(pose.qw);
}

bool PathRecordNode::normalize_quaternion(SlamPathPose &pose) {
  const double norm = std::sqrt(pose.qx * pose.qx + pose.qy * pose.qy +
                                pose.qz * pose.qz + pose.qw * pose.qw);
  if (!std::isfinite(norm) || norm < 1e-9) {
    return false;
  }
  pose.qx /= norm;
  pose.qy /= norm;
  pose.qz /= norm;
  pose.qw /= norm;
  return true;
}

double PathRecordNode::position_distance(const SlamPathPose &a,
                                         const SlamPathPose &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double PathRecordNode::horizontal_distance(const SlamPathPose &a,
                                           const SlamPathPose &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double PathRecordNode::vertical_distance(const SlamPathPose &a,
                                         const SlamPathPose &b) {
  return std::fabs(a.z - b.z);
}

// 本节点始终保存 FAST-LIO 原始 SLAM 坐标，不在录制阶段进行 ENU/NED 转换。
PathRecordNode::PathRecordNode() : rclcpp::Node("path_record_node") {
  // FAST-LIO 原始里程计话题，消息类型必须为 nav_msgs/msg/Odometry。
  slam_topic_ =
      this->declare_parameter<std::string>("slam_topic", "/localization");
  // 期望的 frame_id；空字符串表示采用首帧值，之后仍要求录制期间保持不变。
  expected_frame_id_ =
      this->declare_parameter<std::string>("expected_frame_id", "");
  // RViz2 可直接订阅的录制轨迹，坐标仍保持 SLAM 原始 frame。
  visualization_topic_ = this->declare_parameter<std::string>(
      "visualization_topic", "/path_record/visualization");
  // 输出目录；空字符串表示使用 share/px4_path/path。
  path_dir_ = this->declare_parameter<std::string>("path_dir", "");
  // 输出文件名，只允许文件名，不允许在这里携带目录。
  output_file_ =
      this->declare_parameter<std::string>("output_file", "recorded_path.txt");
  // 是否覆盖 output_file 指定的航路文件；默认 true，便于重复录制同名航路。
  overwrite_existing_ =
      this->declare_parameter<bool>("overwrite_existing", true);
  // true 使用独立 PathRecordProcessor；false 直接用原始有效位姿抽取关键点。
  enable_filter_smoothing_ =
      this->declare_parameter<bool>("enable_filter_smoothing", true);
  // 相对上一个关键点的水平和竖直位移门限，任一达到即记录新点。
  min_xy_m_ = this->declare_parameter<double>("min_xy_m", 0.10);
  min_z_m_ = this->declare_parameter<double>("min_z_m", 0.10);
  // 相邻原始帧和最终航路点允许的最大位移；超过时按定位跳变丢弃。
  max_neighbor_dist_m_ =
      this->declare_parameter<double>("max_neighbor_dist_m", 5.0);
  // 两个关键帧之间的最短时间，单位 s，用于限制高频抖动造成的密集采样。
  min_record_interval_s_ =
      this->declare_parameter<double>("min_record_interval_s", 0.10);
  // 允许写入文件的最少点数；path_loader 至少需要两个点形成航段。
  min_points_ = this->declare_parameter<int>("min_points", 2);

  bool parameters_valid = validate_parameters();
  if (enable_filter_smoothing_) {
    path_processor_ = std::make_unique<PathRecordProcessor>(*this);
    const bool processing_parameters_valid =
        path_processor_->validate_parameters();
    parameters_valid = processing_parameters_valid && parameters_valid;
    if (processing_parameters_valid) {
      path_processor_->log_configuration();
    }
  }

  sub_slam_ = this->create_subscription<nav_msgs::msg::Odometry>(
      slam_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PathRecordNode::on_slam_odom, this, std::placeholders::_1));
  // Rviz2可视化路径
  pub_visualization_ = this->create_publisher<nav_msgs::msg::Path>(
      visualization_topic_, rclcpp::QoS(1).reliable().transient_local());

  RCLCPP_INFO(this->get_logger(),
              "path_record_node up. slam_topic=%s visualization=%s "
              "min_xy=%.2fm min_z=%.2fm max_jump=%.2fm processing=%s",
              slam_topic_.c_str(), visualization_topic_.c_str(), min_xy_m_,
              min_z_m_, max_neighbor_dist_m_,
              enable_filter_smoothing_ ? "enabled" : "disabled");
  if (parameters_valid) {
    start_recording();
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "path recording cannot start because parameters are invalid");
  }
}

PathRecordNode::~PathRecordNode() {
  if (recording_) {
    stop_recording("node shutdown");
  }
}

// 参数不合法时采用失效安全策略：保留节点运行和错误日志，但关闭录制功能。
bool PathRecordNode::validate_parameters() {
  bool valid = true;
  const auto reject = [this, &valid](const char *name,
                                     const std::string &reason) {
    RCLCPP_ERROR(this->get_logger(), "invalid parameter '%s': %s", name,
                 reason.c_str());
    valid = false;
  };

  if (slam_topic_.empty()) {
    reject("slam_topic", "must not be empty");
  }
  if (visualization_topic_.empty()) {
    reject("visualization_topic", "must not be empty");
  }
  if (path_dir_.empty()) {
    reject("path_dir", "must be configured in px4_path/config/params.yaml");
  }
  if (output_file_.empty()) {
    reject("output_file", "must not be empty");
  }
  const std::filesystem::path output_path(output_file_);
  if (!output_path.has_filename() || output_path == "." ||
      output_path == ".." || output_path.filename().string() != output_file_) {
    reject("output_file", "must be a file name; configure path_dir separately");
  }
  if (!std::isfinite(min_xy_m_) || min_xy_m_ <= 0.0) {
    reject("min_xy_m", "must be finite and > 0");
  }
  if (!std::isfinite(min_z_m_) || min_z_m_ <= 0.0) {
    reject("min_z_m", "must be finite and > 0");
  }
  if (!std::isfinite(max_neighbor_dist_m_) || max_neighbor_dist_m_ <= 0.0) {
    reject("max_neighbor_dist_m", "must be finite and > 0");
  }
  if (!std::isfinite(min_record_interval_s_) || min_record_interval_s_ < 0.0) {
    reject("min_record_interval_s", "must be finite and >= 0");
  }
  if (min_points_ < 2) {
    reject("min_points", "must be >= 2 for path_loader compatibility");
  }
  return valid;
}

bool PathRecordNode::prepare_output_path() {
  namespace fs = std::filesystem;

  const fs::path directory(path_dir_);

  std::error_code error;
  fs::create_directories(directory, error);
  if (error) {
    RCLCPP_ERROR(this->get_logger(), "cannot create path directory '%s': %s",
                 directory.string().c_str(), error.message().c_str());
    return false;
  }

  fs::path candidate = directory / output_file_;
  if (!overwrite_existing_ && fs::exists(candidate, error)) {
    const fs::path stem = candidate.stem();
    const fs::path extension = candidate.extension();
    for (std::size_t suffix = 1;; ++suffix) {
      candidate = directory / (stem.string() + "_" + std::to_string(suffix) +
                               extension.string());
      if (!fs::exists(candidate, error)) {
        break;
      }
    }
  }
  if (error) {
    RCLCPP_ERROR(this->get_logger(), "cannot inspect output path '%s': %s",
                 candidate.string().c_str(), error.message().c_str());
    return false;
  }

  session_output_path_ = candidate;
  return true;
}

void PathRecordNode::start_recording() {
  if (recording_) {
    RCLCPP_INFO(this->get_logger(), "record start ignored: already recording");
    return;
  }
  if (!prepare_output_path()) {
    return;
  }

  // 先用上一次的 frame 发布空 Path，让 RViz2 清除旧轨迹。
  publish_visualization({});
  recorded_points_.clear();
  latest_raw_pose_.reset();
  if (path_processor_) {
    path_processor_->reset();
  }
  last_record_stamp_.reset();
  source_frame_id_.clear();
  source_frame_initialized_ = false;
  recording_ = true;
  RCLCPP_INFO(this->get_logger(), "path recording START -> %s",
              session_output_path_->string().c_str());
}

void PathRecordNode::append_final_pose() {
  // 最终原始位姿作为真实录制终点补入；保存前处理仍保持首尾点不漂移。
  if (!latest_raw_pose_.has_value()) {
    return;
  }
  if (recorded_points_.empty()) {
    recorded_points_.push_back(*latest_raw_pose_);
    return;
  }

  const double distance =
      position_distance(recorded_points_.back(), *latest_raw_pose_);
  if (distance > max_neighbor_dist_m_) {
    RCLCPP_WARN(
        this->get_logger(),
        "final SLAM pose is %.2f m from the last recorded point (> %.2f m); "
        "terminal jump ignored",
        distance, max_neighbor_dist_m_);
    return;
  }
  const double xy_distance =
      horizontal_distance(recorded_points_.back(), *latest_raw_pose_);
  const double z_distance =
      vertical_distance(recorded_points_.back(), *latest_raw_pose_);
  if (xy_distance >= min_xy_m_ || z_distance >= min_z_m_) {
    recorded_points_.push_back(*latest_raw_pose_);
  }
}

void PathRecordNode::stop_recording(const char *reason) {
  if (!recording_) {
    return;
  }

  recording_ = false;
  append_final_pose();
  if (recorded_points_.size() < static_cast<std::size_t>(min_points_)) {
    publish_visualization(recorded_points_);
    RCLCPP_ERROR(this->get_logger(),
                 "path recording stopped (%s), but only %zu point(s) were "
                 "captured; file not written",
                 reason, recorded_points_.size());
    return;
  }

  const std::vector<SlamPathPose> finalized =
      path_processor_ ? path_processor_->finalize(recorded_points_)
                      : recorded_points_;
  if (finalized.size() < recorded_points_.size()) {
    RCLCPP_INFO(this->get_logger(),
                "filter/smoothing processing reduced path from %zu to %zu "
                "waypoint(s)",
                recorded_points_.size(), finalized.size());
  }
  if (finalized.size() < static_cast<std::size_t>(min_points_)) {
    publish_visualization(finalized);
    RCLCPP_ERROR(this->get_logger(),
                 "processed path only contains %zu point(s), fewer than "
                 "min_points=%d; file not written",
                 finalized.size(), min_points_);
    return;
  }
  // 停止时发布与即将写入文件完全一致的最终轨迹。
  publish_visualization(finalized);
  if (write_path_file(finalized)) {
    RCLCPP_INFO(this->get_logger(),
                "path recording STOP (%s): saved %zu SLAM points to %s", reason,
                finalized.size(), session_output_path_->string().c_str());
  }
}

bool PathRecordNode::write_path_file(const std::vector<SlamPathPose> &poses) {
  namespace fs = std::filesystem;
  if (!session_output_path_.has_value()) {
    RCLCPP_ERROR(this->get_logger(),
                 "cannot save path: output path is not prepared");
    return false;
  }

  // 保存前做最后一道与 path_loader 相同的相邻距离检查
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const double distance = position_distance(poses[i - 1], poses[i]);
    if (distance > max_neighbor_dist_m_) {
      RCLCPP_ERROR(
          this->get_logger(),
          "cannot save path: point %zu -> %zu distance %.2f m exceeds %.2f m",
          i - 1, i, distance, max_neighbor_dist_m_);
      return false;
    }
  }

  // 先完整写入临时文件，再 rename 为目标文件，避免掉电或写入失败留下半截路径。
  const fs::path temporary_path = session_output_path_->string() + ".tmp";
  std::ofstream output(temporary_path, std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "cannot open temporary path file '%s'",
                 temporary_path.string().c_str());
    return false;
  }

  output << "# px4_path recorded path (FAST-LIO SLAM frame)\n";
  output << "# format: x y z qx qy qz qw\n";
  output << "# source_topic: " << slam_topic_ << "\n";
  output << "# source_frame: " << source_frame_id_ << "\n";
  output << "# sampling: min_xy=" << min_xy_m_ << "m min_z=" << min_z_m_
         << "m min_interval=" << min_record_interval_s_
         << "s max_neighbor=" << max_neighbor_dist_m_ << "m\n";
  if (path_processor_) {
    path_processor_->write_metadata(output);
  } else {
    output << "# filter_smoothing_enabled: false\n";
  }
  output << "# path_loader will convert SLAM -> ENU -> PX4 NED.\n";
  output << std::fixed << std::setprecision(9);
  for (const SlamPathPose &pose : poses) {
    output << pose.x << ' ' << pose.y << ' ' << pose.z << ' ' << pose.qx << ' '
           << pose.qy << ' ' << pose.qz << ' ' << pose.qw << '\n';
  }
  output.flush();
  if (!output.good()) {
    RCLCPP_ERROR(this->get_logger(),
                 "failed while writing temporary path file '%s'",
                 temporary_path.string().c_str());
    output.close();
    return false;
  }
  output.close();

  std::error_code error;
  fs::rename(temporary_path, *session_output_path_, error);
  if (error) {
    RCLCPP_ERROR(this->get_logger(), "cannot finalize path file '%s': %s",
                 session_output_path_->string().c_str(),
                 error.message().c_str());
    return false;
  }
  return true;
}

void PathRecordNode::publish_visualization(
    const std::vector<SlamPathPose> &poses) {
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id =
      source_frame_id_.empty() ? expected_frame_id_ : source_frame_id_;
  path.poses.reserve(poses.size());

  for (const SlamPathPose &recorded_pose : poses) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = recorded_pose.x;
    pose.pose.position.y = recorded_pose.y;
    pose.pose.position.z = recorded_pose.z;
    pose.pose.orientation.x = recorded_pose.qx;
    pose.pose.orientation.y = recorded_pose.qy;
    pose.pose.orientation.z = recorded_pose.qz;
    pose.pose.orientation.w = recorded_pose.qw;
    path.poses.push_back(std::move(pose));
  }

  pub_visualization_->publish(path);
}

void PathRecordNode::on_slam_odom(
    const nav_msgs::msg::Odometry::SharedPtr message) {
  if (!recording_) {
    return;
  }
  if (!expected_frame_id_.empty() &&
      message->header.frame_id != expected_frame_id_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "SLAM frame '%s' does not match expected_frame_id '%s'; sample ignored",
        message->header.frame_id.c_str(), expected_frame_id_.c_str());
    return;
  }
  if (!source_frame_initialized_) {
    source_frame_id_ = message->header.frame_id;
    source_frame_initialized_ = true;
  } else if (message->header.frame_id != source_frame_id_) {
    RCLCPP_ERROR(this->get_logger(),
                 "SLAM frame changed during recording: '%s' -> '%s'",
                 source_frame_id_.c_str(), message->header.frame_id.c_str());
    stop_recording("SLAM frame changed");
    return;
  }

  const auto &position = message->pose.pose.position;
  const auto &orientation = message->pose.pose.orientation;
  SlamPathPose raw_pose;
  raw_pose.x = position.x;
  raw_pose.y = position.y;
  raw_pose.z = position.z;
  raw_pose.qx = orientation.x;
  raw_pose.qy = orientation.y;
  raw_pose.qz = orientation.z;
  raw_pose.qw = orientation.w;
  if (!is_finite_pose(raw_pose) || !normalize_quaternion(raw_pose)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "invalid SLAM pose received; sample ignored");
    return;
  }

  // 必须在进入滑动窗口前检测原始定位跳变。否则一个大跳变会被均值滤波拆成
  // 多个小位移，最终绕过航路点的相邻距离检查。
  if (latest_raw_pose_.has_value()) {
    const double raw_step = position_distance(*latest_raw_pose_, raw_pose);
    if (raw_step > max_neighbor_dist_m_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "SLAM position jump %.2f m exceeds %.2f m; sample ignored", raw_step,
          max_neighbor_dist_m_);
      return;
    }
  }
  latest_raw_pose_ = raw_pose;

  // 可选处理开启时用在线滑动均值结果判定关键帧；关闭时直接使用原始有效位姿。
  const SlamPathPose record_pose =
      path_processor_ ? path_processor_->filter(raw_pose) : raw_pose;

  const rclcpp::Time now = this->get_clock()->now();
  if (recorded_points_.empty()) {
    recorded_points_.push_back(record_pose);
    last_record_stamp_ = now;
    publish_visualization(recorded_points_);
    return;
  }

  const double elapsed = last_record_stamp_.has_value()
                             ? (now - *last_record_stamp_).seconds()
                             : min_record_interval_s_;
  if (elapsed >= 0.0 && elapsed < min_record_interval_s_) {
    return;
  }

  const double distance =
      position_distance(recorded_points_.back(), record_pose);
  if (distance > max_neighbor_dist_m_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "filtered path point is %.2f m from the last recorded "
                         "point (> %.2f m); "
                         "point ignored",
                         distance, max_neighbor_dist_m_);
    return;
  }
  const double xy_distance =
      horizontal_distance(recorded_points_.back(), record_pose);
  const double z_distance =
      vertical_distance(recorded_points_.back(), record_pose);
  // 第二层稀疏化：水平或竖直位移任一达到各自阈值才保留。
  if (xy_distance >= min_xy_m_ || z_distance >= min_z_m_) {
    recorded_points_.push_back(record_pose);
    last_record_stamp_ = now;
    publish_visualization(recorded_points_);
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "recording path: %zu points, latest=(%.2f, %.2f, %.2f)",
        recorded_points_.size(), record_pose.x, record_pose.y, record_pose.z);
  }
}

} // namespace px4_path

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto recorder = std::make_shared<px4_path::PathRecordNode>();
  rclcpp::spin(recorder);
  recorder.reset();
  rclcpp::shutdown();
  return 0;
}
