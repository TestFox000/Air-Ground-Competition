#include "px4_path/path_record_process.hpp"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <stdexcept>
#include <string>

namespace px4_path {

PathRecordProcessor::PathRecordProcessor(rclcpp::Node &node)
    : logger_(node.get_logger()) {
  filter_window_ = node.declare_parameter<int>("filter_window", 5);
  smoothing_window_ = node.declare_parameter<int>("smoothing_window", 3);
  min_smoothed_point_distance_m_ =
      node.declare_parameter<double>("min_smoothed_point_dist_m", 0.05);
}

bool PathRecordProcessor::validate_parameters() const {
  bool valid = true;
  const auto reject = [this, &valid](const char *name,
                                     const std::string &reason) {
    RCLCPP_ERROR(logger_, "invalid processing parameter '%s': %s", name,
                 reason.c_str());
    valid = false;
  };

  if (filter_window_ < 1 || filter_window_ > 101) {
    reject("filter_window", "must be in [1, 101]");
  }
  if (smoothing_window_ < 1 || smoothing_window_ > 101) {
    reject("smoothing_window", "must be in [1, 101]");
  }
  if (!std::isfinite(min_smoothed_point_distance_m_) ||
      min_smoothed_point_distance_m_ < 0.0) {
    reject("min_smoothed_point_dist_m", "must be finite and >= 0");
  }
  return valid;
}

void PathRecordProcessor::log_configuration() const {
  RCLCPP_INFO(logger_,
              "path filter/smoothing enabled: filter_window=%d "
              "smoothing_window=%d min_smoothed_step=%.3fm",
              filter_window_, smoothing_window_,
              min_smoothed_point_distance_m_);
  if (smoothing_window_ > 1) {
    RCLCPP_WARN(
        logger_,
        "path smoothing is enabled: saved interior waypoints may move and "
        "sharp turns may be shortened");
  }
}

void PathRecordProcessor::write_metadata(std::ostream &output) const {
  output << "# filter_smoothing_enabled: true\n";
  output << "# filtering: input_window=" << filter_window_
         << " path_smoothing_window=" << smoothing_window_
         << " min_smoothed_point_dist=" << min_smoothed_point_distance_m_
         << "m\n";
}

void PathRecordProcessor::reset() { filter_buffer_.clear(); }

SlamPathPose PathRecordProcessor::filter(const SlamPathPose &pose) {
  filter_buffer_.push_back(pose);
  const std::size_t filter_size = static_cast<std::size_t>(filter_window_);
  if (filter_buffer_.size() > filter_size) {
    filter_buffer_.erase(filter_buffer_.begin());
  }
  return average_poses(filter_buffer_, 0, filter_buffer_.size());
}

std::vector<SlamPathPose>
PathRecordProcessor::finalize(const std::vector<SlamPathPose> &poses) const {
  return remove_close_points(
      smooth_path(poses, static_cast<std::size_t>(smoothing_window_)),
      min_smoothed_point_distance_m_);
}

bool PathRecordProcessor::normalize_quaternion(SlamPathPose &pose) {
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

double PathRecordProcessor::position_distance(const SlamPathPose &a,
                                              const SlamPathPose &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double PathRecordProcessor::quaternion_dot(const SlamPathPose &a,
                                           const SlamPathPose &b) {
  return a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw;
}

SlamPathPose
PathRecordProcessor::average_poses(const std::vector<SlamPathPose> &poses,
                                   std::size_t begin, std::size_t end) {
  if (begin >= end || end > poses.size()) {
    throw std::invalid_argument("average_poses received an invalid range");
  }

  SlamPathPose mean{};
  mean.qx = 0.0;
  mean.qy = 0.0;
  mean.qz = 0.0;
  mean.qw = 0.0;

  const SlamPathPose &reference = poses[begin];
  for (std::size_t i = begin; i < end; ++i) {
    const SlamPathPose &pose = poses[i];
    mean.x += pose.x;
    mean.y += pose.y;
    mean.z += pose.z;

    const double sign = quaternion_dot(reference, pose) < 0.0 ? -1.0 : 1.0;
    mean.qx += sign * pose.qx;
    mean.qy += sign * pose.qy;
    mean.qz += sign * pose.qz;
    mean.qw += sign * pose.qw;
  }

  const double count = static_cast<double>(end - begin);
  mean.x /= count;
  mean.y /= count;
  mean.z /= count;
  if (!normalize_quaternion(mean)) {
    mean.qx = reference.qx;
    mean.qy = reference.qy;
    mean.qz = reference.qz;
    mean.qw = reference.qw;
  }
  return mean;
}

std::vector<SlamPathPose>
PathRecordProcessor::smooth_path(const std::vector<SlamPathPose> &poses,
                                 std::size_t window_size) {
  if (poses.size() < 3 || window_size <= 1) {
    return poses;
  }
  if (window_size % 2 == 0) {
    ++window_size;
  }

  std::vector<SlamPathPose> smoothed = poses;
  const std::size_t half_window = window_size / 2;
  for (std::size_t i = 1; i + 1 < poses.size(); ++i) {
    const std::size_t begin = i > half_window ? i - half_window : 0;
    const std::size_t end = std::min(poses.size(), i + half_window + 1);
    smoothed[i] = average_poses(poses, begin, end);
  }
  return smoothed;
}

std::vector<SlamPathPose>
PathRecordProcessor::remove_close_points(const std::vector<SlamPathPose> &poses,
                                         double min_distance_m) {
  if (poses.size() < 3 || min_distance_m <= 0.0) {
    return poses;
  }

  std::vector<SlamPathPose> compacted;
  compacted.reserve(poses.size());
  compacted.push_back(poses.front());
  for (std::size_t i = 1; i + 1 < poses.size(); ++i) {
    if (position_distance(compacted.back(), poses[i]) >= min_distance_m) {
      compacted.push_back(poses[i]);
    }
  }

  while (compacted.size() > 1 &&
         position_distance(compacted.back(), poses.back()) < min_distance_m) {
    compacted.pop_back();
  }
  compacted.push_back(poses.back());
  return compacted;
}

} // namespace px4_path
