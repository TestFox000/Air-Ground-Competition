#include "px4_path/path_follow_trajectory_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>

using namespace std::chrono_literals;

namespace px4_path {

PathFollowTrajectoryNode::PathFollowTrajectoryNode()
    : rclcpp::Node("path_follow_node") {
  path_dir_ = this->declare_parameter<std::string>("path_dir", "");
  path_file_ =
      this->declare_parameter<std::string>("path_file", "recorded_path.txt");
  pursuit_rate_ = this->declare_parameter<double>("pursuit_rate", 50.0);
  terminal_tol_ = this->declare_parameter<double>("terminal_tol", 0.20);
  max_step_dist_ = this->declare_parameter<double>("max_step_dist", 1.5);
  max_neighbor_dist_m_ =
      this->declare_parameter<double>("max_neighbor_dist_m", 5.0);
  max_speed_mps_ = this->declare_parameter<double>("max_speed_mps", 0.5);
  max_accel_mps2_ = this->declare_parameter<double>("max_accel_mps2", 0.5);
  max_jerk_mps3_ = this->declare_parameter<double>("max_jerk_mps3", 1.0);
  max_centripetal_accel_mps2_ =
      this->declare_parameter<double>("max_centripetal_accel_mps2", 0.35);
  profile_resolution_m_ =
      this->declare_parameter<double>("profile_resolution_m", 0.10);
  segment_split_turn_deg_ =
      this->declare_parameter<double>("segment_split_turn_deg", 150.0);
  segment_switch_speed_mps_ =
      this->declare_parameter<double>("segment_switch_speed_mps", 0.15);
  max_tracking_error_m_ =
      this->declare_parameter<double>("max_tracking_error_m", 1.0);
  tracking_slowdown_error_m_ =
      this->declare_parameter<double>("tracking_slowdown_error_m", 0.30);
  tracking_pause_error_m_ =
      this->declare_parameter<double>("tracking_pause_error_m", 0.70);
  max_time_scale_rate_per_s_ =
      this->declare_parameter<double>("max_time_scale_rate_per_s", 1.0);
  yaw_mode_ = this->declare_parameter<std::string>("yaw_mode", "fixed");
  yaw_lookahead_s_ = this->declare_parameter<double>("yaw_lookahead_s", 0.5);
  max_yaw_rate_rps_ = this->declare_parameter<double>("max_yaw_rate_rps", 0.5);

  auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
  sub_local_pos_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/fmu/out/vehicle_local_position", px4_qos,
          std::bind(&PathFollowTrajectoryNode::on_local_position, this,
                    std::placeholders::_1));

  sub_goto_enabled_ = this->create_subscription<std_msgs::msg::Bool>(
      "/px4_control/goto_enabled", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PathFollowTrajectoryNode::on_goto_enabled, this,
                std::placeholders::_1));

  pub_goto_ = this->create_publisher<px4_control::msg::GotoTarget>(
      "/px4_control/goto", 10);

  const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(1.0, pursuit_rate_)));
  timer_ = this->create_wall_timer(
      period, std::bind(&PathFollowTrajectoryNode::on_timer, this));

  if (!prepare_path_directory()) {
    RCLCPP_ERROR(
        this->get_logger(),
        "path directory setup failed; node will stay in INIT and not publish");
    return;
  }

  if (reload_path()) {
    set_state(TrajectoryFollowState::LOADED, "path loaded on startup");
  } else {
    RCLCPP_WARN(
        this->get_logger(),
        "startup path load failed; press '4' after creating a valid path");
  }

  RCLCPP_INFO(this->get_logger(),
              "trajectory follower up. file=%s rate=%.1fHz terminal_tol=%.2f "
              "limits(v=%.2f a=%.2f j=%.2f alat=%.2f) split=%.1fdeg "
              "yaw=%s max_yaw_rate=%.2f "
              "tracking(slow=%.2f pause=%.2f abort=%.2f)",
              resolved_path_file_.c_str(), pursuit_rate_, terminal_tol_,
              max_speed_mps_, max_accel_mps2_, max_jerk_mps3_,
              max_centripetal_accel_mps2_, segment_split_turn_deg_,
              yaw_mode_.c_str(), max_yaw_rate_rps_, tracking_slowdown_error_m_,
              tracking_pause_error_m_, max_tracking_error_m_);
}

// ============================================================================
// helpers
// ============================================================================

const char *
PathFollowTrajectoryNode::state_name(TrajectoryFollowState s) const {
  switch (s) {
  case TrajectoryFollowState::INIT:
    return "INIT";
  case TrajectoryFollowState::LOADED:
    return "LOADED";
  case TrajectoryFollowState::PURSUING:
    return "PURSUING";
  case TrajectoryFollowState::PAUSED:
    return "PAUSED";
  case TrajectoryFollowState::HOLD:
    return "HOLD";
  }
  return "?";
}

void PathFollowTrajectoryNode::set_state(TrajectoryFollowState s,
                                         const char *reason) {
  if (state_ == s) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "pursuit: %s -> %s (%s)", state_name(state_),
              state_name(s), reason ? reason : "");
  state_ = s;
}

double PathFollowTrajectoryNode::distance(const Waypoint &a,
                                          const Waypoint &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double PathFollowTrajectoryNode::wrap_pi(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

bool PathFollowTrajectoryNode::prepare_path_directory() {
  namespace fs = std::filesystem;

  if (path_dir_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "path_dir is empty; configure it in "
                                     "px4_path/config/params.yaml");
    return false;
  }
  const std::string &dir = path_dir_;

  std::error_code ec;
  if (!fs::exists(dir, ec)) {
    if (!fs::create_directories(dir, ec)) {
      RCLCPP_ERROR(this->get_logger(), "cannot create path dir '%s': %s",
                   dir.c_str(), ec.message().c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "created path dir: %s", dir.c_str());
  }

  resolved_path_file_ = dir + "/" + path_file_;
  return true;
}

bool PathFollowTrajectoryNode::reload_path() {
  if (!std::isfinite(max_neighbor_dist_m_) || max_neighbor_dist_m_ <= 0.0) {
    points_.clear();
    RCLCPP_ERROR(
        this->get_logger(),
        "path load refused: max_neighbor_dist_m must be finite and > 0; "
        "any previously loaded path has been invalidated");
    return false;
  }

  LoadOptions options;
  options.max_neighbor_dist = max_neighbor_dist_m_;
  LoadResult r = load_path(resolved_path_file_, options);
  for (const auto &w : r.warnings) {
    RCLCPP_WARN(this->get_logger(), "path warn: %s", w.c_str());
  }
  if (!r.ok) {
    points_.clear();
    RCLCPP_ERROR(
        this->get_logger(),
        "path load failed: %s; any previously loaded path has been invalidated",
        r.error.c_str());
    return false;
  }
  points_ = std::move(r.points);
  RCLCPP_INFO(this->get_logger(),
              "path loaded: %zu FAST-LIO SLAM points converted via ENU to PX4 "
              "NED from %s",
              points_.size(), resolved_path_file_.c_str());
  return true;
}

std::vector<std::vector<Waypoint>>
PathFollowTrajectoryNode::make_execution_segments() const {
  std::vector<std::vector<Waypoint>> segments;
  if (points_.empty()) {
    return segments;
  }

  const std::vector<PathTurnSplit> splits =
      find_path_turn_splits(points_, segment_split_turn_deg_);
  std::vector<Waypoint> join;
  join.reserve(5);
  join.push_back(current_pos_);

  const double start_distance = distance(current_pos_, points_.front());
  if (start_distance > 1e-4) {
    const double max_join_step = std::max(0.05, max_step_dist_);
    const std::size_t segment_count =
        static_cast<std::size_t>(std::ceil(start_distance / max_join_step));
    for (std::size_t i = 1; i < segment_count; ++i) {
      const double ratio = static_cast<double>(i) / segment_count;
      Waypoint intermediate{};
      intermediate.x = static_cast<float>(
          current_pos_.x + (points_.front().x - current_pos_.x) * ratio);
      intermediate.y = static_cast<float>(
          current_pos_.y + (points_.front().y - current_pos_.y) * ratio);
      intermediate.z = static_cast<float>(
          current_pos_.z + (points_.front().z - current_pos_.z) * ratio);
      intermediate.yaw = static_cast<float>(
          wrap_pi(current_pos_.yaw +
                  wrap_pi(points_.front().yaw - current_pos_.yaw) * ratio));
      join.push_back(intermediate);
    }
  }

  std::size_t begin_index = 0;
  for (std::size_t segment_index = 0; segment_index <= splits.size();
       ++segment_index) {
    const std::size_t end_index = segment_index < splits.size()
                                      ? splits[segment_index].waypoint_index
                                      : points_.size() - 1;
    std::vector<Waypoint> segment;
    if (segment_index == 0) {
      segment = join;
    }
    segment.insert(segment.end(), points_.begin() + begin_index,
                   points_.begin() + end_index + 1);
    segments.push_back(std::move(segment));
    begin_index = end_index;
  }
  return segments;
}

bool PathFollowTrajectoryNode::start_trajectory() {
  if (!local_pos_valid_ || points_.empty()) {
    return false;
  }
  if (!std::isfinite(max_step_dist_) || max_step_dist_ <= 0.0 ||
      !std::isfinite(terminal_tol_) || terminal_tol_ <= 0.0 ||
      !std::isfinite(max_tracking_error_m_) ||
      !std::isfinite(yaw_lookahead_s_) || yaw_lookahead_s_ < 0.0 ||
      !std::isfinite(max_yaw_rate_rps_) || max_yaw_rate_rps_ < 0.0 ||
      !std::isfinite(segment_split_turn_deg_) ||
      segment_split_turn_deg_ < 0.0 || segment_split_turn_deg_ > 180.0 ||
      !std::isfinite(segment_switch_speed_mps_) ||
      segment_switch_speed_mps_ < 0.0 ||
      !std::isfinite(tracking_slowdown_error_m_) ||
      tracking_slowdown_error_m_ < 0.0 ||
      !std::isfinite(tracking_pause_error_m_) ||
      tracking_pause_error_m_ <= tracking_slowdown_error_m_ ||
      !std::isfinite(max_time_scale_rate_per_s_) ||
      max_time_scale_rate_per_s_ <= 0.0 ||
      (max_tracking_error_m_ > 0.0 &&
       tracking_pause_error_m_ >= max_tracking_error_m_)) {
    RCLCPP_ERROR(
        this->get_logger(),
        "trajectory start refused: invalid join/tolerance/yaw parameter");
    return false;
  }
  if (yaw_mode_ != "tangent" && yaw_mode_ != "recorded" &&
      yaw_mode_ != "fixed") {
    RCLCPP_ERROR(this->get_logger(), "trajectory start refused: yaw_mode must "
                                     "be tangent, recorded or fixed");
    return false;
  }

  TrajectoryLimits limits;
  limits.max_speed_mps = max_speed_mps_;
  limits.max_accel_mps2 = max_accel_mps2_;
  limits.max_jerk_mps3 = max_jerk_mps3_;
  limits.max_centripetal_accel_mps2 = max_centripetal_accel_mps2_;
  limits.profile_resolution_m = profile_resolution_m_;
  const std::vector<PathTurnSplit> splits =
      find_path_turn_splits(points_, segment_split_turn_deg_);
  for (const PathTurnSplit &split : splits) {
    RCLCPP_INFO(this->get_logger(),
                "automatic trajectory split at waypoint %zu (turn=%.1fdeg)",
                split.waypoint_index, split.turn_angle_deg);
  }

  trajectories_.clear();
  active_segment_index_ = 0;
  const auto execution_segments = make_execution_segments();
  double total_length = 0.0;
  double total_duration = 0.0;
  for (std::size_t i = 0; i < execution_segments.size(); ++i) {
    MotionTrajectory trajectory;
    std::string error;
    if (!trajectory.build(execution_segments[i], limits, error)) {
      trajectories_.clear();
      RCLCPP_ERROR(this->get_logger(),
                   "trajectory segment %zu/%zu generation failed: %s", i + 1,
                   execution_segments.size(), error.c_str());
      return false;
    }
    total_length += trajectory.length();
    total_duration += trajectory.duration();
    RCLCPP_INFO(this->get_logger(),
                "trajectory segment %zu/%zu: length=%.2fm duration=%.2fs "
                "measured(v=%.2f a=%.2f j=%.2f alat=%.2f) scale=%.2f",
                i + 1, execution_segments.size(), trajectory.length(),
                trajectory.duration(), trajectory.max_sampled_speed(),
                trajectory.max_sampled_accel(), trajectory.max_sampled_jerk(),
                trajectory.max_sampled_centripetal_accel(),
                trajectory.time_scale());
    trajectories_.push_back(std::move(trajectory));
  }

  trajectory_start_time_ = this->get_clock()->now();
  last_timer_time_ = trajectory_start_time_;
  trajectory_elapsed_s_ = 0.0;
  reference_time_scale_ = 1.0;
  command_yaw_ = current_pos_.yaw;
  RCLCPP_INFO(this->get_logger(),
              "trajectory ready: %zu segment(s), total_length=%.2fm "
              "nominal_duration=%.2fs",
              trajectories_.size(), total_length, total_duration);
  return true;
}

void PathFollowTrajectoryNode::publish_reference(const TrajectorySample &input,
                                                 double timer_dt_s,
                                                 double time_scale,
                                                 double time_scale_rate) {
  TrajectorySample sample = input;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    sample.acceleration[axis] =
        input.acceleration[axis] * time_scale * time_scale +
        input.velocity[axis] * time_scale_rate;
    sample.velocity[axis] = input.velocity[axis] * time_scale;
  }
  double desired_yaw = command_yaw_;

  if (yaw_mode_ == "recorded") {
    desired_yaw = sample.yaw;
  } else if (yaw_mode_ == "tangent") {
    const MotionTrajectory &trajectory = trajectories_[active_segment_index_];
    const TrajectorySample forward = trajectory.sample(std::min(
        trajectory.duration(), trajectory_elapsed_s_ + yaw_lookahead_s_));
    const double dx = forward.position[0] - sample.position[0];
    const double dy = forward.position[1] - sample.position[1];
    if (std::hypot(dx, dy) > 0.05) {
      desired_yaw = std::atan2(dy, dx);
    }
  }

  const double safe_dt =
      timer_dt_s > 0.0 ? timer_dt_s : 1.0 / std::max(1.0, pursuit_rate_);
  const double max_yaw_step = std::max(0.0, max_yaw_rate_rps_) * safe_dt;
  const double yaw_delta = std::clamp(wrap_pi(desired_yaw - command_yaw_),
                                      -max_yaw_step, max_yaw_step);
  command_yaw_ = wrap_pi(command_yaw_ + yaw_delta);
  sample.yaw = command_yaw_;
  sample.yaw_speed = yaw_delta / safe_dt;

  px4_control::msg::GotoTarget message;
  message.active = true;
  message.x = static_cast<float>(sample.position[0]);
  message.y = static_cast<float>(sample.position[1]);
  message.z = static_cast<float>(sample.position[2]);
  message.vx = static_cast<float>(sample.velocity[0]);
  message.vy = static_cast<float>(sample.velocity[1]);
  message.vz = static_cast<float>(sample.velocity[2]);
  message.ax = static_cast<float>(sample.acceleration[0]);
  message.ay = static_cast<float>(sample.acceleration[1]);
  message.az = static_cast<float>(sample.acceleration[2]);
  message.yaw = static_cast<float>(sample.yaw);
  message.yawspeed = static_cast<float>(sample.yaw_speed);
  pub_goto_->publish(message);
}

void PathFollowTrajectoryNode::publish_stop() {
  px4_control::msg::GotoTarget m;
  m.active = false;
  pub_goto_->publish(m);
}

void PathFollowTrajectoryNode::activate_if_ready() {
  if (!goto_enabled_ || !local_pos_valid_ ||
      state_ != TrajectoryFollowState::LOADED) {
    return;
  }
  if (start_trajectory()) {
    set_state(TrajectoryFollowState::PURSUING, "GOTO enabled");
  } else {
    publish_stop();
  }
}

// ============================================================================
// callbacks
// ============================================================================

void PathFollowTrajectoryNode::on_local_position(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
  current_pos_.x = m->x;
  current_pos_.y = m->y;
  current_pos_.z = m->z;
  current_pos_.yaw = m->heading;
  current_speed_mps_ = std::sqrt(static_cast<double>(m->vx) * m->vx +
                                 static_cast<double>(m->vy) * m->vy +
                                 static_cast<double>(m->vz) * m->vz);
  local_pos_valid_ =
      m->xy_valid && m->z_valid && std::isfinite(current_pos_.x) &&
      std::isfinite(current_pos_.y) && std::isfinite(current_pos_.z) &&
      std::isfinite(current_pos_.yaw);

  if (state_ == TrajectoryFollowState::INIT && local_pos_valid_ &&
      !points_.empty()) {
    set_state(TrajectoryFollowState::LOADED, "local_position first sample");
  }
  activate_if_ready();
}

void PathFollowTrajectoryNode::on_goto_enabled(
    const std_msgs::msg::Bool::SharedPtr m) {
  goto_enabled_ = m->data;
  if (!goto_enabled_) {
    if (state_ == TrajectoryFollowState::PURSUING ||
        state_ == TrajectoryFollowState::HOLD) {
      paused_from_ = state_;
      set_state(TrajectoryFollowState::PAUSED, "GOTO disabled");
    }
    return;
  }

  if (state_ == TrajectoryFollowState::PAUSED) {
    last_timer_time_ = this->get_clock()->now();
    set_state(paused_from_, "GOTO re-enabled; resume saved trajectory phase");
    return;
  }
  activate_if_ready();
}

void PathFollowTrajectoryNode::on_timer() {
  if (!local_pos_valid_) {
    return;
  }

  if (state_ == TrajectoryFollowState::PURSUING) {
    if (trajectories_.empty() ||
        active_segment_index_ >= trajectories_.size()) {
      RCLCPP_ERROR(this->get_logger(), "trajectory state is empty or invalid");
      set_state(TrajectoryFollowState::LOADED, "invalid trajectory state");
      publish_stop();
      return;
    }
    const MotionTrajectory &trajectory = trajectories_[active_segment_index_];
    const rclcpp::Time now = this->get_clock()->now();
    double timer_dt = (now - last_timer_time_).seconds();
    last_timer_time_ = now;
    if (timer_dt <= 0.0 || timer_dt > 0.5) {
      timer_dt = 1.0 / std::max(1.0, pursuit_rate_);
    }

    const TrajectorySample current_reference =
        trajectory.sample(trajectory_elapsed_s_);
    Waypoint current_reference_position{};
    current_reference_position.x =
        static_cast<float>(current_reference.position[0]);
    current_reference_position.y =
        static_cast<float>(current_reference.position[1]);
    current_reference_position.z =
        static_cast<float>(current_reference.position[2]);
    const double current_error =
        distance(current_pos_, current_reference_position);

    double desired_time_scale = 1.0;
    if (current_error >= tracking_pause_error_m_) {
      desired_time_scale = 0.0;
    } else if (current_error > tracking_slowdown_error_m_) {
      desired_time_scale =
          (tracking_pause_error_m_ - current_error) /
          (tracking_pause_error_m_ - tracking_slowdown_error_m_);
    }
    const double previous_time_scale = reference_time_scale_;
    double time_scale_rate =
        std::clamp((desired_time_scale - previous_time_scale) / timer_dt,
                   -max_time_scale_rate_per_s_, max_time_scale_rate_per_s_);

    // Retiming changes acceleration by v * d(time_scale)/dt. Clamp that scalar
    // rate so the feed-forward remains inside the configured 3-D acceleration
    // sphere while the reference clock slows down or recovers.
    double velocity_sq = 0.0;
    double accel_sq = 0.0;
    double velocity_dot_accel = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double velocity = current_reference.velocity[axis];
      const double acceleration = current_reference.acceleration[axis] *
                                  previous_time_scale * previous_time_scale;
      velocity_sq += velocity * velocity;
      accel_sq += acceleration * acceleration;
      velocity_dot_accel += velocity * acceleration;
    }
    if (velocity_sq > 1e-9) {
      const double discriminant =
          velocity_dot_accel * velocity_dot_accel -
          velocity_sq * (accel_sq - max_accel_mps2_ * max_accel_mps2_);
      if (discriminant >= 0.0) {
        const double root = std::sqrt(discriminant);
        const double minimum_rate = (-velocity_dot_accel - root) / velocity_sq;
        const double maximum_rate = (-velocity_dot_accel + root) / velocity_sq;
        time_scale_rate =
            std::clamp(time_scale_rate, minimum_rate, maximum_rate);
      } else {
        time_scale_rate = 0.0;
      }
    }
    reference_time_scale_ =
        std::clamp(previous_time_scale + time_scale_rate * timer_dt, 0.0, 1.0);
    time_scale_rate = (reference_time_scale_ - previous_time_scale) / timer_dt;
    trajectory_elapsed_s_ = std::min(
        trajectory.duration(),
        trajectory_elapsed_s_ +
            0.5 * (previous_time_scale + reference_time_scale_) * timer_dt);

    const TrajectorySample sample = trajectory.sample(trajectory_elapsed_s_);
    Waypoint reference{};
    reference.x = static_cast<float>(sample.position[0]);
    reference.y = static_cast<float>(sample.position[1]);
    reference.z = static_cast<float>(sample.position[2]);
    const double tracking_error = distance(current_pos_, reference);
    if (max_tracking_error_m_ > 0.0 && tracking_error > max_tracking_error_m_) {
      RCLCPP_ERROR(this->get_logger(),
                   "trajectory aborted: tracking error %.2fm exceeds %.2fm",
                   tracking_error, max_tracking_error_m_);
      set_state(TrajectoryFollowState::LOADED, "tracking error limit");
      publish_stop();
      return;
    }

    const bool segment_time_complete =
        trajectory_elapsed_s_ >= trajectory.duration();
    if (segment_time_complete) {
      publish_reference(sample, timer_dt, 0.0, 0.0);
    } else {
      publish_reference(sample, timer_dt, reference_time_scale_,
                        time_scale_rate);
    }

    if (segment_time_complete && tracking_error < terminal_tol_) {
      const bool speed_ready =
          segment_switch_speed_mps_ <= 0.0 ||
          (std::isfinite(current_speed_mps_) &&
           current_speed_mps_ <= segment_switch_speed_mps_);
      if (active_segment_index_ + 1 < trajectories_.size()) {
        if (speed_ready) {
          ++active_segment_index_;
          trajectory_elapsed_s_ = 0.0;
          reference_time_scale_ = 1.0;
          last_timer_time_ = now;
          RCLCPP_INFO(this->get_logger(),
                      "trajectory segment switch: now executing %zu/%zu",
                      active_segment_index_ + 1, trajectories_.size());
        } else {
          RCLCPP_INFO_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "waiting at segment %zu endpoint: speed %.2fm/s > %.2fm/s",
              active_segment_index_ + 1, current_speed_mps_,
              segment_switch_speed_mps_);
        }
      } else {
        set_state(TrajectoryFollowState::HOLD, "reached terminal point");
      }
    }
  } else if (state_ == TrajectoryFollowState::HOLD) {
    // Re-assert the final p/v/a reference so control_node never times out.
    const double timer_dt = 1.0 / std::max(1.0, pursuit_rate_);
    const MotionTrajectory &trajectory = trajectories_.back();
    publish_reference(trajectory.sample(trajectory.duration()), timer_dt, 0.0,
                      0.0);
  }
}

} // namespace px4_path
