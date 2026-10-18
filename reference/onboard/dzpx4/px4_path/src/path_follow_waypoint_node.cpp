#include "px4_path/path_follow_waypoint_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>

using namespace std::chrono_literals;

namespace px4_path {

PathFollowWaypointNode::PathFollowWaypointNode()
    : rclcpp::Node("path_follow_node") {
  path_dir_ = this->declare_parameter<std::string>("path_dir", "");
  path_file_ =
      this->declare_parameter<std::string>("path_file", "recorded_path.txt");
  pursuit_rate_ = this->declare_parameter<double>("pursuit_rate", 10.0);
  switch_tol_ = this->declare_parameter<double>("switch_tol", 0.30);
  terminal_tol_ = this->declare_parameter<double>("terminal_tol", 0.20);
  max_step_dist_ = this->declare_parameter<double>("max_step_dist", 1.5);
  max_neighbor_dist_m_ =
      this->declare_parameter<double>("max_neighbor_dist_m", 5.0);

  auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
  sub_local_pos_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/fmu/out/vehicle_local_position", px4_qos,
          std::bind(&PathFollowWaypointNode::on_local_position, this,
                    std::placeholders::_1));

  sub_goto_enabled_ = this->create_subscription<std_msgs::msg::Bool>(
      "/px4_control/goto_enabled", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PathFollowWaypointNode::on_goto_enabled, this,
                std::placeholders::_1));

  pub_goto_ = this->create_publisher<px4_control::msg::GotoTarget>(
      "/px4_control/goto", 10);

  const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(1.0, pursuit_rate_)));
  timer_ = this->create_wall_timer(
      period, std::bind(&PathFollowWaypointNode::on_timer, this));

  if (!prepare_path_directory()) {
    RCLCPP_ERROR(
        this->get_logger(),
        "path directory setup failed; node will stay in INIT and not publish");
    return;
  }

  if (reload_path()) {
    set_state(WaypointFollowState::LOADED, "path loaded on startup");
  } else {
    RCLCPP_WARN(
        this->get_logger(),
        "startup path load failed; press '4' after creating a valid path");
  }

  RCLCPP_INFO(this->get_logger(),
              "waypoint follower up. file=%s rate=%.1fHz switch_tol=%.2f "
              "terminal_tol=%.2f "
              "max_step=%.2f max_neighbor=%.2f",
              resolved_path_file_.c_str(), pursuit_rate_, switch_tol_,
              terminal_tol_, max_step_dist_, max_neighbor_dist_m_);
}

// ============================================================================
// helpers
// ============================================================================

const char *PathFollowWaypointNode::state_name(WaypointFollowState s) const {
  switch (s) {
  case WaypointFollowState::INIT:
    return "INIT";
  case WaypointFollowState::LOADED:
    return "LOADED";
  case WaypointFollowState::PURSUING:
    return "PURSUING";
  case WaypointFollowState::PAUSED:
    return "PAUSED";
  case WaypointFollowState::HOLD:
    return "HOLD";
  }
  return "?";
}

void PathFollowWaypointNode::set_state(WaypointFollowState s,
                                       const char *reason) {
  if (state_ == s) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "pursuit: %s -> %s (%s)", state_name(state_),
              state_name(s), reason ? reason : "");
  state_ = s;
}

double PathFollowWaypointNode::distance(const Waypoint &a, const Waypoint &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Waypoint PathFollowWaypointNode::clamp_step(const Waypoint &from_pos,
                                            const Waypoint &target) const {
  const double d = distance(from_pos, target);
  if (d <= max_step_dist_ || d < 1e-6) {
    return target;
  }
  const double s = max_step_dist_ / d;
  Waypoint c = target;
  c.x = static_cast<float>(from_pos.x + (target.x - from_pos.x) * s);
  c.y = static_cast<float>(from_pos.y + (target.y - from_pos.y) * s);
  c.z = static_cast<float>(from_pos.z + (target.z - from_pos.z) * s);
  // yaw stays at target yaw — clamping xyz only; PX4 will slew heading.
  return c;
}

bool PathFollowWaypointNode::prepare_path_directory() {
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

bool PathFollowWaypointNode::reload_path() {
  if (!std::isfinite(max_neighbor_dist_m_) || max_neighbor_dist_m_ <= 0.0) {
    points_.clear();
    cursor_ = 0;
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
    cursor_ = 0;
    RCLCPP_ERROR(
        this->get_logger(),
        "path load failed: %s; any previously loaded path has been invalidated",
        r.error.c_str());
    return false;
  }
  points_ = std::move(r.points);
  cursor_ = 0;
  RCLCPP_INFO(this->get_logger(),
              "path loaded: %zu FAST-LIO SLAM points converted via ENU to PX4 "
              "NED from %s",
              points_.size(), resolved_path_file_.c_str());
  return true;
}

void PathFollowWaypointNode::publish_goto(const Waypoint &wp) {
  px4_control::msg::GotoTarget m;
  m.active = true;
  m.x = wp.x;
  m.y = wp.y;
  m.z = wp.z;
  m.vx = 0.0f;
  m.vy = 0.0f;
  m.vz = 0.0f;
  m.ax = 0.0f;
  m.ay = 0.0f;
  m.az = 0.0f;
  m.yaw = wp.yaw;
  m.yawspeed = 0.0f;
  pub_goto_->publish(m);
}

void PathFollowWaypointNode::publish_stop() {
  px4_control::msg::GotoTarget m;
  m.active = false;
  pub_goto_->publish(m);
}

void PathFollowWaypointNode::activate_if_ready() {
  if (!goto_enabled_ || !local_pos_valid_ ||
      state_ != WaypointFollowState::LOADED) {
    return;
  }
  if (points_.empty()) {
    set_state(WaypointFollowState::INIT, "loaded path is empty");
    publish_stop();
    return;
  }
  if (!std::isfinite(switch_tol_) || switch_tol_ <= 0.0 ||
      !std::isfinite(terminal_tol_) || terminal_tol_ <= 0.0 ||
      !std::isfinite(max_step_dist_) || max_step_dist_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "GOTO start refused: switch_tol, terminal_tol and "
                 "max_step_dist must be finite and > 0");
    publish_stop();
    return;
  }
  set_state(WaypointFollowState::PURSUING, "GOTO enabled");
}

// ============================================================================
// callbacks
// ============================================================================

void PathFollowWaypointNode::on_local_position(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
  current_pos_.x = m->x;
  current_pos_.y = m->y;
  current_pos_.z = m->z;
  local_pos_valid_ = m->xy_valid && m->z_valid;

  if (state_ == WaypointFollowState::INIT && local_pos_valid_ &&
      !points_.empty()) {
    set_state(WaypointFollowState::LOADED, "local_position first sample");
  }
  activate_if_ready();
}

void PathFollowWaypointNode::on_goto_enabled(
    const std_msgs::msg::Bool::SharedPtr m) {
  goto_enabled_ = m->data;
  if (!goto_enabled_) {
    if (state_ == WaypointFollowState::PURSUING ||
        state_ == WaypointFollowState::HOLD) {
      paused_from_ = state_;
      set_state(WaypointFollowState::PAUSED, "GOTO disabled");
    }
    return;
  }

  if (state_ == WaypointFollowState::PAUSED) {
    set_state(paused_from_, "GOTO re-enabled; resume saved cursor");
    return;
  }
  activate_if_ready();
}

void PathFollowWaypointNode::on_timer() {
  if (!local_pos_valid_) {
    return;
  }

  if (state_ == WaypointFollowState::PURSUING) {
    // Advance cursor: while we're inside switch_tol of the current target and
    // more points remain, jump to the next one. Index-based, no arc length.
    while (cursor_ + 1 < points_.size() &&
           distance(current_pos_, points_[cursor_]) < switch_tol_) {
      ++cursor_;
      RCLCPP_INFO(this->get_logger(), "cursor -> %zu / %zu", cursor_,
                  points_.size());
    }

    // Safety valve: if the raw target is >max_step_dist away (SLAM jump, or
    // path just plain far from wherever pursuit started), clamp to a point
    // max_step_dist toward the true target. Position setpoint stays finite,
    // and PX4's MPC_XY_VEL_MAX will still cap real speed.
    const Waypoint raw = points_[cursor_];
    const Waypoint sent = clamp_step(current_pos_, raw);
    publish_goto(sent);

    // Terminal condition: we're on the last point and inside terminal_tol.
    if (cursor_ + 1 == points_.size() &&
        distance(current_pos_, points_.back()) < terminal_tol_) {
      set_state(WaypointFollowState::HOLD, "reached terminal point");
    }
  } else if (state_ == WaypointFollowState::HOLD) {
    // Keep re-asserting the terminal point so control_node's pursuit_active_
    // never times out and WASDQE stays locked out.
    publish_goto(points_.back());
  }
}

} // namespace px4_path
