#include "px4_control/px4_control_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

namespace px4_control {

namespace {
constexpr uint8_t NAV_STATE_OFFBOARD = 14;
constexpr uint8_t ARMING_STATE_STANDBY = 1; // disarmed
constexpr uint8_t ARMING_STATE_ARMED = 2;

constexpr uint16_t VC_DO_SET_MODE = 176;
constexpr uint16_t VC_NAV_LAND = 21;
constexpr uint16_t VC_COMPONENT_ARM_DISARM = 400;

constexpr float ARM_PARAM1_ARM = 1.0f;
constexpr float ARM_PARAM1_DISARM = 0.0f;

constexpr float SET_MODE_PARAM1_CUSTOM = 1.0f;
constexpr float SET_MODE_PARAM2_OFFBOARD = 6.0f;
} // namespace

ControlNode::ControlNode() : rclcpp::Node("px4_control_node") {
  takeoff_height_ = this->declare_parameter<double>("takeoff_height", 1.2);
  step_xy_ = this->declare_parameter<double>("step_xy", 0.5);
  step_z_ = this->declare_parameter<double>("step_z", 0.1);
  loop_rate_hz_ = this->declare_parameter<int>("loop_rate_hz", 50);
  arrive_tol_ = this->declare_parameter<double>("arrive_tol", 0.15);
  heartbeat_before_offboard_ =
      this->declare_parameter<int>("heartbeat_before_offboard", 20);

  land_control_ = this->declare_parameter<bool>("land_control", false);
  land_fast_speed_ = this->declare_parameter<double>("land_fast_speed", 0.4);
  land_slow_speed_ = this->declare_parameter<double>("land_slow_speed", 0.2);
  land_slow_alt_ = this->declare_parameter<double>("land_slow_alt", 0.5);
  land_touch_z_ = this->declare_parameter<double>("land_touch_z", 0.05);

  yaw_step_rad_ = this->declare_parameter<double>("yaw_step_rad", 0.15);
  pursuit_goto_timeout_ =
      this->declare_parameter<double>("pursuit_goto_timeout", 1.0);

  auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  sub_local_pos_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/fmu/out/vehicle_local_position", px4_qos,
          std::bind(&ControlNode::on_local_position, this,
                    std::placeholders::_1));

  sub_status_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos,
      std::bind(&ControlNode::on_vehicle_status, this, std::placeholders::_1));

  sub_cmd_ = this->create_subscription<msg::ControlCommand>(
      "/px4_control/command", 10,
      std::bind(&ControlNode::on_command, this, std::placeholders::_1));

  sub_goto_ = this->create_subscription<msg::GotoTarget>(
      "/px4_control/goto", 10,
      std::bind(&ControlNode::on_goto, this, std::placeholders::_1));

  pub_offboard_mode_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(
          "/fmu/in/offboard_control_mode", 10);
  pub_trajectory_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
  pub_command_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);
  pub_goto_enabled_ = this->create_publisher<std_msgs::msg::Bool>(
      "/px4_control/goto_enabled", rclcpp::QoS(1).reliable().transient_local());
  const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(1, loop_rate_hz_)));
  timer_ =
      this->create_wall_timer(period, std::bind(&ControlNode::on_timer, this));

  RCLCPP_INFO(
      this->get_logger(),
      "px4_control_node up. takeoff=%.2fm step_xy=%.2f step_z=%.2f rate=%dHz "
      "land_control=%s (fast=%.2f slow=%.2f slow_alt=%.2f)",
      takeoff_height_, step_xy_, step_z_, loop_rate_hz_,
      land_control_ ? "NAV_LAND" : "self-descent", land_fast_speed_,
      land_slow_speed_, land_slow_alt_);
  publish_goto_enabled();
}

// ============================================================================
// helpers
// ============================================================================

uint64_t ControlNode::now_us() {
  return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
}

bool ControlNode::is_offboard() const {
  return status_received_ && last_status_.nav_state == NAV_STATE_OFFBOARD;
}

bool ControlNode::is_armed() const {
  return status_received_ && last_status_.arming_state == ARMING_STATE_ARMED;
}

bool ControlNode::is_on_ground() const {
  if (!status_received_) {
    return false;
  }
  if (last_status_.arming_state == ARMING_STATE_STANDBY) {
    return true;
  }
  // Armed: pose-based only. takeoff_time is unreliable when we're actively
  // sending descent setpoints (thrust stays up, so land_detector never fires).
  return local_pos_valid_ && std::fabs(current_.z) < 0.20f &&
         std::fabs(current_vz_) < 0.15f;
}

const char *ControlNode::state_name(FlightState s) const {
  switch (s) {
  case FlightState::DISARMED:
    return "DISARMED";
  case FlightState::ARMING:
    return "ARMING";
  case FlightState::TAKEOFF:
    return "TAKEOFF";
  case FlightState::HOVER:
    return "HOVER";
  case FlightState::LANDING:
    return "LANDING";
  case FlightState::LANDED:
    return "LANDED";
  }
  return "?";
}

void ControlNode::set_state(FlightState s, const char *reason) {
  if (state_ == s) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "state: %s -> %s (%s)", state_name(state_),
              state_name(s), reason ? reason : "");
  state_ = s;
}

void ControlNode::publish_goto_enabled() {
  std_msgs::msg::Bool message;
  message.data = goto_enabled_;
  pub_goto_enabled_->publish(message);
}

void ControlNode::disable_goto_and_hold(const char *reason) {
  const bool was_enabled = goto_enabled_;
  const bool was_active = pursuit_active_;
  goto_enabled_ = false;
  pursuit_active_ = false;
  pursuit_feedforward_ = {};

  if (state_ == FlightState::HOVER && local_pos_valid_) {
    target_ = current_;
  }
  publish_goto_enabled();

  if (was_enabled || was_active) {
    RCLCPP_INFO(this->get_logger(), "GOTO disabled (%s)%s",
                reason ? reason : "",
                state_ == FlightState::HOVER && local_pos_valid_
                    ? "; current pose fixed as hold target"
                    : "");
  }
}

// ============================================================================
// subscription callbacks
// ============================================================================

void ControlNode::on_local_position(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
  std::lock_guard<std::mutex> lk(mtx_);
  current_.x = m->x;
  current_.y = m->y;
  current_.z = m->z;
  current_.yaw = m->heading;
  current_vz_ = m->vz;
  local_pos_valid_ = m->xy_valid && m->z_valid && std::isfinite(current_.x) &&
                     std::isfinite(current_.y) && std::isfinite(current_.z) &&
                     std::isfinite(current_.yaw);

  if (goto_enabled_ && !local_pos_valid_) {
    disable_goto_and_hold("local position became invalid");
  }

  // While not actively controlling, keep target glued to current pose so that
  // whenever Offboard becomes active the drone will hold, not jump.
  if (state_ == FlightState::DISARMED || state_ == FlightState::LANDED) {
    target_ = current_;
  }
}

void ControlNode::on_vehicle_status(
    const px4_msgs::msg::VehicleStatus::SharedPtr m) {
  std::lock_guard<std::mutex> lk(mtx_);

  last_status_ = *m;
  status_received_ = true;

  // Landing completion must be determined by the real PX4 arming state.
  // This applies to both:
  //   1. PX4 NAV_LAND
  //   2. Self-managed Offboard descent
  //
  // It must be checked before the Offboard-loss handling because PX4 may
  // leave Offboard at the same time that it disarms.
  if (state_ == FlightState::LANDING &&
      m->arming_state == ARMING_STATE_STANDBY) {
    // PX4 has really disarmed. It is now safe to stop the descent target
    // and attach the target to the current position.
    target_ = current_;
    disable_goto_and_hold("PX4 disarmed after landing");
    had_offboard_ = false;

    set_state(FlightState::LANDED, "PX4 confirmed disarmed after landing");

    RCLCPP_INFO(this->get_logger(),
                "landing completed: PX4 arming_state=STANDBY");

    return;
  }

  const bool offb_now = (m->nav_state == NAV_STATE_OFFBOARD);

  if (offb_now) {
    had_offboard_ = true;

  } else if (had_offboard_ && state_ != FlightState::DISARMED &&
             state_ != FlightState::LANDED) {
    // RC or failsafe pulled us out of Offboard.
    RCLCPP_WARN(this->get_logger(),
                "left OFFBOARD (nav_state=%u), yielding control", m->nav_state);

    target_ = current_;
    disable_goto_and_hold("PX4 left Offboard");
    had_offboard_ = false;

    set_state(FlightState::DISARMED, "offboard lost");
  }
}

// ============================================================================
// command handling
// ============================================================================

void ControlNode::on_command(const msg::ControlCommand::SharedPtr m) {
  std::lock_guard<std::mutex> lk(mtx_);
  switch (m->cmd) {
  case msg::ControlCommand::CMD_OFFBOARD:
    handle_offboard();
    break;
  case msg::ControlCommand::CMD_ARM:
    handle_arm();
    break;
  case msg::ControlCommand::CMD_DISARM:
    handle_disarm();
    break;
  case msg::ControlCommand::CMD_LAND:
    handle_land();
    break;
  case msg::ControlCommand::CMD_MOVE:
    handle_move(m->dx, m->dy, m->dz);
    break;
  case msg::ControlCommand::CMD_YAW:
    handle_yaw(m->dx);
    break;
  case msg::ControlCommand::CMD_GOTO_ENABLE:
    handle_goto_enable();
    break;
  case msg::ControlCommand::CMD_GOTO_DISABLE:
    handle_goto_disable();
    break;
  default:
    break;
  }
}

void ControlNode::on_goto(const msg::GotoTarget::SharedPtr m) {
  std::lock_guard<std::mutex> lk(mtx_);

  if (!m->active) {
    disable_goto_and_hold("GotoTarget(active=false)");
    return;
  }

  if (!goto_enabled_) {
    return;
  }

  if (state_ != FlightState::HOVER || !local_pos_valid_) {
    return;
  }
  const bool target_valid =
      std::isfinite(m->x) && std::isfinite(m->y) && std::isfinite(m->z) &&
      std::isfinite(m->vx) && std::isfinite(m->vy) && std::isfinite(m->vz) &&
      std::isfinite(m->ax) && std::isfinite(m->ay) && std::isfinite(m->az) &&
      std::isfinite(m->yaw) && std::isfinite(m->yawspeed);
  if (!target_valid) {
    RCLCPP_ERROR(
        this->get_logger(),
        "pursuit reference rejected: required fields contain NaN or Inf");
    disable_goto_and_hold("invalid GotoTarget");
    return;
  }
  const bool first_pursuit_reference = !pursuit_active_;
  target_.x = m->x;
  target_.y = m->y;
  target_.z = m->z;
  target_.yaw = m->yaw;
  pursuit_feedforward_.vx = m->vx;
  pursuit_feedforward_.vy = m->vy;
  pursuit_feedforward_.vz = m->vz;
  pursuit_feedforward_.ax = m->ax;
  pursuit_feedforward_.ay = m->ay;
  pursuit_feedforward_.az = m->az;
  pursuit_feedforward_.yawspeed = m->yawspeed;
  pursuit_active_ = true;
  last_goto_stamp_ = this->get_clock()->now();
  if (first_pursuit_reference) {
    RCLCPP_INFO(this->get_logger(),
                "pursuit accepted: target=(%.2f, %.2f, %.2f) "
                "vel=(%.2f, %.2f, %.2f) yaw=%.2f",
                target_.x, target_.y, target_.z, pursuit_feedforward_.vx,
                pursuit_feedforward_.vy, pursuit_feedforward_.vz, target_.yaw);
  }
}

void ControlNode::handle_offboard() {
  if (heartbeat_count_ < static_cast<uint64_t>(heartbeat_before_offboard_)) {
    RCLCPP_WARN(this->get_logger(),
                "not enough heartbeats (%lu/%d), refusing to switch Offboard",
                static_cast<unsigned long>(heartbeat_count_),
                heartbeat_before_offboard_);
    return;
  }
  publish_vehicle_command(VC_DO_SET_MODE, SET_MODE_PARAM1_CUSTOM,
                          SET_MODE_PARAM2_OFFBOARD);
  RCLCPP_INFO(this->get_logger(), "sent DO_SET_MODE(Offboard)");
}

void ControlNode::handle_arm() {
  if (state_ != FlightState::DISARMED && state_ != FlightState::LANDED) {
    RCLCPP_WARN(this->get_logger(),
                "ARM refused: state=%s (must be DISARMED/LANDED)",
                state_name(state_));
    return;
  }
  if (!is_on_ground()) {
    RCLCPP_WARN(this->get_logger(), "ARM refused: not on ground");
    return;
  }
  if (!is_offboard()) {
    RCLCPP_WARN(this->get_logger(),
                "ARM refused: not in Offboard yet (press 0 first)");
    return;
  }
  if (!local_pos_valid_) {
    RCLCPP_WARN(this->get_logger(), "ARM refused: local position not valid");
    return;
  }

  home_ = current_;
  target_.x = home_.x;
  target_.y = home_.y;
  target_.z = -static_cast<float>(takeoff_height_);
  target_.yaw = home_.yaw;

  publish_vehicle_command(VC_COMPONENT_ARM_DISARM, ARM_PARAM1_ARM);
  set_state(FlightState::ARMING, "arm command sent");
  RCLCPP_INFO(this->get_logger(),
              "ARM sent, target=(%.2f, %.2f, %.2f) yaw=%.2f", target_.x,
              target_.y, target_.z, target_.yaw);
}

void ControlNode::handle_disarm() {
  if (!is_on_ground()) {
    disable_goto_and_hold("manual disarm requested while airborne");
    RCLCPP_ERROR(this->get_logger(),
                 "DISARM REFUSED: vehicle appears airborne (takeoff_time=%lu "
                 "z=%.2f vz=%.2f)",
                 static_cast<unsigned long>(last_status_.takeoff_time),
                 current_.z, current_vz_);
    return;
  }
  disable_goto_and_hold("manual disarm requested");
  publish_vehicle_command(VC_COMPONENT_ARM_DISARM, ARM_PARAM1_DISARM);
  // set_state(FlightState::DISARMED, "disarm command sent");
  RCLCPP_INFO(this->get_logger(), "DISARM sent");
}

void ControlNode::handle_land() {
  disable_goto_and_hold("landing requested");
  if (state_ == FlightState::DISARMED || state_ == FlightState::LANDED ||
      state_ == FlightState::LANDING) {
    RCLCPP_WARN(this->get_logger(), "LAND ignored: state=%s",
                state_name(state_));
    return;
  }

  if (land_control_) {
    publish_vehicle_command(VC_NAV_LAND);
    set_state(FlightState::LANDING, "NAV_LAND sent");
    RCLCPP_INFO(this->get_logger(),
                "LAND: NAV_LAND sent, PX4 handles descent+disarm");
  } else {
    // Self-managed descent: freeze XY at the current SLAM/EKF position, and let
    // on_timer() slowly pull target_.z downward. Never touches velocity
    // setpoints, so the PX4 position controller is fully in charge — same
    // control loop that held station in Position mode.
    target_.x = current_.x;
    target_.y = current_.y;
    target_.yaw = current_.yaw;
    // Start target_.z from wherever we are right now, not the previous target,
    // so we don't get a sudden drop if target_ had drifted above current.
    target_.z = current_.z;
    last_land_tick_ = this->get_clock()->now();
    set_state(FlightState::LANDING, "self-managed descent begin");
    RCLCPP_INFO(this->get_logger(),
                "LAND: self-descent, XY hold=(%.2f, %.2f) start_z=%.2f",
                target_.x, target_.y, target_.z);
  }
}

void ControlNode::handle_move(float dx, float dy, float dz) {
  if (state_ != FlightState::HOVER) {
    RCLCPP_INFO(this->get_logger(), "move ignored: state=%s (must be HOVER)",
                state_name(state_));
    return;
  }
  if (goto_enabled_) {
    RCLCPP_INFO(this->get_logger(), "move ignored: GOTO is enabled");
    return;
  }
  target_.x += dx;
  target_.y += dy;
  target_.z += dz;
  RCLCPP_INFO(this->get_logger(), "target -> (%.2f, %.2f, %.2f)", target_.x,
              target_.y, target_.z);
}

void ControlNode::handle_yaw(float dyaw) {
  if (state_ != FlightState::HOVER) {
    RCLCPP_INFO(this->get_logger(), "yaw ignored: state=%s (must be HOVER)",
                state_name(state_));
    return;
  }
  if (goto_enabled_) {
    RCLCPP_INFO(this->get_logger(), "yaw ignored: GOTO is enabled");
    return;
  }
  target_.yaw += dyaw;
  while (target_.yaw > static_cast<float>(M_PI)) {
    target_.yaw -= 2.0f * static_cast<float>(M_PI);
  }
  while (target_.yaw < -static_cast<float>(M_PI)) {
    target_.yaw += 2.0f * static_cast<float>(M_PI);
  }
  RCLCPP_INFO(this->get_logger(), "target yaw -> %.3f rad", target_.yaw);
}

void ControlNode::handle_goto_enable() {
  if (state_ != FlightState::HOVER || !local_pos_valid_) {
    RCLCPP_WARN(this->get_logger(),
                "GOTO enable refused: state=%s local_valid=%s",
                state_name(state_), local_pos_valid_ ? "true" : "false");
    return;
  }
  if (goto_enabled_) {
    return;
  }
  goto_enabled_ = true;
  pursuit_active_ = false;
  pursuit_feedforward_ = {};
  publish_goto_enabled();
  RCLCPP_INFO(this->get_logger(),
              "GOTO execution enabled; holding until first valid target");
}

void ControlNode::handle_goto_disable() {
  disable_goto_and_hold("keyboard '5'");
}

// ============================================================================
// px4 publishers
// ============================================================================

void ControlNode::publish_offboard_control_mode() {
  px4_msgs::msg::OffboardControlMode m{};
  m.position = true;
  m.velocity = false;
  m.acceleration = false;
  m.attitude = false;
  m.body_rate = false;
  m.timestamp = now_us();
  pub_offboard_mode_->publish(m);
}

void ControlNode::publish_trajectory_setpoint() {
  // PX4 v1.14+ expects unused setpoint components filled with NaN. A zero in
  // velocity/acceleration/jerk is treated as a feed-forward, which fights the
  // position controller and causes overshoot / bounce on takeoff.
  const float NaN = std::numeric_limits<float>::quiet_NaN();

  px4_msgs::msg::TrajectorySetpoint m{};
  m.position = {target_.x, target_.y, target_.z};

  if (state_ == FlightState::LANDING && !land_control_) {
    const double height_above_ground = -static_cast<double>(current_.z);

    const float descent_vz = static_cast<float>(
        height_above_ground > land_slow_alt_ ? land_fast_speed_
                                             : land_slow_speed_);

    // NED：正 Z 表示向下。
    m.velocity = {NaN, NaN, descent_vz};
  } else if (pursuit_active_ && state_ == FlightState::HOVER) {
    // Position remains the active outer loop. Velocity and acceleration are
    // the kinematically consistent trajectory feed-forwards generated by
    // the active trajectory source; they are not feedback measurements.
    m.velocity = {pursuit_feedforward_.vx, pursuit_feedforward_.vy,
                  pursuit_feedforward_.vz};
  } else {
    m.velocity = {NaN, NaN, NaN};
  }

  if (pursuit_active_ && state_ == FlightState::HOVER) {
    m.acceleration = {pursuit_feedforward_.ax, pursuit_feedforward_.ay,
                      pursuit_feedforward_.az};
  } else {
    m.acceleration = {NaN, NaN, NaN};
  }
  m.jerk = {NaN, NaN, NaN};
  m.yaw = target_.yaw;
  m.yawspeed = pursuit_active_ && state_ == FlightState::HOVER
                   ? pursuit_feedforward_.yawspeed
                   : NaN;
  m.timestamp = now_us();
  pub_trajectory_->publish(m);
}

void ControlNode::publish_vehicle_command(uint16_t command, float p1, float p2,
                                          float p3, float p4, float p5,
                                          float p6, float p7) {
  px4_msgs::msg::VehicleCommand m{};
  m.command = command;
  m.param1 = p1;
  m.param2 = p2;
  m.param3 = p3;
  m.param4 = p4;
  m.param5 = p5;
  m.param6 = p6;
  m.param7 = p7;
  m.target_system = 1;
  m.target_component = 1;
  m.source_system = 1;
  m.source_component = 1;
  m.from_external = true;
  m.timestamp = now_us();
  pub_command_->publish(m);
}

// ============================================================================
// timer: heartbeat + state progression
// ============================================================================

void ControlNode::on_timer() {
  std::lock_guard<std::mutex> lk(mtx_);

  // Drop stale feed-forward before publishing this tick. This is the fallback
  // for a crashed/disconnected external reference source; normal stops arrive
  // immediately as GotoTarget(active=false).
  if (pursuit_active_) {
    const double age = (this->get_clock()->now() - last_goto_stamp_).seconds();
    if (age > pursuit_goto_timeout_) {
      disable_goto_and_hold("Goto timeout");
      RCLCPP_WARN(this->get_logger(),
                  "pursuit timed out after %.2fs; holding current pose", age);
    }
  }

  // Always publish heartbeat + current target — PX4 requires >2Hz.
  publish_offboard_control_mode();
  publish_trajectory_setpoint();
  if (heartbeat_count_ < std::numeric_limits<uint64_t>::max()) {
    ++heartbeat_count_;
  }

  // State progression driven by observed telemetry
  switch (state_) {
  case FlightState::ARMING:
    if (is_armed()) {
      set_state(FlightState::TAKEOFF, "armed detected");
    }
    break;

  case FlightState::TAKEOFF:
    if (local_pos_valid_ &&
        std::fabs(current_.z - target_.z) < static_cast<float>(arrive_tol_)) {
      set_state(FlightState::HOVER, "reached takeoff altitude");
    }
    break;

  case FlightState::LANDING:
    if (land_control_) {
      // Primary exit: vehicle_status callback flips to LANDED on auto-disarm.
      // Backup: if we somehow miss the disarm event but see grounded+disarmed.
      if (last_status_.arming_state == ARMING_STATE_STANDBY && is_on_ground()) {
        set_state(FlightState::LANDED, "detected grounded+disarmed");
      }
    } else {
      // Self-managed: slowly pull target_.z downward at fast/slow rate.
      // The PX4 position controller decides the actual descent velocity.
      const rclcpp::Time now = this->get_clock()->now();
      double dt = (now - last_land_tick_).seconds();
      last_land_tick_ = now;
      if (dt <= 0.0 || dt > 0.5) {
        dt = 1.0 / std::max(1, loop_rate_hz_);
      }

      // Current height above ground (NED z is negative up).
      const double agl = -static_cast<double>(current_.z);
      const double v =
          (agl > land_slow_alt_) ? land_fast_speed_ : land_slow_speed_;

      // Pull target down, but clamp a bit below ground so PX4 keeps pushing
      // thrust down (helps land_detector eventually agree, and stops us from
      // driving target infinitely negative if disarm is delayed).
      const float target_z_floor = static_cast<float>(land_touch_z_ + 0.30);
      if (target_.z < target_z_floor) {
        target_.z += static_cast<float>(v * dt); // NED: descending = z+
        if (target_.z > target_z_floor) {
          target_.z = target_z_floor;
        }
      }

      // PX4 land detector and auto-disarm own touchdown completion. This node
      // keeps publishing the descent target until VehicleStatus reports
      // arming_state=STANDBY.
    }
    break;

  default:
    break;
  }
}

} // namespace px4_control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<px4_control::ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
