#ifndef PX4_CONTROL__PX4_CONTROL_NODE_HPP_
#define PX4_CONTROL__PX4_CONTROL_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "px4_control/msg/control_command.hpp"
#include "px4_control/msg/goto_target.hpp"

namespace px4_control {

enum class FlightState : uint8_t {
  DISARMED = 0,
  ARMING,
  TAKEOFF,
  HOVER,
  LANDING,
  LANDED,
};

class ControlNode : public rclcpp::Node {
public:
  ControlNode();
  ~ControlNode() override = default;

  ControlNode(const ControlNode &) = delete;
  ControlNode &operator=(const ControlNode &) = delete;

private:
  // ---- callbacks ----
  void on_command(const msg::ControlCommand::SharedPtr msg);
  void on_goto(const msg::GotoTarget::SharedPtr msg);
  void
  on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void on_timer();

  // ---- command handlers ----
  void handle_offboard();
  void handle_arm();
  void handle_disarm();
  void handle_land();
  void handle_move(float dx, float dy, float dz);
  void handle_yaw(float dyaw);
  void handle_goto_enable();
  void handle_goto_disable();

  // ---- px4 publishers ----
  void publish_offboard_control_mode();
  void publish_goto_enabled();
  void publish_trajectory_setpoint();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0f,
                               float param2 = 0.0f, float param3 = 0.0f,
                               float param4 = 0.0f, float param5 = 0.0f,
                               float param6 = 0.0f, float param7 = 0.0f);

  // ---- helpers ----
  bool is_offboard() const;
  bool is_armed() const;
  bool is_on_ground() const; // uses takeoff_time + local position
  void disable_goto_and_hold(const char *reason);
  void set_state(FlightState s, const char *reason);
  const char *state_name(FlightState s) const;
  uint64_t now_us();

  // ---- params ----
  double takeoff_height_{
      1.2}; // meters, positive; internally z = -takeoff_height_
  double step_xy_{0.5};
  double step_z_{0.1};
  int loop_rate_hz_{50};
  double arrive_tol_{0.15};
  int heartbeat_before_offboard_{20};

  // Landing configuration
  bool land_control_{
      false}; // true = send NAV_LAND, false = self-managed descent
  double land_fast_speed_{0.4}; // m/s, descent rate above land_slow_alt_
  double land_slow_speed_{0.2}; // m/s, descent rate below land_slow_alt_
  double land_slow_alt_{0.5}; // meters AGL, threshold to switch to slow descent
  double land_touch_z_{
      0.05}; // meters, |current.z| below this + low vz = touched down

  // Path-follow / pursuit
  double yaw_step_rad_{0.15}; // rad per L/R keypress (teleop-side default too)
  double pursuit_goto_timeout_{
      1.0}; // seconds without goto -> clear pursuit_active_

  // ---- ros ----
  rclcpp::Subscription<msg::ControlCommand>::SharedPtr sub_cmd_;
  rclcpp::Subscription<msg::GotoTarget>::SharedPtr sub_goto_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      sub_local_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      pub_offboard_mode_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      pub_trajectory_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_command_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goto_enabled_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- state ----
  mutable std::mutex mtx_;

  struct Vec4 {
    float x{0.f}, y{0.f}, z{0.f}, yaw{0.f};
  };
  Vec4 target_;
  Vec4 current_;
  Vec4 home_;
  struct Feedforward {
    float vx{0.f};
    float vy{0.f};
    float vz{0.f};
    float ax{0.f};
    float ay{0.f};
    float az{0.f};
    float yawspeed{0.f};
  };
  Feedforward pursuit_feedforward_;
  float current_vz_{0.f};
  bool local_pos_valid_{false};

  px4_msgs::msg::VehicleStatus last_status_{};
  bool status_received_{false};

  FlightState state_{FlightState::DISARMED};
  uint64_t heartbeat_count_{0};

  rclcpp::Time
      last_land_tick_{}; // last on_timer call while LANDING (self-managed)

  // remember whether we ever saw OFFBOARD active,
  // so we only trigger "RC took over" once we actually had it
  bool had_offboard_{false};

  // Operator-owned admission gate. While true, active Goto messages may update
  // the target and manual WASDQE/L/R commands remain locked out.
  bool goto_enabled_{false};
  bool pursuit_active_{false};
  rclcpp::Time last_goto_stamp_{};
};

} // namespace px4_control

#endif // PX4_CONTROL__PX4_CONTROL_NODE_HPP_
