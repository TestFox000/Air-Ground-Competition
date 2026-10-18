#ifndef PX4_CONTROL__PX4_TELEOP_NODE_HPP_
#define PX4_CONTROL__PX4_TELEOP_NODE_HPP_

#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <termios.h>

#include "px4_control/msg/control_command.hpp"

namespace px4_control {

class TeleopNode : public rclcpp::Node {
public:
  TeleopNode();
  ~TeleopNode() override;

  TeleopNode(const TeleopNode &) = delete;
  TeleopNode &operator=(const TeleopNode &) = delete;

private:
  void keyboard_loop();
  void publish_move(float dx, float dy, float dz);
  void publish_yaw(float dyaw);
  void publish_cmd(uint8_t cmd);

  void enable_raw_mode();
  void restore_terminal();

  rclcpp::Publisher<msg::ControlCommand>::SharedPtr pub_;

  std::thread kb_thread_;
  std::atomic<bool> running_{true};

  bool termios_saved_{false};
  termios orig_termios_{};

  double step_xy_{0.5};
  double step_z_{0.1};
  double yaw_step_rad_{0.15};
};

} // namespace px4_control

#endif // PX4_CONTROL__PX4_TELEOP_NODE_HPP_
