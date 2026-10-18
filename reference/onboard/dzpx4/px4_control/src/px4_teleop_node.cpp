#include "px4_control/px4_teleop_node.hpp"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>

namespace px4_control {

TeleopNode::TeleopNode() : rclcpp::Node("px4_teleop_node") {
  step_xy_ = this->declare_parameter<double>("step_xy", 0.5);
  step_z_ = this->declare_parameter<double>("step_z", 0.1);
  yaw_step_rad_ = this->declare_parameter<double>("yaw_step_rad", 0.15);

  pub_ =
      this->create_publisher<msg::ControlCommand>("/px4_control/command", 10);

  enable_raw_mode();

  kb_thread_ = std::thread(&TeleopNode::keyboard_loop, this);

  RCLCPP_INFO(
      this->get_logger(),
      "Teleop ready.\n"
      "  0 = switch to Offboard\n"
      "  1 = ARM (auto takeoff)\n"
      "  2 = DISARM (only when landed)\n"
      "  3 = LAND (self-descent or NAV_LAND)\n"
      "  W/S = +/- X %.2fm    A/D = +/- Y %.2fm    Q/E = up/down %.2fm\n"
      "  L/R = yaw +/- %.3f rad (HOVER, ignored while GOTO is enabled)\n"
      "  4 = enable external GOTO    5 = disable GOTO and hold current pose\n"
      "  Ctrl+C to quit",
      step_xy_, step_xy_, step_z_, yaw_step_rad_);
}

TeleopNode::~TeleopNode() {
  running_ = false;
  if (kb_thread_.joinable()) {
    kb_thread_.join();
  }
  restore_terminal();
}

void TeleopNode::enable_raw_mode() {
  if (tcgetattr(STDIN_FILENO, &orig_termios_) != 0) {
    RCLCPP_WARN(this->get_logger(), "tcgetattr failed, keyboard may not work");
    return;
  }
  termios_saved_ = true;

  termios raw = orig_termios_;
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 1; // 100ms read timeout
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

void TeleopNode::restore_terminal() {
  if (termios_saved_) {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    termios_saved_ = false;
  }
}

void TeleopNode::publish_cmd(uint8_t cmd) {
  msg::ControlCommand m;
  m.cmd = cmd;
  m.dx = 0.f;
  m.dy = 0.f;
  m.dz = 0.f;
  pub_->publish(m);
}

void TeleopNode::publish_move(float dx, float dy, float dz) {
  msg::ControlCommand m;
  m.cmd = msg::ControlCommand::CMD_MOVE;
  m.dx = dx;
  m.dy = dy;
  m.dz = dz;
  pub_->publish(m);
}

void TeleopNode::publish_yaw(float dyaw) {
  msg::ControlCommand m;
  m.cmd = msg::ControlCommand::CMD_YAW;
  m.dx = dyaw; // dx carries dyaw in radians (see ControlCommand.msg)
  m.dy = 0.f;
  m.dz = 0.f;
  pub_->publish(m);
}

void TeleopNode::keyboard_loop() {
  while (rclcpp::ok() && running_) {
    char c = 0;
    ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n <= 0) {
      continue;
    }

    const char k =
        static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    switch (k) {
    case '0':
      publish_cmd(msg::ControlCommand::CMD_OFFBOARD);
      RCLCPP_INFO(this->get_logger(), "[key 0] request OFFBOARD");
      break;
    case '1':
      publish_cmd(msg::ControlCommand::CMD_ARM);
      RCLCPP_INFO(this->get_logger(), "[key 1] request ARM + takeoff");
      break;
    case '2':
      publish_cmd(msg::ControlCommand::CMD_DISARM);
      RCLCPP_INFO(this->get_logger(), "[key 2] request DISARM");
      break;
    case '3':
      publish_cmd(msg::ControlCommand::CMD_LAND);
      RCLCPP_INFO(this->get_logger(), "[key 3] request LAND");
      break;

    // NED frame: X=north, Y=east, Z=down
    case 'w':
      publish_move(+step_xy_, 0.f, 0.f);
      break;
    case 's':
      publish_move(-step_xy_, 0.f, 0.f);
      break;
    case 'a':
      publish_move(0.f, -step_xy_, 0.f); // A -> Y-
      break;
    case 'd':
      publish_move(0.f, +step_xy_, 0.f); // D -> Y+
      break;
    case 'q':
      publish_move(0.f, 0.f, -step_z_); // Q -> up  (NED z-)
      break;
    case 'e':
      publish_move(0.f, 0.f, +step_z_); // E -> down (NED z+)
      break;

    // Yaw ±. NED heading is 0=north, positive = clockwise from above.
    // If real flight sees the drone yaw the "wrong way" for L/R, just flip
    // the sign of dyaw here — control_node yaw logic is symmetric.
    case 'l':
      publish_yaw(+static_cast<float>(yaw_step_rad_));
      break;
    case 'r':
      publish_yaw(-static_cast<float>(yaw_step_rad_));
      break;

    case '4':
      publish_cmd(msg::ControlCommand::CMD_GOTO_ENABLE);
      RCLCPP_INFO(this->get_logger(), "[key 4] enable external GOTO");
      break;
    case '5':
      publish_cmd(msg::ControlCommand::CMD_GOTO_DISABLE);
      RCLCPP_INFO(this->get_logger(),
                  "[key 5] disable GOTO and hold current pose");
      break;

    default:
      break;
    }
  }
}

} // namespace px4_control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<px4_control::TeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
