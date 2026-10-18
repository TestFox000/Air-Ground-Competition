#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "px4_path/path_follow_trajectory_node.hpp"
#include "px4_path/path_follow_waypoint_node.hpp"

namespace {

std::string read_follow_mode() {
  auto selector = std::make_shared<rclcpp::Node>("path_follow_node");
  return selector->declare_parameter<std::string>("follow_mode", "trajectory");
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  const std::string follow_mode = read_follow_mode();
  std::shared_ptr<rclcpp::Node> follower;

  if (follow_mode == "waypoint") {
    follower = std::make_shared<px4_path::PathFollowWaypointNode>();
  } else if (follow_mode == "trajectory") {
    follower = std::make_shared<px4_path::PathFollowTrajectoryNode>();
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("path_follow"),
                 "invalid follow_mode '%s'; expected waypoint or trajectory",
                 follow_mode.c_str());
    rclcpp::shutdown();
    return 2;
  }

  RCLCPP_INFO(follower->get_logger(), "selected follow_mode=%s",
              follow_mode.c_str());
  rclcpp::spin(follower);
  follower.reset();
  rclcpp::shutdown();
  return 0;
}
