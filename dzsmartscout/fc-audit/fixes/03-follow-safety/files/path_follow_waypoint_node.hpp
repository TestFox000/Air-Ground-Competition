#ifndef PX4_PATH__PATH_FOLLOW_WAYPOINT_NODE_HPP_
#define PX4_PATH__PATH_FOLLOW_WAYPOINT_NODE_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "px4_control/msg/goto_target.hpp"
#include "px4_path/path_loader.hpp"

namespace px4_path {

enum class WaypointFollowState : uint8_t {
  INIT = 0, // waiting for local_position first sample
  LOADED,   // path file parsed OK, idle until start
  PURSUING, // sending goto = points_[cursor_], advancing cursor
  PAUSED,   // goto disabled; cursor remains frozen
  HOLD,     // reached terminal point, keep sending it
};

class PathFollowWaypointNode : public rclcpp::Node {
public:
  PathFollowWaypointNode();
  ~PathFollowWaypointNode() override = default;

  PathFollowWaypointNode(const PathFollowWaypointNode &) = delete;
  PathFollowWaypointNode &operator=(const PathFollowWaypointNode &) = delete;

private:
  // ---- callbacks ----
  void
  on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void on_goto_enabled(const std_msgs::msg::Bool::SharedPtr msg);
  void on_timer();

  // ---- helpers ----
  bool prepare_path_directory(); // 创建目录并解析统一的目标文件路径
  bool reload_path();            // (re)parse file, populate points_
  void activate_if_ready();
  void publish_goto(const Waypoint &wp);
  void publish_stop();
  Waypoint clamp_step(const Waypoint &from_pos, const Waypoint &target) const;
  static double distance(const Waypoint &a, const Waypoint &b);
  const char *state_name(WaypointFollowState s) const;
  void set_state(WaypointFollowState s, const char *reason);

  // ---- params ----
  std::string path_dir_;
  std::string path_file_{"recorded_path.txt"};
  double pursuit_rate_{10.0};
  double switch_tol_{0.30};
  double terminal_tol_{0.20};
  double max_step_dist_{1.5};
  double max_neighbor_dist_m_{5.0};
  // 补丁 03：要求一次显式的 OFF->ON 使能跳变才允许开始跟随。
  bool require_fresh_enable_{true};

  // ---- ros ----
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      sub_local_pos_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_goto_enabled_;
  rclcpp::Publisher<px4_control::msg::GotoTarget>::SharedPtr pub_goto_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- state ----
  WaypointFollowState state_{WaypointFollowState::INIT};
  WaypointFollowState paused_from_{WaypointFollowState::PURSUING};
  std::vector<Waypoint> points_; // NED, from path_loader
  std::size_t cursor_{0};

  bool local_pos_valid_{false};
  bool goto_enabled_{false};
  // 补丁 03：/px4_control/goto_enabled 是锁存的，首帧只是"基线"，不是"命令"。
  bool goto_permission_observed_{false};
  bool enable_transition_seen_{false};
  Waypoint current_pos_{}; // yaw field unused here

  std::string resolved_path_file_; // final absolute path used to load
};

} // namespace px4_path

#endif // PX4_PATH__PATH_FOLLOW_WAYPOINT_NODE_HPP_
