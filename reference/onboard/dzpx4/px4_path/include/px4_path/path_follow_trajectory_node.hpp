#ifndef PX4_PATH__PATH_FOLLOW_TRAJECTORY_NODE_HPP_
#define PX4_PATH__PATH_FOLLOW_TRAJECTORY_NODE_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "px4_control/msg/goto_target.hpp"
#include "px4_path/motion_trajectory.hpp"
#include "px4_path/path_loader.hpp"

namespace px4_path {

enum class TrajectoryFollowState : uint8_t {
  INIT = 0, // waiting for local_position first sample
  LOADED,   // path file parsed OK, idle until start
  PURSUING, // sampling and sending a continuous time-parameterized reference
  PAUSED,   // goto disabled; trajectory phase remains frozen
  HOLD,     // reached terminal point, keep sending it
};

class PathFollowTrajectoryNode : public rclcpp::Node {
public:
  PathFollowTrajectoryNode();
  ~PathFollowTrajectoryNode() override = default;

  PathFollowTrajectoryNode(const PathFollowTrajectoryNode &) = delete;
  PathFollowTrajectoryNode &
  operator=(const PathFollowTrajectoryNode &) = delete;

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
  bool start_trajectory();
  void publish_reference(const TrajectorySample &sample, double timer_dt_s,
                         double time_scale = 1.0, double time_scale_rate = 0.0);
  void publish_stop();
  std::vector<std::vector<Waypoint>> make_execution_segments() const;
  static double distance(const Waypoint &a, const Waypoint &b);
  static double wrap_pi(double angle);
  const char *state_name(TrajectoryFollowState s) const;
  void set_state(TrajectoryFollowState s, const char *reason);

  // ---- params ----
  std::string path_dir_;
  std::string path_file_{"recorded_path.txt"};
  double pursuit_rate_{50.0};
  double terminal_tol_{0.20};
  double max_step_dist_{1.5};
  double max_neighbor_dist_m_{5.0};
  double max_speed_mps_{0.5};
  double max_accel_mps2_{0.5};
  double max_jerk_mps3_{1.0};
  double max_centripetal_accel_mps2_{0.35};
  double profile_resolution_m_{0.10};
  double segment_split_turn_deg_{150.0};
  double segment_switch_speed_mps_{0.15};
  double max_tracking_error_m_{1.0};
  double tracking_slowdown_error_m_{0.30};
  double tracking_pause_error_m_{0.70};
  double max_time_scale_rate_per_s_{1.0};
  std::string yaw_mode_{"fixed"};
  double yaw_lookahead_s_{0.5};
  double max_yaw_rate_rps_{0.5};

  // ---- ros ----
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      sub_local_pos_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_goto_enabled_;
  rclcpp::Publisher<px4_control::msg::GotoTarget>::SharedPtr pub_goto_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- state ----
  TrajectoryFollowState state_{TrajectoryFollowState::INIT};
  TrajectoryFollowState paused_from_{TrajectoryFollowState::PURSUING};
  std::vector<Waypoint> points_; // NED, from path_loader
  std::vector<MotionTrajectory> trajectories_;
  std::size_t active_segment_index_{0};
  rclcpp::Time trajectory_start_time_{};
  rclcpp::Time last_timer_time_{};
  double trajectory_elapsed_s_{0.0};
  double reference_time_scale_{1.0};
  double command_yaw_{0.0};

  bool local_pos_valid_{false};
  bool goto_enabled_{false};
  double current_speed_mps_{0.0};
  Waypoint current_pos_{};

  std::string resolved_path_file_; // final absolute path used to load
};

} // namespace px4_path

#endif // PX4_PATH__PATH_FOLLOW_TRAJECTORY_NODE_HPP_
