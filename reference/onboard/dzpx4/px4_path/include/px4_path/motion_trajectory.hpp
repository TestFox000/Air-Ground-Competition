#ifndef PX4_PATH__MOTION_TRAJECTORY_HPP_
#define PX4_PATH__MOTION_TRAJECTORY_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "px4_path/path_loader.hpp"

namespace px4_path {

struct TrajectoryLimits {
  double max_speed_mps{0.5};
  double max_accel_mps2{0.5};
  double max_jerk_mps3{1.0};
  double max_centripetal_accel_mps2{0.35};
  double profile_resolution_m{0.10};
};

struct TrajectorySample {
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
  std::array<double, 3> acceleration{};
  std::array<double, 3> jerk{};
  double yaw{0.0};
  double yaw_speed{0.0};
};

struct PathTurnSplit {
  std::size_t waypoint_index{0};
  double turn_angle_deg{0.0};
};

// Find interior waypoints where the outgoing direction reverses sharply from
// the incoming direction. A threshold <= 0 disables automatic splitting.
std::vector<PathTurnSplit>
find_path_turn_splits(const std::vector<Waypoint> &points,
                      double threshold_deg);

// A time-parameterized, interpolating cubic spline through NED waypoints.
// Spatial position is C2-continuous. Curvature sets a local speed ceiling, a
// forward/backward pass enforces acceleration-aware braking, and a clamped
// cubic time spline connects the local profile with zero endpoint speed. Full
// geometric velocity, acceleration and jerk are sampled and the time allocation
// is enlarged until all configured limits hold.
class MotionTrajectory {
public:
  bool build(const std::vector<Waypoint> &points,
             const TrajectoryLimits &limits, std::string &error);

  TrajectorySample sample(double time_s) const;

  bool valid() const { return valid_; }
  double duration() const { return duration_s_; }
  double length() const { return geometric_length_m_; }
  double max_sampled_speed() const { return max_sampled_speed_mps_; }
  double max_sampled_accel() const { return max_sampled_accel_mps2_; }
  double max_sampled_jerk() const { return max_sampled_jerk_mps3_; }
  double max_sampled_centripetal_accel() const {
    return max_sampled_centripetal_accel_mps2_;
  }
  double time_scale() const { return applied_time_scale_; }

private:
  class NaturalCubicSpline {
  public:
    bool build(const std::vector<double> &knots,
               const std::vector<double> &values);
    bool build_clamped(const std::vector<double> &knots,
                       const std::vector<double> &values, double first_slope,
                       double last_slope);
    std::array<double, 4> evaluate(double knot) const;

  private:
    std::vector<double> knots_;
    std::vector<double> values_;
    std::vector<double> second_derivatives_;
  };

  struct SpatialSample {
    std::array<double, 3> position{};
    std::array<double, 3> first{};
    std::array<double, 3> second{};
    std::array<double, 3> third{};
    double yaw{0.0};
    double yaw_first{0.0};
  };

  struct ProgressSample {
    double path_s{0.0};
    double speed{0.0};
    double accel{0.0};
    double jerk{0.0};
  };

  SpatialSample sample_spatial(double path_s) const;
  ProgressSample sample_progress(double time_s) const;
  TrajectorySample sample_kinematics(const ProgressSample &progress) const;
  bool rebuild_progress_spline();
  void measure_limits();
  static double wrap_pi(double angle);

  std::array<NaturalCubicSpline, 3> position_splines_;
  NaturalCubicSpline yaw_spline_;
  NaturalCubicSpline progress_spline_;
  std::vector<double> progress_time_knots_;
  std::vector<double> progress_path_knots_;
  double path_length_m_{0.0};
  double geometric_length_m_{0.0};
  double duration_s_{0.0};
  double max_sampled_speed_mps_{0.0};
  double max_sampled_accel_mps2_{0.0};
  double max_sampled_jerk_mps3_{0.0};
  double max_sampled_centripetal_accel_mps2_{0.0};
  double applied_time_scale_{1.0};
  bool valid_{false};
};

} // namespace px4_path

#endif // PX4_PATH__MOTION_TRAJECTORY_HPP_
