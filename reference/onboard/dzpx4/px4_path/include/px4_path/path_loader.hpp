#ifndef PX4_PATH__PATH_LOADER_HPP_
#define PX4_PATH__PATH_LOADER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace px4_path {

// One path point in PX4 NED frame (converted from FAST-LIO SLAM via ROS ENU).
struct Waypoint {
  float x;   // NED north, m
  float y;   // NED east,  m
  float z;   // NED down,  m (positive = below origin)
  float yaw; // rad, PX4 heading (0=north, +CW), wrapped to [-pi, pi]
};

struct LoadResult {
  bool ok{false};
  std::string error; // human-readable, empty on success
  std::vector<std::string> warnings;
  std::vector<Waypoint> points; // NED, ready to publish
};

// Loader options / limits. Height validation is done after SLAM -> ENU.
struct LoadOptions {
  double z_min_enu{0.2};      // warn if z_enu below
  double z_max_enu{5.0};      // warn if z_enu above
  double quat_norm_min{0.98}; // warn if |q| below (still normalized+used)
  double quat_norm_max{1.02}; // warn if |q| above
  double max_neighbor_dist{
      5.0}; // hard-fail if neighbor pts farther apart than this
  std::size_t min_points{2}; // hard-fail below
};

// Pure function: read `path` (FAST-LIO SLAM, "x y z qx qy qz qw" per line),
// validate, convert SLAM -> ENU -> NED, and return NED waypoints. The SLAM ->
// ENU transform matches px4_odom_convert. On failure `ok=false` with reason.
LoadResult load_path(const std::string &path, const LoadOptions &opts = {});

} // namespace px4_path

#endif // PX4_PATH__PATH_LOADER_HPP_
