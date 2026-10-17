#include "px4_path/path_loader.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

namespace px4_path {

namespace {

double wrap_pi(double a) {
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

struct Position3 {
  double x;
  double y;
  double z;
};

// Keep this transform identical to px4_odom_convert's R_enu_slam_ and
// t_enu_slam_. FAST-LIO's SLAM frame is rotated 180 degrees about +Z relative
// to the ENU frame used before conversion to PX4 NED.
Position3 slam_to_enu(double x_slam, double y_slam, double z_slam) {
  return {-x_slam, -y_slam, z_slam};
}

Position3 enu_to_ned(const Position3 &enu) { return {enu.y, enu.x, -enu.z}; }

std::string trim(const std::string &s) {
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

} // namespace

LoadResult load_path(const std::string &path, const LoadOptions &opts) {
  LoadResult r;

  std::ifstream f(path);
  if (!f.is_open()) {
    r.error = "cannot open path file: " + path;
    return r;
  }

  std::string raw_line;
  std::size_t lineno = 0;
  while (std::getline(f, raw_line)) {
    ++lineno;
    const std::string line = trim(raw_line);
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);
    double x_slam, y_slam, z_slam, qx, qy, qz, qw;
    if (!(iss >> x_slam >> y_slam >> z_slam >> qx >> qy >> qz >> qw)) {
      std::ostringstream os;
      os << "line " << lineno
         << ": expected 7 floats (x y z qx qy qz qw), got: " << line;
      r.error = os.str();
      return r;
    }
    // Anything else on the line is a hard error too — likely a typo the user
    // will want to know about rather than silently drop.
    std::string extra;
    if (iss >> extra) {
      std::ostringstream os;
      os << "line " << lineno << ": unexpected extra token '" << extra << "'";
      r.error = os.str();
      return r;
    }

    const Position3 position_enu = slam_to_enu(x_slam, y_slam, z_slam);
    const Position3 position_ned = enu_to_ned(position_enu);

    // Validate after SLAM -> ENU so diagnostics describe the intermediate
    // frame used by the standard ENU -> NED conversion.
    if (position_enu.z < opts.z_min_enu || position_enu.z > opts.z_max_enu) {
      std::ostringstream os;
      os << "line " << lineno << ": z_enu=" << position_enu.z << " outside ["
         << opts.z_min_enu << ", " << opts.z_max_enu << "] (soft warn)";
      r.warnings.push_back(os.str());
    }

    double qn = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (qn < 1e-6) {
      std::ostringstream os;
      os << "line " << lineno << ": quaternion has zero norm";
      r.error = os.str();
      return r;
    }
    if (qn < opts.quat_norm_min || qn > opts.quat_norm_max) {
      std::ostringstream os;
      os << "line " << lineno << ": |q|=" << qn << " outside ["
         << opts.quat_norm_min << ", " << opts.quat_norm_max
         << "], will normalize";
      r.warnings.push_back(os.str());
    }
    // Normalize before yaw extraction so out-of-tolerance quaternions still
    // produce a valid heading.
    qx /= qn;
    qy /= qn;
    qz /= qn;
    qw /= qn;

    // Extract FAST-LIO yaw, apply the same 180-degree SLAM -> ENU rotation as
    // px4_odom_convert, then convert the ENU heading to PX4 NED.
    const double yaw_slam =
        std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    const double yaw_enu = wrap_pi(yaw_slam + M_PI);
    const double yaw_ned = wrap_pi(M_PI / 2.0 - yaw_enu);

    Waypoint w;
    w.x = static_cast<float>(position_ned.x);
    w.y = static_cast<float>(position_ned.y);
    w.z = static_cast<float>(position_ned.z);
    w.yaw = static_cast<float>(yaw_ned);
    r.points.push_back(w);
  }

  if (r.points.size() < opts.min_points) {
    std::ostringstream os;
    os << "need at least " << opts.min_points << " points, got "
       << r.points.size();
    r.error = os.str();
    return r;
  }

  // Neighbor-distance hard fail. Catches "typed 100 instead of 10" mistakes
  // before the drone tries to fly there.
  for (std::size_t i = 1; i < r.points.size(); ++i) {
    const float dx = r.points[i].x - r.points[i - 1].x;
    const float dy = r.points[i].y - r.points[i - 1].y;
    const float dz = r.points[i].z - r.points[i - 1].z;
    const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (d > opts.max_neighbor_dist) {
      std::ostringstream os;
      os << "point " << i - 1 << " -> " << i << " distance " << d
         << " m > max_neighbor_dist " << opts.max_neighbor_dist << " m";
      r.error = os.str();
      return r;
    }
  }

  r.ok = true;
  return r;
}

} // namespace px4_path
