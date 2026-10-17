#include "px4_path/motion_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace px4_path {

namespace {

constexpr double kMinPointDistanceM = 1e-2;
constexpr double kMinSpatialDerivative = 1e-6;
constexpr double kLimitSafetyScale = 1.001;

double norm3(const std::array<double, 3> &value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

std::array<double, 3> cross3(const std::array<double, 3> &a,
                             const std::array<double, 3> &b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

} // namespace

std::vector<PathTurnSplit>
find_path_turn_splits(const std::vector<Waypoint> &points,
                      double threshold_deg) {
  std::vector<PathTurnSplit> splits;
  if (points.size() < 3 || !std::isfinite(threshold_deg) ||
      threshold_deg <= 0.0 || threshold_deg > 180.0) {
    return splits;
  }

  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const double incoming_x = points[i].x - points[i - 1].x;
    const double incoming_y = points[i].y - points[i - 1].y;
    const double incoming_z = points[i].z - points[i - 1].z;
    const double outgoing_x = points[i + 1].x - points[i].x;
    const double outgoing_y = points[i + 1].y - points[i].y;
    const double outgoing_z = points[i + 1].z - points[i].z;
    const double incoming_norm =
        std::sqrt(incoming_x * incoming_x + incoming_y * incoming_y +
                  incoming_z * incoming_z);
    const double outgoing_norm =
        std::sqrt(outgoing_x * outgoing_x + outgoing_y * outgoing_y +
                  outgoing_z * outgoing_z);
    if (incoming_norm < 1e-6 || outgoing_norm < 1e-6) {
      continue;
    }

    const double cosine =
        std::clamp((incoming_x * outgoing_x + incoming_y * outgoing_y +
                    incoming_z * outgoing_z) /
                       (incoming_norm * outgoing_norm),
                   -1.0, 1.0);
    const double angle_deg = std::acos(cosine) * 180.0 / M_PI;
    if (angle_deg >= threshold_deg) {
      splits.push_back({i, angle_deg});
    }
  }
  return splits;
}

bool MotionTrajectory::NaturalCubicSpline::build(
    const std::vector<double> &knots, const std::vector<double> &values) {
  if (knots.size() != values.size() || knots.size() < 2) {
    return false;
  }
  for (std::size_t i = 1; i < knots.size(); ++i) {
    if (!std::isfinite(knots[i]) || knots[i] <= knots[i - 1] ||
        !std::isfinite(values[i])) {
      return false;
    }
  }
  if (!std::isfinite(knots.front()) || !std::isfinite(values.front())) {
    return false;
  }

  knots_ = knots;
  values_ = values;
  second_derivatives_.assign(knots.size(), 0.0);

  if (knots.size() == 2) {
    return true;
  }

  const std::size_t internal_count = knots.size() - 2;
  std::vector<double> lower(internal_count, 0.0);
  std::vector<double> diagonal(internal_count, 0.0);
  std::vector<double> upper(internal_count, 0.0);
  std::vector<double> rhs(internal_count, 0.0);

  for (std::size_t row = 0; row < internal_count; ++row) {
    const std::size_t i = row + 1;
    const double h_prev = knots[i] - knots[i - 1];
    const double h_next = knots[i + 1] - knots[i];
    lower[row] = h_prev;
    diagonal[row] = 2.0 * (h_prev + h_next);
    upper[row] = h_next;
    rhs[row] = 6.0 * ((values[i + 1] - values[i]) / h_next -
                      (values[i] - values[i - 1]) / h_prev);
  }

  for (std::size_t row = 1; row < internal_count; ++row) {
    const double factor = lower[row] / diagonal[row - 1];
    diagonal[row] -= factor * upper[row - 1];
    rhs[row] -= factor * rhs[row - 1];
  }

  second_derivatives_[knots.size() - 2] = rhs.back() / diagonal.back();
  for (std::size_t row = internal_count - 1; row-- > 0;) {
    second_derivatives_[row + 1] =
        (rhs[row] - upper[row] * second_derivatives_[row + 2]) / diagonal[row];
  }
  return true;
}

bool MotionTrajectory::NaturalCubicSpline::build_clamped(
    const std::vector<double> &knots, const std::vector<double> &values,
    double first_slope, double last_slope) {
  if (knots.size() != values.size() || knots.size() < 2 ||
      !std::isfinite(first_slope) || !std::isfinite(last_slope)) {
    return false;
  }
  for (std::size_t i = 0; i < knots.size(); ++i) {
    if (!std::isfinite(knots[i]) || !std::isfinite(values[i]) ||
        (i > 0 && knots[i] <= knots[i - 1])) {
      return false;
    }
  }

  knots_ = knots;
  values_ = values;
  const std::size_t count = knots.size();
  std::vector<double> lower(count, 0.0);
  std::vector<double> diagonal(count, 0.0);
  std::vector<double> upper(count, 0.0);
  std::vector<double> rhs(count, 0.0);

  const double first_h = knots[1] - knots[0];
  diagonal[0] = 2.0 * first_h;
  upper[0] = first_h;
  rhs[0] = 6.0 * ((values[1] - values[0]) / first_h - first_slope);
  for (std::size_t i = 1; i + 1 < count; ++i) {
    const double h_prev = knots[i] - knots[i - 1];
    const double h_next = knots[i + 1] - knots[i];
    lower[i] = h_prev;
    diagonal[i] = 2.0 * (h_prev + h_next);
    upper[i] = h_next;
    rhs[i] = 6.0 * ((values[i + 1] - values[i]) / h_next -
                    (values[i] - values[i - 1]) / h_prev);
  }
  const double last_h = knots[count - 1] - knots[count - 2];
  lower[count - 1] = last_h;
  diagonal[count - 1] = 2.0 * last_h;
  rhs[count - 1] =
      6.0 * (last_slope - (values[count - 1] - values[count - 2]) / last_h);

  for (std::size_t row = 1; row < count; ++row) {
    const double factor = lower[row] / diagonal[row - 1];
    diagonal[row] -= factor * upper[row - 1];
    rhs[row] -= factor * rhs[row - 1];
  }
  second_derivatives_.assign(count, 0.0);
  second_derivatives_.back() = rhs.back() / diagonal.back();
  for (std::size_t row = count - 1; row-- > 0;) {
    second_derivatives_[row] =
        (rhs[row] - upper[row] * second_derivatives_[row + 1]) / diagonal[row];
  }
  return true;
}

std::array<double, 4>
MotionTrajectory::NaturalCubicSpline::evaluate(double knot) const {
  if (knots_.size() < 2) {
    return {};
  }

  knot = std::clamp(knot, knots_.front(), knots_.back());
  auto upper = std::upper_bound(knots_.begin(), knots_.end(), knot);
  std::size_t i = upper == knots_.begin()
                      ? 0
                      : static_cast<std::size_t>(upper - knots_.begin() - 1);
  if (i + 1 >= knots_.size()) {
    i = knots_.size() - 2;
  }

  const double h = knots_[i + 1] - knots_[i];
  const double a = (knots_[i + 1] - knot) / h;
  const double b = (knot - knots_[i]) / h;
  const double m0 = second_derivatives_[i];
  const double m1 = second_derivatives_[i + 1];

  const double value =
      a * values_[i] + b * values_[i + 1] +
      ((a * a * a - a) * m0 + (b * b * b - b) * m1) * h * h / 6.0;
  const double first =
      (values_[i + 1] - values_[i]) / h +
      h * ((-3.0 * a * a + 1.0) * m0 + (3.0 * b * b - 1.0) * m1) / 6.0;
  const double second = a * m0 + b * m1;
  const double third = (m1 - m0) / h;
  return {value, first, second, third};
}

double MotionTrajectory::wrap_pi(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

bool MotionTrajectory::build(const std::vector<Waypoint> &points,
                             const TrajectoryLimits &limits,
                             std::string &error) {
  valid_ = false;
  duration_s_ = 0.0;
  path_length_m_ = 0.0;
  geometric_length_m_ = 0.0;
  progress_time_knots_.clear();
  progress_path_knots_.clear();
  max_sampled_speed_mps_ = 0.0;
  max_sampled_accel_mps2_ = 0.0;
  max_sampled_jerk_mps3_ = 0.0;
  max_sampled_centripetal_accel_mps2_ = 0.0;
  applied_time_scale_ = 1.0;
  error.clear();

  if (!std::isfinite(limits.max_speed_mps) || limits.max_speed_mps <= 0.0 ||
      !std::isfinite(limits.max_accel_mps2) || limits.max_accel_mps2 <= 0.0 ||
      !std::isfinite(limits.max_jerk_mps3) || limits.max_jerk_mps3 <= 0.0 ||
      !std::isfinite(limits.max_centripetal_accel_mps2) ||
      limits.max_centripetal_accel_mps2 <= 0.0 ||
      !std::isfinite(limits.profile_resolution_m) ||
      limits.profile_resolution_m <= 0.0) {
    error = "trajectory limits must be finite and greater than zero";
    return false;
  }

  std::vector<Waypoint> unique_points;
  unique_points.reserve(points.size());
  for (const Waypoint &point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.yaw)) {
      error = "trajectory contains a non-finite waypoint";
      return false;
    }
    if (!unique_points.empty()) {
      const double dx = point.x - unique_points.back().x;
      const double dy = point.y - unique_points.back().y;
      const double dz = point.z - unique_points.back().z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < kMinPointDistanceM) {
        unique_points.back().yaw = point.yaw;
        continue;
      }
    }
    unique_points.push_back(point);
  }

  if (unique_points.size() < 2) {
    error = "trajectory needs at least two distinct positions";
    return false;
  }

  std::vector<double> knots(unique_points.size(), 0.0);
  std::array<std::vector<double>, 3> coordinates;
  for (auto &coordinate : coordinates) {
    coordinate.reserve(unique_points.size());
  }
  std::vector<double> unwrapped_yaw;
  unwrapped_yaw.reserve(unique_points.size());

  for (std::size_t i = 0; i < unique_points.size(); ++i) {
    const Waypoint &point = unique_points[i];
    coordinates[0].push_back(point.x);
    coordinates[1].push_back(point.y);
    coordinates[2].push_back(point.z);

    if (i == 0) {
      unwrapped_yaw.push_back(point.yaw);
    } else {
      const double delta = wrap_pi(point.yaw - unique_points[i - 1].yaw);
      unwrapped_yaw.push_back(unwrapped_yaw.back() + delta);

      const double dx = point.x - unique_points[i - 1].x;
      const double dy = point.y - unique_points[i - 1].y;
      const double dz = point.z - unique_points[i - 1].z;
      knots[i] = knots[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
    }
  }
  path_length_m_ = knots.back();

  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!position_splines_[axis].build(knots, coordinates[axis])) {
      error = "failed to build position spline";
      return false;
    }
  }
  if (!yaw_spline_.build(knots, unwrapped_yaw)) {
    error = "failed to build yaw spline";
    return false;
  }

  // Build a curvature-aware speed profile over the spatial spline. The
  // forward/backward passes make every speed reachable from both the start and
  // the terminal stop under the translational acceleration limit.
  const std::size_t profile_count = std::max<std::size_t>(
      {2, unique_points.size() * 2,
       static_cast<std::size_t>(
           std::ceil(path_length_m_ / limits.profile_resolution_m)) +
           1});
  std::vector<double> profile_s(profile_count, 0.0);
  std::vector<double> arc_step(profile_count, 0.0);
  std::vector<double> spatial_norm(profile_count, 0.0);
  std::vector<double> speed(profile_count, limits.max_speed_mps);
  SpatialSample previous_spatial = sample_spatial(0.0);

  for (std::size_t i = 0; i < profile_count; ++i) {
    profile_s[i] = path_length_m_ * static_cast<double>(i) /
                   static_cast<double>(profile_count - 1);
    const SpatialSample spatial = sample_spatial(profile_s[i]);
    spatial_norm[i] = norm3(spatial.first);
    if (!std::isfinite(spatial_norm[i]) ||
        spatial_norm[i] < kMinSpatialDerivative) {
      error = "spatial spline has a degenerate derivative";
      return false;
    }

    const double curvature = norm3(cross3(spatial.first, spatial.second)) /
                             std::pow(spatial_norm[i], 3);
    if (std::isfinite(curvature) && curvature > 1e-9) {
      speed[i] = std::min(
          speed[i], std::sqrt(limits.max_centripetal_accel_mps2 / curvature));
    }

    if (i > 0) {
      std::array<double, 3> delta{};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        delta[axis] = spatial.position[axis] - previous_spatial.position[axis];
      }
      arc_step[i] = norm3(delta);
      geometric_length_m_ += arc_step[i];
    }
    previous_spatial = spatial;
  }

  speed.front() = 0.0;
  speed.back() = 0.0;
  for (std::size_t i = 1; i < profile_count; ++i) {
    speed[i] = std::min(speed[i],
                        std::sqrt(speed[i - 1] * speed[i - 1] +
                                  2.0 * limits.max_accel_mps2 * arc_step[i]));
  }
  for (std::size_t i = profile_count - 1; i-- > 0;) {
    speed[i] = std::min(
        speed[i], std::sqrt(speed[i + 1] * speed[i + 1] +
                            2.0 * limits.max_accel_mps2 * arc_step[i + 1]));
  }

  progress_time_knots_.assign(profile_count, 0.0);
  progress_path_knots_ = profile_s;
  for (std::size_t i = 0; i + 1 < profile_count; ++i) {
    const double speed_sum = speed[i] + speed[i + 1];
    const double segment_duration =
        speed_sum > 1e-8
            ? 2.0 * arc_step[i + 1] / speed_sum
            : std::sqrt(2.0 * arc_step[i + 1] / limits.max_accel_mps2);
    if (!std::isfinite(segment_duration) || segment_duration <= 0.0) {
      error = "local speed profile produced an invalid segment duration";
      return false;
    }
    progress_time_knots_[i + 1] =
        progress_time_knots_[i] + std::max(1e-3, segment_duration);
  }

  if (!rebuild_progress_spline()) {
    error = "failed to build the local time-allocation spline";
    return false;
  }
  measure_limits();
  if (!std::isfinite(max_sampled_speed_mps_) ||
      !std::isfinite(max_sampled_accel_mps2_) ||
      !std::isfinite(max_sampled_jerk_mps3_) ||
      !std::isfinite(max_sampled_centripetal_accel_mps2_)) {
    error = "local time allocation is not monotonic or finite";
    return false;
  }
  applied_time_scale_ =
      std::max({1.0, max_sampled_speed_mps_ / limits.max_speed_mps,
                std::sqrt(max_sampled_accel_mps2_ / limits.max_accel_mps2),
                std::cbrt(max_sampled_jerk_mps3_ / limits.max_jerk_mps3),
                std::sqrt(max_sampled_centripetal_accel_mps2_ /
                          limits.max_centripetal_accel_mps2)});
  if (applied_time_scale_ > 1.0) {
    applied_time_scale_ *= kLimitSafetyScale;
    for (double &time_s : progress_time_knots_) {
      time_s *= applied_time_scale_;
    }
    if (!rebuild_progress_spline()) {
      error = "failed to rebuild the scaled time-allocation spline";
      return false;
    }
    measure_limits();
  }

  if (!std::isfinite(duration_s_) || duration_s_ <= 0.0) {
    error = "computed trajectory duration is invalid";
    return false;
  }

  valid_ = true;
  return true;
}

MotionTrajectory::SpatialSample
MotionTrajectory::sample_spatial(double path_s) const {
  SpatialSample result;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const auto values = position_splines_[axis].evaluate(path_s);
    result.position[axis] = values[0];
    result.first[axis] = values[1];
    result.second[axis] = values[2];
    result.third[axis] = values[3];
  }
  const auto yaw_values = yaw_spline_.evaluate(path_s);
  result.yaw = yaw_values[0];
  result.yaw_first = yaw_values[1];
  return result;
}

TrajectorySample
MotionTrajectory::sample_kinematics(const ProgressSample &progress) const {
  TrajectorySample result;
  const SpatialSample spatial = sample_spatial(progress.path_s);

  for (std::size_t axis = 0; axis < 3; ++axis) {
    result.position[axis] = spatial.position[axis];
    result.velocity[axis] = spatial.first[axis] * progress.speed;
    result.acceleration[axis] =
        spatial.second[axis] * progress.speed * progress.speed +
        spatial.first[axis] * progress.accel;
    result.jerk[axis] =
        spatial.third[axis] * progress.speed * progress.speed * progress.speed +
        3.0 * spatial.second[axis] * progress.speed * progress.accel +
        spatial.first[axis] * progress.jerk;
  }
  result.yaw = wrap_pi(spatial.yaw);
  result.yaw_speed = spatial.yaw_first * progress.speed;
  return result;
}

MotionTrajectory::ProgressSample
MotionTrajectory::sample_progress(double time_s) const {
  ProgressSample result;
  if (progress_time_knots_.size() < 2) {
    return result;
  }

  time_s = std::clamp(time_s, 0.0, duration_s_);
  const auto values = progress_spline_.evaluate(time_s);
  result.path_s = std::clamp(values[0], 0.0, path_length_m_);
  result.speed = values[1];
  result.accel = values[2];
  result.jerk = values[3];
  return result;
}

bool MotionTrajectory::rebuild_progress_spline() {
  if (!progress_spline_.build_clamped(progress_time_knots_,
                                      progress_path_knots_, 0.0, 0.0)) {
    return false;
  }
  duration_s_ = progress_time_knots_.back();
  return std::isfinite(duration_s_) && duration_s_ > 0.0;
}

void MotionTrajectory::measure_limits() {
  max_sampled_speed_mps_ = 0.0;
  max_sampled_accel_mps2_ = 0.0;
  max_sampled_jerk_mps3_ = 0.0;
  max_sampled_centripetal_accel_mps2_ = 0.0;
  const std::size_t sample_count =
      std::max<std::size_t>(2000, progress_time_knots_.size() * 40);
  for (std::size_t i = 0; i <= sample_count; ++i) {
    const double time_s = duration_s_ * static_cast<double>(i) / sample_count;
    const ProgressSample progress = sample_progress(time_s);
    if (progress.speed < -1e-6) {
      max_sampled_speed_mps_ = std::numeric_limits<double>::infinity();
      max_sampled_accel_mps2_ = std::numeric_limits<double>::infinity();
      max_sampled_jerk_mps3_ = std::numeric_limits<double>::infinity();
      max_sampled_centripetal_accel_mps2_ =
          std::numeric_limits<double>::infinity();
      return;
    }
    const TrajectorySample sample = sample_kinematics(progress);
    max_sampled_speed_mps_ =
        std::max(max_sampled_speed_mps_, norm3(sample.velocity));
    max_sampled_accel_mps2_ =
        std::max(max_sampled_accel_mps2_, norm3(sample.acceleration));
    max_sampled_jerk_mps3_ =
        std::max(max_sampled_jerk_mps3_, norm3(sample.jerk));
    const SpatialSample spatial = sample_spatial(progress.path_s);
    const double derivative_norm = norm3(spatial.first);
    const double curvature =
        derivative_norm > kMinSpatialDerivative
            ? norm3(cross3(spatial.first, spatial.second)) /
                  std::pow(derivative_norm, 3)
            : std::numeric_limits<double>::infinity();
    max_sampled_centripetal_accel_mps2_ =
        std::max(max_sampled_centripetal_accel_mps2_,
                 curvature * norm3(sample.velocity) * norm3(sample.velocity));
  }
}

TrajectorySample MotionTrajectory::sample(double time_s) const {
  if (!valid_) {
    return {};
  }
  return sample_kinematics(sample_progress(time_s));
}

} // namespace px4_path
