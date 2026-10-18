#include "dzmission/eskf.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace dzmission
{

namespace
{
constexpr double kMinDt = 1e-4;
constexpr double kMaxDt = 0.5;
constexpr int kInitSamples = 200;

Eigen::Matrix3d skew(const Eigen::Vector3d &a)
{
  Eigen::Matrix3d m;
  m << 0.0, -a.z(), a.y(),
       a.z(), 0.0, -a.x(),
      -a.y(), a.x(), 0.0;
  return m;
}
}  // namespace

Eskf::Eskf()
{
  P_.setZero();
  // 初始不确定度
  P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-4;   // 位置
  P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-2;   // 速度
  P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1e-4;   // 姿态
  P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-4;   // 陀螺零偏
  P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-2; // 加速度零偏
}

void Eskf::initializeFromImu(const sensor_msgs::msg::Imu &imu)
{
  if (initialized_)
  {
    return;
  }
  gyro_sum_ += Eigen::Vector3d(imu.angular_velocity.x,
                               imu.angular_velocity.y,
                               imu.angular_velocity.z);
  accel_sum_ += Eigen::Vector3d(imu.linear_acceleration.x,
                                imu.linear_acceleration.y,
                                imu.linear_acceleration.z);
  ++init_samples_;
}

bool Eskf::tryFinishInit(double gravity)
{
  if (initialized_ || init_samples_ < kInitSamples)
  {
    return false;
  }

  const Eigen::Vector3d g = gyro_sum_ / static_cast<double>(init_samples_);
  bg_ = g;
  bg_z_ = g.z();

  const Eigen::Vector3d a = accel_sum_ / static_cast<double>(init_samples_);
  // 用重力方向估计 roll/pitch（地面车近似水平）
  const double ax = a.x();
  const double ay = a.y();
  const double az = a.z();
  const double norm = std::sqrt(ax * ax + ay * ay + az * az);
  if (norm > 1e-3)
  {
    roll_ = std::atan2(ay, az);
    pitch_ = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    q_ = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX());
  }
  (void)gravity;

  v_.setZero();
  initialized_ = true;
  return true;
}

void Eskf::normalizeQuaternion() { q_.normalize(); }

void Eskf::projectToPlanar()
{
  // 地面车约束：把姿态投影成纯航向，抑制 roll/pitch 漂移带来的速度投影误差
  yaw_ = std::atan2(2.0 * (q_.w() * q_.z() + q_.x() * q_.y()),
                    1.0 - 2.0 * (q_.y() * q_.y() + q_.z() * q_.z()));
  roll_ = 0.0;
  pitch_ = 0.0;
  q_ = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
}

void Eskf::predict(const sensor_msgs::msg::Imu &imu, double dt)
{
  if (!initialized_)
  {
    return;
  }
  if (dt < kMinDt)
  {
    return;
  }
  dt = std::min(dt, kMaxDt);

  const Eigen::Vector3d omega(imu.angular_velocity.x,
                              imu.angular_velocity.y,
                              imu.angular_velocity.z);
  const Eigen::Vector3d acc(imu.linear_acceleration.x,
                            imu.linear_acceleration.y,
                            imu.linear_acceleration.z);

  const Eigen::Vector3d omega_c = omega - bg_;
  const Eigen::Vector3d acc_c = acc - ba_;

  // ---- 标称状态传播 ----
  const Eigen::Vector3d theta = omega_c * dt;
  const double angle = theta.norm();
  Eigen::Quaterniond dq;
  if (angle > 1e-8)
  {
    dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, theta / angle));
  }
  else
  {
    dq = Eigen::Quaterniond(1.0, 0.5 * theta.x(), 0.5 * theta.y(), 0.5 * theta.z());
  }
  Eigen::Quaterniond q_new = q_ * dq;
  q_new.normalize();

  // 世界系加速度：去掉重力（IMU 测的是比力 f = a - g，即静止时读到 +g 向上）
  // ★ a_world 必须**非 const**：下面要改它的 z 分量（扣重力）。
  //   曾写成 `const Eigen::Vector3d a_world = ...` 而下一行 `a_world.z() -= ...`，
  //   报 "assignment of read-only location"（Eigen 的 .z() 返回可写引用，
  //   但对象是 const 时就变成只读）。
  Eigen::Vector3d a_world = q_new * acc_c;
  a_world.z() -= 9.80665;

  v_ += a_world * dt;
  p_ += v_ * dt + 0.5 * a_world * dt * dt;

  q_ = q_new;

  // ---- 误差状态转移矩阵 ----
  const Eigen::Matrix3d R = q_.toRotationMatrix();
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d a_skew = skew(acc_c);

  Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
  F.block<3, 3>(0, 3) = I * dt;
  F.block<3, 3>(3, 6) = -R * a_skew * dt;
  F.block<3, 3>(3, 12) = -R * dt;
  F.block<3, 3>(6, 6) = I - skew(omega_c) * dt;
  F.block<3, 3>(6, 9) = -I * dt;

  // ---- 过程噪声 ----
  Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
  const double gyro_var = (gyro_noise_ * gyro_noise_) * dt * dt;
  const double accel_var = (accel_noise_ * accel_noise_) * dt * dt;
  const double gbrw_var = (gyro_bias_rw_ * gyro_bias_rw_) * dt;
  const double abrw_var = (accel_bias_rw_ * accel_bias_rw_) * dt;

  Q.block<3, 3>(0, 0) = I * (accel_var * dt * dt * 0.25);
  Q.block<3, 3>(3, 3) = I * accel_var;
  Q.block<3, 3>(6, 6) = I * gyro_var;
  Q.block<3, 3>(9, 9) = I * gbrw_var;
  Q.block<3, 3>(12, 12) = I * abrw_var;

  P_ = F * P_ * F.transpose() + Q;
  P_ = 0.5 * (P_ + P_.transpose()).eval();
  normalizeQuaternion();
}

void Eskf::updateWheelVelocity(double vx, double vy, double sigma_v)
{
  if (!initialized_)
  {
    return;
  }

  // 观测：车体系速度 = 世界速度投影到车体
  const double c = std::cos(yaw_);
  const double s = std::sin(yaw_);
  const double vx_pred = c * v_.x() + s * v_.y();
  const double vy_pred = -s * v_.x() + c * v_.y();

  Eigen::Vector2d residual;
  residual << vx - vx_pred, vy - vy_pred;

  Eigen::Matrix<double, 2, 15> H = Eigen::Matrix<double, 2, 15>::Zero();
  H(0, 3) = c;
  H(0, 4) = s;
  H(0, 6) = -s * v_.x() + c * v_.y();  // d vx_body / d yaw
  H(1, 3) = -s;
  H(1, 4) = c;
  H(1, 6) = -c * v_.x() - s * v_.y();  // d vy_body / d yaw

  Eigen::Matrix2d Rm = Eigen::Matrix2d::Identity() * (sigma_v * sigma_v);

  const Eigen::Matrix<double, 15, 2> K =
      P_ * H.transpose() * (H * P_ * H.transpose() + Rm).inverse();
  const Eigen::Matrix<double, 15, 1> dx = K * residual;

  p_ += dx.segment<3>(0);
  v_ += dx.segment<3>(3);
  const Eigen::Vector3d dtheta = dx.segment<3>(6);
  if (dtheta.norm() > 1e-9)
  {
    q_ = q_ * Eigen::Quaterniond(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
    q_.normalize();
  }
  bg_ += dx.segment<3>(9);
  ba_ += dx.segment<3>(12);
  bg_z_ = bg_.z();

  const Eigen::Matrix<double, 15, 15> IKH =
      Eigen::Matrix<double, 15, 15>::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * Rm * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose()).eval();
  projectToPlanar();
}

void Eskf::updateGyroZ(double wz_measured, double sigma_w)
{
  if (!initialized_)
  {
    return;
  }
  // 仅在"真值角速度为 0"（ZUPT）时调用：
  //   观测模型  wz_measured = 0 + bg_z
  //   观测矩阵  H = d(wz_measured)/d(bg_z) = +1
  const double residual = wz_measured - bg_z_;

  Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
  H(0, 9) = 1.0;

  Eigen::Matrix<double, 1, 1> Rm;
  Rm(0, 0) = sigma_w * sigma_w;

  const Eigen::Matrix<double, 15, 1> K =
      P_ * H.transpose() * (H * P_ * H.transpose() + Rm).inverse();
  const Eigen::Matrix<double, 15, 1> dx = K * residual;

  p_ += dx.segment<3>(0);
  v_ += dx.segment<3>(3);
  bg_ += dx.segment<3>(9);
  ba_ += dx.segment<3>(12);
  bg_z_ = bg_.z();

  P_ = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P_;
  P_ = 0.5 * (P_ + P_.transpose()).eval();
}

void Eskf::updateAbsolutePose(double x, double y, double yaw,
                              double sigma_pos, double sigma_yaw)
{
  if (!initialized_)
  {
    // 尚未初始化时，直接把绝对位姿当作起点
    p_.x() = x;
    p_.y() = y;
    p_.z() = 0.0;
    yaw_ = yaw;
    q_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    v_.setZero();
    return;
  }

  // 航向残差需要归一化到 (-pi, pi]
  double dyaw = yaw - yaw_;
  while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
  while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

  Eigen::Matrix<double, 3, 1> residual;
  residual << x - p_.x(), y - p_.y(), dyaw;

  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 6) = 1.0;

  Eigen::Matrix3d Rm = Eigen::Matrix3d::Zero();
  Rm(0, 0) = sigma_pos * sigma_pos;
  Rm(1, 1) = sigma_pos * sigma_pos;
  Rm(2, 2) = sigma_yaw * sigma_yaw;

  const Eigen::Matrix<double, 15, 3> K =
      P_ * H.transpose() * (H * P_ * H.transpose() + Rm).inverse();
  const Eigen::Matrix<double, 15, 1> dx = K * residual;

  p_ += dx.segment<3>(0);
  v_ += dx.segment<3>(3);
  const Eigen::Vector3d dtheta = dx.segment<3>(6);
  if (dtheta.norm() > 1e-9)
  {
    q_ = q_ * Eigen::Quaterniond(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
    q_.normalize();
  }
  bg_ += dx.segment<3>(9);
  ba_ += dx.segment<3>(12);
  bg_z_ = bg_.z();

  const Eigen::Matrix<double, 15, 15> IKH =
      Eigen::Matrix<double, 15, 15>::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * Rm * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose()).eval();
  projectToPlanar();
}

void Eskf::zeroVelocityUpdate(double sigma_v)
{
  if (!initialized_)
  {
    return;
  }
  const double c = std::cos(yaw_);
  const double s = std::sin(yaw_);
  const double vx_pred = c * v_.x() + s * v_.y();
  const double vy_pred = -s * v_.x() + c * v_.y();

  Eigen::Vector2d residual;
  residual << -vx_pred, -vy_pred;

  Eigen::Matrix<double, 2, 15> H = Eigen::Matrix<double, 2, 15>::Zero();
  H(0, 3) = c;
  H(0, 4) = s;
  H(1, 3) = -s;
  H(1, 4) = c;

  Eigen::Matrix2d Rm = Eigen::Matrix2d::Identity() * (sigma_v * sigma_v);
  const Eigen::Matrix<double, 15, 2> K =
      P_ * H.transpose() * (H * P_ * H.transpose() + Rm).inverse();
  const Eigen::Matrix<double, 15, 1> dx = K * residual;

  p_ += dx.segment<3>(0);
  v_ += dx.segment<3>(3);
  bg_ += dx.segment<3>(9);
  ba_ += dx.segment<3>(12);
  bg_z_ = bg_.z();

  P_ = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P_;
  P_ = 0.5 * (P_ + P_.transpose()).eval();
}

}  // namespace dzmission
