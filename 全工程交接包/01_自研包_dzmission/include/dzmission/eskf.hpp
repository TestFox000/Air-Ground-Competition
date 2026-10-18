#ifndef DZMISSION_ESKF_HPP
#define DZMISSION_ESKF_HPP

#include <Eigen/Dense>

#include "sensor_msgs/msg/imu.hpp"
namespace dzmission
{

// =============================================================================
// Eskf —— 轻量误差状态卡尔曼滤波器（2.5D：位置/速度/航向 + IMU 零偏）
// -----------------------------------------------------------------------------
// 为什么需要它：
//   dzactuator 的里程计存在两个已知问题
//     （1）只有 linear_Speed>0 时才积分，纯旋转时 Robot_Pos.Z 不更新
//     （2）角速度由左右轮速差推出，打滑与机械误差直接变成航向漂移
//   两者叠加导致纯里程计航向漂移很快，无法支撑长距离路径跟踪。
//
// 本滤波器的作用：
//   以 IMU 陀螺的 yaw 角速度为主做航向传播（IMU 精度远高于轮速差），
//   用轮式里程计的速度做速度与位置观测，并在线估计陀螺 z 轴零偏，
//   从而抑制漂移；再把绝对位姿（AMCL / 视觉）作为位置+航向观测做重置。
//
// 状态量（15 维误差状态）：
//   δp(3) δv(3) δθ(3, roll/pitch/yaw) δbg(3) δba(3)
//   标称状态：p(3) v(3) q(4) bg(3) ba(3)
//
// 注意：这是为低成本 IMU + 轮式里程计设计的工程实现，不是高精度惯导。
//       重力对齐、roll/pitch 可观测性在本车上意义有限（车体近水平），
//       因此 roll/pitch 只做姿态投影，不参与位置传播。
// =============================================================================
class Eskf
{
public:
  Eskf();

  bool initialized() const { return initialized_; }
  double gyroBiasZ() const { return bg_z_; }

  // 静止初始化：累积若干帧 IMU，估计陀螺零偏与初始姿态
  void initializeFromImu(const sensor_msgs::msg::Imu &imu);
  bool tryFinishInit(double gravity);

  // 预测：IMU（角速度 + 线加速度）
  void predict(const sensor_msgs::msg::Imu &imu, double dt);

  // 更新：轮式里程计（速度观测，body frame）
  //   vx, vy 单位 m/s；sigma_v 为速度标准差
  void updateWheelVelocity(double vx, double vy, double sigma_v);

  // 更新：陀螺 z 轴角速度直接观测（可选，主要靠 predict 的零偏在线估计）
  void updateGyroZ(double wz, double sigma_w);

  // 更新：绝对位姿观测（map 系位置 + 航向），用于 AMCL / 视觉重置
  //   sigma_pos 单位 m，sigma_yaw 单位 rad
  void updateAbsolutePose(double x, double y, double yaw,
                          double sigma_pos, double sigma_yaw);

  // 姿态零速修正（ZUPT）：车辆静止时把速度与角速度归零
  void zeroVelocityUpdate(double sigma_v);

  // ---- 状态读取 ----
  const Eigen::Vector3d &position() const { return p_; }
  const Eigen::Vector3d &velocity() const { return v_; }
  double yaw() const { return yaw_; }
  double roll() const { return roll_; }
  double pitch() const { return pitch_; }
  Eigen::Quaterniond quaternion() const { return q_; }
  Eigen::Matrix3d positionCovariance() const { return P_.block<3, 3>(0, 0); }

private:
  void normalizeQuaternion();
  void projectToPlanar();  // 把姿态投影为纯航向（地面车约束）

  bool initialized_{false};
  int init_samples_{0};
  Eigen::Vector3d gyro_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_sum_{Eigen::Vector3d::Zero()};

  // 标称状态
  Eigen::Vector3d p_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()};

  double roll_{0.0};
  double pitch_{0.0};
  double yaw_{0.0};

  // 误差状态协方差 15x15
  Eigen::Matrix<double, 15, 15> P_;

  // 噪声参数
  double gyro_noise_{0.002};    // rad/s/sqrt(Hz)
  double accel_noise_{0.02};    // m/s^2/sqrt(Hz)
  double gyro_bias_rw_{0.00002};  // rad/s^2/sqrt(Hz)
  double accel_bias_rw_{0.0002};  // m/s^3/sqrt(Hz)

  double bg_z_{0.0};
  double last_yaw_rate_{0.0};
};

}  // namespace dzmission

#endif  // DZMISSION_ESKF_HPP
