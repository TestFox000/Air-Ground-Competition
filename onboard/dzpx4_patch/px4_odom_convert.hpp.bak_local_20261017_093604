#ifndef PX4_ODOM_CONVERT__PX4_ODOM_CONVERT_HPP_
#define PX4_ODOM_CONVERT__PX4_ODOM_CONVERT_HPP_
// ============================================================================
// px4_odom_convert 增强版（air_ground 韧性定位体系，2026-10-16）
//
// 与官方原版的差异（全部来自两天四轮实飞的实测证据，见注释）：
//   1) auto-align：SLAM 位姿直通 + EV reset_counter 一次重锚（团队实验版）
//   2) jump guard：单步大位移 hold 防护（实测 SLAM 跟丢跳变 37~62m）
//   3) armed 门控：解锁状态下跳变永不接受（物理不可能 = 100% 跟丢）
//   4) z 换源：高度用 EKF 反馈 z（气压计主导），SLAM z（漂 5.56/8.9m）绝缘
//
// 部署：tools/patch_deploy.py 一键完成（备份机上原版 → 推本文件 → colcon）。
// 机上对应备份：px4_odom_convert.cpp.bak_20261016 / .bak_v2_20261016。
// ============================================================================

#include <string>
#include <cmath>
#include <memory>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_ros_com/frame_transforms.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

class Px4OdomConvert : public rclcpp::Node
{
public:
  Px4OdomConvert();
  void run();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void onFcuStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  void onEkfLocalPosition(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  void publishVehicleVisualOdometry(
    const nav_msgs::msg::Odometry & odom);

  void computeAlignment(double slam_yaw_ned, double ekf_heading);

  void lookupTfAndPublish();
  uint64_t nowUs();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    sub_local_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vio_pub_;

  // Used only by TF mode to avoid publishing the same TF sample repeatedly.
  builtin_interfaces::msg::Time last_tf_stamp_{};
  bool has_last_tf_stamp_{false};

  std::string source_mode_;
  std::string odom_topic_;
  std::string localization_topic_;
  std::string tf_map_frame_;
  std::string tf_lidar_frame_;
  double z_offset_m_;
  double loop_rate_hz_;

  int odom_ready_count_{0};
  bool odom_ready_{false};

  float nan = std::numeric_limits<float>::quiet_NaN();

  bool has_previous_published_position_{false};
  Eigen::Vector3f previous_published_position_ned_{Eigen::Vector3f::Zero()};

  Eigen::Matrix3d R_enu_slam_;
  Eigen::Vector3d t_enu_slam_;

  // ---- one-shot alignment to EKF local NED frame ----
  // With align_yaw_to_ekf=false (default for SLAM-only flight): the SLAM pose
  // is published unchanged (fixed ENU->NED rotation only) and the EKF is
  // re-anchored to it once via a single reset_counter bump. The EKF frame
  // then coincides with the SLAM map frame, which is what px4_path replay
  // and rviz overlay expect.
  // With align_yaw_to_ekf=true: the SLAM yaw is rotated once (averaged over
  // ~1s, then frozen) to match the EKF heading convention; used when the EKF
  // heading source (mag/GPS) must stay authoritative.
  bool enable_auto_align_{true};
  bool align_yaw_to_ekf_{false};

  bool aligned_{false};
  double yaw_offset_{0.0};

  int align_sample_count_{0};
  double align_ekf_heading_ref_{0.0};
  double align_slam_yaw_ref_{0.0};
  double align_ekf_heading_sum_{0.0};
  double align_slam_yaw_sum_{0.0};

  bool ekf_pos_valid_{false};
  Eigen::Vector3d ekf_pos_ned_{Eigen::Vector3d::Zero()};
  double ekf_heading_{0.0};

  bool has_reset_counters_{false};
  uint8_t last_xy_reset_counter_{0};
  uint8_t last_heading_reset_counter_{0};

  // ---- jump guard：SLAM 跟丢/错配的单步大位移防护 ----
  // 2026-10-16 试飞实测：FAST-LIO 在被搬运/剧烈晃动时跟丢重捕，/localization
  // 单步输出 37~62m 跳变、z 漂 5m；直通策略让 EKF 瞬间被打崩、解锁坐标漂走。
  // 分级处理：小位移放行；大位移先 hold（重发上一有效位姿 + 放大方差，
  // EKF 视角=静止且不可信）；地面待命时连续 jump_confirm_n_ 帧仍大 ->
  // 判定真实大位移（被搬运/换场），接受新位置并 bump EV reset_counter 让
  // EKF 平滑重锚。armed 状态永不接受（见 fcu_armed_）。
  double jump_hold_m_{0.5};     // 单步位置跳变阈值（米）
  double jump_yaw_rad_{0.8};   // 单步 yaw 跳变阈值（弧度，~46°）
  int jump_confirm_n_{8};       // 连续多少帧大位移才判定"真实大位移"（43Hz 下约 0.2s）

  bool gate_has_prev_{false};
  int gate_hold_count_{0};
  Eigen::Vector3d gate_prev_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond gate_prev_quat_{Eigen::Quaterniond::Identity()};

  // 飞控解锁状态（2026-10-16）：armed 时 jump 永不接受——无人机物理上
  // 不可能在 0.2s 内移动数米，armed 大位移 100% 是 SLAM 跟丢，hold 到底
  // 保 EKF；仅地面待命（disarmed）允许"被搬运"式重锚。
  bool fcu_armed_{false};

  // 我们维护的 EV reset_counter：对齐完成时 bump 到 1（EKF 重锚到 SLAM 帧），
  // 之后每次"接受真实大位移"再 +1（EKF 重锚到新位姿），其余时间保持不变。
  uint8_t ev_reset_counter_{0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // PX4_ODOM_CONVERT__PX4_ODOM_CONVERT_HPP_
