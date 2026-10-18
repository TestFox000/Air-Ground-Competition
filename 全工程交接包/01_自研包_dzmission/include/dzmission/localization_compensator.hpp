#ifndef DZMISSION_LOCALIZATION_COMPENSATOR_HPP
#define DZMISSION_LOCALIZATION_COMPENSATOR_HPP

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "dzmission/eskf.hpp"

namespace dzmission
{

// =============================================================================
// LocalizationCompensator —— 自身定位补偿
// -----------------------------------------------------------------------------
// ★★ 位置来源：不用 dzactuator 积分出的位姿，改用「行驶距离 + IMU 偏航」★★
//
// 依据（学员代码的实测记录，见 发学员/src(走斜线)/car_localization/serial_reader.py）：
//   "dzactuator's integrated pose lives in an encoder-yaw frame whose rotation
//    does not match the physical world (verified 2026-09-06: after the same 90deg
//    right turn, the raw track drifted to opposite sides across two runs and
//    spirals outward monotonically), so raw pose deltas are unusable as world
//    coordinates. The sample-to-sample travel DISTANCE is rotation-invariant and
//    accurate, so it is integrated along the IMU map yaw instead."
//
// 翻译成工程结论：
//   · 底盘积分位姿的**朝向系是坏的** -> 绝不能把 odom.pose 当世界坐标，也不能
//     把它的位姿当观测量喂给滤波器（会污染状态）
//   · 但**相邻采样的行驶距离**是旋转不变量，且实测准确
//   ⇒ 做法：step = |Δpos_raw|，沿 IMU 偏航角积分位置
//
// 偏航角的来源：
//   用 ESKF 的 IMU 积分航向（连续、不受打滑影响），但需要一个
//   「IMU 偏航 -> 地图偏航」的固定偏移量。该偏移量通过锚点指令标定：
//       /nav/anchor  std_msgs/String  "anchor:COL,ROW,HEADING_DEG"
//     （例如车放在 M7 朝向网格 +x 方向 -> "anchor:M,7,0"）
//   锚点同时把位置吸附到该格中心 —— 这就是"转角位姿快照"的人工版本。
//
// 解决的问题（对应 dzactuator 的已知缺陷）：
//   1. 里程计仅在 linear_Speed > 1e-5 时积分：原地旋转时航向完全丢失。
//      -> 本节点航向全程由 IMU 积分，原地转向也不丢。
//   2. 航向由左右轮速差推算，打滑即漂移 -> 用 IMU。
//   3. 长距离累积误差 -> 锚点 + 转角快照（snap）把漂移限制在一段之内。
//
// 输入：
//   /imu            sensor_msgs/Imu        （航向来源）
//   /odom           nav_msgs/Odometry      （**只用相邻采样的距离**）
//   /nav/anchor     std_msgs/String        （"anchor:COL,ROW,HEADING_DEG"）
//   /amcl_pose      PoseWithCovarianceStamped（可选绝对位姿，仅用于重锚）
// 输出：
//   /odometry/filtered        nav_msgs/Odometry      （map 系位姿）
//   /localization/healthy     std_msgs/UInt8         （1=健康 0=发散，供任务层联锁）
//   /localization/yaw_deg     std_msgs/Float32
//   /localization/dr_step_cm  std_msgs/Float32       （最近一步行程，用于标定）
//   TF: map -> odom（动态重锚，可关）、odom -> base_link（可关）
// =============================================================================
class LocalizationCompensator : public rclcpp::Node
{
public:
  explicit LocalizationCompensator(const rclcpp::NodeOptions &options);
  ~LocalizationCompensator() override;

  bool healthy() const { return healthy_; }
  // 以下四个访问器读的是**航位推算**结果，不是 ESKF 的绝对状态
  double yaw() const { return map_yaw_; }
  double x() const { return dr_x_m_; }
  double y() const { return dr_y_m_; }
  double v() const { return last_step_v_; }
  double w() const { return last_w_; }
  bool hasAbsoluteFix() const { return has_fix_; }
  // 供任务层在"到达某格心"时调用，把位置吸附到格心（限制漂移累积）
  void snapToCell(int col, int row);

private:
  void loadParams();

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void anchorCallback(const std_msgs::msg::String::SharedPtr msg);
  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void watchdogCallback();

  // IMU / 里程计是否新鲜 + 状态是否有限；调用者需持有 mtx_
  bool evaluateHealth() const;

  void publishEstimate();
  void publishTransform(double x, double y, double yaw);

  // ---- 参数 ----
  std::string imu_topic_{"imu_data"};
  std::string odom_topic_{"odom"};
  std::string amcl_topic_{"amcl_pose"};
  std::string anchor_topic_{"nav/anchor"};
  double imu_timeout_ms_{200.0};
  double odom_timeout_ms_{300.0};
  double startup_grace_ms_{3000.0};  // 启动后多久开始判定故障
  double zupt_linear_thresh_{0.02};   // m/s
  double zupt_gyro_thresh_{0.05};     // rad/s
  double zupt_min_duration_ms_{300.0};
  double wheel_sigma_v_{0.05};
  double gyro_sigma_z_{0.01};
  bool enable_absolute_fix_{true};
  double fix_sigma_pos_{0.10};
  double fix_sigma_yaw_{0.05};
  double fix_min_interval_ms_{200.0};
  double fix_max_jump_m_{2.0};        // 超过视为异常观测，拒绝
  bool publish_tf_{true};
  bool publish_map_to_odom_{true};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string map_frame_{"map"};

  // ---- ★ 航位推算（DR）参数 ----
  // 忽略小于此值的单步行程（m），用于滤掉静止时的抖动
  double dr_min_step_m_{0.0005};
  // 单步行程上限（m）：超过视为采样跳变/异常，丢弃该步
  double dr_max_step_m_{0.5};
  // 启动时的地图偏航角（度）。若已知车的初始朝向，直接填；
  // 否则用 /nav/anchor 标定
  double initial_map_yaw_deg_{0.0};
  // 初始位置（格坐标）。默认 M7 = (12, 6)
  int initial_col_{12};
  int initial_row_{6};
  // 是否在启动时就用初始格/朝向做一次锚定
  bool anchor_at_startup_{true};
  // 每格边长（m），用于 anchor 指令换算格坐标为米
  double block_size_m_{0.35};
  // 锚点指令前缀
  std::string anchor_prefix_{"anchor:"};

  // ---- 状态 ----
  Eskf eskf_;
  std::mutex mtx_;

  rclcpp::Time last_imu_time_;
  rclcpp::Time last_imu_rx_time_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_fix_time_;
  rclcpp::Time last_pub_time_;
  rclcpp::Time first_data_time_;
  bool have_imu_{false};
  bool have_odom_{false};
  bool has_fix_{false};

  bool healthy_{false};
  double last_w_{0.0};
  double stationary_since_{-1.0};

  // ---- ★ 航位推算状态 ----
  // IMU 原始偏航的第一帧读数（作为相对零点）与到地图系的固定偏移
  double imu_yaw_bias_{0.0};
  double imu_yaw_rel_{0.0};
  bool have_imu_yaw_{false};
  double yaw_offset_rad_{0.0};      // map_yaw = imu_yaw_rel + yaw_offset
  bool yaw_offset_valid_{false};
  // 地图系偏航（度）与航位推算位置（m）
  double map_yaw_{0.0};
  double dr_x_m_{0.0};
  double dr_y_m_{0.0};
  // 上一步的原始位姿（只用来算距离）+ 累计行程
  bool have_prev_raw_{false};
  double prev_raw_x_{0.0};
  double prev_raw_y_{0.0};
  double total_travel_m_{0.0};
  double last_step_m_{0.0};
  double last_step_v_{0.0};
  rclcpp::Time last_step_time_;

  // map->odom 锚点
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  double anchor_yaw_{0.0};
  bool anchor_valid_{false};

  // 保存最近一次原始轮速，供 ZUPT 与判活
  double wheel_vx_{0.0};
  double wheel_vy_{0.0};

  // 订阅/发布
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_anchor_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_healthy_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yaw_deg_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_bias_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_step_cm_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace dzmission

#endif  // DZMISSION_LOCALIZATION_COMPENSATOR_HPP
