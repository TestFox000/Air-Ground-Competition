#include "px4_odom_convert.hpp"

#include <iomanip>
#include <iostream>

#include <tf2/exceptions.h>

namespace {

double wrapPi(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

// Yaw (rad, NED convention) extracted from a PX4-convention quaternion
// (w, x, y, z), rotation from FRD body to NED world.
double yawFromQuat(const Eigen::Quaterniond & q)
{
  const double siny = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny, cosy);
}

Eigen::Quaterniond quatFromYaw(double yaw)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

}  // namespace


Px4OdomConvert::Px4OdomConvert()
: Node("px4_odom_convert")
{
  source_mode_          = this->declare_parameter<std::string>("source_mode", "odom");
  odom_topic_           = this->declare_parameter<std::string>("odom_topic", "/Odometry");
  localization_topic_   = this->declare_parameter<std::string>("localization_topic", "/localization");
  tf_map_frame_         = this->declare_parameter<std::string>("tf_map_frame", "map");
  tf_lidar_frame_       = this->declare_parameter<std::string>("tf_lidar_frame", "lidar");
  z_offset_m_           = this->declare_parameter<double>("z_offset_m", 0.0);
  loop_rate_hz_         = this->declare_parameter<double>("loop_rate_hz", 50.0);

  R_enu_slam_ <<
    -1.0, 0.0, 0.0,
     0.0,-1.0, 0.0,
     0.0, 0.0, 1.0;

  t_enu_slam_ << 
    0.0,
    0.0,
    0.0;

  vio_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
    "/fmu/in/vehicle_visual_odometry",
    rclcpp::SensorDataQoS());

  enable_auto_align_ =
    this->declare_parameter<bool>("enable_auto_align", true);
  align_yaw_to_ekf_ =
    this->declare_parameter<bool>("align_yaw_to_ekf", false);

  if (enable_auto_align_) {
    sub_local_pos_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position",
        rclcpp::SensorDataQoS(),
        std::bind(
          &Px4OdomConvert::onEkfLocalPosition,
          this,
          std::placeholders::_1));

    // 解锁状态监测：armed 时 jump guard 永不接受大位移（物理上不可能）
    sub_status_ =
      this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status",
        rclcpp::SensorDataQoS(),
        std::bind(
          &Px4OdomConvert::onFcuStatus,
          this,
          std::placeholders::_1));

    if (align_yaw_to_ekf_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Auto-align: SLAM yaw rotated once (~1s average) to match EKF "
        "heading, then frozen; reset_counter bumped once");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Auto-align: SLAM pose published unchanged; EKF re-anchored to "
        "the SLAM frame once via reset_counter bump");
    }
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Auto-align disabled: fixed ENU rotation only");
  }

  if (source_mode_ == "tf") {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(this->get_logger(), "Mode: tf  [%s -> %s]",
      tf_map_frame_.c_str(), tf_lidar_frame_.c_str());
  } else {
    const std::string topic =
      (source_mode_ == "localization") ? localization_topic_ : odom_topic_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      topic,
      rclcpp::SensorDataQoS(),
      std::bind(&Px4OdomConvert::odomCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Mode: %s  topic: %s",
      source_mode_.c_str(), topic.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Publish PX4 visual odom: /fmu/in/vehicle_visual_odometry");
  if (source_mode_ == "tf") {
    RCLCPP_INFO(this->get_logger(), "TF lookup rate: %.1f Hz  z_offset: %.3f m",
      loop_rate_hz_, z_offset_m_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Input-driven publishing (no frame filling)  z_offset: %.3f m",
      z_offset_m_);
  }
  RCLCPP_INFO(this->get_logger(),
    "Resilience layer: jump_guard(%.2fm/%.0fdeg, armed=never-accept) + z-from-EKF",
    0.5, 0.8 * 180.0 / M_PI);
}

uint64_t Px4OdomConvert::nowUs()
{
  return static_cast<uint64_t>(this->now().nanoseconds() / 1000);
}

void Px4OdomConvert::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!odom_ready_)
    {
        odom_ready_count_++;

        if (odom_ready_count_ >= 20)
        {
            odom_ready_ = true;

            RCLCPP_INFO(
              this->get_logger(),
              "Odometry initialized, start publishing PX4 VIO"
            );
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
              this->get_logger(),
              *this->get_clock(),
              1000,
              "Waiting odom initialization: %d/20",
              odom_ready_count_
            );
        }
    }

    if (odom_ready_)
    {
        publishVehicleVisualOdometry(*msg);
    }
}

void Px4OdomConvert::onFcuStatus(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  fcu_armed_ = (msg->arming_state == 2);  // 2 = ARMED
}

void Px4OdomConvert::onEkfLocalPosition(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  const bool valid =
    msg->xy_valid && msg->z_valid &&
    std::isfinite(msg->x) && std::isfinite(msg->y) &&
    std::isfinite(msg->z) && std::isfinite(msg->heading);

  if (valid) {
    ekf_pos_ned_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
    ekf_heading_ = msg->heading;

    if (has_reset_counters_ &&
      (msg->xy_reset_counter != last_xy_reset_counter_ ||
       msg->heading_reset_counter != last_heading_reset_counter_))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "EKF frame reset detected (xy_reset=%u heading_reset=%u); "
        "alignment transform stays frozen, EKF will re-converge to SLAM",
        msg->xy_reset_counter, msg->heading_reset_counter);

      // If the reset hits while we are still averaging the initial
      // alignment, the average is meaningless — start over.
      if (!aligned_) {
        align_sample_count_ = 0;
        align_ekf_heading_sum_ = 0.0;
        align_slam_yaw_sum_ = 0.0;
      }
    }

    last_xy_reset_counter_ = msg->xy_reset_counter;
    last_heading_reset_counter_ = msg->heading_reset_counter;
    has_reset_counters_ = true;
  }

  ekf_pos_valid_ = valid;
}

void Px4OdomConvert::computeAlignment(
  double slam_yaw_ned,
  double ekf_heading)
{
  yaw_offset_ = wrapPi(ekf_heading - slam_yaw_ned);

  aligned_ = true;
  ev_reset_counter_ = 1;  // 对齐完成：bump 0->1，EKF 重锚到 SLAM 帧

  RCLCPP_INFO(
    this->get_logger(),
    "Aligned SLAM->EKF: yaw_offset=%.1f deg; reset_counter bumped 0->1, "
    "EKF will re-anchor to the SLAM pose once",
    yaw_offset_ * 180.0 / M_PI);
}

void Px4OdomConvert::publishVehicleVisualOdometry(
  const nav_msgs::msg::Odometry & odom)
{
  namespace ft = px4_ros_com::frame_transforms;

  px4_msgs::msg::VehicleOdometry vio{};


  const auto & p = odom.pose.pose.position;
  const auto & q = odom.pose.pose.orientation;

  vio.timestamp = nowUs();
  vio.timestamp_sample =
    static_cast<uint64_t>(odom.header.stamp.sec) * 1000000ULL +
    static_cast<uint64_t>(odom.header.stamp.nanosec) / 1000ULL;

  vio.pose_frame =
    px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

  vio.velocity_frame =
    px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;

  Eigen::Vector3d position_slam(
    p.x,
    p.y,
    p.z - z_offset_m_
  );

  Eigen::Vector3d position_enu =
    R_enu_slam_ * position_slam
    + t_enu_slam_;

  Eigen::Quaterniond q_slam(
    q.w,
    q.x,
    q.y,
    q.z
  );

  Eigen::Matrix3d R_slam_body =
    q_slam.toRotationMatrix();

  Eigen::Matrix3d R_enu_body =
    R_enu_slam_ * R_slam_body;

  Eigen::Quaterniond q_enu(
    R_enu_body
  );

  Eigen::Quaterniond q_px4 =
    ft::ros_to_px4_orientation(q_enu);

  Eigen::Vector3d position_ned =
    ft::enu_to_ned_local_frame(
        position_enu
    );

  if (enable_auto_align_) {
    const double slam_yaw_ned = yawFromQuat(q_px4);

    if (!aligned_ && ekf_pos_valid_) {
      align_sample_count_++;

      if (align_yaw_to_ekf_) {
        if (align_sample_count_ == 1) {
          align_ekf_heading_ref_ = ekf_heading_;
          align_slam_yaw_ref_ = slam_yaw_ned;
        }
        align_ekf_heading_sum_ +=
          wrapPi(ekf_heading_ - align_ekf_heading_ref_);
        align_slam_yaw_sum_ +=
          wrapPi(slam_yaw_ned - align_slam_yaw_ref_);

        if (align_sample_count_ >= 100) {
          const double slam_yaw_avg = wrapPi(
            align_slam_yaw_ref_ +
            align_slam_yaw_sum_ / static_cast<double>(align_sample_count_));
          const double ekf_heading_avg = wrapPi(
            align_ekf_heading_ref_ +
            align_ekf_heading_sum_ / static_cast<double>(align_sample_count_));

          computeAlignment(slam_yaw_avg, ekf_heading_avg);
        }
      } else if (align_sample_count_ >= 30) {
        aligned_ = true;
        ev_reset_counter_ = 1;  // 对齐完成（SLAM 直通模式）：bump 0->1 让 EKF 重锚
        RCLCPP_INFO(
          this->get_logger(),
          "Publishing SLAM pose unchanged; reset_counter bumped 0->1, "
          "EKF re-anchors to the SLAM frame once");
      }
    }

    if (aligned_) {
      const Eigen::Matrix3d Rz =
        Eigen::AngleAxisd(yaw_offset_, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();

      position_ned = Rz * position_ned;
      q_px4 = quatFromYaw(yaw_offset_) * q_px4;
    }
  }

  // ===== z 轴换源（2026-10-16 二次实飞实测落地）=====
  // 两天两个场地 SLAM 的 z 分别漂 5.56m / 8.9m（雷达 z 观测随环境劣化），
  // 而 EKF 的 z 由气压计主导、悬停高度始终正常。这里用 EKF 反馈 z 替换
  // SLAM z：EV z 与 EKF z 零残差（对 z 不提供信息），等效于关闭 EV-z
  // 融合——高度从此完全由气压计决定，SLAM z 漂移不再影响飞控。
  if (ekf_pos_valid_) {
    position_ned.z() = ekf_pos_ned_.z();
  }

  // ===== jump guard：SLAM 跟丢/错配的单步大位移防护 =====
  // 试飞实测（2026-10-16）：/localization 单步跳变 37~62m、z 漂 5m 会瞬间
  // 打崩 EKF（解锁坐标漂走、ARM 目标跑到 5m 外）。小位移放行；大位移先
  // hold（重发上一有效位姿 + 放大方差）；地面待命时连续 jump_confirm_n_
  // 帧仍大 -> 判定真实大位移（被搬运/换场），接受新位置并 bump reset_counter
  // 平滑重锚。armed 时永不接受（见 fcu_armed_）。
  bool gate_holding = false;
  if (gate_has_prev_) {
    const double gate_dp = (position_ned - gate_prev_pos_).norm();
    const double gate_dy =
      std::fabs(wrapPi(yawFromQuat(q_px4) - yawFromQuat(gate_prev_quat_)));
    if (gate_dp > jump_hold_m_ || gate_dy > jump_yaw_rad_) {
      ++gate_hold_count_;
      // armed 时永不接受：无人机物理上不可能 0.2s 内移动数米，armed 大
      // 位移 100% 是 SLAM 跟丢——hold 到底保 EKF，SLAM 恢复后自然放行。
      // 仅地面待命（disarmed）允许"被搬运"式重锚（连续 N 帧一致才接受）。
      const bool accept = !fcu_armed_ && gate_hold_count_ >= jump_confirm_n_;
      if (accept) {
        RCLCPP_WARN(
          this->get_logger(),
          "jump guard: real large displacement accepted (dpos=%.2f m, "
          "dyaw=%.0f deg after %d held frames); bumping EV reset_counter "
          "for EKF re-anchor",
          gate_dp, gate_dy * 180.0 / M_PI, gate_hold_count_);
        gate_hold_count_ = 0;
        gate_prev_pos_ = position_ned;
        gate_prev_quat_ = q_px4;
        ++ev_reset_counter_;  // EKF 重锚到新位姿（uint8 自动回绕）
      } else {
        gate_holding = true;
        position_ned = gate_prev_pos_;
        q_px4 = gate_prev_quat_;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "jump guard: holding previous pose (dpos=%.2f m, dyaw=%.0f deg, "
          "frame %d%s)",
          gate_dp, gate_dy * 180.0 / M_PI, gate_hold_count_,
          fcu_armed_ ? ", armed=永不接受" : "");
      }
    } else {
      gate_hold_count_ = 0;
      gate_prev_pos_ = position_ned;
      gate_prev_quat_ = q_px4;
    }
  } else {
    gate_prev_pos_ = position_ned;
    gate_prev_quat_ = q_px4;
    gate_has_prev_ = true;
  }

  vio.position[0] =
    static_cast<float>(position_ned.x());

  vio.position[1] =
    static_cast<float>(position_ned.y());

  vio.position[2] =
    static_cast<float>(position_ned.z());

  vio.velocity[0] = nan;
  vio.velocity[1] = nan;
  vio.velocity[2] = nan;

  vio.q[0] =
    static_cast<float>(q_px4.w());

  vio.q[1] =
    static_cast<float>(q_px4.x());

  vio.q[2] =
    static_cast<float>(q_px4.y());

  vio.q[3] =
    static_cast<float>(q_px4.z());

  vio.angular_velocity[0] = nan;
  vio.angular_velocity[1] = nan;
  vio.angular_velocity[2] = nan;

  // hold 期间位置不可信：方差放大两个数量级，EKF 会大幅降低该测量权重
  const float gate_pos_var = gate_holding ? 100.0f : 0.01f;
  const float gate_quat_var = gate_holding ? 100.0f : 0.01f;

  vio.position_variance[0] = gate_pos_var;
  vio.position_variance[1] = gate_pos_var;
  vio.position_variance[2] = gate_pos_var;

  vio.orientation_variance[0] = gate_quat_var;
  vio.orientation_variance[1] = gate_quat_var;
  vio.orientation_variance[2] = gate_quat_var;

  vio.velocity_variance[0] = nan;
  vio.velocity_variance[1] = nan;
  vio.velocity_variance[2] = nan;

  // reset_counter 变化即"EV 帧变化"：对齐完成 0->1（重锚 SLAM 帧），之后
  // jump guard 每次接受真实大位移再 +1（重锚新位姿）。其余时间恒定，
  // 不触发 EKF reset。hold 期间保持不变（EKF 视角=同一帧的静止测量）。
  vio.reset_counter = ev_reset_counter_;

  vio.quality = 100;

  const Eigen::Vector3f current_position_ned(
    vio.position[0], vio.position[1], vio.position[2]);

  if (has_previous_published_position_) {
    const Eigen::Vector3f delta_position =
      current_position_ned - previous_published_position_ned_;

    std::cout << std::fixed << std::setprecision(4)
              << "[px4_odom_convert] position delta NED [m]: "
              << "dx=" << delta_position.x() << "  "
              << "dy=" << delta_position.y() << "  "
              << "dz=" << delta_position.z()
              << std::endl;
  } else {
    std::cout << std::fixed << std::setprecision(4)
              << "[px4_odom_convert] first position NED [m]: "
              << "x=" << current_position_ned.x() << "  "
              << "y=" << current_position_ned.y() << "  "
              << "z=" << current_position_ned.z()
              << std::endl;
  }

  previous_published_position_ned_ = current_position_ned;
  has_previous_published_position_ = true;

  vio_pub_->publish(vio);
}

void Px4OdomConvert::lookupTfAndPublish()
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      tf_map_frame_, tf_lidar_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "TF lookup [%s->%s] failed: %s",
      tf_map_frame_.c_str(), tf_lidar_frame_.c_str(), ex.what());
    return;
  }

  if (has_last_tf_stamp_ && tf_stamped.header.stamp == last_tf_stamp_) {
    return;
  }
  last_tf_stamp_ = tf_stamped.header.stamp;
  has_last_tf_stamp_ = true;

  nav_msgs::msg::Odometry odom{};
  odom.header.stamp = tf_stamped.header.stamp;
  odom.header.frame_id = tf_stamped.header.frame_id;
  odom.child_frame_id = tf_stamped.child_frame_id;
  odom.pose.pose.position.x = tf_stamped.transform.translation.x;
  odom.pose.pose.position.y = tf_stamped.transform.translation.y;
  odom.pose.pose.position.z = tf_stamped.transform.translation.z;
  odom.pose.pose.orientation = tf_stamped.transform.rotation;
  // velocity fields remain zero — PX4 EKF2 is configured to not fuse velocity

  publishVehicleVisualOdometry(odom);
}

void Px4OdomConvert::run()
{
  if (source_mode_ != "tf") {
    rclcpp::spin(shared_from_this());
    return;
  }

  rclcpp::Rate rate(loop_rate_hz_);

  while (rclcpp::ok()) {
    rclcpp::spin_some(shared_from_this());
    lookupTfAndPublish();
    rate.sleep();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Px4OdomConvert>();
  node->run();

  rclcpp::shutdown();
  return 0;
}
