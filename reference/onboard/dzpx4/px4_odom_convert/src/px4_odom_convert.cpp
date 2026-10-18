#include "px4_odom_convert.hpp"

#include <iomanip>
#include <iostream>

#include <tf2/exceptions.h>


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

  vio.position_variance[0] = 0.01f;
  vio.position_variance[1] = 0.01f;
  vio.position_variance[2] = 0.01f;

  vio.orientation_variance[0] = 0.01f;
  vio.orientation_variance[1] = 0.01f;
  vio.orientation_variance[2] = 0.01f;

  vio.velocity_variance[0] = nan;
  vio.velocity_variance[1] = nan;
  vio.velocity_variance[2] = nan;

  vio.reset_counter = 0;

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
