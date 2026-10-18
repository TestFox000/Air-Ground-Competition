#ifndef PX4_ODOM_CONVERT__PX4_ODOM_CONVERT_HPP_
#define PX4_ODOM_CONVERT__PX4_ODOM_CONVERT_HPP_

#include <string>
#include <cmath>
#include <memory>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
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

  void publishVehicleVisualOdometry(
    const nav_msgs::msg::Odometry & odom);

  void lookupTfAndPublish();
  uint64_t nowUs();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // PX4_ODOM_CONVERT__PX4_ODOM_CONVERT_HPP_
