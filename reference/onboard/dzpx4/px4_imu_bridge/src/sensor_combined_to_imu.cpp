/*
*@brief PX4 SensorCombined to ROS Imu cpp
*/

#include "../include/sensor_combined_to_imu.hpp"

#include <array>
#include <chrono>
#include <memory>

namespace px4_imu_bridge
{

SensorCombinedToImu::SensorCombinedToImu()
: Node("sensor_combined_to_imu")
{
  input_topic_ = this->declare_parameter<std::string>(
    "input_topic", "/fmu/out/sensor_combined");
  output_topic_ = this->declare_parameter<std::string>(
    "output_topic", "/fmu/out/imu_data");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "body");
  convert_frd_to_flu_ = this->declare_parameter<bool>("convert_frd_to_flu", true);
  log_conversion_time_ = this->declare_parameter<bool>("log_conversion_time", true);
  timing_log_period_ms_ = this->declare_parameter<int>("timing_log_period_ms", 1000);
  last_timing_log_time_ = std::chrono::steady_clock::now();

  const auto qos = rclcpp::SensorDataQoS();

  sensor_combined_sub_ = this->create_subscription<SensorCombined>(
    input_topic_,
    qos,
    std::bind(&SensorCombinedToImu::sensorCombinedCallback, this, std::placeholders::_1));

  imu_pub_ = this->create_publisher<Imu>(output_topic_, qos);

  RCLCPP_INFO(this->get_logger(), "  subscribe: %s ",
    input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  publish:   %s ",
    output_topic_.c_str());
}

std::array<double, 3> SensorCombinedToImu::convertVectorFrdToRosFlu(
  const std::array<float, 3> & input) const
{
  if (!convert_frd_to_flu_) {
    return {
      static_cast<double>(input[0]),
      static_cast<double>(input[1]),
      static_cast<double>(input[2])
    };
  }

  // PX4 body frame is usually FRD (Forward, Right, Down), while ROS uses FLU
  // (Forward, Left, Up). X is unchanged; Y and Z change sign.
  return {
    static_cast<double>(input[0]),
    -static_cast<double>(input[1]),
    -static_cast<double>(input[2])
  };
}

void SensorCombinedToImu::recordConversionTime(double conversion_time_us)
{
  if (!log_conversion_time_) {
    return;
  }

  timing_sample_count_++;
  timing_total_us_ += conversion_time_us;
  if (conversion_time_us > timing_max_us_) {
    timing_max_us_ = conversion_time_us;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto elapsed_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_timing_log_time_).count();

  if (elapsed_ms < timing_log_period_ms_) {
    return;
  }

  const double timing_avg_us = timing_total_us_ / static_cast<double>(timing_sample_count_);

  RCLCPP_INFO(
    this->get_logger(),
    "IMU conversion+publish time: latest=%.3f us, avg=%.3f us, max=%.3f us, samples=%zu",
    conversion_time_us, timing_avg_us, timing_max_us_, timing_sample_count_);

  last_timing_log_time_ = now;
  timing_sample_count_ = 0;
  timing_total_us_ = 0.0;
  timing_max_us_ = 0.0;
}

void SensorCombinedToImu::sensorCombinedCallback(const SensorCombined::SharedPtr msg)
{
  const auto conversion_start_time = std::chrono::steady_clock::now();

  Imu imu_msg;

  // SensorCombined::timestamp is the PX4 gyro sample timestamp in microseconds
  // since PX4 boot.  Preserve that clock domain instead of using the ROS
  // callback time, so consumers can align this IMU message with PX4 data.
  constexpr uint64_t kMicrosecondsPerSecond = 1000000ULL;
  constexpr uint64_t kNanosecondsPerMicrosecond = 1000ULL;
  imu_msg.header.stamp.sec =
    static_cast<int32_t>(msg->timestamp / kMicrosecondsPerSecond);
  imu_msg.header.stamp.nanosec = static_cast<uint32_t>(
    (msg->timestamp % kMicrosecondsPerSecond) * kNanosecondsPerMicrosecond);
  imu_msg.header.frame_id = frame_id_;

  // SensorCombined 不包含姿态四元数
  // 用 covariance[0] = -1 标记 orientation 无效
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation_covariance[0] = -1.0;

  const auto angular_velocity = convertVectorFrdToRosFlu(msg->gyro_rad);
  imu_msg.angular_velocity.x = angular_velocity[0];
  imu_msg.angular_velocity.y = angular_velocity[1];
  imu_msg.angular_velocity.z = angular_velocity[2];

  const auto linear_acceleration = convertVectorFrdToRosFlu(msg->accelerometer_m_s2);
  imu_msg.linear_acceleration.x = linear_acceleration[0];
  imu_msg.linear_acceleration.y = linear_acceleration[1];
  imu_msg.linear_acceleration.z = linear_acceleration[2];

  imu_pub_->publish(imu_msg);

  const auto conversion_end_time = std::chrono::steady_clock::now();
  const double conversion_time_us =
    std::chrono::duration<double, std::micro>(
    conversion_end_time - conversion_start_time).count();
  recordConversionTime(conversion_time_us);
}

}  // namespace px4_imu_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_imu_bridge::SensorCombinedToImu>());

  rclcpp::shutdown();
  return 0;
}
