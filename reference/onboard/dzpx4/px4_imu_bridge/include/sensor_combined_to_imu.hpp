#ifndef PX4_IMU_BRIDGE__SENSOR_COMBINED_TO_IMU_HPP_
#define PX4_IMU_BRIDGE__SENSOR_COMBINED_TO_IMU_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <string>

#include "px4_msgs/msg/sensor_combined.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace px4_imu_bridge
{

class SensorCombinedToImu : public rclcpp::Node
{
public:
  SensorCombinedToImu();

private:
  using SensorCombined = px4_msgs::msg::SensorCombined;
  using Imu = sensor_msgs::msg::Imu;

  void sensorCombinedCallback(const SensorCombined::SharedPtr msg);
  void recordConversionTime(double conversion_time_us);
  std::array<double, 3> convertVectorFrdToRosFlu(const std::array<float, 3> & input) const;

  rclcpp::Subscription<SensorCombined>::SharedPtr sensor_combined_sub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  bool convert_frd_to_flu_{true};
  bool log_conversion_time_{true};
  int timing_log_period_ms_{1000};

  std::chrono::steady_clock::time_point last_timing_log_time_;
  std::size_t timing_sample_count_{0};
  double timing_total_us_{0.0};
  double timing_max_us_{0.0};
};

}  // namespace px4_imu_bridge

#endif  // PX4_IMU_BRIDGE__SENSOR_COMBINED_TO_IMU_HPP_
