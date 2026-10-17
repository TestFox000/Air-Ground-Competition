// dzlink_pc2_bridge —— Livox CustomMsg -> sensor_msgs/PointCloud2 + 单调序号信标
//
// 设计要点（为什么不用 Python）：
//   rclpy 订阅 CustomMsg 会把 19968 个嵌套点对象逐个物化成 Python 对象，
//   实测每帧 ~300ms，只能跑到 3~4Hz，且会通过 DDS 可靠传输流控反压驱动本体。
//   C++ 路径只做一次内存布局转换（float 拷贝），微秒级，稳跑 10Hz。
//
// 话题：
//   订阅  /livox/lidar   livox_ros_driver2/msg/CustomMsg   (可选参数 in_topic)
//   发布  /livox/points  sensor_msgs/msg/PointCloud2       (x,y,z,intensity float32)
//   发布  /dzlink/seq    std_msgs/msg/Int64                50Hz 单调递增（丢包精确对账）
//
// 刻意不碰 /livox/lidar 的类型：无人机 FAST-LIO 栈按 CustomMsg 订阅，契约保持不变。

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/int64.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

using namespace std::chrono_literals;

class DzlinkPc2Bridge : public rclcpp::Node
{
public:
  DzlinkPc2Bridge()
  : rclcpp::Node("dzlink_pc2_bridge")
  {
    const std::string in_topic = this->declare_parameter<std::string>("in_topic", "/livox/lidar");
    const std::string out_topic = this->declare_parameter<std::string>("out_topic", "/livox/points");
    const std::string seq_topic = this->declare_parameter<std::string>("seq_topic", "/dzlink/seq");
    const double seq_rate = this->declare_parameter<double>("seq_rate", 50.0);

    // 与驱动一致的 QoS：RELIABLE + VOLATILE（对 BEST_EFFORT 订阅者同样兼容）
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, qos);
    seq_pub_ = this->create_publisher<std_msgs::msg::Int64>(seq_topic, qos);
    sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      in_topic, qos,
      std::bind(&DzlinkPc2Bridge::onCloud, this, std::placeholders::_1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / (seq_rate > 0.0 ? seq_rate : 50.0)));
    seq_timer_ = this->create_wall_timer(period, std::bind(&DzlinkPc2Bridge::onSeq, this));
    stat_timer_ = this->create_wall_timer(5s, std::bind(&DzlinkPc2Bridge::onStat, this));

    t_start_ = this->now();
    RCLCPP_INFO(this->get_logger(),
      "bridge up: %s(CustomMsg) -> %s(PointCloud2, x/y/z/intensity) | %s @ %.1fHz",
      in_topic.c_str(), out_topic.c_str(), seq_topic.c_str(), seq_rate);
  }

private:
  void onSeq()
  {
    std_msgs::msg::Int64 m;
    m.data = seq_;
    seq_pub_->publish(m);
    ++seq_;
  }

  void onCloud(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    const size_t n = msg->points.size();

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = 1;
    out.width = static_cast<uint32_t>(n);

    out.fields.resize(4);
    auto set_field = [&out](size_t i, const char * name, uint32_t offset) {
      out.fields[i].name = name;
      out.fields[i].offset = offset;
      out.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      out.fields[i].count = 1;
    };
    set_field(0, "x", 0);
    set_field(1, "y", 4);
    set_field(2, "z", 8);
    set_field(3, "intensity", 12);

    out.is_bigendian = false;
    out.point_step = 16;
    out.row_step = static_cast<uint32_t>(16 * n);
    out.is_dense = false;              // MID-360 存在 (0,0,0) 无效点
    out.data.resize(n * 16);

    float * p = reinterpret_cast<float *>(out.data.data());
    for (size_t i = 0; i < n; ++i) {
      const auto & pt = msg->points[i];
      p[4 * i + 0] = pt.x;
      p[4 * i + 1] = pt.y;
      p[4 * i + 2] = pt.z;
      p[4 * i + 3] = static_cast<float>(pt.reflectivity);
    }

    pc_pub_->publish(out);

    ++frames_;
    points_ += n;
    last_points_ = n;
  }

  void onStat()
  {
    const double uptime = (this->now() - t_start_).seconds();
    RCLCPP_INFO(this->get_logger(),
      "STAT frames=%zu points_total=%zu pts_last=%zu seq=%ld subs_points=%zu subs_seq=%zu uptime=%.0fs",
      frames_, points_, last_points_, static_cast<long>(seq_),
      pc_pub_->get_subscription_count(), seq_pub_->get_subscription_count(), uptime);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr seq_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr seq_timer_;
  rclcpp::TimerBase::SharedPtr stat_timer_;

  int64_t seq_{0};
  size_t frames_{0};
  size_t points_{0};
  size_t last_points_{0};
  rclcpp::Time t_start_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DzlinkPc2Bridge>());
  rclcpp::shutdown();
  return 0;
}
