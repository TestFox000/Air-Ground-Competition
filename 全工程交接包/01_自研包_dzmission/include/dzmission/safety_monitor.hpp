#ifndef DZMISSION_SAFETY_MONITOR_HPP
#define DZMISSION_SAFETY_MONITOR_HPP

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace dzmission
{

// =============================================================================
// SafetyMonitor —— 近距离避碰（★ 无雷达版）
// -----------------------------------------------------------------------------
// 为什么必须重写：
//   赛题传感器白名单**禁止激光雷达 / 深度相机 / 双目 / UWB / 任何带深度反馈
//   的传感器**。原实现基于 sensor_msgs/LaserScan 做扇区测距，在本赛题里
//   输入永远是空的 —— 它会一直报"急停"，或者（更糟）一直判定"无障碍"。
//
// 没有测距传感器时，"避碰"只能靠**底盘自身的反馈**实现。本实现四路信号：
//
// ① 堵转/卡死检测（★ 主力手段）
//    规则依据：与固定场地相撞持续超 10 秒 -> 裁判切手动。
//    物理原理：如果**发了速度但车不走**，就是撞上东西了。
//       卡死判据：指令速度 >= stall_cmd_v 但实测速度 < stall_actual_v，
//                 持续 stall_hold_ms（默认 1.2s，远小于 10s 以留出脱困时间）
//    代价：这是"先撞上再发现"，不能预防。所以任务层必须另配**结构化规避**：
//       地图驱动避让（FieldGrid 的连通性规划）才是主路径 —— 见 §A1。
//
// ② 姿态异常
//    IMU 俯仰/横滚超阈值 -> 可能压到东西/上台阶。分别判"瞬时冲击"与"持续倾斜"。
//
// ③ 视觉近场占用
//    车端相机的检测框（/offset_center）如果**又大又靠下**，说明障碍物很近。
//    用框高占画面比例做"近场占用"粗判据 —— 精度不高，只作为降速档。
//
// ④ 地图新鲜度
//    没有地雷/障碍清单就贸然行驶会压到水桶。地图过期 -> 降速档而非急停
//    （因为起终点保底路径不依赖地图，全停会白丢 10 分）。
//
// 安全哲学（★ 重要）：
//   "失明"不等于"危险"。原本雷达失效要按失明急停，现在没有雷达，
//   改成：**只有检测到实际异常（堵转/倾斜）才急停**，
//   信息不足只降速。否则整局都在急停。
//
// 与 dztrack 既有 obstacleDetect 的关系：
//   dztrack 的 obstacleDetect 基于 pcl::KdTreeFLANN + LaserScan，在本赛题里
//   同样是死代码。两者都不可用 -> 地图驱动规避是唯一的结构化手段。
// =============================================================================
class SafetyMonitor
{
public:
  // 触发急停/降速的原因，供任务层区分处置策略
  enum class Reason : int
  {
    kNone = 0,
    kStall = 1,        // 发了速度车不走 -> 撞上东西
    kTilt = 2,         // 姿态异常 -> 压到东西/上台阶
    kVisionBlock = 3,  // 视觉近场占用
    kMapStale = 4,     // 地图过期
    kNoOdom = 5,       // 里程计断流（无法判堵转）
  };

  struct Level
  {
    int brake_level{0};         // 0/1/2
    double v_limit{0.0};        // level 1 时的线速度上限（m/s）
    double steer_bias{0.0};     // 建议叠加的舵机角度（度，正=左转）
    Reason reason{Reason::kNone};
    std::string detail;         // 人类可读的判定依据，直接打日志

    // 诊断量
    double cmd_v{0.0};          // 本帧指令速度
    double actual_v{0.0};       // 本帧实测速度
    double stall_s{0.0};        // 连续堵转时长
    double tilt_pitch_deg{0.0};
    double tilt_roll_deg{0.0};
    bool odom_fresh{false};
    bool vision_fresh{false};
    bool map_fresh{false};
  };

  SafetyMonitor(rclcpp::Node *node, const std::string &ns);
  ~SafetyMonitor() = default;

  SafetyMonitor(const SafetyMonitor &) = delete;
  SafetyMonitor &operator=(const SafetyMonitor &) = delete;

  void loadParams();

  // 最近一帧评估结果（线程安全拷贝）
  Level level() const;

  // 里程计是否新鲜。不新鲜时**无法做堵转判定**，
  // 任务层必须按"失去卡死保护"处理（降速，而不是全速跑）。
  bool odomFresh() const;

  // 是否处于急停状态
  bool stopped() const { return stopped_.load(); }

  // 连续堵转时长（秒）—— 任务层用它决定何时启动"脱困倒车"
  double stuckDuration() const;

  // 从堵转中恢复：任务层执行完一次倒车/转向脱困后调用，清零计时
  void reset();

  // 任务层每帧注入自己下发的速度指令，供堵转比对。
  // 不注入时无法判堵转（会保守地视为"无指令"，不触发堵转）。
  void setCommandedVelocity(double v);

  // 任务层/地图模块上报地图新鲜度
  void setMapFreshness(bool fresh);

  // ---- 脱困建议 ----
  // 堵转超过 recover_after_s 后，任务层应执行一次脱困动作。
  // 本函数给出建议的脱困指令（倒车 + 反打方向），由任务层决定是否执行。
  //   out_v : m/s（负 = 倒车）
  //   out_steer_deg : 舵机角度（度）
  bool suggestion(double &out_v, double &out_steer_deg) const;
  bool recoveryDue() const;

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  // ★ 视觉消息是 std_msgs/Int32MultiArray（rknn_pt/det_node 的 /offset_center），
  //   不是 Float32MultiArray。原实现按 Float32 订阅，**类型不匹配 -> 永远收不到**。
  void visionCallbackInt(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  // 兼容旧接口保留（Float32 版本，仅作占位）
  void visionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  // 仲裁器发布的下发速度（/arbiter_speed）。有了它才谈得上"堵转"判据：
  // 必须知道"我让它走多快"，才能判断"它没走"。
  void cmdCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void evalCallback();          // 定时评估（不依赖任何输入话题的到达）
  void publishMarkers(const Level &lv);

  static double normalizeAngle(double a);
  static double quatToRollPitchYawZ(const sensor_msgs::msg::Imu &m,
                                    double &roll, double &pitch);

  rclcpp::Node *node_{nullptr};

  // ---- 参数：话题 ----
  std::string odom_topic_{"odom"};
  std::string imu_topic_{"imu_data"};
  std::string vision_topic_{"offset_center"};
  // 可选的"实际速度"来源：dzactuator 的 odom 或 localization 的输出
  std::string cmd_topic_{"arbiter_speed"};   // 仲裁器发布的下发速度
  // 若给了 localization 的里程计，则用它做堵转比对（更干净）
  std::string actual_odom_topic_{"odometry/filtered"};

  // ---- 参数：堵转判据 ----
  double stall_cmd_v_{0.05};       // m/s，指令速度达到此值才算"应该动"
  // 实测速度阈值。★ 注意 dzactuator 的 odom.twist 有 *50 的标度问题，
  // 所以这里不要求"实测速度接近指令速度"，只要求"明显不是 0"。
  // 若现场标定出正确的标度，用 odom_speed_scale 修正后再收紧本阈值。
  double stall_actual_v_{0.02};    // m/s
  // odom.twist 线速度的标度修正系数（1.0 = 信任原值；0.02 = 若原值放大 50 倍）
  double odom_speed_scale_{1.0};
  double stall_hold_ms_{1200.0};   // ms，持续多久判定堵转
  double stall_grace_ms_{400.0};   // ms，指令变化后的宽限期（起转/换向需要时间）
  double recover_after_s_{2.0};    // s，堵转多久后建议脱困
  double recover_reverse_v_{-0.10};// m/s，脱困倒车速度
  double recover_steer_deg_{25.0}; // 度，脱困打舵角度（正=左）

  // ---- 参数：姿态判据 ----
  double tilt_pitch_warn_deg_{12.0};
  double tilt_pitch_stop_deg_{20.0};
  double tilt_roll_warn_deg_{12.0};
  double tilt_roll_stop_deg_{20.0};
  double tilt_hold_ms_{500.0};     // 持续倾斜多久才算（滤掉瞬时冲击）

  // ---- 参数：视觉近场占用 ----
  // 检测框高度占画面高度的比例超过此值，认为障碍物很近
  double vision_near_box_ratio_{0.55};
  double vision_timeout_ms_{500.0};
  double vision_crawl_speed_{0.10}; // m/s，视觉占用时的限速
  // 画面高度（像素），用于把 bbox_h 换算成占比。usb_cam 默认 480
  double vision_frame_height_{480.0};

  // ---- 参数：里程计新鲜度 ----
  double odom_timeout_ms_{300.0};
  double no_odom_crawl_speed_{0.15}; // m/s，里程计断流时的限速
  double no_odom_stop_ms_{3000.0};   // ms，断流多久后升级为急停

  // ---- 参数：地图新鲜度 ----
  double map_crawl_speed_{0.25};     // m/s，地图过期时的限速
  bool map_stale_blocks_{false};     // 地图过期是否也急停（默认否）

  // ---- 参数：其它 ----
  double eval_hz_{20.0};
  double crawl_speed_{0.12};
  double clear_hold_ms_{400.0};   // 异常消失后持续多久才解除
  double stuck_timeout_s_{10.0};  // ★ 规则硬限：撞固定场地超 10s 裁判切手动
  bool publish_markers_{true};

  // ---- 状态 ----
  mutable std::mutex mtx_;
  Level level_;
  std::atomic<bool> stopped_{false};
  // ★ 多线程注意：evalCallback 由定时器（可能在另一线程）执行，
  //   它会读写下面这些计时量。任务层同时可能调用 reset()/setCommandedVelocity()。
  //   为避免数据竞争，所有跨线程读写的标量都用原子类型；
  //   毫秒时间戳用 double（秒）表示，原子 double 用 AT 不保证无锁，
  //   所以这里统一用 std::atomic<int64_t> 存纳秒。
  std::atomic<int64_t> last_odom_ns_{0};
  std::atomic<int64_t> last_imu_ns_{0};
  std::atomic<int64_t> last_vision_ns_{0};
  std::atomic<int64_t> last_cmd_ns_{0};
  std::atomic<int64_t> stall_start_ns_{0};
  std::atomic<int64_t> tilt_start_ns_{0};
  std::atomic<int64_t> clear_since_ns_{0};
  std::atomic<int64_t> stop_since_ns_{0};
  std::atomic<int64_t> eval_now_ns_{0};

  std::atomic<bool> have_odom_{false};
  std::atomic<bool> have_imu_{false};
  std::atomic<bool> have_vision_{false};
  std::atomic<bool> have_cmd_{false};
  std::atomic<bool> clear_pending_{false};

  std::atomic<double> cmd_v_{0.0};        // 任务层注入的指令速度
  std::atomic<double> actual_v_{0.0};     // 实测速度（已按标度修正）
  std::atomic<double> prev_cmd_v_{0.0};
  std::atomic<double> tilt_pitch_{0.0};
  std::atomic<double> tilt_roll_{0.0};
  std::atomic<bool> map_fresh_{true};
  std::atomic<double> vision_box_ratio_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  // ★ 与 rknn_pt/det_node 的 /offset_center 类型一致（Int32MultiArray）
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_vision_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cmd_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_level_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_reason_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_stall_s_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dzmission

#endif  // DZMISSION_SAFETY_MONITOR_HPP
