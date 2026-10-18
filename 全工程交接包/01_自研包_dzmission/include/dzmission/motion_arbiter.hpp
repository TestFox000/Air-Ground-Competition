#ifndef DZMISSION_MOTION_ARBITER_HPP
#define DZMISSION_MOTION_ARBITER_HPP

#include <atomic>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace dzmission
{

// =============================================================================
// MotionArbiter —— 速度指令唯一出口
// -----------------------------------------------------------------------------
// 问题：dztrack 的 pursuitAngle、nav2 的 cmd_vel、遥控 logicontrol、避障停车
//       可能同时往下位机送速度，谁最后到谁生效，无法保证安全优先级。
//
// 解法：所有速度源先注入本仲裁器，由本节点统一发布 cmd_vel / pursuitAngle，
//       并保证优先级（高 -> 低）：
//         1. 原地转向（pivot）—— 必须独占，否则会与导航源打架
//         2. 雷达近距急停 / 硬急停
//         3. 遥控接管 logicontrol
//         4. 云台锁车（交战瞄准时禁止底盘乱动，可配置）
//         5. 任务层停止（STOP 指令 / 到达终点 / 链路丢失）
//         6. 导航源（dztrack 优先于 nav2）
//         7. 默认：零速
//       各源带 TTL，超时自动失效，避免"最后一条速度永远生效"。
//
// ★★ 两个话题的 angular.z 语义**完全不同**（这是最容易搞错的地方）★★
//
//   pursuitAngle.angular.z  = **舵机角度，单位「度」**，0 = 直行，
//                             有效范围 [-35, +35]
//                             依据：dzactuator 内部 servo_angle = angular.z + 60，
//                             再 clamp 到 [25, 95]；dztrack 的 sLineControl()
//                             返回的就是角度。
//
//   cmd_vel.angular.z       = **角速度，单位 rad/s**
//                             依据：nav2 的 DWA / velocity_smoother 标准约定；
//                             dzactuator 在 DWA 模式下用 atan(L*w/v) 自行换算。
//
//   因此：
//     · 纯跟踪类源（dztrack / 原地转向）走**伺服通道**，量纲是度
//     · nav2 / 任务层速度源走**速度通道**，量纲是 rad/s
//     · 两者绝不能混存于同一个 double，否则会出现"角度当角速度限幅"
//       或"角速度当角度发给舵机"的事故
//
// 发布约定（与 dzactuator 现有逻辑对齐）：
//   pursuitAngle : linear.x=v, angular.z=舵机角度(度), linear.y>=0.5 表示刹车请求
//   cmd_vel      : linear.x=v, angular.z=角速度(rad/s)
//
// 模式切换说明（走直角版学员代码的实测结论）：
//   **不需要显式发 logicontrol 切模式**。dzactuator 的 modeTimerCallback
//   会在"pursuitAngle 是唯一活跃话题"时自动切到 Pursuit 模式。
//   所以本仲裁器的发布策略是：
//     · 只发 pursuitAngle  -> 自动 Pursuit（用角度）
//     · 只发 cmd_vel       -> 自动 DWA（用角速度）
//     · 刹车时两路都发且 linear.y=1，保证任一模式下都停住
// =============================================================================
class MotionArbiter
{
public:
  enum class Source : int
  {
    kNone = 0,
    kDzTrack = 1,  // dztrack 纯跟踪（pursuitAngle，角度）
    kNav2 = 2,     // nav2 velocity_smoother -> cmd_vel（角速度）
    kAim = 3,      // 云台瞄准时的底盘微调（角速度）
    kMission = 4,  // 任务层直接给定速度（角速度）
    kPivot = 5,    // ★ 原地转向（pursuitAngle，角度）—— 优先级最高
  };

  MotionArbiter(rclcpp::Node *node, const std::string &ns);
  ~MotionArbiter() = default;

  MotionArbiter(const MotionArbiter &) = delete;
  MotionArbiter &operator=(const MotionArbiter &) = delete;

  // ---- 参数加载（在构造函数后由节点的 declare/get_parameter 完成） ----
  void loadParams(const std::string &prefix);

  // ---- 速度源注入（角度/角速度按源自动判别，见上表） ----
  //   kDzTrack / kPivot : w 的单位是**度**（舵机角度）
  //   其余源            : w 的单位是 rad/s（角速度）
  void setSource(Source s, double v, double w);
  void setSource(Source s, const geometry_msgs::msg::Twist &t);
  void clearSource(Source s);
  void clearAllSources();

  // ---- 显式伺服通道（推荐给原地转向/纯跟踪用，避免量纲歧义） ----
  //   steer_deg 为舵机角度（度），自动 clamp 到 [-max_steer_deg, +max_steer_deg]
  void setServoSource(Source s, double v, double steer_deg);
  double maxSteerDeg() const { return max_steer_deg; }
  // 最近一次真正发给 pursuitAngle 的舵机角度（度），供观测/日志
  double activeSteerDeg() const { return out_steer_deg_; }
  // 最近一次发给 cmd_vel 的角速度（rad/s），供观测/日志
  double activeW() const { return out_w_; }

  // ---- 避障输入（独立于导航源） ----
  // brake_level: 0=无限制, 1=减速档, 2=急停
  void setObstacle(int brake_level, double v_limited);

  // ---- 硬急停 ----
  void setEmergencyStop(bool on);
  bool emergencyStop() const { return e_stop_.load(); }

  // ---- 约束开关 ----
  void setGimbalLock(bool on) { gimbal_lock_ = on; }
  void setMissionStop(bool on) { mission_stop_ = on; }
  void setTeleopActive(bool on);

  // ---- 周期性调用（建议 50Hz） ----
  void tick();

  // ---- 观测 ----
  double activeV() const { return out_v_; }
  int brakeLevel() const { return brake_level_; }
  const char *activeSourceName() const;

  // ---- 参数 ----
  double max_v{0.6};
  double max_servo_deg{35.0};    // ★ 舵机角度上限（度），不是角速度
  double max_steer_deg{30.0};    // 控制层允许的舵机角度上限（度）
  double max_w{1.2};             // cmd_vel 通道角速度上限（rad/s）
  double max_acc_v{1.5};         // m/s^2
  double max_acc_servo{120.0};   // 度/s，舵机角度变化率上限（保护舵机）
  double max_acc_w{4.0};         // rad/s^2，仅速度通道
  double source_ttl{0.4};
  bool publish_cmd_vel{true};
  bool publish_pursuit{true};
  // 刹车时是否两路同发 linear.y=1（dzactuator 双模式停车所必需）
  bool brake_publish_both{true};

private:
  struct Slot
  {
    double v{0.0};
    double w{0.0};        // 度（伺服通道）或 rad/s（速度通道）
    bool valid{false};
    rclcpp::Time stamp;
  };

  Slot &slot(Source s);
  static const char *sourceName(Source s);
  // 该源是否使用伺服（角度）通道
  static bool isServoSource(Source s);

  rclcpp::Node *node_{nullptr};

  Slot dztrack_;
  Slot nav2_;
  Slot aim_;
  Slot mission_;
  Slot pivot_;

  int brake_level_{0};       // 0/1/2
  double brake_v_limit_{0.0};

  std::atomic<bool> e_stop_{false};
  bool gimbal_lock_{false};
  bool mission_stop_{false};
  bool teleop_active_{false};
  rclcpp::Time teleop_stamp_;

  double out_v_{0.0};
  double out_steer_deg_{0.0};   // 伺服通道输出（度）
  double out_w_{0.0};           // 速度通道输出（rad/s）
  Source out_source_{Source::kNone};
  bool last_brake_{false};
  bool last_servo_channel_{false};
  rclcpp::Time last_tick_;
  bool have_last_tick_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_pursuit_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_arbiter_v_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_arbiter_steer_;
};

}  // namespace dzmission

#endif  // DZMISSION_MOTION_ARBITER_HPP
