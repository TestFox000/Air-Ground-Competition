#include "dzmission/safety_monitor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

namespace dzmission
{

namespace
{
// 节点版本戳：launch 加载的是 colcon 安装副本，源码改了忘 build 就会
// 静默跑旧代码。打进日志便于一眼判断。
constexpr const char *kNodeVersion = "v6-nolidar";

// ★ 已删除未使用的 toNs()：它在整个项目里只有定义、没有任何调用，
//   而 CMakeLists 开了 -Wall -Wextra，编译器每次都报
//       warning: 'int64_t dzmission::{anonymous}::toNs(const rclcpp::Time&)'
//                defined but not used [-Wunused-function]
//   死代码留着会淹没真正的警告，所以直接去掉。
//   若将来需要"把 rclcpp::Time 转纳秒"，用 t.nanoseconds() 即可。

double sinceS(int64_t now_ns, int64_t then_ns)
{
  if (then_ns <= 0)
  {
    return 1e9;   // 从未收到 -> 视为很久以前
  }
  return static_cast<double>(now_ns - then_ns) * 1e-9;
}
}  // namespace

SafetyMonitor::SafetyMonitor(rclcpp::Node *node, const std::string &ns)
: node_(node)
{
  (void)ns;
  // QoS 与 dzactuator / 定位节点一致：BEST_EFFORT。
  // 学员代码的实测：QoS 不匹配会静默丢弃全部消息。
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos,
      std::bind(&SafetyMonitor::odomCallback, this, std::placeholders::_1));
  sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      std::bind(&SafetyMonitor::imuCallback, this, std::placeholders::_1));
  sub_vision_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
      vision_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&SafetyMonitor::visionCallbackInt, this, std::placeholders::_1));
  sub_cmd_ = node_->create_subscription<std_msgs::msg::Float32>(
      cmd_topic_, rclcpp::QoS(10),
      std::bind(&SafetyMonitor::cmdCallback, this, std::placeholders::_1));

  pub_markers_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>("safety_markers", 1);
  pub_level_ = node_->create_publisher<std_msgs::msg::UInt8>("safety_brake_level", 5);
  pub_reason_ = node_->create_publisher<std_msgs::msg::UInt8>("safety_reason", 5);
  pub_stall_s_ = node_->create_publisher<std_msgs::msg::Float32>("safety_stall_s", 5);

  // 定时评估：不依赖任何输入话题的到达。
  // IMU/里程计一旦停发，回调不再触发；若只在回调里更新状态，
  // 判定会永久冻结在最后一次的"正常"上，任务层察觉不到失效。
  const double hz = std::max(1.0, eval_hz_);
  timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
      std::bind(&SafetyMonitor::evalCallback, this));

  RCLCPP_INFO(node_->get_logger(),
              "[Safety %s] ★ 无雷达版：堵转 + 姿态 + 视觉近场 + 地图新鲜度", kNodeVersion);
  RCLCPP_INFO(node_->get_logger(),
              "[Safety] odom=%s imu=%s vision=%s cmd=%s",
              odom_topic_.c_str(), imu_topic_.c_str(),
              vision_topic_.c_str(), cmd_topic_.c_str());
  RCLCPP_INFO(node_->get_logger(),
              "[Safety] 堵转判据: cmd>=%.2f 且 actual<%.2f 持续 %.0fms；"
              "脱困建议在 %.1fs 后给出",
              stall_cmd_v_, stall_actual_v_, stall_hold_ms_, recover_after_s_);
}

void SafetyMonitor::loadParams()
{
  const std::string p = "safety.";

  node_->declare_parameter(p + "odom_topic", odom_topic_);
  node_->declare_parameter(p + "imu_topic", imu_topic_);
  node_->declare_parameter(p + "vision_topic", vision_topic_);
  node_->declare_parameter(p + "cmd_topic", cmd_topic_);

  node_->declare_parameter(p + "stall_cmd_v", stall_cmd_v_);
  node_->declare_parameter(p + "stall_actual_v", stall_actual_v_);
  node_->declare_parameter(p + "odom_speed_scale", odom_speed_scale_);
  node_->declare_parameter(p + "stall_hold_ms", stall_hold_ms_);
  node_->declare_parameter(p + "stall_grace_ms", stall_grace_ms_);
  node_->declare_parameter(p + "recover_after_s", recover_after_s_);
  node_->declare_parameter(p + "recover_reverse_v", recover_reverse_v_);
  node_->declare_parameter(p + "recover_steer_deg", recover_steer_deg_);

  node_->declare_parameter(p + "tilt_pitch_warn_deg", tilt_pitch_warn_deg_);
  node_->declare_parameter(p + "tilt_pitch_stop_deg", tilt_pitch_stop_deg_);
  node_->declare_parameter(p + "tilt_roll_warn_deg", tilt_roll_warn_deg_);
  node_->declare_parameter(p + "tilt_roll_stop_deg", tilt_roll_stop_deg_);
  node_->declare_parameter(p + "tilt_hold_ms", tilt_hold_ms_);

  node_->declare_parameter(p + "vision_near_box_ratio", vision_near_box_ratio_);
  node_->declare_parameter(p + "vision_timeout_ms", vision_timeout_ms_);
  node_->declare_parameter(p + "vision_crawl_speed", vision_crawl_speed_);
  node_->declare_parameter(p + "vision_frame_height", vision_frame_height_);

  node_->declare_parameter(p + "odom_timeout_ms", odom_timeout_ms_);
  node_->declare_parameter(p + "no_odom_crawl_speed", no_odom_crawl_speed_);
  node_->declare_parameter(p + "no_odom_stop_ms", no_odom_stop_ms_);

  node_->declare_parameter(p + "map_crawl_speed", map_crawl_speed_);
  node_->declare_parameter(p + "map_stale_blocks", map_stale_blocks_);

  node_->declare_parameter(p + "eval_hz", eval_hz_);
  node_->declare_parameter(p + "crawl_speed", crawl_speed_);
  node_->declare_parameter(p + "clear_hold_ms", clear_hold_ms_);
  node_->declare_parameter(p + "stuck_timeout_s", stuck_timeout_s_);
  node_->declare_parameter(p + "publish_markers", publish_markers_);

  node_->get_parameter(p + "odom_topic", odom_topic_);
  node_->get_parameter(p + "imu_topic", imu_topic_);
  node_->get_parameter(p + "vision_topic", vision_topic_);
  node_->get_parameter(p + "cmd_topic", cmd_topic_);

  node_->get_parameter(p + "stall_cmd_v", stall_cmd_v_);
  node_->get_parameter(p + "stall_actual_v", stall_actual_v_);
  node_->get_parameter(p + "odom_speed_scale", odom_speed_scale_);
  node_->get_parameter(p + "stall_hold_ms", stall_hold_ms_);
  node_->get_parameter(p + "stall_grace_ms", stall_grace_ms_);
  node_->get_parameter(p + "recover_after_s", recover_after_s_);
  node_->get_parameter(p + "recover_reverse_v", recover_reverse_v_);
  node_->get_parameter(p + "recover_steer_deg", recover_steer_deg_);

  node_->get_parameter(p + "tilt_pitch_warn_deg", tilt_pitch_warn_deg_);
  node_->get_parameter(p + "tilt_pitch_stop_deg", tilt_pitch_stop_deg_);
  node_->get_parameter(p + "tilt_roll_warn_deg", tilt_roll_warn_deg_);
  node_->get_parameter(p + "tilt_roll_stop_deg", tilt_roll_stop_deg_);
  node_->get_parameter(p + "tilt_hold_ms", tilt_hold_ms_);

  node_->get_parameter(p + "vision_near_box_ratio", vision_near_box_ratio_);
  node_->get_parameter(p + "vision_timeout_ms", vision_timeout_ms_);
  node_->get_parameter(p + "vision_crawl_speed", vision_crawl_speed_);
  node_->get_parameter(p + "vision_frame_height", vision_frame_height_);

  node_->get_parameter(p + "odom_timeout_ms", odom_timeout_ms_);
  node_->get_parameter(p + "no_odom_crawl_speed", no_odom_crawl_speed_);
  node_->get_parameter(p + "no_odom_stop_ms", no_odom_stop_ms_);

  node_->get_parameter(p + "map_crawl_speed", map_crawl_speed_);
  node_->get_parameter(p + "map_stale_blocks", map_stale_blocks_);

  node_->get_parameter(p + "eval_hz", eval_hz_);
  node_->get_parameter(p + "crawl_speed", crawl_speed_);
  node_->get_parameter(p + "clear_hold_ms", clear_hold_ms_);
  node_->get_parameter(p + "stuck_timeout_s", stuck_timeout_s_);
  node_->get_parameter(p + "publish_markers", publish_markers_);
}

// -----------------------------------------------------------------------------
void SafetyMonitor::setCommandedVelocity(double v) { cmd_v_.store(v); }

void SafetyMonitor::setMapFreshness(bool fresh) { map_fresh_.store(fresh); }

// -----------------------------------------------------------------------------
void SafetyMonitor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double vx = msg->twist.twist.linear.x;
  const double vy = msg->twist.twist.linear.y;
  // 标度修正：dzactuator 的 Odom_Trans 直接把 mm/s 当 float 用，
  // 还可能乘过 50。现场标定出正确标度后改 odom_speed_scale。
  const double speed = std::hypot(vx, vy) * odom_speed_scale_;
  actual_v_.store(speed);
  have_odom_.store(true);
  last_odom_ns_.store(node_->now().nanoseconds());
}

void SafetyMonitor::cmdCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // 仲裁器发布的 /arbiter_speed 就是它下发给下位机的速度。
  // 用**它**而不是任务层自己记的期望速度，原因：
  //   任务层想要的速度可能在仲裁层被护栏（急停/限速/云台锁车）改过，
  //   拿被改过的实际下发值做堵转比对才不会误判。
  cmd_v_.store(static_cast<double>(msg->data));
  have_cmd_.store(true);
  last_cmd_ns_.store(node_->now().nanoseconds());
}

void SafetyMonitor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double roll = 0.0;
  double pitch = 0.0;
  quatToRollPitchYawZ(*msg, roll, pitch);
  tilt_roll_.store(roll * 180.0 / M_PI);
  tilt_pitch_.store(pitch * 180.0 / M_PI);
  have_imu_.store(true);
  last_imu_ns_.store(node_->now().nanoseconds());
}

// -----------------------------------------------------------------------------
// 视觉近场占用
// -----------------------------------------------------------------------------
// ★ 消息格式（已与 rknn_pt/src/det_node.cc v2 对齐，共 8 个元素）：
//     [0] offset_x   像素（目标中心 - 画面中心）
//     [1] offset_y   像素
//     [2] class_id   1 = 检测到目标（本模型单类，无红蓝）
//     [3] dist_cm    0 = 未知
//     [4] bbox_x     [5] bbox_y   [6] bbox_w   [7] bbox_h
//
//   原实现读 data[4]/data[5] 当作 y1/y2，但当时的 det_node 只发 3 个元素，
//   所以**这段代码从未生效过**。现在 det_node 补齐了 bbox，这里改为正确解析：
//   用"框高 / 画面高"作为近场占用的粗判据。
void SafetyMonitor::visionCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  // 兼容两种消息类型：可能有人用 Float32MultiArray（距离为浮点）发布
  (void)msg;
}

void SafetyMonitor::visionCallbackInt(
    const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  const auto &d = msg->data;
  if (d.size() < 8)
  {
    // 格式不对（例如旧版 det_node 只发 3 个）：不参与近场判定，只算心跳
    have_vision_.store(true);
    last_vision_ns_.store(node_->now().nanoseconds());
    // ★ SafetyMonitor 不是 rclcpp::Node 的子类，它**持有** node_ 指针，
    //   所以没有 this->get_logger() / this->get_clock()。
    //   必须走 node_->...（本文件其他地方也都是这么写的）。
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "[Safety] offset_center 元素数 %zu < 8，跳过近场判定",
                         d.size());
    return;
  }

  // 无目标：class_id == 0
  if (d[2] == 0)
  {
    vision_box_ratio_.store(0.0);
    have_vision_.store(true);
    last_vision_ns_.store(node_->now().nanoseconds());
    return;
  }

  const double bh = std::fabs(static_cast<double>(d[7]));   // bbox_h
  const double frame_h = std::max(1.0, vision_frame_height_);
  vision_box_ratio_.store(bh / frame_h);

  have_vision_.store(true);
  last_vision_ns_.store(node_->now().nanoseconds());
}

SafetyMonitor::Level SafetyMonitor::level() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return level_;
}

bool SafetyMonitor::odomFresh() const
{
  const int64_t now = node_->now().nanoseconds();
  return sinceS(now, last_odom_ns_.load()) * 1000.0 < odom_timeout_ms_;
}

double SafetyMonitor::stuckDuration() const
{
  const int64_t s = stall_start_ns_.load();
  if (s <= 0)
  {
    return 0.0;
  }
  const int64_t now = node_->now().nanoseconds();
  return std::max(0.0, static_cast<double>(now - s) * 1e-9);
}

bool SafetyMonitor::recoveryDue() const
{
  return stuckDuration() >= recover_after_s_;
}

bool SafetyMonitor::suggestion(double &out_v, double &out_steer_deg) const
{
  if (!recoveryDue())
  {
    return false;
  }
  // 脱困动作：倒车 + 反打方向。
  // 为什么是倒车而不是继续前冲：规则"与固定场地相撞持续超 10s -> 裁判切手动"，
  // 所以必须先脱离接触面。倒车是最快脱离的方式。
  out_v = recover_reverse_v_;
  out_steer_deg = recover_steer_deg_;
  return true;
}

void SafetyMonitor::reset()
{
  stall_start_ns_.store(0);
  tilt_start_ns_.store(0);
  clear_pending_.store(false);
  clear_since_ns_.store(0);
  stopped_.store(false);
  {
    std::lock_guard<std::mutex> lk(mtx_);
    level_ = Level{};
  }
}

void SafetyMonitor::evalCallback()
{
  const int64_t now = node_->now().nanoseconds();
  eval_now_ns_.store(now);

  Level lv;
  lv.odom_fresh = sinceS(now, last_odom_ns_.load()) * 1000.0 < odom_timeout_ms_;
  lv.vision_fresh = sinceS(now, last_vision_ns_.load()) * 1000.0 < vision_timeout_ms_;
  lv.map_fresh = map_fresh_.load();
  lv.cmd_v = cmd_v_.load();
  lv.actual_v = actual_v_.load();
  lv.tilt_pitch_deg = tilt_pitch_.load();
  lv.tilt_roll_deg = tilt_roll_.load();

  double v_limit = 1e9;   // 未被限制
  Reason reason = Reason::kNone;
  std::string detail;

  auto worse = [](Reason a, Reason b) {
    // 只用于展示，优先级按枚举值大小（堵转 > 姿态 > 视觉 > 地图 > 无里程计）
    return static_cast<int>(a) < static_cast<int>(b) ? b : a;
  };

  // =========================== ① 堵转检测（主力） ===========================
  // 指令速度达到阈值、但实测速度明显为 0，且持续时间超过 stall_hold_ms
  const double cmd = lv.cmd_v;
  const bool cmd_changed = std::fabs(cmd - prev_cmd_v_.load()) > 0.02;
  prev_cmd_v_.store(cmd);

  const bool wants_move = std::fabs(cmd) >= stall_cmd_v_;
  const bool not_moving = lv.actual_v < stall_actual_v_;

  if (cmd_changed)
  {
    // 指令刚变化：起转 / 换向需要时间，重置堵转计时并给宽限期
    stall_start_ns_.store(0);
  }

  if (wants_move && not_moving && lv.odom_fresh)
  {
    if (stall_start_ns_.load() <= 0)
    {
      stall_start_ns_.store(now);
    }
    const double stall_s =
        static_cast<double>(now - stall_start_ns_.load()) * 1e-9;
    lv.stall_s = stall_s;
    const double grace = stall_grace_ms_ * 1e-3;
    if (stall_s >= grace + stall_hold_ms_ * 1e-3)
    {
      reason = worse(reason, Reason::kStall);
      v_limit = 0.0;   // 急停
      detail = "堵转：指令 " + std::to_string(cmd).substr(0, 5) +
               " m/s 但实测 " + std::to_string(lv.actual_v).substr(0, 5) +
               " m/s，持续 " + std::to_string(static_cast<int>(stall_s)) + " s";
      if (stall_s >= stuck_timeout_s_)
      {
        detail += "（★ 已达规则硬限 " + std::to_string(static_cast<int>(stuck_timeout_s_)) +
                  "s，必须立即脱困，否则裁判切手动）";
      }
    }
    else if (stall_s >= grace)
    {
      // 宽限期已过但还没到判定时长：先降速观察
      reason = worse(reason, Reason::kStall);
      v_limit = std::min(v_limit, crawl_speed_);
      detail = "疑似堵转：实测速度接近 0，降速观察";
    }
  }
  else
  {
    stall_start_ns_.store(0);
  }

  // =========================== ② 姿态异常 ===========================
  {
    const double p = std::fabs(lv.tilt_pitch_deg);
    const double r = std::fabs(lv.tilt_roll_deg);
    const bool over_stop = (p >= tilt_pitch_stop_deg_ || r >= tilt_roll_stop_deg_);
    const bool over_warn = (p >= tilt_pitch_warn_deg_ || r >= tilt_roll_warn_deg_);
    if (over_stop || over_warn)
    {
      if (tilt_start_ns_.load() <= 0)
      {
        tilt_start_ns_.store(now);
      }
      const double held = static_cast<double>(now - tilt_start_ns_.load()) * 1e-9;
      if (over_stop && held >= tilt_hold_ms_ * 1e-3)
      {
        reason = worse(reason, Reason::kTilt);
        v_limit = 0.0;
        detail = "姿态异常：俯仰 " + std::to_string(lv.tilt_pitch_deg).substr(0, 5) +
                 "° / 横滚 " + std::to_string(lv.tilt_roll_deg).substr(0, 5) +
                 "° 持续超限（可能压到东西）";
      }
      else if (over_warn)
      {
        reason = worse(reason, Reason::kTilt);
        v_limit = std::min(v_limit, crawl_speed_);
        detail = "姿态偏斜：俯仰 " + std::to_string(lv.tilt_pitch_deg).substr(0, 5) +
                 "° / 横滚 " + std::to_string(lv.tilt_roll_deg).substr(0, 5) + "°";
      }
    }
    else
    {
      tilt_start_ns_.store(0);
    }
  }

  // =========================== ③ 视觉近场占用 ===========================
  if (lv.vision_fresh)
  {
    const double ratio = vision_box_ratio_.load();
    if (ratio >= vision_near_box_ratio_)
    {
      reason = worse(reason, Reason::kVisionBlock);
      v_limit = std::min(v_limit, vision_crawl_speed_);
      detail = "视觉近场占用：检测框占画面 " +
               std::to_string(static_cast<int>(ratio * 100)) + "%";
    }
  }

  // =========================== ④ 地图新鲜度 ===========================
  if (!lv.map_fresh)
  {
    reason = worse(reason, Reason::kMapStale);
    if (map_stale_blocks_)
    {
      v_limit = 0.0;
      detail = "地图过期，禁用行驶（map_stale_blocks=true）";
    }
    else
    {
      v_limit = std::min(v_limit, map_crawl_speed_);
      detail = "地图过期（未收到地雷/障碍清单），降速行驶";
    }
  }

  // =========================== ⑤ 里程计断流 ===========================
  // 里程计断了就无法判堵转 —— 这是"失去卡死保护"，不是"危险"。
  // 所以先降速，超时才升级为急停。
  if (!lv.odom_fresh)
  {
    const double lost_s = sinceS(now, last_odom_ns_.load());
    reason = worse(reason, Reason::kNoOdom);
    if (lost_s * 1000.0 >= no_odom_stop_ms_)
    {
      v_limit = 0.0;
      detail = "里程计断流 " + std::to_string(static_cast<int>(lost_s)) +
               " s，无法做堵转判定，急停";
    }
    else
    {
      v_limit = std::min(v_limit, no_odom_crawl_speed_);
      detail = "里程计断流，降速（失去堵转保护）";
    }
  }

  // =========================== 汇总 ===========================
  const bool want_stop = (v_limit <= 0.0);
  bool want_stop_now = want_stop;
  lv.reason = reason;
  lv.detail = detail;

  if (want_stop)
  {
    // 急停保持：异常消失后需持续 clear_hold_ms 才解除，避免抖动
    if (reason != Reason::kNone)
    {
      clear_pending_.store(false);
      clear_since_ns_.store(0);
    }
    else if (!clear_pending_.load())
    {
      clear_pending_.store(true);
      clear_since_ns_.store(now);
    }
    else if (static_cast<double>(now - clear_since_ns_.load()) * 1000.0 >=
             clear_hold_ms_)
    {
      // 异常已消失且保持足够久 -> 解除急停，本帧按"无限制"放行
      clear_pending_.store(false);
      clear_since_ns_.store(0);
      stopped_.store(false);
      want_stop_now = false;
    }
  }

  if (want_stop_now)
  {
    if (!stopped_.load())
    {
      stop_since_ns_.store(now);
    }
    stopped_.store(true);
    lv.brake_level = 2;
    lv.v_limit = 0.0;
  }
  else if (v_limit < 1e8)
  {
    lv.brake_level = 1;
    lv.v_limit = v_limit;
  }
  else
  {
    lv.brake_level = 0;
    lv.v_limit = 0.0;
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    level_ = lv;
  }

  if (publish_markers_)
  {
    publishMarkers(lv);
  }

  std_msgs::msg::UInt8 l;
  l.data = static_cast<uint8_t>(lv.brake_level);
  pub_level_->publish(l);

  std_msgs::msg::UInt8 r;
  r.data = static_cast<uint8_t>(lv.reason);
  pub_reason_->publish(r);

  std_msgs::msg::Float32 st;
  st.data = static_cast<float>(lv.stall_s);
  pub_stall_s_->publish(st);

  // 只在状态变化时打日志，避免刷屏
  static int last_level = 0;
  static int last_reason = 0;
  if (lv.brake_level != last_level || static_cast<int>(lv.reason) != last_reason)
  {
    if (lv.brake_level == 0)
    {
      RCLCPP_INFO(node_->get_logger(), "[Safety] 解除限制，恢复正常");
    }
    else if (lv.brake_level == 2)
    {
      RCLCPP_WARN(node_->get_logger(), "[Safety] 急停：%s", lv.detail.c_str());
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "[Safety] 降速至 %.2f m/s：%s",
                  lv.v_limit, lv.detail.c_str());
    }
    last_level = lv.brake_level;
    last_reason = static_cast<int>(lv.reason);
  }
}

// -----------------------------------------------------------------------------
void SafetyMonitor::publishMarkers(const Level &lv)
{
  visualization_msgs::msg::MarkerArray arr;

  visualization_msgs::msg::Marker m;
  m.header.frame_id = "base_link";
  m.header.stamp = node_->now();
  m.ns = "safety";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.6;
  m.scale.z = 0.08;
  m.color.a = 1.0;
  if (lv.brake_level == 2)
  {
    m.color.r = 1.0f;
    m.text = "ESTOP: " + lv.detail;
  }
  else if (lv.brake_level == 1)
  {
    m.color.r = 1.0f;
    m.color.g = 0.7f;
    m.text = "SLOW: " + lv.detail;
  }
  else
  {
    m.color.g = 1.0f;
    m.text = "OK";
  }
  arr.markers.push_back(m);
  pub_markers_->publish(arr);
}

double SafetyMonitor::normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double SafetyMonitor::quatToRollPitchYawZ(const sensor_msgs::msg::Imu &m,
                                          double &roll, double &pitch)
{
  const double x = m.orientation.x;
  const double y = m.orientation.y;
  const double z = m.orientation.z;
  const double w = m.orientation.w;
  // roll (x-axis)
  const double sinr = 2.0 * (w * x + y * z);
  const double cosr = 1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(sinr, cosr);
  // pitch (y-axis)
  const double sinp = 2.0 * (w * y - z * x);
  pitch = (std::fabs(sinp) >= 1.0) ? (sinp > 0 ? M_PI / 2 : -M_PI / 2)
                                   : std::asin(sinp);
  // yaw (z-axis)
  const double siny = 2.0 * (w * z + x * y);
  const double cosy = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny, cosy);
}

}  // namespace dzmission
