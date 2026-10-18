#include "dzmission/motion_arbiter.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace dzmission
{

MotionArbiter::MotionArbiter(rclcpp::Node *node, const std::string &ns) : node_(node)
{
  (void)ns;
  // QoS 必须与 dzactuator 的订阅端一致：BEST_EFFORT + depth 1。
  // 学员代码的实测记录：QoS 不匹配会**静默丢弃全部消息**，看起来像"车不动"，
  // 而且 publisher count 不为 0（所以光看 publisher 数量判断不出问题）。
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
  pub_pursuit_ = node_->create_publisher<geometry_msgs::msg::Twist>("pursuitAngle", qos);
  pub_arbiter_v_ =
      node_->create_publisher<std_msgs::msg::Float32>("arbiter_speed", 10);
  pub_arbiter_steer_ =
      node_->create_publisher<std_msgs::msg::Float32>("arbiter_steer_deg", 10);

  last_tick_ = node_->now();
  teleop_stamp_ = node_->now();
}

void MotionArbiter::loadParams(const std::string &prefix)
{
  const std::string p = prefix.empty() ? "" : prefix + ".";

  node_->declare_parameter(p + "arbiter.max_v", max_v);
  node_->declare_parameter(p + "arbiter.max_servo_deg", max_servo_deg);
  node_->declare_parameter(p + "arbiter.max_steer_deg", max_steer_deg);
  node_->declare_parameter(p + "arbiter.max_w", max_w);
  node_->declare_parameter(p + "arbiter.max_acc_v", max_acc_v);
  node_->declare_parameter(p + "arbiter.max_acc_servo", max_acc_servo);
  node_->declare_parameter(p + "arbiter.max_acc_w", max_acc_w);
  node_->declare_parameter(p + "arbiter.source_ttl", source_ttl);
  node_->declare_parameter(p + "arbiter.publish_cmd_vel", publish_cmd_vel);
  node_->declare_parameter(p + "arbiter.publish_pursuit", publish_pursuit);
  node_->declare_parameter(p + "arbiter.brake_publish_both", brake_publish_both);

  node_->get_parameter(p + "arbiter.max_v", max_v);
  node_->get_parameter(p + "arbiter.max_servo_deg", max_servo_deg);
  node_->get_parameter(p + "arbiter.max_steer_deg", max_steer_deg);
  node_->get_parameter(p + "arbiter.max_w", max_w);
  node_->get_parameter(p + "arbiter.max_acc_v", max_acc_v);
  node_->get_parameter(p + "arbiter.max_acc_servo", max_acc_servo);
  node_->get_parameter(p + "arbiter.max_acc_w", max_acc_w);
  node_->get_parameter(p + "arbiter.source_ttl", source_ttl);
  node_->get_parameter(p + "arbiter.publish_cmd_vel", publish_cmd_vel);
  node_->get_parameter(p + "arbiter.publish_pursuit", publish_pursuit);
  node_->get_parameter(p + "arbiter.brake_publish_both", brake_publish_both);

  // 控制层允许的舵机角度不能超过硬件允许范围，否则 dzactuator 会 clamp，
  // 造成"指令与实际不符"的隐性偏差
  if (max_steer_deg > max_servo_deg)
  {
    RCLCPP_WARN(node_->get_logger(),
                "[Arbiter] max_steer_deg=%.1f > max_servo_deg=%.1f，已收紧到硬件上限",
                max_steer_deg, max_servo_deg);
    max_steer_deg = max_servo_deg;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[Arbiter] max_v=%.2f max_steer=%.1fdeg (hw %.1f) max_w=%.2frad/s "
              "ttl=%.2f publish(cmd_vel=%d pursuit=%d brake_both=%d)",
              max_v, max_steer_deg, max_servo_deg, max_w, source_ttl,
              static_cast<int>(publish_cmd_vel), static_cast<int>(publish_pursuit),
              static_cast<int>(brake_publish_both));
}

bool MotionArbiter::isServoSource(Source s)
{
  // 只有这两个源走 pursuitAngle（角度）通道
  return s == Source::kDzTrack || s == Source::kPivot;
}

MotionArbiter::Slot &MotionArbiter::slot(Source s)
{
  switch (s)
  {
    case Source::kDzTrack: return dztrack_;
    case Source::kNav2: return nav2_;
    case Source::kAim: return aim_;
    case Source::kMission: return mission_;
    case Source::kPivot: return pivot_;
    default: return mission_;
  }
}

const char *MotionArbiter::sourceName(Source s)
{
  switch (s)
  {
    case Source::kNone: return "NONE";
    case Source::kDzTrack: return "DZTRACK";
    case Source::kNav2: return "NAV2";
    case Source::kAim: return "AIM";
    case Source::kMission: return "MISSION";
    case Source::kPivot: return "PIVOT";
    default: return "?";
  }
}

const char *MotionArbiter::activeSourceName() const { return sourceName(out_source_); }

void MotionArbiter::setSource(Source s, double v, double w)
{
  Slot &sl = slot(s);
  sl.v = v;
  // 伺服通道的量纲是度，按控制层允许的舵机角度限幅；
  // 速度通道的量纲是 rad/s，按角速度限幅。两者不能互相 clamp。
  if (isServoSource(s))
  {
    sl.w = std::clamp(w, -max_steer_deg, max_steer_deg);
  }
  else
  {
    sl.w = std::clamp(w, -max_w, max_w);
  }
  sl.v = std::clamp(v, -max_v, max_v);
  sl.valid = true;
  sl.stamp = node_->now();
}

void MotionArbiter::setSource(Source s, const geometry_msgs::msg::Twist &t)
{
  setSource(s, t.linear.x, t.angular.z);
}

void MotionArbiter::setServoSource(Source s, double v, double steer_deg)
{
  // 强制按伺服通道处理（不依赖 isServoSource，便于将来复用）
  Slot &sl = slot(s);
  sl.v = std::clamp(v, -max_v, max_v);
  sl.w = std::clamp(steer_deg, -max_steer_deg, max_steer_deg);
  sl.valid = true;
  sl.stamp = node_->now();
}

void MotionArbiter::clearSource(Source s) { slot(s).valid = false; }

void MotionArbiter::clearAllSources()
{
  dztrack_.valid = false;
  nav2_.valid = false;
  aim_.valid = false;
  mission_.valid = false;
  pivot_.valid = false;
}

void MotionArbiter::setObstacle(int brake_level, double v_limited)
{
  brake_level_ = std::clamp(brake_level, 0, 2);
  brake_v_limit_ = v_limited;
}

void MotionArbiter::setEmergencyStop(bool on) { e_stop_.store(on); }

void MotionArbiter::setTeleopActive(bool on)
{
  teleop_active_ = on;
  teleop_stamp_ = node_->now();
}

namespace
{
double rateLimit(double target, double current, double max_delta)
{
  const double d = target - current;
  if (d > max_delta) return current + max_delta;
  if (d < -max_delta) return current - max_delta;
  return target;
}
}  // namespace

void MotionArbiter::tick()
{
  const rclcpp::Time now = node_->now();
  double dt = (now - last_tick_).seconds();
  if (!have_last_tick_)
  {
    dt = 0.02;
    have_last_tick_ = true;
  }
  if (dt <= 0.0 || dt > 1.0)
  {
    dt = 0.02;
  }
  last_tick_ = now;

  // ---- 1. 源 TTL 失效 ----
  Slot *all[5] = {&dztrack_, &nav2_, &aim_, &mission_, &pivot_};
  for (Slot *s : all)
  {
    if (s->valid && (now - s->stamp).seconds() > source_ttl)
    {
      s->valid = false;
    }
  }

  // ---- 2. 遥控接管 TTL ----
  const bool teleop_ok = teleop_active_ && (now - teleop_stamp_).seconds() < source_ttl;

  // ---- 3. 选源 ----
  double v = 0.0;
  double w = 0.0;                 // 按被选中源自己的量纲
  Source chosen = Source::kNone;
  bool brake = false;
  bool servo_channel = false;

  // ① 原地转向独占最高优先级：转向过程中底盘必须只服从转向控制器，
  //    否则 dztrack/nav2 会同时发角度，造成"边转边被拽回来"的抖动。
  if (pivot_.valid && !e_stop_.load() && brake_level_ < 2)
  {
    v = pivot_.v;
    w = pivot_.w;
    chosen = Source::kPivot;
    servo_channel = true;
  }
  else if (e_stop_.load() || brake_level_ >= 2)
  {
    v = 0.0;
    w = 0.0;
    chosen = Source::kNone;
    brake = true;
  }
  else if (teleop_ok || gimbal_lock_ || mission_stop_)
  {
    // 遥控接管 / 云台锁车 / 任务停止：一律零速（dzactuator 有自己的遥控通道，
    // 这里只负责不让导航速度继续流向下位机）
    v = 0.0;
    w = 0.0;
    chosen = Source::kNone;
    brake = teleop_ok || gimbal_lock_;
  }
  else
  {
    Slot *pick = nullptr;
    if (dztrack_.valid)
    {
      pick = &dztrack_;
      chosen = Source::kDzTrack;
    }
    else if (nav2_.valid)
    {
      pick = &nav2_;
      chosen = Source::kNav2;
    }
    else if (aim_.valid)
    {
      pick = &aim_;
      chosen = Source::kAim;
    }
    else if (mission_.valid)
    {
      pick = &mission_;
      chosen = Source::kMission;
    }

    if (pick != nullptr)
    {
      v = pick->v;
      w = pick->w;
      servo_channel = isServoSource(chosen);
    }

    // 避障减速档：只削线速度，角速度保留以便转向绕障
    if (brake_level_ == 1)
    {
      const double limit = std::max(0.0, brake_v_limit_);
      if (v > limit) v = limit;
      if (v < -limit) v = -limit;
      brake = true;
    }
  }

  // ---- 4. 限幅（按通道分别限幅，绝不可混用） ----
  v = std::clamp(v, -max_v, max_v);
  if (servo_channel)
  {
    w = std::clamp(w, -max_steer_deg, max_steer_deg);
  }
  else
  {
    w = std::clamp(w, -max_w, max_w);
  }

  // ---- 5. 速率限幅（防止下位机收到阶跃指令 / 保护舵机） ----
  v = rateLimit(v, out_v_, max_acc_v * dt);
  if (servo_channel)
  {
    // 舵机走角度通道：度/s
    out_steer_deg_ = rateLimit(w, out_steer_deg_, max_acc_servo * dt);
    w = out_steer_deg_;
    // 切到伺服通道时把速度通道输出同步，避免下次切回去时产生跳变
    out_w_ = 0.0;
  }
  else
  {
    out_w_ = rateLimit(w, out_w_, max_acc_w * dt);
    w = out_w_;
    out_steer_deg_ = 0.0;
  }

  // 硬急停时不限幅，直接归零
  if (e_stop_.load() || brake_level_ >= 2)
  {
    v = 0.0;
    w = 0.0;
    out_steer_deg_ = 0.0;
    out_w_ = 0.0;
  }

  out_v_ = v;
  out_source_ = chosen;

  // ---- 6. 发布 ----
  // 发布策略必须与 dzactuator 的 modeTimerCallback 兼容，否则会出现
  // "自动驾驶命令被误判为纯跟踪命令"的错位：
  //
  //   dzactuator 以"最近 0.5s 内哪个话题有消息"来选择模式
  //     pursuitAngle 有消息 -> Pursuit 模式（把 angular.z 当**角度**用）
  //     cmd_vel      有消息 -> DWA 模式（用 atan(L*w/v) 把角速度换算成角度）
  //   raw[3] 是 mode：
  //     若最近 100ms 收到过 linear.y >= 0.5 的 pursuitAngle -> DWA
  //     否则                                                -> Pursuit
  //
  // 因此：
  //   * 伺服通道源（dztrack / pivot）只发 pursuitAngle，angular.z = 角度(度)
  //   * 速度通道源（nav2 / mission / aim）只发 cmd_vel，angular.z = 角速度(rad/s)
  //   * 刹车时向两个话题都发 linear.y=1 且 v=0：
  //       - Pursuit 模式下靠 dzactuator 自己的停机逻辑，需要 v=0 才会停
  //       - DWA 模式下 dzactuator 会因 linear.y>=0.5 把 raw[3] 置回 Pursuit
  //     所以刹车必须两路同发，才能保证任一模式下都停住。
  const bool publish_brake = brake || e_stop_.load();

  if (publish_brake)
  {
    geometry_msgs::msg::Twist tb;
    tb.linear.x = 0.0;
    tb.angular.z = 0.0;          // 两个通道都归零，量纲无歧义
    tb.linear.y = 1.0;
    if (publish_cmd_vel && (brake_publish_both || !servo_channel))
    {
      pub_cmd_vel_->publish(tb);
    }
    if (publish_pursuit && (brake_publish_both || servo_channel))
    {
      pub_pursuit_->publish(tb);
    }
    last_brake_ = true;
  }
  else
  {
    // 非刹车：只向被选中通道发布，另一通道按需发一帧 y=0 的释放帧，
    // 把上一次刹车可能残留的模式判断拉回来
    geometry_msgs::msg::Twist t;
    t.linear.x = v;
    t.linear.y = 0.0;

    if (servo_channel)
    {
      t.angular.z = w;           // 度
      if (publish_pursuit)
      {
        pub_pursuit_->publish(t);
      }
      if (publish_cmd_vel && last_brake_)
      {
        geometry_msgs::msg::Twist trel;
        trel.linear.x = 0.0;
        trel.angular.z = 0.0;
        trel.linear.y = 0.0;
        pub_cmd_vel_->publish(trel);
      }
    }
    else
    {
      t.angular.z = w;           // rad/s
      if (publish_cmd_vel)
      {
        pub_cmd_vel_->publish(t);
      }
      if (publish_pursuit && last_brake_)
      {
        geometry_msgs::msg::Twist trel;
        trel.linear.x = 0.0;
        trel.angular.z = 0.0;
        trel.linear.y = 0.0;
        pub_pursuit_->publish(trel);
      }
    }
    last_brake_ = false;
  }

  last_servo_channel_ = servo_channel;

  std_msgs::msg::Float32 fs;
  fs.data = static_cast<float>(std::fabs(v));
  pub_arbiter_v_->publish(fs);

  std_msgs::msg::Float32 fd;
  fd.data = static_cast<float>(out_steer_deg_);
  pub_arbiter_steer_->publish(fd);
}

}  // namespace dzmission
