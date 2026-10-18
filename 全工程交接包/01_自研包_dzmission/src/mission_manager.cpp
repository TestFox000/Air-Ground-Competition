#include "dzmission/mission_manager.hpp"

#include "dzmission/air_protocol.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace dzmission
{

namespace
{
double deg2rad(double d) { return d * M_PI / 180.0; }
}  // namespace

const char *MissionManager::stateName(State s)
{
  switch (s)
  {
    case State::kDisabled: return "DISABLED";
    case State::kIdle: return "IDLE";
    case State::kNavigating: return "NAVIGATING";
    case State::kEngaging: return "ENGAGING";
    case State::kReturning: return "RETURNING";
    case State::kHolding: return "HOLDING";
    case State::kEstop: return "ESTOP";
    case State::kArrived: return "ARRIVED";
    default: return "?";
  }
}

MissionManager::MissionManager(const rclcpp::NodeOptions &options)
: Node("dzmission", options)
{
  loadParams();

  // ===========================================================================
  // ★★ 把全部 rclcpp::Time 成员对齐到本节点时钟 ★★
  // ---------------------------------------------------------------------------
  // 为什么必须做这一步：
  //   `rclcpp::Time` 的**默认构造**使用 RCL_SYSTEM_TIME(=1)，
  //   而 `this->now()` 返回本节点时钟 RCL_ROS_TIME(=2)。
  //   两者**相减会抛异常**：
  //       std::runtime_error: can't subtract times with different time sources [1 != 2]
  //   而 tick() 里第一件事就是 `(now - pose_stamp_).seconds()`，
  //   于是节点在启动后的第一次 tick **直接 terminate**（退出码 -6）。
  //
  //   ★ 触发条件：在**收到任何定位数据之前**就跑到了 tick()。
  //     正常比赛里定位数据很快就来，所以这个坑一直没暴露；
  //     离线测试（没有 IMU/odom/amcl 数据源）会**必然**命中。
  //
  //   显式初始化为 now() 后，所有时间源统一，减法安全；
  //   语义上也更合理：这些成员表示"上次发生时刻"，
  //   启动时视为"刚刚发生"比"1970 年"更接近真实。
  // ===========================================================================
  const rclcpp::Time t_start = this->now();
  pose_stamp_ = t_start;
  target_stamp_ = t_start;
  dztrack_stamp_ = t_start;
  nav2_stamp_ = t_start;
  teleop_stamp_ = t_start;
  goal_sent_time_ = t_start;
  goal_last_retry_time_ = t_start;
  last_status_time_ = t_start;
  last_fault_tx_time_ = t_start;
  link_lost_since_ = t_start;
  estop_since_ = t_start;
  bridge_last_request_time_ = t_start;

  // 子系统
  cmd_link_ = std::make_unique<CmdLink>(this, "");
  arbiter_ = std::make_unique<MotionArbiter>(this, "");
  arbiter_->loadParams("");
  safety_ = std::make_unique<SafetyMonitor>(this, "");
  safety_->loadParams();
  aimer_ = std::make_unique<TargetAimer>(this, "");
  aimer_->loadParams();
  referee_ = std::make_unique<RefereeBoard>(this, "");

  // ★ 格子图（场地拓扑 v5）与桥洞过门判定
  //   规格必须与 FieldGrid::Spec 一致；configure 把它们交给 BridgeTracker，
  //   使其 selfCheck() 能验证 30 种合法门位组合全被接受、非法组合被拒绝。
  grid_ = std::make_unique<FieldGrid>();
  bridge_ = std::make_unique<BridgeTracker>();
  {
    const auto &sp = grid_->spec();
    bridge_->configure(sp.cols, sp.rows, sp.bridge_col,
                       sp.bridge_a_lo, sp.bridge_a_hi,
                       sp.bridge_b_lo, sp.bridge_b_hi);
    const std::string err = bridge_->selfCheck();
    if (err.empty())
    {
      RCLCPP_INFO(get_logger(),
                  "[Mission] BridgeTracker 自检通过（30 种门位组合）");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "[Mission] ★ BridgeTracker 自检失败：%s",
                   err.c_str());
    }
  }
  referee_->loadParams();

  setupRosInterfaces();

  cmd_link_->start();
  referee_->start();

  startTimer();

  RCLCPP_INFO(get_logger(),
              "===== dzmission 启动 ===== autonomy=%d arrive=%.2fm battery_low=%.0f%%",
              static_cast<int>(enable_autonomy_), arrive_dist_, low_battery_pct_);
}

MissionManager::~MissionManager()
{
  if (cmd_link_)
  {
    cmd_link_->stop();
  }
  if (referee_)
  {
    referee_->stop();
  }
}

void MissionManager::loadParams()
{
  const std::string p = "mission.";

  declare_parameter(p + "enable_autonomy", enable_autonomy_);
  declare_parameter(p + "require_motion_enable", require_motion_enable_);
  declare_parameter(p + "arrive_dist", arrive_dist_);
  declare_parameter(p + "arrive_yaw_tol_deg", arrive_yaw_tol_deg_);
  declare_parameter(p + "goal_timeout_s", goal_timeout_s_);
  declare_parameter(p + "goal_retry_period_s", goal_retry_period_s_);
  declare_parameter(p + "low_battery_pct", low_battery_pct_);
  declare_parameter(p + "link_lost_return_delay_s", link_lost_return_delay_s_);
  declare_parameter(p + "auto_return_on_link_loss", auto_return_on_link_loss_);
  declare_parameter(p + "engage_requires_localization", engage_requires_localization_);
  declare_parameter(p + "engage_approach_dist", engage_approach_dist_);
  declare_parameter(p + "gimbal_lock_chassis", gimbal_lock_chassis_);
  declare_parameter(p + "clear_recovery_delay_s", clear_recovery_delay_s_);
  declare_parameter(p + "dztrack_active_timeout_s", dztrack_active_timeout_s_);
  declare_parameter(p + "use_dztrack_when_active", use_dztrack_when_active_);
  declare_parameter(p + "default_ammo_when_link_lost", default_ammo_when_link_lost_);

  get_parameter(p + "enable_autonomy", enable_autonomy_);
  get_parameter(p + "require_motion_enable", require_motion_enable_);
  get_parameter(p + "arrive_dist", arrive_dist_);
  get_parameter(p + "arrive_yaw_tol_deg", arrive_yaw_tol_deg_);
  get_parameter(p + "goal_timeout_s", goal_timeout_s_);
  get_parameter(p + "goal_retry_period_s", goal_retry_period_s_);
  get_parameter(p + "low_battery_pct", low_battery_pct_);
  get_parameter(p + "link_lost_return_delay_s", link_lost_return_delay_s_);
  get_parameter(p + "auto_return_on_link_loss", auto_return_on_link_loss_);
  get_parameter(p + "engage_requires_localization", engage_requires_localization_);
  get_parameter(p + "engage_approach_dist", engage_approach_dist_);
  get_parameter(p + "gimbal_lock_chassis", gimbal_lock_chassis_);
  get_parameter(p + "clear_recovery_delay_s", clear_recovery_delay_s_);
  get_parameter(p + "dztrack_active_timeout_s", dztrack_active_timeout_s_);
  get_parameter(p + "use_dztrack_when_active", use_dztrack_when_active_);
  get_parameter(p + "default_ammo_when_link_lost", default_ammo_when_link_lost_);
}

void MissionManager::setupRosInterfaces()
{
  sub_cmd_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "mission_cmd", rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      std::bind(&MissionManager::onMissionCommand, this, std::placeholders::_1));

  sub_pose_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&MissionManager::poseCallback, this, std::placeholders::_1));

  // dztrack 的速度输出：启动 dztrack 时必须把 pursuitAngle 重映射为
  // dztrack_raw_cmd（见 doc/INTEGRATION.md 2.1），否则会与仲裁器抢底盘
  sub_dztrack_ = create_subscription<geometry_msgs::msg::Twist>(
      "dztrack_raw_cmd", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
      std::bind(&MissionManager::dztrackCallback, this, std::placeholders::_1));

  // nav2 的速度输出：velocity_smoother 的 cmd_vel_smoothed 必须重映射为
  // cmd_vel_nav_raw（见 doc/INTEGRATION.md 2.1），由仲裁器决定是否真正下发
  sub_nav2_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav_raw", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
      std::bind(&MissionManager::nav2Callback, this, std::placeholders::_1));

  // 遥控接管：与 dzactuator 的 logicontrol 同源
  sub_teleop_ = create_subscription<geometry_msgs::msg::Twist>(
      "logicontrol", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
      std::bind(&MissionManager::teleopCallback, this, std::placeholders::_1));

  sub_battery_ = create_subscription<std_msgs::msg::Float32>(
      "Battery_Percentage", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
      std::bind(&MissionManager::batteryCallback, this, std::placeholders::_1));

  sub_loc_healthy_ = create_subscription<std_msgs::msg::UInt8>(
      "localization/healthy", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
      std::bind(&MissionManager::locHealthyCallback, this, std::placeholders::_1));

  // ★ FIELD_FIXED（场地固定结构）—— 无人机端上线时发一次，可选消息。
  //   本车端从这里取**无人机起降点**（v2.1 新增），把这两格设为禁入格。
  //   话题名 "field" 与 AirLink::publishMulti("field", d.payload) 一致。
  sub_field_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "field", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
      std::bind(&MissionManager::fieldFixedCallback, this, std::placeholders::_1));

  pub_state_ = create_publisher<std_msgs::msg::UInt8>("mission_state", 5);
  pub_fault_ = create_publisher<std_msgs::msg::UInt8>("mission_fault", 5);
  pub_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>("mission_goal", 5);
  pub_debug_ = create_publisher<std_msgs::msg::UInt8MultiArray>("mission_debug", 5);

  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");

  // 初始状态
  setState(enable_autonomy_ ? State::kIdle : State::kDisabled, "startup");
  last_status_time_ = now();
  last_fault_tx_time_ = now();
  link_lost_since_ = now();
  estop_since_ = now();
  goal_sent_time_ = now();
  goal_last_retry_time_ = now();
  dztrack_stamp_ = now();
  teleop_stamp_ = now();
  pose_stamp_ = now();
  target_stamp_ = now();
}

void MissionManager::startTimer()
{
  // 50Hz：仲裁 + 云台 + 状态机
  timer_ = create_wall_timer(std::chrono::milliseconds(20),
                             std::bind(&MissionManager::tick, this));
}

// =============================================================================
// 主循环
// =============================================================================
void MissionManager::tick()
{
  const rclcpp::Time now = this->now();

  // ---- 1. 定位健康 ----
  const double pose_age = (now - pose_stamp_).seconds();
  const bool pose_fresh = pose_valid_ && pose_age < 0.5;

  // ---- 2. 近场安全等级 -> 仲裁器 ----
  // 关键：近场（原为雷达，现为**视觉近场**）数据不新鲜时，绝不能沿用上一帧的
  //       "前方畅通"结论，必须按急停处理（失明即停车），否则会盲开。
  //
  // ★ 本赛题**禁止激光雷达**，所以 SafetyMonitor 已改为"无雷达版"
  //   （视觉近场 + 堵转 + IMU 姿态 + 地图新鲜度）。
  //   这里原为 safety_->scanFresh()（LaserScan 新鲜度），该接口随雷达一并移除；
  //   现用 Level::vision_fresh 替代 —— 语义一致：近场感知不新鲜就当急停。
  const SafetyMonitor::Level lv = safety_->level();
  const bool near_field_fresh = lv.vision_fresh;
  if (near_field_fresh)
  {
    arbiter_->setObstacle(lv.brake_level, lv.v_limit);
  }
  else
  {
    arbiter_->setObstacle(2, 0.0);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "[Mission] 近场感知数据超时未更新，按急停处理（失明即停）");
  }

  // ---- 3. 链路状态（只在 true->false 边沿记录丢失起点） ----
  const bool link_alive = cmd_link_->linkAlive();
  if (link_alive)
  {
    link_was_alive_ = true;
    link_lost_reported_ = false;
  }
  else if (link_was_alive_ && !link_lost_reported_)
  {
    // 真正从"曾经连通"变为"断开"的那一刻
    link_lost_since_ = now;
    link_lost_reported_ = true;
    if (auto_return_on_link_loss_ && state_ != State::kReturning &&
        state_ != State::kDisabled)
    {
      RCLCPP_WARN(get_logger(), "[Mission] 指令链路丢失，%.1fs 后自动返航",
                  link_lost_return_delay_s_);
    }
  }

  // ---- 4. 遥控接管检测（只在上/下沿改动仲裁器，避免每 tick 刷新其时间戳） ----
  const bool teleop_active = (now - teleop_stamp_).seconds() < 0.5;
  if (teleop_active != teleop_prev_active_)
  {
    arbiter_->setTeleopActive(teleop_active);
    teleop_prev_active_ = teleop_active;
  }

  // ---- 5. 云台锁车 ----
  const bool engaging = (state_ == State::kEngaging);
  arbiter_->setGimbalLock(engaging && gimbal_lock_chassis_ && safety_->stopped());

  // ---- 6. 安全急停判定与自动恢复 ----
  // 定位故障必须"锁存"：/odometry/filtered 从未到达时，pose_valid_ 恒为 false，
  // 若每 tick 都重新判定，就会出现 ESTOP -> 恢复 -> 立刻 ESTOP 的 1Hz 振荡。
  const bool safety_stop = safety_->stopped();
  const bool loc_fault = !pose_fresh ||
                         (engage_requires_localization_ && !loc_healthy_ && engaging);

  setFaultBit(kBitNoLocalization, !pose_fresh);
  setFaultBit(kBitObstacle, safety_stop);

  if (safety_stop || loc_fault)
  {
    arbiter_->setEmergencyStop(true);
    if (state_ != State::kEstop)
    {
      resume_state_ = (state_ == State::kEstop) ? State::kIdle : state_;
      estop_since_ = now;
      // ★ 文案必须能区分原因 —— 否则排查时会被误导。
      //   原写法在 `!pose_fresh` 时也打印 "localization unhealthy"，
      //   但几种根因完全不同：
      //     · 从未收到 /odometry/filtered  -> pose_valid_ 恒 false
      //     · 收到了但太旧                 -> 定位节点停了 / 话题或 QoS 不对
      //     · loc_healthy_ 为 false        -> 定位节点自评不健康（传感器数据问题）
      std::string why;
      if (safety_stop)
      {
        why = "safety e-stop (obstacle)";
      }
      else if (!pose_valid_)
      {
        why = "localization: never received /odometry/filtered";
      }
      else if (pose_age >= 0.5)
      {
        why = "localization: /odometry/filtered stale (" +
              std::to_string(pose_age) + "s)";
      }
      else
      {
        why = "localization: loc_healthy=false (sensor unhealthy)";
      }
      setState(State::kEstop, why);
      reportFaults();
    }
  }
  else if (state_ == State::kEstop)
  {
    // 只有在危险确实解除后（且已经过了恢复延时）才离开 ESTOP
    if ((now - estop_since_).seconds() >= clear_recovery_delay_s_)
    {
      resume_state_ = resume_state_ == State::kEstop ? State::kIdle : resume_state_;
      setState(resume_state_, "e-stop cleared, resume");
      arbiter_->setEmergencyStop(false);
    }
    else
    {
      arbiter_->setEmergencyStop(true);
    }
  }
  else
  {
    arbiter_->setEmergencyStop(false);
  }

  // ---- 7. 链路丢失返航 ----
  setFaultBit(kBitCmdLink, !link_alive && link_was_alive_);
  if (!link_alive && link_was_alive_ && auto_return_on_link_loss_ &&
      (now - link_lost_since_).seconds() > link_lost_return_delay_s_ &&
      state_ != State::kReturning && state_ != State::kEstop &&
      state_ != State::kDisabled)
  {
    handleReturnBase(MissionCommand{});
  }

  // ---- 8. 低电量处理 ----
  setFaultBit(kBitLowBattery, battery_pct_ < low_battery_pct_);
  if (battery_pct_ < low_battery_pct_)
  {
    if (state_ == State::kEngaging)
    {
      aimer_->ceaseFire();
    }
    if (home_valid_ && state_ != State::kReturning && state_ != State::kEstop &&
        state_ != State::kDisabled)
    {
      handleReturnBase(MissionCommand{});
    }
  }

  // ---- 9. 交战输入联锁 ----
  const RefereeBoard::Status rs = referee_->status();
  // 裁判板链路不可用时：是否用一个保守的假定弹量继续允许交战
  const int ammo_for_aim = rs.link_ok ? static_cast<int>(rs.ammo)
                                      : default_ammo_when_link_lost_;
  aimer_->setAmmo(ammo_for_aim);
  aimer_->setBeingHit(rs.being_hit);
  aimer_->setChassisStationary(std::fabs(vel_v_) < 0.05);
  // ★ 红蓝面联锁 / 靶标击毁 / 弹量纪律所需的裁判板反馈
  aimer_->setTargetHp(static_cast<int>(rs.target_hp));
  aimer_->setBoardShotCount(static_cast<int>(rs.shoot_count));
  aimer_->setRefereeLinkOk(rs.link_ok);

  setFaultBit(kBitRefereeLink, !rs.link_ok);
  setFaultBit(kBitNoAmmo, rs.link_ok && engaging && rs.ammo < 1);

  // ---- 10. 云台/激光 ----
  aimer_->tick();

  // ★ 桥洞过门判定（规则 image7：从通道通过即完成穿棱，每区段 10 分）
  //   放进 tick() 而不是单独定时器：它需要 pose_x_/pose_y_（刚更新过），
  //   且不需要高频 —— 50Hz 足够捕捉过门动作（车过门至少几百毫秒）。
  tickBridgeCrossing();
  {
    const TargetAimer::AimResult ar = aimer_->result();
    setFaultBit(kBitVisionLost, engaging && !ar.target_visible &&
                                    ar.last_fire_age_s > 3.0);
  }

  // ---- 11. 速度源注入仲裁 ----
  // dztrack 优先于 nav2：dztrack 自带 obstacleDetect，且它是"预录路径跟踪"模式，
  // 一旦它在发速度就说明操作者希望走这条路，此时忽略 nav2 的输出。
  const bool dztrack_recent = (now - dztrack_stamp_).seconds() < dztrack_active_timeout_s_;
  const bool nav2_recent = nav2_seen_ &&
                           (now - nav2_stamp_).seconds() < dztrack_active_timeout_s_;
  const bool motion_allowed = (state_ != State::kEstop) && !safety_stop;

  if (use_dztrack_when_active_ && dztrack_recent && motion_allowed)
  {
    arbiter_->setSource(MotionArbiter::Source::kDzTrack, dztrack_v_, dztrack_w_);
  }
  if (nav2_recent && motion_allowed)
  {
    // nav2 的速度由其 velocity_smoother 产出并经 launch remap 到 cmd_vel_nav_raw
    arbiter_->setSource(MotionArbiter::Source::kNav2, nav2_v_, nav2_w_);
  }

  // ---- 12. 任务停止标志 ----
  // require_motion_enable=true 时，除遥控外一律不允许底盘运动，
  // 直到上层下发任意一条有效任务指令（收到指令即视为使能）。
  const bool stop_for_enable = require_motion_enable_ && !motion_enabled_;
  const bool mission_should_stop =
      stop_for_enable || (state_ == State::kHolding) || (state_ == State::kArrived) ||
      (state_ == State::kIdle) || (state_ == State::kDisabled);
  arbiter_->setMissionStop(mission_should_stop && !dztrack_recent && !nav2_recent);

  // ---- 13. 仲裁输出 ----
  arbiter_->tick();
  vel_v_ = arbiter_->activeV();
  vel_w_ = arbiter_->activeW();

  // ---- 14. 导航到达检测 / 目标重发 ----
  if (state_ == State::kNavigating)
  {
    checkArrival();

    // 服务未就绪或目标丢失时，按 goal_retry_period_s_ 周期性重发
    if (goal_valid_ && !nav_goal_active_.load() &&
        (now - goal_last_retry_time_).seconds() >= goal_retry_period_s_)
    {
      goal_last_retry_time_ = now;
      sendNavGoal(goal_x_, goal_y_, goal_yaw_, goal_has_yaw_);
    }

    // 整体超时保护：长时间未到达则告警并重新下发
    if ((now - goal_sent_time_).seconds() > goal_timeout_s_)
    {
      RCLCPP_WARN(get_logger(), "[Mission] 导航超时 %.1fs，重新下发目标",
                  goal_timeout_s_);
      if (goal_valid_)
      {
        goal_sent_time_ = now;
        sendNavGoal(goal_x_, goal_y_, goal_yaw_, goal_has_yaw_);
      }
    }
  }

  // ---- 15. 状态回传 ----
  reportFaults();

  if ((now - last_status_time_).seconds() > 0.2)
  {
    last_status_time_ = now;
    publishStatus();
  }

  // ---- 16. 调试信息 ----
  {
    std_msgs::msg::UInt8MultiArray dbg;
    dbg.data.resize(8);
    dbg.data[0] = static_cast<uint8_t>(state_);
    dbg.data[1] = static_cast<uint8_t>(arbiter_->brakeLevel());
    dbg.data[2] = static_cast<uint8_t>(static_cast<int>(vel_v_ * 100.0) & 0xFF);
    dbg.data[3] = static_cast<uint8_t>(link_alive ? 1 : 0);
    dbg.data[4] = static_cast<uint8_t>(loc_healthy_ ? 1 : 0);
    dbg.data[5] = static_cast<uint8_t>(rs.ammo);
    dbg.data[6] = static_cast<uint8_t>(rs.self_hp);
    dbg.data[7] = static_cast<uint8_t>(aimer_->result().state);
    pub_debug_->publish(dbg);
  }

  std_msgs::msg::UInt8 st;
  st.data = static_cast<uint8_t>(state_);
  pub_state_->publish(st);
  std_msgs::msg::UInt8 ft;
  ft.data = fault_;
  pub_fault_->publish(ft);
}

// =============================================================================
// 指令处理
// =============================================================================
void MissionManager::onMissionCommand(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 16)
  {
    return;
  }

  MissionCommand m;
  m.cmd = msg->data[0];
  m.seq = msg->data[1];
  m.target_id = msg->data[2];
  m.confidence = msg->data[3];
  m.stop = msg->data[4];
  m.has_yaw = msg->data[5] != 0;

  const int16_t bearing_raw = static_cast<int16_t>(msg->data[6] | (msg->data[7] << 8));
  const uint16_t range_raw = static_cast<uint16_t>(msg->data[8] | (msg->data[9] << 8));
  m.bearing_deg = static_cast<double>(bearing_raw) * 0.1;
  m.range_m = static_cast<double>(range_raw) * 0.01;

  // ===========================================================================
  // ★★ 批量侦察消息的载荷解析（尾部扩展，见 cmd_link.cpp publishCommand）★★
  // ---------------------------------------------------------------------------
  // 布局： [16..19] ts_ms(u32 LE)   [20] item_count(u8)   [21..] payload
  //
  // ★ 为什么不在协议层（mission_command.cpp）解析：
  //   协议层不该知道"场地有几行几列"；领域校验必须在持有 FieldGrid 的这里。
  // ★ 老指令只有 16 字节，必须容忍（此时 m.payload 保持为空）。
  // ===========================================================================
  if (msg->data.size() >= 21)
  {
    const size_t base = 16;
    m.ts_ms = static_cast<uint32_t>(msg->data[base + 0]) |
              (static_cast<uint32_t>(msg->data[base + 1]) << 8) |
              (static_cast<uint32_t>(msg->data[base + 2]) << 16) |
              (static_cast<uint32_t>(msg->data[base + 3]) << 24);
    m.item_count = msg->data[base + 4];
    m.payload.assign(msg->data.begin() + static_cast<long>(base + 5),
                     msg->data.end());
  }

  RCLCPP_INFO(get_logger(), "[Mission] 收到指令 %s seq=%u", cmdName(m.cmd), m.seq);

  // 任何一条有效指令都视为"运动使能"（当 require_motion_enable 打开时生效）
  motion_enabled_ = true;

  if (state_ == State::kDisabled)
  {
    if (!enable_autonomy_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "[Mission] enable_autonomy=false，忽略指令");
      return;
    }
    setState(State::kIdle, "first command received");
  }

  switch (m.cmd)
  {
    case kCmdNavTo: handleNavTo(m); break;
    case kCmdOpStop: handleStop(m); break;
    case kCmdOpEngage: handleEngage(m); break;
    case kCmdCeaseFire: handleCeaseFire(m); break;
    case kCmdOpReturnBase: handleReturnBase(m); break;
    case kCmdOpHold: handleHold(m); break;
    case kCmdTargetReport: handleTargetReport(m); break;
    case kCmdSetHome: handleSetHome(m); break;
    // ★★ 无人机侦察结果四类（规则 image5/image6）★★
    //   ★ 这四类**不改变任务状态**，只更新地图/门位缓存 ——
    //     侦察是"给养"，不是"指令"。任务状态仍由 NAV_TO/ENGAGE 等驱动。
    case kCmdMineList: handleMineList(m); break;
    case kCmdObstacleList: handleObstacleList(m); break;
    case kCmdTargetList: handleTargetList(m); break;
    case kCmdBridgeGate: handleBridgeGate(m); break;
    case kCmdMapFull: handleMapFull(m); break;
    // ★ 场地固定结构（v2.1 含无人机起降点）—— 设为禁入格
    case kCmdFieldFixed: handleFieldFixed(m); break;
    case kCmdOpNop:
    case kCmdHeartbeat:
      break;
    default:
      RCLCPP_WARN(get_logger(), "[Mission] 未知指令 0x%02x", m.cmd);
      break;
  }
}

void MissionManager::handleNavTo(const MissionCommand &m)
{
  if (state_ == State::kEstop)
  {
    RCLCPP_WARN(get_logger(), "[Mission] ESTOP 中拒绝 NAV_TO");
    return;
  }

  goal_x_ = m.x;
  goal_y_ = m.y;
  goal_yaw_ = m.yaw;
  goal_has_yaw_ = m.has_yaw;
  goal_valid_ = true;
  goal_retry_count_ = 0;

  aimer_->ceaseFire();
  sendNavGoal(goal_x_, goal_y_, goal_yaw_, goal_has_yaw_);
  setState(State::kNavigating, "NAV_TO");
}

void MissionManager::handleStop(const MissionCommand &m)
{
  if (m.stop == 0)
  {
    setState(State::kIdle, "STOP released");
    return;
  }
  cancelNavGoal();
  aimer_->abort();
  goal_valid_ = false;
  arbiter_->clearAllSources();
  // 急停由 tick() 依据 mission_should_stop 统一决定，避免在多处状态切换
  setState(State::kHolding, "STOP command");
}

void MissionManager::handleEngage(const MissionCommand &m)
{
  if (state_ == State::kEstop)
  {
    RCLCPP_WARN(get_logger(), "[Mission] ESTOP 中拒绝 ENGAGE");
    return;
  }
  setFaultBit(kBitLowBattery, battery_pct_ < low_battery_pct_);
  if (battery_pct_ < low_battery_pct_)
  {
    RCLCPP_WARN(get_logger(), "[Mission] 电量 %.0f%% 过低，拒绝交战", battery_pct_);
    reportFaults();
    sendTargetAck(kHitAborted, m);
    return;
  }
  if (engage_requires_localization_ && !loc_healthy_)
  {
    RCLCPP_WARN(get_logger(), "[Mission] 定位不健康，拒绝交战");
    setFaultBit(kBitNoLocalization, true);
    reportFaults();
    sendTargetAck(kHitAborted, m);
    return;
  }

  target_id_ = m.target_id;
  target_confidence_ = m.confidence;
  target_bearing_deg_ = m.bearing_deg;
  target_range_m_ = m.range_m;
  target_stamp_ = now();
  target_valid_ = true;

  shots_at_engage_start_ = referee_->status().shoot_count;
  target_hp_at_engage_start_ = referee_->status().target_hp;

  // 若无人机给了相对方位与距离，先解算目标点并前往；否则原地交战
  if (m.range_m > 0.05 && std::fabs(m.bearing_deg) <= 180.0)
  {
    const double bearing = deg2rad(m.bearing_deg);
    // 目标点 = 本车在 map 系位置 + 旋转后的相对矢量
    const double cx = std::cos(pose_yaw_);
    const double sx = std::sin(pose_yaw_);
    const double dx = m.range_m * std::cos(bearing);
    const double dy = m.range_m * std::sin(bearing);

    double tx = pose_x_ + cx * dx - sx * dy;
    double ty = pose_y_ + sx * dx + cx * dy;

    // 保持 engage_approach_dist 的打击距离：朝目标点收缩
    const double ux = tx - pose_x_;
    const double uy = ty - pose_y_;
    const double d = std::hypot(ux, uy);
    if (d > engage_approach_dist_)
    {
      const double k = (d - engage_approach_dist_) / d;
      tx = pose_x_ + ux * (1.0 - k);
      ty = pose_y_ + uy * (1.0 - k);
    }

    goal_x_ = tx;
    goal_y_ = ty;
    goal_has_yaw_ = false;
    goal_valid_ = true;
    sendNavGoal(goal_x_, goal_y_, 0.0, false);
    RCLCPP_INFO(get_logger(),
                "[Mission] ENGAGE target=%u 解算目标点 (%.2f, %.2f) 距离=%.2fm",
                m.target_id, tx, ty, m.range_m);
  }
  else
  {
    goal_valid_ = false;
    cancelNavGoal();
  }

  // 与 tick() 保持一致的链路/弹量判定（此处不能引用 tick 的局部变量）
  const RefereeBoard::Status rs = referee_->status();
  aimer_->setAmmo(rs.link_ok ? static_cast<int>(rs.ammo) : default_ammo_when_link_lost_);
  aimer_->setBeingHit(rs.being_hit);
  aimer_->requestEngage(m.target_id);

  setState(State::kEngaging, "ENGAGE");
}

void MissionManager::handleCeaseFire(const MissionCommand &m)
{
  (void)m;
  aimer_->ceaseFire();
  if (state_ == State::kEngaging)
  {
    setState(goal_valid_ ? State::kNavigating : State::kHolding, "CEASE_FIRE");
  }
}

void MissionManager::handleReturnBase(const MissionCommand &m)
{
  (void)m;
  aimer_->abort();

  if (!home_valid_)
  {
    RCLCPP_WARN(get_logger(), "[Mission] 未设置 home 点，返航退化为原地待命");
    cancelNavGoal();
    goal_valid_ = false;
    setState(State::kHolding, "RETURN_BASE without home");
    sendFault(kFaultNone, 1);
    return;
  }

  // 位置反投影：返航点固定，用 NavigateToPose 走
  goal_x_ = home_x_;
  goal_y_ = home_y_;
  goal_yaw_ = home_yaw_;
  goal_has_yaw_ = true;
  goal_valid_ = true;
  goal_retry_count_ = 0;
  sendNavGoal(goal_x_, goal_y_, goal_yaw_, true);
  setState(State::kReturning, "RETURN_BASE");
}

void MissionManager::handleHold(const MissionCommand &m)
{
  (void)m;
  cancelNavGoal();
  goal_valid_ = false;
  arbiter_->clearAllSources();
  setState(State::kHolding, "HOLD");
}

void MissionManager::handleTargetReport(const MissionCommand &m)
{
  target_id_ = m.target_id;
  target_confidence_ = m.confidence;
  target_bearing_deg_ = m.bearing_deg;
  target_range_m_ = m.range_m;
  target_stamp_ = now();
  target_valid_ = true;

  RCLCPP_INFO(get_logger(),
              "[Mission] 目标上报 id=%u 方位=%.1fdeg 距离=%.2fm 置信=%u%%",
              m.target_id, m.bearing_deg, m.range_m, m.confidence);
}

// =============================================================================
// ★★ 无人机侦察结果处理（规则 image5/image6）★★
// =============================================================================
// 无人机四类识别：地雷位置 · 打击区位置 · 障碍物 · 桥洞通道。
// 协议层（mission_command.cpp）只做**长度/自洽性**检查；
// **领域校验在这里做**，因为只有 MissionManager 持有 FieldGrid 的场地规格。
//
// ★ 设计原则：**校验不过就当没收**，绝不带病规划。
//   无人机端尚未实现，字段若有偏差，最可能的后果是"静默错误"
//   （例如门位越界被当成合法行 -> 规划出撞墙的路径 -> 撞 10s 切手动）。
// =============================================================================

namespace
{
// 内容哈希（FNV-1a）：用于判断"这份数据与上次是否相同"，
// 避免无人机周期重发同一份数据时反复打日志/重规划。
uint32_t hashRecon(const std::vector<uint8_t> &v)
{
  uint32_t h = 2166136261u;
  for (uint8_t b : v)
  {
    h ^= b;
    h *= 16777619u;
  }
  return h;
}
}  // namespace

// -----------------------------------------------------------------------------
// EVENT 帧发送
// -----------------------------------------------------------------------------
// 规则要求的关键事件（image7 桥洞通过、image11 计分项）必须主动上报，
// 否则裁判系统无法确认我方完成了动作。
bool MissionManager::sendEvent(uint8_t ev, uint8_t a0, uint8_t a1, uint8_t a2)
{
  if (!cmd_link_)
  {
    return false;
  }
  const uint32_t ts = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
  const std::vector<uint8_t> payload = air::enc::event(ts, ev, a0, a1, a2);
  std::vector<uint8_t> frame(air::kMaxFrameLen);
  const size_t n = encodeFrame(air::kEvent, payload, frame.data(), frame.size());
  if (n == 0)
  {
    RCLCPP_WARN(get_logger(), "[Mission] EVENT 编码失败 ev=%u", ev);
    return false;
  }
  const bool ok = cmd_link_->sendFrame(frame.data(), n);
  RCLCPP_INFO(get_logger(), "[Mission] EVENT 上报 ev=%u a0=%u a1=%u a2=%u -> %s",
              ev, a0, a1, a2, ok ? "已发送" : "发送失败");
  return ok;
}

// -----------------------------------------------------------------------------
// 地雷列表
// -----------------------------------------------------------------------------
void MissionManager::handleMineList(const MissionCommand &m)
{
  const auto &p = m.payload;
  const size_t cnt = m.item_count;
  std::vector<ReconPoint> pts;
  pts.reserve(cnt);
  int rejected = 0;

  for (size_t i = 0; i < cnt; ++i)
  {
    const size_t off = 5 + i * 3;   // ts(4) count(1) 之后，每条 3 字节
    if (off + 3 > p.size())
    {
      break;
    }
    ReconPoint rp;
    rp.col = static_cast<int>(p[off]);
    rp.row = static_cast<int>(p[off + 1]);
    rp.confidence = static_cast<int>(p[off + 2]);

    if (!grid_->inBounds(rp.col, rp.row))
    {
      ++rejected;
      continue;
    }
    // 去重（同一格重复上报只保留一条）
    bool dup = false;
    for (const auto &q : pts)
    {
      if (q.col == rp.col && q.row == rp.row) { dup = true; break; }
    }
    if (!dup)
    {
      pts.push_back(rp);
    }
  }

  const uint32_t h = hashRecon(p);
  if (h == recon_mine_hash_ && pts.size() == recon_mines_.size())
  {
    return;   // 内容未变
  }
  recon_mine_hash_ = h;
  recon_mines_ = pts;
  recon_mine_ts_ = m.ts_ms;

  RCLCPP_INFO(get_logger(),
              "[Mission] 地雷列表：收到 %zu 条，采纳 %zu 条，越界拒绝 %d 条",
              cnt, pts.size(), rejected);
  if (rejected > 0)
  {
    RCLCPP_WARN(get_logger(),
                "[Mission] ★ 有 %d 条地雷坐标越界（场地 %d 列 x %d 行）"
                "—— 请核对无人机端的字段单位（应为 0-based 列/行）",
                rejected, grid_->spec().cols, grid_->spec().rows);
  }
  setFaultBit(kBitReconIncomplete, pts.empty());
  applyReconToGrid();
}

// -----------------------------------------------------------------------------
// 障碍物列表
// -----------------------------------------------------------------------------
void MissionManager::handleObstacleList(const MissionCommand &m)
{
  const auto &p = m.payload;
  const size_t cnt = m.item_count;
  std::vector<ReconPoint> pts;
  pts.reserve(cnt);
  int rejected = 0;

  for (size_t i = 0; i < cnt; ++i)
  {
    const size_t off = 5 + i * 3;
    if (off + 3 > p.size())
    {
      break;
    }
    ReconPoint rp;
    rp.col = static_cast<int>(p[off]);
    rp.row = static_cast<int>(p[off + 1]);
    rp.confidence = static_cast<int>(p[off + 2]);

    if (!grid_->inBounds(rp.col, rp.row))
    {
      ++rejected;
      continue;
    }
    // ★ 起终点格不允许被标为障碍 —— 若无人机误报，会导致任务无法开始/结束
    const auto &sp = grid_->spec();
    if ((rp.col == sp.start_col && rp.row == sp.start_row) ||
        (rp.col == sp.goal_col && rp.row == sp.goal_row))
    {
      RCLCPP_WARN(get_logger(),
                  "[Mission] ★ 障碍物上报落在起/终点格 %s，已忽略",
                  FieldGrid::cellName(rp.col, rp.row).c_str());
      ++rejected;
      continue;
    }
    bool dup = false;
    for (const auto &q : pts)
    {
      if (q.col == rp.col && q.row == rp.row) { dup = true; break; }
    }
    if (!dup)
    {
      pts.push_back(rp);
    }
  }

  const uint32_t h = hashRecon(p);
  if (h == recon_obstacle_hash_ && pts.size() == recon_obstacles_.size())
  {
    return;
  }
  recon_obstacle_hash_ = h;
  recon_obstacles_ = pts;
  recon_obstacle_ts_ = m.ts_ms;

  RCLCPP_INFO(get_logger(),
              "[Mission] 障碍物列表：收到 %zu 条，采纳 %zu 条，拒绝 %d 条",
              cnt, pts.size(), rejected);
  applyReconToGrid();
}

// -----------------------------------------------------------------------------
// 打击区列表
// -----------------------------------------------------------------------------
void MissionManager::handleTargetList(const MissionCommand &m)
{
  const auto &p = m.payload;
  const size_t cnt = m.item_count;
  std::vector<ReconPoint> pts;
  pts.reserve(cnt);
  int rejected = 0;

  for (size_t i = 0; i < cnt; ++i)
  {
    const size_t off = 5 + i * 4;   // 每条 4 字节（多一个 face）
    if (off + 4 > p.size())
    {
      break;
    }
    ReconPoint rp;
    rp.col = static_cast<int>(p[off]);
    rp.row = static_cast<int>(p[off + 1]);
    rp.face = static_cast<int>(p[off + 2]);
    rp.confidence = static_cast<int>(p[off + 3]);

    if (!grid_->inBounds(rp.col, rp.row))
    {
      ++rejected;
      continue;
    }
    // face 只接受 0/1/2（未知/红友/蓝敌）
    if (rp.face > 2)
    {
      RCLCPP_WARN(get_logger(),
                  "[Mission] ★ 打击区 face=%d 非法（应为 0/1/2），按未知处理",
                  rp.face);
      rp.face = 0;
    }
    pts.push_back(rp);
  }

  recon_targets_ = pts;
  recon_target_ts_ = m.ts_ms;

  RCLCPP_INFO(get_logger(),
              "[Mission] 打击区列表：收到 %zu 条，采纳 %zu 条，越界拒绝 %d 条",
              cnt, pts.size(), rejected);
  // ★ 注意：规则 image4 说打击区是"相连的两个横/纵向单元"。
  //   这里只做单格校验；"两格连通性"留到使用时判断，
  //   因为无人机可能分两帧下发两条记录。
}

// -----------------------------------------------------------------------------
// 桥洞门位
// -----------------------------------------------------------------------------
void MissionManager::handleBridgeGate(const MissionCommand &m)
{
  const auto &p = m.payload;
  if (p.size() < 4)
  {
    return;
  }
  const uint8_t bridge_id = p[0];
  const uint8_t gate_row = p[1];
  const uint8_t confidence = p[2];
  const uint8_t flags = p[3];

  const auto &sp = grid_->spec();
  const bool in_a = (gate_row >= sp.bridge_a_lo && gate_row <= sp.bridge_a_hi);
  const bool in_b = (gate_row >= sp.bridge_b_lo && gate_row <= sp.bridge_b_hi);

  // ★ 无人机端尚未实现，不能假设它一定填对 bridge_id。
  //   所以：优先按 bridge_id 归位，但若 bridge_id 不合法则**按行号落在哪个
  //   区段自动归位** —— 行号是物理事实，比编号可靠。
  int row_a = -1;
  int row_b = -1;
  if (in_a)
  {
    row_a = gate_row;
  }
  else if (in_b)
  {
    row_b = gate_row;
  }
  else
  {
    RCLCPP_WARN(get_logger(),
                "[Mission] ★ 桥洞门位非法：bridge_id=%u gate_row=%u "
                "（区段一 K%d..K%d / 区段二 K%d..K%d）—— 已拒绝",
                bridge_id, gate_row, sp.bridge_a_lo + 1, sp.bridge_a_hi + 1,
                sp.bridge_b_lo + 1, sp.bridge_b_hi + 1);
    sendEvent(air::kEvGateInvalid, gate_row, bridge_id, 0);
    setFaultBit(kBitReconIncomplete, true);
    return;
  }

  const bool was_ready = bridge_->ready();
  const bool now_ready = bridge_->setGates(row_a, row_b);

  RCLCPP_INFO(get_logger(),
              "[Mission] 桥洞门位：bridge_id=%u row=K%d conf=%u flags=0x%02X"
              " -> %s（%s）",
              bridge_id, gate_row + 1, confidence, flags,
              bridgeStateName(bridge_->state()), bridge_->note().c_str());

  for (const auto &r : bridge_->rejected())
  {
    RCLCPP_WARN(get_logger(), "[Mission] ★ 门位被拒：%s", r.c_str());
  }

  if (now_ready)
  {
    // ★★ 门位就绪：打开 FieldGrid 里 K 列的缺口 ★★
    grid_->openCrossing(bridge_->gateRowA());
    grid_->openCrossing(bridge_->gateRowB());
    if (!was_ready)
    {
      RCLCPP_INFO(get_logger(), "[Mission] ★ 桥洞就绪：K%d + K%d（可跨桥）",
                  bridge_->gateRowA() + 1, bridge_->gateRowB() + 1);
      sendEvent(air::kEvGateConfirmed, bridge_->gateRowA(),
                bridge_->gateRowB(), 100);
      setFaultBit(kBitReconIncomplete, false);
    }
  }
  else
  {
    // ★ 只有一个门（或非法）：不可用于规划。
    //   v5 拓扑下单门时起点到终点**不可达**（水平隔离线只有 K 列一个缺口，
    //   进西侧和回东侧需要两个不同高度的缺口）。
    //   硬走会撞隔板 -> 撞 10s 切手动（image9）-> 全盘皆输。
    setFaultBit(kBitReconIncomplete, true);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "[Mission] ★ 桥洞信息不完整（%s）—— 暂不跨桥。"
                         "单门时起终点不可达，硬走会撞隔板",
                         bridgeStateName(bridge_->state()));
  }
}

// -----------------------------------------------------------------------------
// 全图占据栅格（可选）
// -----------------------------------------------------------------------------
void MissionManager::handleMapFull(const MissionCommand &m)
{
  const auto &p = m.payload;
  if (p.size() < 10)
  {
    return;
  }
  const uint8_t cols = p[8];
  const uint8_t rows = p[9];
  const auto &sp = grid_->spec();
  if (cols != sp.cols || rows != sp.rows)
  {
    RCLCPP_WARN(get_logger(),
                "[Mission] ★ 全图尺寸不符：收到 %ux%u，期望 %dx%d —— 已忽略",
                cols, rows, sp.cols, sp.rows);
    return;
  }

  int applied = 0;
  for (int r = 0; r < rows; ++r)
  {
    for (int c = 0; c < cols; ++c)
    {
      const size_t off = 10 + (static_cast<size_t>(r) * cols + c) * 2;
      if (off + 2 > p.size())
      {
        break;
      }
      const uint16_t cell = FieldGrid::rdU16Pub(&p[off]);
      uint8_t occ = 0;
      uint8_t conf = 0;
      uint8_t age = 0;
      air::enc::cellDecode(cell, occ, conf, age);
      // 只应用"确定被占据"且置信度足够的格
      if (occ == 2 && conf >= 50)
      {
        if (grid_->markBlockedByRecon(c, r))
        {
          ++applied;
        }
      }
    }
  }
  RCLCPP_INFO(get_logger(), "[Mission] 全图栅格：应用 %d 个阻挡格", applied);
}

// -----------------------------------------------------------------------------
// 把侦察数据应用到格子图
// -----------------------------------------------------------------------------
void MissionManager::applyReconToGrid()
{
  int mine_ok = 0;
  for (const auto &rp : recon_mines_)
  {
    if (grid_->markMineByRecon(rp.col, rp.row))
    {
      ++mine_ok;
    }
  }
  int obs_ok = 0;
  for (const auto &rp : recon_obstacles_)
  {
    if (grid_->markBlockedByRecon(rp.col, rp.row))
    {
      ++obs_ok;
    }
  }
  for (const auto &rp : recon_targets_)
  {
    grid_->markTargetByRecon(rp.col, rp.row);
  }
  RCLCPP_INFO(get_logger(),
              "[Mission] 侦察数据已应用：地雷 %d 格、障碍 %d 格", mine_ok, obs_ok);
}

// -----------------------------------------------------------------------------
// ★ 桥洞过门判定（每周期调用）
// -----------------------------------------------------------------------------
// 规则 image7："从可通行通道通过即完成穿棱"，每区段 10 分（共 20 分）。
// ★ 判定必须看"进入 + 穿出"成对出现（见 bridge_tracker.cpp 的说明），
//   只看"到达门格"会把路过也算通过 -> 误报得分。
void MissionManager::tickBridgeCrossing()
{
  if (!bridge_ || !grid_)
  {
    return;
  }

  int col = -1;
  int row = -1;
  if (!grid_->worldToCell(pose_x_, pose_y_, col, row))
  {
    return;
  }

  const BridgeTracker::PassResult r = bridge_->update(col, row);

  if (r.just_passed_a)
  {
    RCLCPP_INFO(get_logger(), "[Mission] ★ 穿过桥洞区段一（K%d）—— 该区段 10 分",
                r.gate_row + 1);
    // ★ 注意命名空间：kEv* 在 air:: 里，而 kBridgeId* 在 dzmission:: 里
    //   （定义在 mission_command.hpp）。不要给 kBridgeId* 加 air:: 前缀。
    sendEvent(air::kEvBridgePassed, kBridgeIdA, r.gate_row, 100);
  }
  if (r.just_passed_b)
  {
    RCLCPP_INFO(get_logger(), "[Mission] ★ 穿过桥洞区段二（K%d）—— 该区段 10 分",
                r.gate_row + 1);
    sendEvent(air::kEvBridgePassed, kBridgeIdB, r.gate_row, 100);
  }
  if (r.all_passed && !bridge_all_reported_)
  {
    bridge_all_reported_ = true;
    RCLCPP_INFO(get_logger(),
                "[Mission] ★★ 桥洞穿梭完成（两个区段都通过，共 20 分）");
    sendEvent(air::kEvBridgePassed, 0, 0, 100);   // bridge_id=0 = 全部完成
  }

  // 门位不完整时周期请求重发（无人机端可能漏发/丢包）
  if (!bridge_->ready())
  {
    const rclcpp::Time now_t = now();
    if ((now_t - bridge_last_request_time_).seconds() > 2.0)
    {
      bridge_last_request_time_ = now_t;
      ++bridge_request_count_;
      RCLCPP_WARN(get_logger(),
                  "[Mission] 桥洞信息不完整（%s），请求重发（第 %d 次）",
                  bridgeStateName(bridge_->state()), bridge_request_count_);
      sendEvent(air::kEvNeedGate, bridge_request_count_, 0, 0);
    }
  }
}

void MissionManager::handleSetHome(const MissionCommand &m)
{
  home_x_ = m.x;
  home_y_ = m.y;
  home_yaw_ = pose_yaw_;
  home_valid_ = true;
  RCLCPP_INFO(get_logger(), "[Mission] home 设置为 (%.2f, %.2f)", home_x_, home_y_);
}

// =============================================================================
// 状态迁移
// =============================================================================
void MissionManager::setState(State s, const std::string &reason)
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  if (state_ == s && reason == last_reason_)
  {
    return;
  }
  RCLCPP_INFO(get_logger(), "[Mission] %s -> %s  (%s)", stateName(state_), stateName(s),
              reason.c_str());
  state_ = s;
  last_reason_ = reason;
}

// =============================================================================
// 导航
// =============================================================================
void MissionManager::sendNavGoal(double x, double y, double yaw, bool has_yaw)
{
  if (!nav_client_)
  {
    return;
  }

  // 关键：绝不能在 50Hz 定时器里阻塞等待，否则会饿死 arbiter_->tick()，
  // 仲裁器停止发布速度 -> 下位机看到的是过期的速度流。
  if (!nav_client_->action_server_is_ready())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                         "[Mission] navigate_to_pose 动作服务未就绪（nav2 未启动？），"
                         "目标已记录，待服务就绪后重试");
    return;
  }

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, has_yaw ? yaw : pose_yaw_);
  goal.pose.pose.orientation = tf2::toMsg(q);

  // 回显目标，便于 RViz 观察
  geometry_msgs::msg::PoseStamped vis = goal.pose;
  pub_goal_->publish(vis);

  auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  opts.goal_response_callback =
      std::bind(&MissionManager::onNavGoalResponse, this, std::placeholders::_1);
  opts.result_callback = &MissionManager::onNavGoalResult;

  nav_goal_active_.store(true);
  nav_goal_rejected_.store(false);
  nav_client_->async_send_goal(goal, opts);
  goal_sent_time_ = now();
  goal_last_retry_time_ = now();

  RCLCPP_INFO(get_logger(), "[Mission] 下发导航目标 (%.2f, %.2f) yaw=%s",
              x, y, has_yaw ? "指定" : "保持");
}

void MissionManager::cancelNavGoal()
{
  if (nav_client_ && nav_goal_active_.load())
  {
    nav_client_->async_cancel_all_goals();
    nav_goal_active_.store(false);
  }
}

void MissionManager::onNavGoalResponse(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr handle)
{
  nav_goal_handle_ = handle;
  if (!handle)
  {
    nav_goal_rejected_.store(true);
    nav_goal_active_.store(false);
    RCLCPP_WARN(get_logger(), "[Mission] 导航目标被拒绝");
  }
}

void MissionManager::onNavGoalResult(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &r)
{
  (void)r;  // 结果由 checkArrival / 超时逻辑兜底处理
}

void MissionManager::checkArrival()
{
  if (!goal_valid_)
  {
    return;
  }
  const double d = distanceToGoal();
  if (d > arrive_dist_)
  {
    return;
  }

  if (goal_has_yaw_)
  {
    const double dyaw = std::fabs(normalizeAngle(pose_yaw_ - goal_yaw_));
    if (dyaw > deg2rad(arrive_yaw_tol_deg_))
    {
      return;
    }
  }

  RCLCPP_INFO(get_logger(), "[Mission] 到达目标 (%.2f, %.2f)，剩余 %.2fm", goal_x_, goal_y_, d);

  goal_valid_ = false;
  cancelNavGoal();

  // 交战中到达打击距离：保持交战，底盘停下继续瞄准
  if (state_ == State::kEngaging)
  {
    arbiter_->clearAllSources();
    return;
  }

  setState(State::kArrived, "goal reached");
}

double MissionManager::distanceToGoal() const
{
  return std::hypot(goal_x_ - pose_x_, goal_y_ - pose_y_);
}

// =============================================================================
// 反馈回传
// =============================================================================
void MissionManager::setFaultBit(uint32_t bit, bool on)
{
  if (bit == 0u)
  {
    return;
  }
  if (on)
  {
    fault_bits_ |= bit;
  }
  else
  {
    fault_bits_ &= ~bit;
  }
}

// 把故障位图按严重程度折算成单一故障码，并按节流上报
void MissionManager::reportFaults()
{
  uint8_t code = kFaultNone;
  if (fault_bits_ == kBitNone)
  {
    code = kFaultNone;
  }
  else if (fault_bits_ & kBitNoLocalization)
  {
    code = kFaultNoLocalization;          // 最严重：位置不可信
  }
  else if (fault_bits_ & kBitObstacle)
  {
    code = kFaultObstacleBlocked;
  }
  else if (fault_bits_ & kBitLowBattery)
  {
    code = kFaultLowBattery;
  }
  else if (fault_bits_ & kBitCmdLink)
  {
    code = kFaultCmdLinkLost;
  }
  else if (fault_bits_ & kBitRefereeLink)
  {
    code = kFaultRefereeLinkLost;
  }
  else if (fault_bits_ & kBitNoAmmo)
  {
    code = kFaultNoAmmo;
  }
  else if (fault_bits_ & kBitVisionLost)
  {
    code = kFaultVisionLost;
  }

  fault_ = code;

  // sendFault 内部：故障码变化时立即发送，未变化则 2s 节流
  const uint8_t detail = (code == kFaultLowBattery)
                             ? static_cast<uint8_t>(std::clamp(battery_pct_, 0.0, 255.0))
                             : 0;
  sendFault(code, detail);
}

void MissionManager::publishStatus()
{
  const RefereeBoard::Status rs = referee_->status();
  const TargetAimer::AimResult ar = aimer_->result();

  VehicleStatus vs;
  vs.seq = ++tx_seq_;
  vs.mission_state = static_cast<uint8_t>(state_);
  vs.fault = fault_;
  vs.self_hp = rs.self_hp;
  vs.ammo = rs.ammo;
  vs.hit_dir = rs.hit_dir;
  vs.shoot_count = rs.shoot_count;
  vs.battery_pct = static_cast<uint8_t>(std::clamp(battery_pct_, 0.0, 100.0));
  vs.x = pose_x_;
  vs.y = pose_y_;
  vs.yaw = pose_yaw_;
  vs.v = vel_v_;
  vs.w = vel_w_;

  sendReport(kReportStatus, vs);

  // 交战过程中目标消失或命中，回执一次
  if (state_ == State::kEngaging)
  {
    const bool hp_dropped = rs.target_hp < target_hp_at_engage_start_;
    const bool shots_made = rs.shoot_count > shots_at_engage_start_;

    if (hp_dropped && target_id_ != kTargetNone)
    {
      MissionCommand req;
      req.seq = tx_seq_;
      req.target_id = target_id_;
      sendTargetAck(kHitSuccess, req);
      RCLCPP_INFO(get_logger(), "[Mission] 判定命中：对方血量 %d -> %d",
                  target_hp_at_engage_start_, rs.target_hp);
      target_hp_at_engage_start_ = rs.target_hp;
      shots_at_engage_start_ = rs.shoot_count;
      target_id_ = kTargetNone;
      aimer_->ceaseFire();
    }
    else if (shots_made && target_id_ != kTargetNone &&
             rs.target_hp >= target_hp_at_engage_start_ &&
             (!ar.target_visible || ar.last_fire_age_s > 2.0))
    {
      MissionCommand req;
      req.seq = tx_seq_;
      req.target_id = target_id_;
      sendTargetAck(kHitMissed, req);
      shots_at_engage_start_ = rs.shoot_count;
    }
  }
}

void MissionManager::sendReport(uint8_t cmd, const VehicleStatus &st)
{
  uint8_t buf[64];
  size_t n = 0;
  if (cmd == kReportStatus)
  {
    n = encodeVehicleStatus(st, buf, sizeof(buf));
  }
  if (n > 0 && cmd_link_)
  {
    cmd_link_->sendFrame(buf, n);
  }
}

void MissionManager::sendTargetAck(uint8_t result, const MissionCommand &req)
{
  TargetAck a;
  a.seq = req.seq;
  a.target_id = req.target_id;
  a.result = result;
  a.ammo_left = static_cast<uint8_t>(referee_->ammo());
  a.aim_x = pose_x_;
  a.aim_y = pose_y_;

  uint8_t buf[64];
  const size_t n = encodeTargetAck(a, buf, sizeof(buf));
  if (n > 0 && cmd_link_)
  {
    cmd_link_->sendFrame(buf, n);
  }
}

void MissionManager::sendFault(uint8_t fault, uint8_t detail)
{
  const rclcpp::Time now = this->now();
  const bool changed = (last_fault_code_ != fault);
  const bool due = (now - last_fault_tx_time_).seconds() > 2.0;

  if (!changed && !due)
  {
    return;  // 同一故障 2s 内不重复刷链路
  }
  last_fault_code_ = fault;
  last_fault_tx_time_ = now;

  uint8_t buf[64];
  const size_t n = encodeFault(fault, detail, buf, sizeof(buf));
  if (n > 0 && cmd_link_)
  {
    cmd_link_->sendFrame(buf, n);
  }
  if (fault != kFaultNone && changed)
  {
    RCLCPP_WARN(get_logger(), "[Mission] 上报故障码 %u (detail=%u)", fault, detail);
  }
}

// =============================================================================
// 回调
// =============================================================================
void MissionManager::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  pose_x_ = msg->pose.pose.position.x;
  pose_y_ = msg->pose.pose.position.y;
  pose_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
  pose_valid_ = true;
  pose_stamp_ = now();
}

void MissionManager::dztrackCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  dztrack_v_ = msg->linear.x;
  dztrack_w_ = msg->angular.z;
  dztrack_stamp_ = now();
}

void MissionManager::nav2Callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  nav2_v_ = msg->linear.x;
  nav2_w_ = msg->angular.z;
  nav2_stamp_ = now();
  nav2_seen_ = true;
}

void MissionManager::teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  (void)msg;
  teleop_stamp_ = now();
}

void MissionManager::batteryCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  battery_pct_ = static_cast<double>(msg->data) * 100.0;
}

void MissionManager::locHealthyCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  loc_healthy_ = msg->data != 0;
}

// =============================================================================
// FIELD_FIXED（场地固定结构）-> 无人机起降点设为禁入格
// =============================================================================
// ★ 为什么需要：
//   无人机起飞前与降落后停在场上两个固定格（现场图标注 M9 / N2）。
//   地面车若规划路径经过就会撞机，而"撞固定场地持续超 10s 裁判切手动"
//   是硬规则 —— 所以这两格必须禁入。
//
// ★ 载荷布局（v2.1，12 字节；旧版 8 字节）：
//     [0] up_row  [1] col_lo  [2] col_hi  [3] bridge_col
//     [4] a_lo    [5] a_hi    [6] b_lo    [7] b_hi
//     [8] drone_start_col  [9] drone_start_row     <-- v2.1 新增
//    [10] drone_end_col   [11] drone_end_row      <-- v2.1 新增
//
// ★ 向后兼容：若只收到 8 字节（旧无人机端），保留 Spec 里的默认起降点，
//   不报错 —— 默认值本来就与现场图一致。
// =============================================================================
void MissionManager::fieldFixedCallback(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  applyFieldFixed(msg->data.data(), msg->data.size());
}

// FIELD_FIXED 的**唯一实现** —— 话题回调与指令分派都走这里
void MissionManager::applyFieldFixed(const uint8_t *d, size_t n)
{
  const auto &sp = grid_->spec();

  // 前 8 字节是场地结构；本车端已固化，只校验不改（改了反而危险）
  if (n >= 8)
  {
    if (d[0] != static_cast<uint8_t>(sp.barrier_up_row) ||
        d[3] != static_cast<uint8_t>(sp.bridge_col))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "[Mission] FIELD_FIXED 与本车端固化结构不一致"
                           "（线 row=%u 期望 %d；桥洞列=%u 期望 %d）—— 以本车端为准",
                           d[0], sp.barrier_up_row, d[3], sp.bridge_col);
    }
  }

  // 无人机起降点：需要 v2.1 的 12 字节
  if (n >= 12)
  {
    const int sc = d[8];
    const int sr = d[9];
    const int ec = d[10];
    const int er = d[11];
    if (grid_->setDroneCells(sc, sr, ec, er))
    {
      field_fixed_applied_ = true;
      RCLCPP_INFO(get_logger(),
                  "[Mission] ★ 无人机起降点已设为禁入格：起点 %s、终点 %s"
                  "（地面车规划将避开这两格，避免撞机）",
                  FieldGrid::cellName(sc, sr).c_str(),
                  FieldGrid::cellName(ec, er).c_str());
    }
    else
    {
      RCLCPP_WARN(get_logger(),
                  "[Mission] ★ FIELD_FIXED 的无人机起降点非法（(%d,%d)/(%d,%d)）"
                  "—— 越界或与起终点重合，已忽略，保留默认 %s / %s",
                  sc, sr, ec, er,
                  FieldGrid::cellName(sp.drone_start_col, sp.drone_start_row).c_str(),
                  FieldGrid::cellName(sp.drone_end_col, sp.drone_end_row).c_str());
    }
  }
  else if (!field_fixed_applied_)
  {
    // 旧版 8 字节帧：用 Spec 默认值，明确告知一次
    field_fixed_applied_ = true;
    RCLCPP_INFO(get_logger(),
                "[Mission] FIELD_FIXED 为旧版 %zu 字节（无起降点字段）—— "
                "沿用默认禁入格 %s / %s",
                n,
                FieldGrid::cellName(sp.drone_start_col, sp.drone_start_row).c_str(),
                FieldGrid::cellName(sp.drone_end_col, sp.drone_end_row).c_str());
  }
}

void MissionManager::handleFieldFixed(const MissionCommand &m)
{
  applyFieldFixed(m.payload.data(), m.payload.size());
}

// =============================================================================
// 工具
// =============================================================================
double MissionManager::normalizeAngle(double a) const
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace dzmission
