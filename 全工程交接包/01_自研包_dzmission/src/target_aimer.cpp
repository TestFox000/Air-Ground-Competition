#include "dzmission/target_aimer.hpp"
// ★ 必须显式 include：kTargetNone 定义在 mission_command.hpp（dzmission 命名空间）。
//   之前没有它，只在用到 kTargetNone 的那一行报 "not declared in this scope"。
//   mission_manager.cpp 能用到是因为它经 cmd_link.hpp 间接获得 —— 不要依赖
//   这种间接包含（上游头文件一改动就会连带失败）。
#include "dzmission/mission_command.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

namespace dzmission
{

namespace
{
int clampi(int v, int lo, int hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

TargetAimer::TargetAimer(rclcpp::Node *node, const std::string &ns) : node_(node)
{
  (void)ns;
  last_vision_time_ = node_->now();
  last_scan_step_time_ = node_->now();
  laser_start_time_ = node_->now();
  last_fire_time_ = node_->now();
  last_feedback_time_ = node_->now();
}

void TargetAimer::loadParams()
{
  const std::string p = "aim.";

  node_->declare_parameter(p + "own_gimbal", own_gimbal_);
  node_->declare_parameter(p + "vision_topic", vision_topic_);
  node_->declare_parameter(p + "gimbal_feedback_topic", gimbal_feedback_topic_);
  node_->declare_parameter(p + "gimbal_cmd_topic", gimbal_cmd_topic_);
  node_->declare_parameter(p + "laser_topic", laser_topic_);

  node_->declare_parameter(p + "gimbal_center_tick", gimbal_center_tick_);
  node_->declare_parameter(p + "gimbal_tick_min", gimbal_tick_min_);
  node_->declare_parameter(p + "gimbal_tick_max", gimbal_tick_max_);
  node_->declare_parameter(p + "gimbal_max_speed", gimbal_max_speed_);
  node_->declare_parameter(p + "yaw_ticks_per_deg", yaw_ticks_per_deg_);
  node_->declare_parameter(p + "pitch_ticks_per_deg", pitch_ticks_per_deg_);
  node_->declare_parameter(p + "yaw_deg_sign", yaw_deg_sign_);
  node_->declare_parameter(p + "pitch_deg_sign", pitch_deg_sign_);

  node_->declare_parameter(p + "scan_yaw_dirs", scan_yaw_dirs_);
  node_->declare_parameter(p + "scan_yaw_min_deg", scan_yaw_min_deg_);
  node_->declare_parameter(p + "scan_yaw_max_deg", scan_yaw_max_deg_);
  node_->declare_parameter(p + "scan_pitch_levels_deg", scan_pitch_levels_deg_);
  node_->declare_parameter(p + "scan_dwell_ms", scan_dwell_ms_);
  node_->declare_parameter(p + "scan_arrive_tol", scan_arrive_tol_);
  node_->declare_parameter(p + "scan_speed", scan_speed_);
  node_->declare_parameter(p + "pitch_scan_speed", pitch_scan_speed_);

  // ★ 扫描模式与丢帧宽限/开火保持窗口（借鉴自学员工程，见头文件注释）
  node_->declare_parameter(p + "scan_mode", scan_mode_);
  node_->declare_parameter(p + "scan_sweep_rate_dps", scan_sweep_rate_dps_);
  node_->declare_parameter(p + "scan_hold_ms", scan_hold_ms_);
  node_->declare_parameter(p + "scan_sweep_multi_pitch", scan_sweep_multi_pitch_);
  node_->declare_parameter(p + "lost_grace_ms", lost_grace_ms_);
  node_->declare_parameter(p + "fire_hold_px", fire_hold_px_);

  node_->declare_parameter(p + "vision_smooth_alpha", vision_smooth_alpha_);
  node_->declare_parameter(p + "fast_alpha", fast_alpha_);
  node_->declare_parameter(p + "fast_error_threshold_px", fast_error_threshold_px_);
  node_->declare_parameter(p + "vision_step_gain", vision_step_gain_);
  node_->declare_parameter(p + "fast_step_gain", fast_step_gain_);
  node_->declare_parameter(p + "max_step_per_frame", max_step_per_frame_);
  node_->declare_parameter(p + "fast_max_step", fast_max_step_);
  node_->declare_parameter(p + "vision_deadband_px", vision_deadband_px_);
  node_->declare_parameter(p + "pitch_deadband_px", pitch_deadband_px_);
  node_->declare_parameter(p + "pitch_step_gain", pitch_step_gain_);
  node_->declare_parameter(p + "pitch_max_step_per_frame", pitch_max_step_per_frame_);
  node_->declare_parameter(p + "vision_timeout_ms", vision_timeout_ms_);
  node_->declare_parameter(p + "lost_frames_to_rescan", lost_frames_to_rescan_);

  node_->declare_parameter(p + "motor_kp", motor_kp_);
  node_->declare_parameter(p + "fast_motor_kp", fast_motor_kp_);
  node_->declare_parameter(p + "pitch_motor_kp", pitch_motor_kp_);
  node_->declare_parameter(p + "pitch_max_speed", pitch_max_speed_);
  node_->declare_parameter(p + "position_deadband_ticks", position_deadband_ticks_);
  node_->declare_parameter(p + "pitch_position_deadband_ticks", pitch_position_deadband_ticks_);

  node_->declare_parameter(p + "fire_tolerance_px", fire_tolerance_px_);
  node_->declare_parameter(p + "fire_debounce_frames", fire_debounce_frames_);
  node_->declare_parameter(p + "fire_cooldown_ms", fire_cooldown_ms_);
  node_->declare_parameter(p + "fire_duration_ms", fire_duration_ms_);
  node_->declare_parameter(p + "require_stationary", require_stationary_);
  node_->declare_parameter(p + "min_ammo_to_fire", min_ammo_to_fire_);
  node_->declare_parameter(p + "max_engage_range_m", max_engage_range_m_);
  node_->declare_parameter(p + "enable_auto_scan", enable_auto_scan_);

  // ★ 红蓝面联锁
  node_->declare_parameter(p + "require_blue_face", require_blue_face_);
  node_->declare_parameter(p + "face_unknown_policy", face_unknown_policy_);
  node_->declare_parameter(p + "face_debounce_frames", face_debounce_frames_);
  // ★ 激光红点联锁
  node_->declare_parameter(p + "require_laser_dot", require_laser_dot_);
  node_->declare_parameter(p + "laser_dot_required_after_first",
                           laser_dot_required_after_first_);
  node_->declare_parameter(p + "laser_dot_debounce_frames",
                           laser_dot_debounce_frames_);
  // ★ 靶标击毁
  node_->declare_parameter(p + "target_hp_destroyed", target_hp_destroyed_);
  // ★ 移动靶（换向点开火）
  node_->declare_parameter(p + "require_turn_around", require_turn_around_);
  node_->declare_parameter(p + "turn_around_v_thresh_px_s",
                           turn_around_v_thresh_px_s_);
  node_->declare_parameter(p + "turn_around_dwell_ms", turn_around_dwell_ms_);
  node_->declare_parameter(p + "turn_around_velocity_alpha",
                           turn_around_velocity_alpha_);
  node_->declare_parameter(p + "turn_around_v_soft_ratio",
                           turn_around_v_soft_ratio_);
  node_->declare_parameter(p + "turn_around_window_ms", turn_around_window_ms_);
  node_->declare_parameter(p + "turn_around_vel_history", turn_around_vel_history_);
  // ★ 弹量纪律
  node_->declare_parameter(p + "ammo_link_lost_policy", ammo_link_lost_policy_);
  node_->declare_parameter(p + "ammo_link_lost_allowance",
                           ammo_link_lost_allowance_);
  node_->declare_parameter(p + "max_local_shots", max_local_shots_);

  node_->get_parameter(p + "own_gimbal", own_gimbal_);
  node_->get_parameter(p + "vision_topic", vision_topic_);
  node_->get_parameter(p + "gimbal_feedback_topic", gimbal_feedback_topic_);
  node_->get_parameter(p + "gimbal_cmd_topic", gimbal_cmd_topic_);
  node_->get_parameter(p + "laser_topic", laser_topic_);

  node_->get_parameter(p + "gimbal_center_tick", gimbal_center_tick_);
  node_->get_parameter(p + "gimbal_tick_min", gimbal_tick_min_);
  node_->get_parameter(p + "gimbal_tick_max", gimbal_tick_max_);
  node_->get_parameter(p + "gimbal_max_speed", gimbal_max_speed_);
  node_->get_parameter(p + "yaw_ticks_per_deg", yaw_ticks_per_deg_);
  node_->get_parameter(p + "pitch_ticks_per_deg", pitch_ticks_per_deg_);
  node_->get_parameter(p + "yaw_deg_sign", yaw_deg_sign_);
  node_->get_parameter(p + "pitch_deg_sign", pitch_deg_sign_);

  node_->get_parameter(p + "scan_yaw_dirs", scan_yaw_dirs_);
  node_->get_parameter(p + "scan_yaw_min_deg", scan_yaw_min_deg_);
  node_->get_parameter(p + "scan_yaw_max_deg", scan_yaw_max_deg_);
  node_->get_parameter(p + "scan_pitch_levels_deg", scan_pitch_levels_deg_);
  node_->get_parameter(p + "scan_dwell_ms", scan_dwell_ms_);
  node_->get_parameter(p + "scan_arrive_tol", scan_arrive_tol_);
  node_->get_parameter(p + "scan_speed", scan_speed_);
  node_->get_parameter(p + "pitch_scan_speed", pitch_scan_speed_);

  node_->get_parameter(p + "scan_mode", scan_mode_);
  node_->get_parameter(p + "scan_sweep_rate_dps", scan_sweep_rate_dps_);
  node_->get_parameter(p + "scan_hold_ms", scan_hold_ms_);
  node_->get_parameter(p + "scan_sweep_multi_pitch", scan_sweep_multi_pitch_);
  node_->get_parameter(p + "lost_grace_ms", lost_grace_ms_);
  node_->get_parameter(p + "fire_hold_px", fire_hold_px_);

  node_->get_parameter(p + "vision_smooth_alpha", vision_smooth_alpha_);
  node_->get_parameter(p + "fast_alpha", fast_alpha_);
  node_->get_parameter(p + "fast_error_threshold_px", fast_error_threshold_px_);
  node_->get_parameter(p + "vision_step_gain", vision_step_gain_);
  node_->get_parameter(p + "fast_step_gain", fast_step_gain_);
  node_->get_parameter(p + "max_step_per_frame", max_step_per_frame_);
  node_->get_parameter(p + "fast_max_step", fast_max_step_);
  node_->get_parameter(p + "vision_deadband_px", vision_deadband_px_);
  node_->get_parameter(p + "pitch_deadband_px", pitch_deadband_px_);
  node_->get_parameter(p + "pitch_step_gain", pitch_step_gain_);
  node_->get_parameter(p + "pitch_max_step_per_frame", pitch_max_step_per_frame_);
  node_->get_parameter(p + "vision_timeout_ms", vision_timeout_ms_);
  node_->get_parameter(p + "lost_frames_to_rescan", lost_frames_to_rescan_);

  node_->get_parameter(p + "motor_kp", motor_kp_);
  node_->get_parameter(p + "fast_motor_kp", fast_motor_kp_);
  node_->get_parameter(p + "pitch_motor_kp", pitch_motor_kp_);
  node_->get_parameter(p + "pitch_max_speed", pitch_max_speed_);
  node_->get_parameter(p + "position_deadband_ticks", position_deadband_ticks_);
  node_->get_parameter(p + "pitch_position_deadband_ticks", pitch_position_deadband_ticks_);

  node_->get_parameter(p + "fire_tolerance_px", fire_tolerance_px_);
  node_->get_parameter(p + "fire_debounce_frames", fire_debounce_frames_);
  node_->get_parameter(p + "fire_cooldown_ms", fire_cooldown_ms_);
  node_->get_parameter(p + "fire_duration_ms", fire_duration_ms_);
  node_->get_parameter(p + "require_stationary", require_stationary_);
  node_->get_parameter(p + "min_ammo_to_fire", min_ammo_to_fire_);
  node_->get_parameter(p + "max_engage_range_m", max_engage_range_m_);
  node_->get_parameter(p + "enable_auto_scan", enable_auto_scan_);

  node_->get_parameter(p + "require_blue_face", require_blue_face_);
  node_->get_parameter(p + "face_unknown_policy", face_unknown_policy_);
  node_->get_parameter(p + "face_debounce_frames", face_debounce_frames_);
  node_->get_parameter(p + "require_laser_dot", require_laser_dot_);
  node_->get_parameter(p + "laser_dot_required_after_first",
                       laser_dot_required_after_first_);
  node_->get_parameter(p + "laser_dot_debounce_frames",
                       laser_dot_debounce_frames_);
  node_->get_parameter(p + "target_hp_destroyed", target_hp_destroyed_);
  node_->get_parameter(p + "require_turn_around", require_turn_around_);
  node_->get_parameter(p + "turn_around_v_thresh_px_s",
                       turn_around_v_thresh_px_s_);
  node_->get_parameter(p + "turn_around_dwell_ms", turn_around_dwell_ms_);
  node_->get_parameter(p + "turn_around_velocity_alpha",
                       turn_around_velocity_alpha_);
  node_->get_parameter(p + "turn_around_v_soft_ratio",
                       turn_around_v_soft_ratio_);
  node_->get_parameter(p + "turn_around_window_ms", turn_around_window_ms_);
  node_->get_parameter(p + "turn_around_vel_history", turn_around_vel_history_);
  node_->get_parameter(p + "ammo_link_lost_policy", ammo_link_lost_policy_);
  node_->get_parameter(p + "ammo_link_lost_allowance",
                       ammo_link_lost_allowance_);
  node_->get_parameter(p + "max_local_shots", max_local_shots_);

  // 参数整理
  if (vision_smooth_alpha_ < 0.05 || vision_smooth_alpha_ > 1.0) vision_smooth_alpha_ = 0.4;
  if (fast_alpha_ < 0.05 || fast_alpha_ > 1.0) fast_alpha_ = 0.85;
  if (max_step_per_frame_ < 1) max_step_per_frame_ = 1;
  if (fast_max_step_ < 1) fast_max_step_ = 1;
  if (scan_yaw_dirs_ < 1) scan_yaw_dirs_ = 1;
  if (pitch_max_speed_ > gimbal_max_speed_) pitch_max_speed_ = gimbal_max_speed_;
  if (pitchLevelCount() < 1) scan_pitch_levels_deg_ = {0.0};

  cmd_pos_yaw_ = gimbal_center_tick_;
  cmd_pos_pitch_ = gimbal_center_tick_;
  fb_pos_yaw_ = gimbal_center_tick_;
  fb_pos_pitch_ = gimbal_center_tick_;

  sub_vision_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
      vision_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&TargetAimer::visionCallback, this, std::placeholders::_1));

  sub_feedback_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
      gimbal_feedback_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&TargetAimer::feedbackCallback, this, std::placeholders::_1));

  pub_gimbal_cmd_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>(
      gimbal_cmd_topic_, 10);
  pub_laser_ = node_->create_publisher<std_msgs::msg::UInt8>(laser_topic_, 10);
  pub_aim_state_ = node_->create_publisher<std_msgs::msg::UInt8>("aim_state", 5);

  RCLCPP_INFO(node_->get_logger(),
              "[Aimer] own_gimbal=%d vision=%s fb=%s cmd=%s laser=%s",
              static_cast<int>(own_gimbal_), vision_topic_.c_str(),
              gimbal_feedback_topic_.c_str(), gimbal_cmd_topic_.c_str(),
              laser_topic_.c_str());
  if (!own_gimbal_)
  {
    RCLCPP_WARN(node_->get_logger(),
                "[Aimer] own_gimbal=false: 云台与激光指令仍由 dzrknn_dataprocess 发布，"
                "本节点只做联锁与状态监视。");
  }
}

// -----------------------------------------------------------------------------
// 外部接口
// -----------------------------------------------------------------------------
void TargetAimer::requestEngage(uint8_t target_id)
{
  std::lock_guard<std::mutex> lk(mtx_);
  engage_target_id_.store(target_id);
  engaging_.store(target_id != 0);
  in_tolerance_count_ = 0;
  lost_target_count_ = 0;
  if (target_id != 0)
  {
    state_ = State::kScanning;
    last_scan_step_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(), "[Aimer] ENGAGE target_id=%u", target_id);
  }
  else
  {
    state_ = State::kIdle;
    laser_active_ = false;
    // 直接置位，不调用 centerGimbal()（它也会加锁，std::mutex 非递归）
    cmd_pos_yaw_ = clampTick(gimbal_center_tick_);
    cmd_pos_pitch_ = clampTick(gimbal_center_tick_);
    cmd_spd_yaw_ = scan_speed_;
    cmd_spd_pitch_ = pitch_scan_speed_;
    RCLCPP_INFO(node_->get_logger(), "[Aimer] ENGAGE off");
  }
}

void TargetAimer::ceaseFire()
{
  std::lock_guard<std::mutex> lk(mtx_);
  laser_active_ = false;
  in_tolerance_count_ = 0;
  if (state_ == State::kFiring || state_ == State::kAligned)
  {
    state_ = State::kTracking;
  }
  RCLCPP_INFO(node_->get_logger(), "[Aimer] CEASE FIRE");
}

void TargetAimer::abort()
{
  std::lock_guard<std::mutex> lk(mtx_);
  engaging_.store(false);
  engage_target_id_.store(0);
  laser_active_ = false;
  in_tolerance_count_ = 0;
  lost_target_count_ = 0;
  vision_smooth_ready_ = false;
  state_ = State::kIdle;
  // 同 requestEngage：不调用加锁的 centerGimbal()
  cmd_pos_yaw_ = clampTick(gimbal_center_tick_);
  cmd_pos_pitch_ = clampTick(gimbal_center_tick_);
  cmd_spd_yaw_ = scan_speed_;
  cmd_spd_pitch_ = pitch_scan_speed_;
}

// -----------------------------------------------------------------------------
// 回调
// -----------------------------------------------------------------------------
void TargetAimer::visionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 3)
  {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  center_x_ = msg->data[0];
  center_y_ = msg->data[1];
  class_id_ = msg->data[2];
  // 可选第 4 位：距离（单位 0.01 m）。视觉节点未提供时保持上次值。
  if (msg->data.size() >= 4 && msg->data[3] > 0)
  {
    last_target_range_m_ = static_cast<double>(msg->data[3]) * 0.01;
  }

  // ★★ 第 9 位：可见面颜色（rknn_pt/det_node v3 起提供）★★
  //   0 = 不确定 / 1 = 红(友方) / 2 = 蓝(敌方)
  //   去抖：连续 face_debounce_frames_ 帧看到同一面才采信，
  //   避免单帧误判导致"该打的不打、不该打的打了"。
  if (msg->data.size() >= 9)
  {
    const int f = msg->data[8];
    if (f == face_candidate_)
    {
      ++face_seen_count_;
    }
    else
    {
      face_candidate_ = f;
      face_seen_count_ = 1;
    }
    if (face_seen_count_ >= std::max(1, face_debounce_frames_))
    {
      face_ = face_candidate_;
    }
  }
  else
  {
    // 视觉节点未提供该字段（旧版 det_node）——保持"不确定"，
    // 这样默认策略（require_blue_face_ + policy 0）会自动禁止开火，
    // 不会因为缺字段而误打友方。
    face_ = 0;
    face_candidate_ = 0;
    face_seen_count_ = 0;
  }

  // ★★ 第 10 位：检测框内是否有激光红点（命中确认）★★
  if (msg->data.size() >= 10)
  {
    const bool dot = (msg->data[9] != 0);
    if (dot)
    {
      ++laser_dot_count_;
    }
    else
    {
      laser_dot_count_ = 0;
    }
    laser_dot_seen_ =
        (laser_dot_count_ >= std::max(1, laser_dot_debounce_frames_));
  }
  else
  {
    laser_dot_seen_ = false;
    laser_dot_count_ = 0;
  }

  target_visible_ = (class_id_ > 0);
  last_vision_time_ = node_->now();
}

void TargetAimer::feedbackCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2)
  {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  fb_pos_pitch_ = msg->data[0];
  fb_pos_yaw_ = msg->data[1];
  has_feedback_ = true;
  last_feedback_time_ = node_->now();
}

// -----------------------------------------------------------------------------
// 主循环
// -----------------------------------------------------------------------------
void TargetAimer::tick()
{
  std::lock_guard<std::mutex> lk(mtx_);

  const rclcpp::Time now = node_->now();

  // ---- 激光状态机（仅交战期间允许维持开火窗口） ----
  bool fire_now = false;
  if (laser_active_ && !engaging_.load())
  {
    // 非交战状态一律不允许激光输出
    laser_active_ = false;
    state_ = State::kIdle;
  }
  if (laser_active_)
  {
    const double active_ms = (now - laser_start_time_).seconds() * 1000.0;
    if (active_ms < static_cast<double>(fire_duration_ms_))
    {
      fire_now = true;
    }
    else
    {
      laser_active_ = false;
      if (state_ == State::kFiring)
      {
        state_ = State::kCooldown;
      }
    }
  }

  const double vision_age_ms = (now - last_vision_time_).seconds() * 1000.0;
  const bool vision_fresh = vision_age_ms < static_cast<double>(vision_timeout_ms_);
  // 只把"交战请求的那个目标类别"当作有效目标，不再硬编码 class 1
  const uint8_t want_id = engage_target_id_.load();
  const bool class_match = (want_id == kTargetNone)
                               ? (class_id_ > 0)
                               : (class_id_ == static_cast<int>(want_id));
  const bool vision_lost = !vision_fresh || !target_visible_ || !class_match;

  if (!engaging_.load())
  {
    state_ = State::kIdle;
    if (own_gimbal_)
    {
      centerGimbal();
      publishGimbal();
    }
    publishLaser(fire_now);
    cmd_spd_yaw_ = 0;
    cmd_spd_pitch_ = 0;
    std_msgs::msg::UInt8 st;
    st.data = static_cast<uint8_t>(state_);
    pub_aim_state_->publish(st);
    return;
  }

  // ---- 丢帧统计 ----
  if (vision_lost)
  {
    in_tolerance_count_ = 0;
    vision_smooth_ready_ = false;

    if (state_ == State::kTracking || state_ == State::kAligned || state_ == State::kFiring)
    {
      ++lost_target_count_;
      // ★ 丢帧宽限（按时间，不按帧数）：
      //   目标刚消失时**保持不动**，等视觉流水线追上，而不是立刻重新扫描。
      //   依据：学员两版的注释 —— 扫描速度超过检测延迟时，云台会扫过目标，
      //   导致"明明指着目标却报没看到"，于是永远扫不到。
      //   为什么用时间而不是帧数：视觉断流时帧计数永远不增加，会一直等下去；
      //   按时间则能可靠超时。
      const double since_seen_s =
          ever_seen_target_ ? (now - last_target_seen_time_).seconds() : 1e9;
      const bool within_grace = since_seen_s < (lost_grace_ms_ / 1000.0);
      const bool by_frames = lost_target_count_ >= lost_frames_to_rescan_;

      if (within_grace && !by_frames)
      {
        // 宽限期内：保持当前云台位置不动
        state_ = State::kTracking;
      }
      else
      {
        state_ = State::kScanning;
        lost_target_count_ = 0;
        last_scan_step_time_ = now;
        // 重新扫描时重置 sweep 状态，从归中位置重新开始
        sweep_initialized_ = false;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[Aimer] target lost (%.1fs) -> SCANNING", since_seen_s);
      }
    }
    else
    {
      state_ = (state_ == State::kCooldown) ? State::kCooldown : State::kScanning;
    }
  }
  else
  {
    lost_target_count_ = 0;
    // ★ 记录"最后一次真正看到目标"的时刻，供丢帧宽限使用
    last_target_seen_time_ = now;
    ever_seen_target_ = true;
    if (state_ == State::kScanning || state_ == State::kIdle || state_ == State::kCooldown)
    {
      state_ = State::kTracking;
    }
  }

  // ---- 状态行为 ----
  if (state_ == State::kScanning)
  {
    if (!enable_auto_scan_)
    {
      // 不扫描：保持归中等待视觉主动出现
      centerGimbal();
    }
    else
    {
      // ★ 两种扫描模式
      if (scan_mode_ == "sweep")
      {
        sweepScanTarget(now.seconds());
      }
      else
      {
        // step 模式：跳到下一个预置角度，停 dwell 等视觉（原行为）
        const double elapsed_ms = (now - last_scan_step_time_).seconds() * 1000.0;
        if (elapsed_ms >= static_cast<double>(scan_dwell_ms_))
        {
          const bool arrived =
              std::abs(fb_pos_yaw_ - cmd_pos_yaw_) <= scan_arrive_tol_ &&
              std::abs(fb_pos_pitch_ - cmd_pos_pitch_) <= scan_arrive_tol_;
          const bool timeout = elapsed_ms >= static_cast<double>(scan_dwell_ms_) * 3.0;
          if (arrived || timeout)
          {
            advanceScanIndex();
            last_scan_step_time_ = now;
          }
        }
        applyScanTarget();
      }
    }
    if (own_gimbal_)
    {
      publishGimbal();
    }
  }
  else
  {
    // TRACKING / ALIGNED / FIRING / COOLDOWN：视觉伺服
    const int abs_cx = std::abs(center_x_);
    const int abs_cy = std::abs(center_y_);
    const bool fast_mode =
        (abs_cx > fast_error_threshold_px_) || (abs_cy > fast_error_threshold_px_);

    const double alpha = fast_mode ? fast_alpha_ : vision_smooth_alpha_;
    const double step_gain = fast_mode ? fast_step_gain_ : vision_step_gain_;
    const int max_step = fast_mode ? fast_max_step_ : max_step_per_frame_;

    if (!vision_smooth_ready_)
    {
      vision_cx_filtered_ = static_cast<double>(center_x_);
      vision_cy_filtered_ = static_cast<double>(center_y_);
      vision_smooth_ready_ = true;
    }
    else
    {
      vision_cx_filtered_ = alpha * static_cast<double>(center_x_) +
                            (1.0 - alpha) * vision_cx_filtered_;
      vision_cy_filtered_ = alpha * static_cast<double>(center_y_) +
                            (1.0 - alpha) * vision_cy_filtered_;
    }

    double ex = vision_cx_filtered_;
    double ey = vision_cy_filtered_;
    if (std::abs(ex) < static_cast<double>(vision_deadband_px_)) ex = 0.0;
    if (std::abs(ey) < static_cast<double>(pitch_deadband_px_)) ey = 0.0;

    int step_yaw = static_cast<int>(std::lround(ex * step_gain));
    int step_pitch = static_cast<int>(std::lround(ey * pitch_step_gain_));
    step_yaw = clampi(step_yaw, -max_step, max_step);
    step_pitch = clampi(step_pitch, -pitch_max_step_per_frame_, pitch_max_step_per_frame_);

    cmd_pos_yaw_ = clampTick(fb_pos_yaw_ + step_yaw);
    cmd_pos_pitch_ = clampTick(fb_pos_pitch_ + step_pitch);

    const double kp_yaw = fast_mode ? fast_motor_kp_ : motor_kp_;
    cmd_spd_yaw_ = static_cast<int>(std::min(
        std::abs(kp_yaw * static_cast<double>(cmd_pos_yaw_ - fb_pos_yaw_)),
        static_cast<double>(gimbal_max_speed_)));
    cmd_spd_pitch_ = computeTrackingSpeed(cmd_pos_pitch_, fb_pos_pitch_, true);

    if (own_gimbal_)
    {
      publishGimbal();
    }

    // ---- 开火判定 ----
    // ★ 两个不同的容差（借鉴自学员工程，取其"对准 5px / 保持 15px"的分层）：
    //   fire_tolerance_px_  —— 进入开火态的**对准**容差（严格）
    //   fire_hold_px_       —— 开火态下"继续开火"的**保持**窗口（宽松）
    //   分开的理由：只有一个阈值时，目标在阈值边缘抖动会让开火↔停火反复切换，
    //   而全局限 10 发弹量，浪费不起。超出保持窗口才暂停开火并微调回中心。
    //   注：abs_cx / abs_cy 在本函数上方（视觉伺服段）已声明，此处复用。
    const bool in_tolerance =
        abs_cx <= fire_tolerance_px_ && abs_cy <= fire_tolerance_px_;
    const bool in_hold_window =
        abs_cx <= fire_hold_px_ && abs_cy <= fire_hold_px_;

    // 已经在开火态时，用宽松窗口维持；尚未开火时，必须先用严格容差证明对准
    const bool treat_as_aligned =
        (state_ == State::kFiring && in_hold_window) || in_tolerance;

    if (treat_as_aligned)
    {
      ++in_tolerance_count_;
      out_of_hold_count_ = 0;
    }
    else
    {
      in_tolerance_count_ = 0;
      ++out_of_hold_count_;
    }

    // =======================================================================
    // ★★ 移动靶"换向点"检测（方案 α 的核心）★★
    // -----------------------------------------------------------------------
    // 背景（见 doc/MOVING_TARGET_PLAN.md）：
    //   靶标沿轨道**匀速往返**，速度 v。视觉流水线有 **4 帧延迟 ≈ 267ms**
    //   （detect_rate_hz=15 → 帧周期 66ms）。靶标在延迟内的位移：
    //       位移 = v × 0.267s
    //   按固件参数推算 v ≈ 0.44 m/s → 位移 ≈ **11.7 cm**
    //   而靶标直径只有 **8~10 cm** → **位移 > 靶标尺寸**
    //   ⇒ 不做提前量补偿，开火时激光必然落在靶标**后面**。
    //
    // α 的规避办法：**只在靶标换向（速度过零）的瞬间开火**。
    //   换向瞬间 v ≈ 0，延迟造成的位移也 ≈ 0 → 无滞后误差。
    //   代价是每个往返周期只有 2 个换向点，但换来"不打偏"，
    //   在只有 10 发弹量、且"每 2 次有效命中才 +1 计数"的规则下，
    //   这个取舍是正确的。
    //
    // 实现：由视觉帧序列估计靶标横向速度（像素/秒），
    //       要求 |速度| 持续低于阈值达到 dwell 时间，且靶标在容差内。
    // =======================================================================
    const bool turn_around_fire = evaluateTurnAround(now, center_x_, center_y_);
    // 若未启用该策略，则视为"随时可打"（退化为原行为）
    const bool dwell_ok = (!require_turn_around_) || turn_around_fire;

    // ★★ 方案 β：提前量射击 ★★
    //   与 α 的关系是**互补 + 兜底**，不是替代：
    //     · β 条件满足（速度可估、靶标朝瞄准点运动、云台已稳）-> 用 β，射速高
    //     · β 条件不满足 -> 自动落回 α 的换向点，安全
    //   这样即使 β 的 T_total 标定不准，也**不会比 α 更差**。
    const bool lead_fire = evaluateLeadFire(now, center_x_, center_y_);
    // ★ 已删除未使用的 lead_used：它赋值后从未被读取（-Wunused-variable）。
    //   β 的统计用的是 lead_fire_count_ / lead_fallback_count_（见 evaluateLeadFire
    //   内部自增），不依赖这个变量。

    // 开火窗口：β 命中 或 α 换向点，二者取或
    const bool moving_ok = lead_fire || dwell_ok;

    const double since_last_fire_ms = (now - last_fire_time_).seconds() * 1000.0;
    const bool posture_ok = !require_stationary_ || chassis_stationary_;
    const bool not_hit = !being_hit_;
    // 射程联锁：仅在视觉提供了距离（>0）时才做限制，避免视觉无距离信息时误锁
    const bool range_ok = (last_target_range_m_ <= 0.0) ||
                          (last_target_range_m_ <= max_engage_range_m_);

    // =======================================================================
    // ★★ 红蓝面联锁（本赛题最关键的安全约束）★★
    // -----------------------------------------------------------------------
    // 规则：靶标双面结构，红=友方、蓝=敌方；**打友方扣分**。
    //   可见面 == 2(蓝) -> 允许
    //   可见面 == 1(红) -> 禁止（那是友方，打了扣分）
    //   可见面 == 0(不确定) -> 由 face_unknown_policy_ 决定：
    //        0 = 禁止（默认，最保守。误打友方扣 5 分/次，比漏打更贵）
    //        1 = 允许（仅当有其它来源确认了对面是敌方时才该这么设）
    // =======================================================================
    bool face_ok = true;
    if (require_blue_face_)
    {
      if (face_ == 2)
      {
        face_ok = true;
      }
      else if (face_ == 1)
      {
        face_ok = false;   // ★ 看到红面：绝对不打
      }
      else
      {
        face_ok = (face_unknown_policy_ == 1);
      }
    }

    // =======================================================================
    // ★★ 靶标击毁联锁 ★★
    //   规则：任一侧累计 3 次则该侧判定击毁；**正反共用同一靶标实体**，
    //   所以任一侧到 3 就是整体失效。继续打只是浪费弹量与赛时。
    // =======================================================================
    const int hp = target_hp_.load();
    const bool target_destroyed = (hp >= target_hp_destroyed_);

    // =======================================================================
    // ★★ 弹量纪律 ★★
    //   规则：共 10 发。裁判板会报 ammo；链路断时按 ammo_link_lost_policy_：
    //     0 = 保守，链路断即禁打
    //     1 = 乐观，按本地记账 10 - local_shots
    //     2 = 折中（推荐），允许再打 N 发，且**每发都要有反馈**
    // =======================================================================
    const bool link_ok = referee_link_ok_.load();
    const int board_ammo = ammo_.load();
    const int board_shots = board_shot_count_.load();

    if (!link_ok && !link_loss_baseline_valid_)
    {
      // 刚发现链路断：记下基准，供"是否有反馈"判据使用
      board_shot_at_link_loss_ = board_shots;
      target_hp_at_link_loss_ = hp;
      link_loss_baseline_valid_ = true;
      link_degraded_ = true;
      RCLCPP_WARN(node_->get_logger(),
                  "[Aimer] 裁判板链路断 -> 本地记账模式（已发 %d 发）",
                  local_shots_);
    }
    else if (link_ok && link_loss_baseline_valid_)
    {
      link_loss_baseline_valid_ = false;
      link_degraded_ = false;
      RCLCPP_INFO(node_->get_logger(), "[Aimer] 裁判板链路恢复");
    }

    int ammo_effective = 0;
    if (link_ok)
    {
      // 裁判板可用：信任裁判板，但**本地硬上限兜底**（防裁判板计数异常）
      ammo_effective = std::min(board_ammo,
                                std::max(0, max_local_shots_ - local_shots_));
    }
    else
    {
      switch (ammo_link_lost_policy_)
      {
        case 0:
          ammo_effective = 0;   // 保守：不打
          break;
        case 1:
          ammo_effective = std::max(0, max_local_shots_ - local_shots_);
          break;
        default:                // 2 = 折中
        {
          const int remaining_allowance =
              ammo_link_lost_allowance_ - local_shots_;
          // 是否已有**反馈**：裁判板计数增加 或 靶标血量增加
          const bool got_feedback =
              (board_shots > board_shot_at_link_loss_) ||
              (hp > target_hp_at_link_loss_);
          if (got_feedback || local_shots_ == 0)
          {
            // 有反馈（或还没打过第一发）-> 允许继续
            ammo_effective = std::min(
                remaining_allowance,
                std::max(0, max_local_shots_ - local_shots_));
          }
          else
          {
            // 打过了但没有任何反馈 -> 停止（要么没命中、要么链路真坏了）
            ammo_effective = 0;
          }
          break;
        }
      }
    }
    const bool ammo_ok = (ammo_effective >= min_ammo_to_fire_) && !target_destroyed;

    // =======================================================================
    // ★★ 激光红点联锁 ★★
    //   首次开火前激光还没打出去，当然看不到红点，所以本联锁只在
    //   "已经打过至少一发"之后才生效（laser_dot_required_after_first_）。
    //   它能确认**激光落点**确实在靶上，比"靶标在画面中心"可靠。
    // =======================================================================
    bool dot_ok = true;
    if (require_laser_dot_ && laser_dot_required_after_first_ &&
        local_shots_ > 0)
    {
      dot_ok = laser_dot_seen_;
    }

    // ---- 四重联锁汇总 ----
    const bool authorized =
        treat_as_aligned &&
        in_tolerance_count_ >= fire_debounce_frames_ &&
        since_last_fire_ms >= static_cast<double>(fire_cooldown_ms_) &&
        ammo_ok && posture_ok && not_hit && range_ok &&
        face_ok && dot_ok && !target_destroyed &&
        moving_ok;                   // ★ 移动靶：β 提前量 或 α 换向点

    if (!laser_active_ && authorized)
    {
      laser_active_ = true;
      laser_start_time_ = now;
      last_fire_time_ = now;
      in_tolerance_count_ = 0;
      ++shots_fired_;
      ++local_shots_;          // ★ 本地记账
      state_ = State::kFiring;
      fire_now = true;
      // ★ 记录本次走的是 β（提前量）还是 α（换向点），便于现场判断补偿是否生效
      if (lead_fire)
      {
        ++lead_fire_count_;
      }
      else
      {
        ++lead_fallback_count_;
      }
      RCLCPP_INFO(node_->get_logger(),
                  "[Aimer] FIRE #%d(local %d) cx=%d cy=%d face=%d dot=%d "
                  "ammo_eff=%d link=%d hp=%d range=%.2fm | %s v=%.0fpx/s "
                  "lead=%.0fpx T_total=%.0fms",
                  shots_fired_, local_shots_, center_x_, center_y_,
                  face_, static_cast<int>(laser_dot_seen_),
                  ammo_effective, static_cast<int>(link_ok), hp,
                  last_target_range_m_,
                  lead_fire ? "beta-lead" : "alpha-dwell",
                  last_track_velocity_px_s_, lead_offset_px_,
                  leadTotalMs());
    }
    else if (!laser_active_)
    {
      // ★ 逐条记录"为什么不开火"，这是现场调试最需要的信息
      if (target_destroyed)
      {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "[Aimer] 靶标已击毁(hp=%d>=%d)，停止交战",
                             hp, target_hp_destroyed_);
      }
      else if (!face_ok)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[Aimer] ★ 可见面为%s -> 禁止开火（打友方要扣分）",
                             face_ == 1 ? "红色(友方)" : "不确定");
      }
      else if (!dot_ok)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[Aimer] 未见激光红点 -> 暂不开火（确认落点中）");
      }
      else if (!ammo_ok)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "[Aimer] 弹量不足(有效 %d, 本地已发 %d, link=%d)",
                             ammo_effective, local_shots_,
                             static_cast<int>(link_ok));
      }

      // ★ 目标漂出"保持窗口"：暂停开火，退回对准（视觉伺服会把它拉回来）。
      //   这就是"层宽容差"的作用 —— 只有漂得比较远才停火，轻微抖动不停。
      if (state_ == State::kFiring && !in_hold_window)
      {
        state_ = State::kTracking;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[Aimer] drifted out of hold window (cx=%d cy=%d > %d px)"
                             " -> pause fire, re-aim",
                             center_x_, center_y_, fire_hold_px_);
      }
      else if (in_tolerance_count_ >= fire_debounce_frames_ && !authorized)
      {
        state_ = State::kAligned;
      }
      else if (since_last_fire_ms < static_cast<double>(fire_cooldown_ms_))
      {
        state_ = State::kCooldown;
      }
      else
      {
        state_ = State::kTracking;
      }
    }
  }

  publishLaser(fire_now);

  std_msgs::msg::UInt8 st;
  st.data = static_cast<uint8_t>(state_);
  pub_aim_state_->publish(st);
}

// -----------------------------------------------------------------------------
// ★ 移动靶"换向点"检测（方案 α）
// -----------------------------------------------------------------------------
// 思路：把"靶标在画面中的横向位置"当作一维时间序列 x(t)。
//   换向点 = v(t) = dx/dt 过零、且靶标处于容差内的时刻。
//
// 三个必要条件（全部满足才算"处于换向点"）：
//   1. 视觉帧新鲜（不是陈旧数据）
//   2. |v| 持续低于阈值（dwell 时间）—— 单帧速度过零不算，
//      因为噪声也会让速度符号抖一下
//   3. 靶标在画面容差内（否则即使停住了，激光也没指对）
//
// 为什么用**像素/秒**而不是直接判"位置到达极值"：
//   极值检测需要先看到两侧的极值才能确认，会漏掉第一个换向点；
//   速度阈值法不需要任何历史极值，第一次停顿时就能开火。
//
// 与"提前量射击"（方案 β）的关系：
//   β 会预测靶标何时到达瞄准点，允许在**运动中**开火；
//   α 只等它自然停下。α 的命中率更高（无滞后误差），
//   但射速低。两者是同一接口的不同实现，β 可在 α 之上叠加。
// -----------------------------------------------------------------------------
bool TargetAimer::evaluateTurnAround(const rclcpp::Time &now, int px, int py)
{
  if (!require_turn_around_)
  {
    return true;   // 未启用该策略 -> 退化为"随时可打"
  }

  // 视觉帧必须新鲜：陈旧数据的速度估计没有意义
  const double vision_age_ms = (now - last_vision_time_).seconds() * 1000.0;
  if (vision_age_ms > static_cast<double>(vision_timeout_ms_))
  {
    turn_around_dwell_start_ = now;   // 重置 dwell 计时
    last_track_velocity_px_s_ = 0.0;
    turn_around_armed_ = false;
    return false;
  }

  // 靶标必须还在容差内 —— 停住了但没指对，开火也是打偏
  if (std::abs(px) > fire_tolerance_px_ || std::abs(py) > fire_tolerance_px_)
  {
    turn_around_dwell_start_ = now;
    turn_around_armed_ = false;
    return false;
  }

  // ---- 速度估计（帧间差分 + 中值滤波）----
  // ★ 为什么用**中值**而不是一阶低通：
  //   量化验证发现一阶低通（alpha=0.35）滞后约 600ms，使换向后残余位移
  //   达 **7.1cm > 靶标半径 4.5cm** —— 等于没解决问题（见
  //   test/moving_target_check.py §2）。
  //   中值滤波对**孤立噪声**（检测框偶尔跳一下）免疫，
  //   而对**单调趋势**（靶标持续加速）几乎无滞后，正合此处需求。
  const rclcpp::Time prev = last_track_time_;
  const double dt = (now - prev).seconds();

  if (last_track_px_valid_ && dt > 1e-4)
  {
    const double v_raw = static_cast<double>(px - last_track_px_) / dt;
    const int cap = std::max(1, turn_around_vel_history_);
    if (static_cast<int>(vel_hist_.size()) < cap)
    {
      vel_hist_.push_back(v_raw);
    }
    else
    {
      vel_hist_[static_cast<size_t>(vel_hist_idx_)] = v_raw;
      vel_hist_idx_ = (vel_hist_idx_ + 1) % cap;
    }
    std::vector<double> tmp(vel_hist_);
    std::sort(tmp.begin(), tmp.end());
    const size_t n = tmp.size();
    last_track_velocity_px_s_ =
        (n % 2 == 1) ? tmp[n / 2] : 0.5 * (tmp[n / 2 - 1] + tmp[n / 2]);
    turn_around_velocity_ready_ = true;
  }
  last_track_px_ = px;
  last_track_time_ = now;
  last_track_px_valid_ = true;

  const double abs_v = std::abs(last_track_velocity_px_s_);

  // ★★ 方向反转检测（主判据）★★
  // -----------------------------------------------------------------------
  // 为什么要用"方向反转"而不是"幅值低于阈值"：
  //   §2 的仿真发现，一阶低通滤波（alpha=0.35）有 ~600ms 的滞后：
  //   靶标真实速度已在 t=800ms 过零，但滤波后的估计值仍有 66.8 px/s，
  //   结果**真换向一次都没被识别**（可开火帧数 0/60）。
  //
  //   物理上"换向"就是**速度变号**，而变号是比幅值更稳健的特征：
  //   幅值会被滤波器拖尾，符号变化不会（它只需要趋势翻转）。
  //   所以：当估计速度的符号翻转、且此刻|速度|已经不大时，
  //   就认为发生了换向，进入一个短促的开火窗口。
  //
  // 为什么还要看幅值：仅凭变号会把噪声抖动也当成换向。
  //   加上"翻转时 |v| 已经衰减到某一水平"这一条（用宽松的第二阈值
  //   turn_around_v_soft_ratio × 硬阈值），就能把两者区分开。
  // -----------------------------------------------------------------------
  const bool have_prev = turn_around_velocity_ready_;
  const bool sign_flip = have_prev &&
                         (last_track_velocity_px_s_ * turn_around_prev_v_) < 0.0;
  const double soft_thresh = turn_around_v_thresh_px_s_ * turn_around_v_soft_ratio_;

  if (sign_flip && abs_v <= soft_thresh)
  {
    // 本次构成一次换向：开始/刷新开火窗口
    turn_around_reversal_time_ = now;
    turn_around_reversal_seen_ = true;
  }
  turn_around_prev_v_ = last_track_velocity_px_s_;

  // 开火窗口：换向发生后保持 turn_around_window_ms 时间
  if (turn_around_reversal_seen_)
  {
    const double since_ms =
        (now - turn_around_reversal_time_).seconds() * 1000.0;
    if (since_ms <= static_cast<double>(turn_around_window_ms_))
    {
      turn_around_armed_ = false;   // 窗口内不依赖 dwell
      return true;                  // ★ 换向窗口内 -> 可开火
    }
    turn_around_reversal_seen_ = false;
  }

  // ★ 兜底判据：幅值法（靶标本来就静止、或速度极低时用）
  if (abs_v <= turn_around_v_thresh_px_s_)
  {
    if (!turn_around_armed_)
    {
      turn_around_armed_ = true;
      turn_around_dwell_start_ = now;
    }
    const double dwell_s = (now - turn_around_dwell_start_).seconds();
    return dwell_s >= (turn_around_dwell_ms_ / 1000.0);
  }

  // 速度还高：重新计 dwell
  turn_around_armed_ = false;
  turn_around_dwell_start_ = now;
  return false;
}

// -----------------------------------------------------------------------------
// ★★ 方案 β：提前量射击 ★★
// -----------------------------------------------------------------------------
double TargetAimer::leadTotalMs() const
{
  const double frame_ms = (detect_rate_hz_ > 0.1) ? (1000.0 / detect_rate_hz_)
                                                  : 66.0;
  const double pipe_ms = static_cast<double>(lead_pipeline_frames_) * frame_ms;
  return pipe_ms + lead_exposure_ms_ + lead_servo_ms_ + lead_fire_ms_;
}

// 判据：靶标将要在"我开火后 T_total"的时刻经过瞄准点（画面中心）
//
// 推导（见 test/moving_target_beta_check.py §3）：
//   设靶标当前在画面横坐标 px、像素速度 v(px/s)，云台已指向瞄准点(px=0)。
//   若现在开火，T_total 后激光到达，此时靶标位置 = px + v*T_total。
//   要命中需 px + v*T_total ≈ 0，即要在 **|px| ≈ |v|*T_total** 时开火，
//   且靶标**正在朝瞄准点运动**（px 与 v 符号相反）。
//
//   ★ 这就是提前量的物理含义：激光要 T_total 才到达，
//     所以必须瞄准靶标**将要到达**的位置，而不是它现在的位置。
//
// 四道门（全满足才用 β）：
//   1. lead_enable_
//   2. 速度估计置信度够（有效观测帧数 >= lead_min_confidence_）
//   3. 靶标接近画面中心（透视在边缘使像素速度失真）
//   4. 靶标正在朝瞄准点运动（否则等它换向，交给 α）
// 兜底：连续未命中 lead_max_misses_ 次后退回 α。
bool TargetAimer::evaluateLeadFire(const rclcpp::Time &now, int px, int py)
{
  lead_active_ = false;
  lead_offset_px_ = 0.0;

  if (!lead_enable_)
  {
    return false;
  }
  // ★ 兜底：连续打空太多 -> 补偿有系统偏差，退回 α 的换向点策略
  if (lead_miss_streak_ >= std::max(1, lead_max_misses_))
  {
    return false;
  }
  // 视觉必须新鲜
  const double vision_age_ms = (now - last_vision_time_).seconds() * 1000.0;
  if (vision_age_ms > static_cast<double>(vision_timeout_ms_))
  {
    return false;
  }
  // 速度估计置信度
  if (static_cast<int>(vel_hist_.size()) < std::max(1, lead_min_confidence_))
  {
    return false;
  }
  // 靶标必须接近画面中心（透视失真控制）
  if (std::abs(px) > lead_trust_center_px_)
  {
    return false;
  }
  // ★ 垂直方向准入：靶标只水平移动，所以**不做**垂直提前量；
  //   但必须确认它当前在垂直容差内 —— 否则即使水平预测命中，
  //   激光也会偏上/偏下打空（云台俯仰来不及在 T_total 内修正）。
  //   ★ 为什么用 fire_tolerance_px_ 而不是 lead_aim_tolerance_px_：
  //     fire_tolerance_px_ 才是"何时允许进入开火态"的定义（见 result() 里
  //     fire_authorized 同时约束 center_x/center_y）。β 在水平上比它更严
  //     （用 lead_aim_tolerance_px_ 判提前量），垂直上没有更严的理由，
  //     用同一把尺子最自洽，也避免 YAML 调参时出现意外的过严。
  if (std::abs(py) > fire_tolerance_px_)
  {
    return false;
  }

  const double v = last_track_velocity_px_s_;
  const double t_s = leadTotalMs() / 1000.0;
  const double offset = v * t_s;      // 提前量（像素，带符号）

  // ★ 要求靶标"正在朝瞄准点运动"：
  //   px > 0（在右边）且 v < 0（向左）  -> 朝中心运动
  //   px < 0（在左边）且 v > 0（向右）  -> 朝中心运动
  //   否则它在远离中心，等它换向更合适（交给 α）
  const bool approaching = (px > 0 && v < 0.0) || (px < 0 && v > 0.0);
  if (!approaching)
  {
    return false;
  }

  lead_offset_px_ = offset;
  lead_active_ = true;

  // ★ 命中判据：现在开火，T_total 后靶标位置应为 0
  //   预测位置 = px + v*T_total
  const double predicted = static_cast<double>(px) + offset;
  const double tol = static_cast<double>(std::max(1, lead_aim_tolerance_px_));
  return std::abs(predicted) <= tol;
}

// -----------------------------------------------------------------------------
void TargetAimer::publishGimbal()
{
  std_msgs::msg::Int32MultiArray cmd;
  cmd.data.resize(4);
  cmd.data[0] = cmd_pos_pitch_;
  cmd.data[1] = cmd_pos_yaw_;
  cmd.data[2] = cmd_spd_pitch_;
  cmd.data[3] = cmd_spd_yaw_;
  pub_gimbal_cmd_->publish(cmd);
}

void TargetAimer::publishLaser(bool on)
{
  std_msgs::msg::UInt8 m;
  m.data = on ? 1 : 0;
  pub_laser_->publish(m);
}

// -----------------------------------------------------------------------------
// 扫描
// -----------------------------------------------------------------------------
int TargetAimer::pitchLevelCount() const
{
  return scan_pitch_levels_deg_.empty() ? 1
                                        : static_cast<int>(scan_pitch_levels_deg_.size());
}

double TargetAimer::scanYawDeg() const
{
  if (scan_yaw_dirs_ <= 1)
  {
    return scan_yaw_min_deg_;
  }
  const double step = (scan_yaw_max_deg_ - scan_yaw_min_deg_) /
                      static_cast<double>(scan_yaw_dirs_ - 1);
  return scan_yaw_min_deg_ + step * static_cast<double>(scan_yaw_idx_);
}

double TargetAimer::scanPitchDeg() const
{
  if (scan_pitch_levels_deg_.empty())
  {
    return 0.0;
  }
  const int idx = std::min(scan_pitch_idx_, pitchLevelCount() - 1);
  return scan_pitch_levels_deg_[static_cast<size_t>(idx)];
}

void TargetAimer::applyScanTarget()
{
  const int yaw_tick = gimbal_center_tick_ +
                       static_cast<int>(std::lround(scanYawDeg() * yaw_ticks_per_deg_ *
                                                    yaw_deg_sign_));
  const int pitch_tick = gimbal_center_tick_ +
                         static_cast<int>(std::lround(scanPitchDeg() * pitch_ticks_per_deg_ *
                                                      pitch_deg_sign_));
  cmd_pos_yaw_ = clampTick(yaw_tick);
  cmd_pos_pitch_ = clampTick(pitch_tick);
  cmd_spd_yaw_ = scan_speed_;
  cmd_spd_pitch_ = pitch_scan_speed_;
}

void TargetAimer::advanceScanIndex()
{
  const int next_yaw = scan_yaw_idx_ + scan_yaw_dir_;
  if (next_yaw >= 0 && next_yaw < scan_yaw_dirs_)
  {
    scan_yaw_idx_ = next_yaw;
    return;
  }
  scan_yaw_dir_ = -scan_yaw_dir_;
  scan_pitch_idx_ = (scan_pitch_idx_ + 1) % pitchLevelCount();
  scan_yaw_idx_ = (scan_yaw_dir_ > 0) ? 0 : (scan_yaw_dirs_ - 1);
}

void TargetAimer::centerGimbal()
{
  cmd_pos_yaw_ = clampTick(gimbal_center_tick_);
  cmd_pos_pitch_ = clampTick(gimbal_center_tick_);
  cmd_spd_yaw_ = scan_speed_;
  cmd_spd_pitch_ = pitch_scan_speed_;
}

// -----------------------------------------------------------------------------
// ★ 连续慢扫模式（sweep）
// -----------------------------------------------------------------------------
// 为什么需要这个模式：
//   学员两版的注释都指出了同一个失败模式 ——
//     "the sweep outran the detection latency, so the target was never
//      reported while the gimbal was actually pointing at it."
//   即扫描速度超过了视觉流水线的延迟：云台扫过一个目标时，检测帧还没算完，
//   于是系统认为"没看到"，继续往前扫，**永远扫不到**。
//
//   跳档式（step）每 300ms 跳一格就已经很快了；但真正的风险是
//   跳档后立刻评估"到了吗"，而视觉还没反应。所以 sweep 模式的思路是：
//   以**可控的角速度**连续推进，并在换向点停顿 hold 一段时间让视觉追上。
//
// 与 step 模式的分工：
//   step  —— 预置方向离散、扫得快、适合已知大致方位
//   sweep —— 连续覆盖、速度可调、适合"完全不知道靶在哪"
// -----------------------------------------------------------------------------
void TargetAimer::sweepScanTarget(double now_s)
{
  if (!sweep_initialized_)
  {
    // 从归中位置开始，方向按上一次的朝向
    sweep_yaw_deg_ = 0.0;
    sweep_pitch_deg_ = scanPitchDeg();
    sweep_dir_ = 1;
    sweep_initialized_ = true;
    last_sweep_tick_ = node_->now();
    sweep_hold_until_ = node_->now();
  }

  const rclcpp::Time now = node_->now();

  // 换向点停顿：让视觉流水线追上（这是关键，不能省）
  if (now < sweep_hold_until_)
  {
    applyScanTarget();          // 保持当前目标位置
    cmd_spd_yaw_ = scan_speed_;
    cmd_spd_pitch_ = pitch_scan_speed_;
    // ★ 必须同步刷新时间基准：否则 hold 结束后 dt 会累加成一大值，
    //   造成单帧跳跃（虽然下面有 per_tick_cap 兜底，但那是最后手段）
    last_sweep_tick_ = now;
    return;
  }

  // dt 取上界保护：定时器抖动/线程调度延迟都可能让 dt 异常大
  double dt = std::max(0.0, now_s - last_sweep_tick_.seconds());
  dt = std::min(dt, 0.1);      // 单次积分不超过 100ms
  last_sweep_tick_ = now;

  // 单帧推进量受两个上界约束：
  //   1) 配置的角速度 scan_sweep_rate_dps_
  //   2) 单帧不超过扫描范围的一半（防止大步跳过目标）
  const double span = std::abs(scan_yaw_max_deg_ - scan_yaw_min_deg_);
  const double per_tick_cap = std::max(1.0, span * 0.5);
  double dyaw = std::min(scan_sweep_rate_dps_ * dt, per_tick_cap);
  if (dyaw <= 0.0)
  {
    dyaw = scan_sweep_rate_dps_ * 0.02;   // 兜底：dt 为 0 时按 20ms 估
  }

  sweep_yaw_deg_ += sweep_dir_ * dyaw;

  // 到达端点：换向 + 停顿；若启用多层，则同时换 pitch 层
  if (sweep_yaw_deg_ >= scan_yaw_max_deg_)
  {
    sweep_yaw_deg_ = scan_yaw_max_deg_;
    sweep_dir_ = -1;
    sweep_hold_until_ = now + rclcpp::Duration::from_seconds(scan_hold_ms_ / 1000.0);
    if (scan_sweep_multi_pitch_)
    {
      scan_pitch_idx_ = (scan_pitch_idx_ + 1) % std::max(1, pitchLevelCount());
      sweep_pitch_deg_ = scanPitchDeg();
    }
  }
  else if (sweep_yaw_deg_ <= scan_yaw_min_deg_)
  {
    sweep_yaw_deg_ = scan_yaw_min_deg_;
    sweep_dir_ = 1;
    sweep_hold_until_ = now + rclcpp::Duration::from_seconds(scan_hold_ms_ / 1000.0);
    if (scan_sweep_multi_pitch_)
    {
      scan_pitch_idx_ = (scan_pitch_idx_ + 1) % std::max(1, pitchLevelCount());
      sweep_pitch_deg_ = scanPitchDeg();
    }
  }

  // 直接按 sweep 角算 tick（不经过 scanYawDeg，它读的是离散索引）
  const int yaw_tick = gimbal_center_tick_ +
                       static_cast<int>(std::lround(sweep_yaw_deg_ *
                                                    yaw_ticks_per_deg_ *
                                                    yaw_deg_sign_));
  const int pitch_tick = gimbal_center_tick_ +
                         static_cast<int>(std::lround(sweep_pitch_deg_ *
                                                      pitch_ticks_per_deg_ *
                                                      pitch_deg_sign_));
  cmd_pos_yaw_ = clampTick(yaw_tick);
  cmd_pos_pitch_ = clampTick(pitch_tick);
  cmd_spd_yaw_ = scan_speed_;
  cmd_spd_pitch_ = pitch_scan_speed_;
}

// 由当前扫描索引/方向推出下一个扫描角（跳档模式用，便于日志与调试）
void TargetAimer::computeNextScanAngle(double &yaw_deg, double &pitch_deg)
{
  const int next_yaw = scan_yaw_idx_ + scan_yaw_dir_;
  const bool wrap = (next_yaw < 0 || next_yaw >= scan_yaw_dirs_);
  if (wrap)
  {
    yaw_deg = (scan_yaw_dir_ > 0) ? scan_yaw_min_deg_ : scan_yaw_max_deg_;
    const int next_pitch = (scan_pitch_idx_ + 1) % std::max(1, pitchLevelCount());
    pitch_deg = scan_pitch_levels_deg_.empty()
                    ? 0.0
                    : scan_pitch_levels_deg_[static_cast<size_t>(next_pitch)];
  }
  else
  {
    const double step = (scan_yaw_dirs_ <= 1)
                            ? 0.0
                            : (scan_yaw_max_deg_ - scan_yaw_min_deg_) /
                                  static_cast<double>(scan_yaw_dirs_ - 1);
    yaw_deg = scan_yaw_min_deg_ + step * static_cast<double>(next_yaw);
    pitch_deg = scanPitchDeg();
  }
}

// -----------------------------------------------------------------------------
int TargetAimer::clampTick(int tick) const
{
  return clampi(tick, gimbal_tick_min_, gimbal_tick_max_);
}

int TargetAimer::computeTrackingSpeed(int target, int current, bool is_pitch) const
{
  const int err = target - current;
  const int abs_err = std::abs(err);

  const int deadband = is_pitch ? pitch_position_deadband_ticks_
                                : position_deadband_ticks_;
  const double kp = is_pitch ? pitch_motor_kp_ : motor_kp_;
  const int max_speed = is_pitch ? pitch_max_speed_ : gimbal_max_speed_;

  if (abs_err <= deadband)
  {
    return 0;
  }

  double speed = std::abs(kp * static_cast<double>(err));
  speed = std::min(speed, static_cast<double>(max_speed));
  return static_cast<int>(std::lround(speed));
}

// -----------------------------------------------------------------------------
TargetAimer::AimResult TargetAimer::result() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  AimResult r;
  r.state = state_;
  r.center_x = center_x_;
  r.center_y = center_y_;
  r.class_id = class_id_;
  r.target_visible = target_visible_;
  r.shots_fired = shots_fired_;
  r.gimbal_yaw_tick = fb_pos_yaw_;
  r.gimbal_pitch_tick = fb_pos_pitch_;
  r.last_fire_age_s = (node_->now() - last_fire_time_).seconds();
  r.range_m = last_target_range_m_;

  // ★ 新增观测项
  r.face = face_;
  r.laser_dot_seen = laser_dot_seen_;
  r.local_shots = local_shots_;
  r.link_degraded = link_degraded_;
  // ★ 移动靶
  r.track_velocity_px_s = last_track_velocity_px_s_;
  // ★ 方案 β 观测项
  r.lead_active = lead_active_;
  r.lead_offset_px = lead_offset_px_;
  r.lead_total_ms = leadTotalMs();
  r.lead_fire_count = lead_fire_count_;
  r.lead_fallback_count = lead_fallback_count_;
  {
    const double vt = std::abs(last_track_velocity_px_s_);
    const bool in_tol = std::abs(center_x_) <= fire_tolerance_px_ &&
                        std::abs(center_y_) <= fire_tolerance_px_;
    r.turn_around_ok = (!require_turn_around_) ||
                       (in_tol && vt <= turn_around_v_thresh_px_s_);
  }
  const int hp = target_hp_.load();
  r.target_destroyed = (hp >= target_hp_destroyed_);

  const bool link_ok = referee_link_ok_.load();
  r.ammo_effective = link_ok
                         ? std::min(ammo_.load(),
                                    std::max(0, max_local_shots_ - local_shots_))
                         : std::max(0, max_local_shots_ - local_shots_);

  const bool ammo_ok = (r.ammo_effective >= min_ammo_to_fire_) &&
                       !r.target_destroyed;
  // ★ 本函数是 **const**，而 std::atomic 的 operator T() 只在**非 const** 版本可用
  //   （const 版本是 = delete）。所以 const 函数里读原子量必须显式 .load()，
  //   否则报 no conversion from 'const std::atomic<bool>' to 'bool'。
  //   注意：tick()（非 const）里可以隐式转换，不要一并改（改了也无害，
  //   但保持一致性更好——见下方 fire_authorized 的写法）。
  const bool posture_ok = !require_stationary_ || chassis_stationary_.load();
  const bool range_ok = (last_target_range_m_ <= 0.0) ||
                        (last_target_range_m_ <= max_engage_range_m_);
  // 面联锁（与 tick 内保持一致）
  bool face_ok = true;
  if (require_blue_face_)
  {
    face_ok = (face_ == 2) || (face_ == 0 && face_unknown_policy_ == 1);
  }
  bool dot_ok = true;
  if (require_laser_dot_ && laser_dot_required_after_first_ && local_shots_ > 0)
  {
    dot_ok = laser_dot_seen_;
  }

  r.fire_authorized = engaging_.load() && ammo_ok && posture_ok && range_ok &&
                      !being_hit_.load() && face_ok && dot_ok &&
                      std::abs(center_x_) <= fire_tolerance_px_ &&
                      std::abs(center_y_) <= fire_tolerance_px_;
  r.fired_this_tick = laser_active_;
  return r;
}

}  // namespace dzmission
