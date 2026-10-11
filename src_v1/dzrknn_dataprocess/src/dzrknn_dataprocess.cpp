#include "dzrknn_dataprocess.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <chrono>
#include <algorithm>

// 定义BOOST_BIND_GLOBAL_PLACEHOLDERS以消除警告
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/bind.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<turn_on_robot>("dzrknn_dataprocess");
  // rclcpp::spin(node); // 使用rclcpp::spin而不是直接调用Control
  node->Control();
  rclcpp::shutdown();
  return 0;
}

void turn_on_robot::callback_offset_center(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  std_msgs::msg::Int32MultiArray temp_msg = *msg; // [0] :centerx偏移 [1] :centery偏移 [2] :物体类别(1=靶子) [3] :框宽 [4] :框高
  last_vision_time_ = this->now(); // 视觉心跳：不管有没有目标，只要收到消息就刷新

  // 目标类别：视觉端做了 +1 偏移 —— 1=蓝靶、2=红靶，0 表示无目标
  const bool is_target_cls = (temp_msg.data[2] == 1) || (temp_msg.data[2] == 2);

  // 长宽比过滤：只跟踪接近正方形(1:1)的目标，长方形的描边目标直接忽略
  const bool is_target = is_target_cls && aspectRatioOk(temp_msg);

  if (is_target) // 识别到合格的方形目标
  {
    if (!isTracking())
    {
      // 由 IDLE 或 SCAN 进入 TRACK：一旦锁定立即转为"边走边跟随"，
      // 正在进行的点位扫描即使没扫完也算结束；同时上报状态让循迹放心继续前进。
      fire_hold_count_ = 0;
      filt_inited_ = false;   // 重新进入跟踪时用当前帧重建滤波状态
      target_seen_count_ = 0; // 重新进入跟踪，扫描确认计数从零开始
      fire_cleared_ = false;
      setAimState(AIM_TRACK);
      std::cout << "==> 发现目标，进入边走边跟随" << std::endl;
    }
    lost_target_count_ = 0; // 检测到就清零丢帧计数

    // ===== 扫描确认：连续识别到目标达到 target_confirm_frames_ 帧才算"确认有目标" =====
    // 确认之前只跟踪、绝不开火 —— 云台横扫时扫过目标的一两帧偶发检出不足以触发射击。
    target_seen_count_++;
    const bool target_confirmed = (target_seen_count_ >= target_confirm_frames_);
    if (target_confirmed && !fire_cleared_)
    {
      fire_cleared_ = true;
      std::cout << "==> 扫描确认目标，允许射击" << std::endl;
    }

    // ===== 第一步：一阶低通滤波(EMA)，压制视觉检测 ±2~5px 的跳变 =====
    const double dx_raw = static_cast<double>(temp_msg.data[0]);
    const double dy_raw = static_cast<double>(temp_msg.data[1]);
    if (!filt_inited_)
    {
      dx_filt_ = dx_raw;
      dy_filt_ = dy_raw;
      // 同步复位"上一帧滤波值"：否则重新进入跟踪的第一帧会拿上一轮跟踪的陈旧值做差，
      // 算出一个虚假的"目标移动速度"，进而误判为画面不稳定而推迟开火。
      dx_prev_filt_ = dx_raw;
      dy_prev_filt_ = dy_raw;
      abs_x_last_ = std::abs(dx_filt_);
      abs_y_last_ = std::abs(dy_filt_);
      filt_inited_ = true;
    }
    dx_filt_ = ema_alpha_ * dx_raw + (1.0 - ema_alpha_) * dx_filt_;
    dy_filt_ = ema_alpha_ * dy_raw + (1.0 - ema_alpha_) * dy_filt_;

    // ===== 第二步：跟踪死区，滤波后仍落在死区内就完全不动位置 =====
    const double dz = static_cast<double>(track_deadzone_px_);
    double eff_x = 0.0;
    double eff_y = 0.0;
    if (std::abs(dx_filt_) > dz)
    {
      eff_x = (dx_filt_ > 0.0) ? (dx_filt_ - dz) : (dx_filt_ + dz);
    }
    if (std::abs(dy_filt_) > dz)
    {
      eff_y = (dy_filt_ > 0.0) ? (dy_filt_ - dz) : (dy_filt_ + dz);
    }

    // ===== 第三步：位置增量（有效偏差的一半 + 目标移动预判）=====
    // 预判：按目标在画面中的移动速度提前一段，补偿"视觉→串口→下位机"的链路延迟
    const double vx = dx_filt_ - dx_prev_filt_; // 像素/帧
    const double vy = dy_filt_ - dy_prev_filt_;
    dx_prev_filt_ = dx_filt_;
    dy_prev_filt_ = dy_filt_;

    if (eff_y != 0.0)
    {
      moveBaseControl.Position_0 = clampPitchTick(
          curYuntai_feedback_data.Position_0 +
          static_cast<int>(eff_y / 2.0 + vy * predict_gain_));
    }
    if (eff_x != 0.0)
    {
      moveBaseControl.Position_1 = clampYawTick(
          curYuntai_feedback_data.Position_1 +
          static_cast<int>(eff_x / 2.0 + vx * predict_gain_));
    }

    // ===== 第四步：PD 速度（死区内速度直接为 0）=====
    const double abs_x = std::abs(dx_filt_);
    const double abs_y = std::abs(dy_filt_);
    moveBaseControl.Speed_0 = trackSpeedFor(abs_y, abs_y - abs_y_last_, eff_y);
    moveBaseControl.Speed_1 = trackSpeedFor(abs_x, abs_x - abs_x_last_, eff_x);
    abs_x_last_ = abs_x;
    abs_y_last_ = abs_y;

    // ===== 第五步：狙击开火判定 =====
    // 同时满足：目标进入(动态)开火框、瞄准已稳定、弹药与冷却允许、并连续保持 N 帧
    // 动态框：用目标框宽估算距离，越近视差越大框越宽
    const double box_w = static_cast<double>(temp_msg.data[3]);
    const double fire_dz = static_cast<double>(fireDeadzonePx(box_w));

    // 下位机 camemo_feedback 回传的云台电机实际速度（旧判据用）
    const int real_speed_0 = std::abs(curYuntai_feedback_data.Speed_0);
    const int real_speed_1 = std::abs(curYuntai_feedback_data.Speed_1);
    const bool motor_still = (real_speed_0 < fire_real_speed_max_) &&
                             (real_speed_1 < fire_real_speed_max_) &&
                             (moveBaseControl.Speed_0 < fire_speed_max_) &&
                             (moveBaseControl.Speed_1 < fire_speed_max_);

    // ===== 稳定判据（两种模式，由 fire_use_image_stable_ 选择）=====
    // 画面稳定(方案B，默认)：vx/vy 是滤波后偏差的单帧变化量，两者都很小即认为"已经瞄住了"。
    //   它衡量的是"目标在画面里稳不稳"，而不是"电机转不转" —— 所以车辆行进中（含缓弯）也能开火。
    // 电机静止(旧判据)：要求云台电机绝对停止 —— 只在车辆完全静止时才可能开火。
    const bool aim_stable = (std::abs(vx) <= fire_stable_px_per_frame_) &&
                            (std::abs(vy) <= fire_stable_px_per_frame_);
    const bool aim_ready = fire_use_image_stable_ ? aim_stable : motor_still;

    // 开火零点取"激光实际中心"：激光与摄像头不同轴时，画面几何中心并非命中点
    const double aim_err_x = std::abs(dx_filt_ - laser_offset_x_);
    const double aim_err_y = std::abs(dy_filt_ - laser_offset_y_);
    const bool in_fire_zone = (aim_err_x <= fire_dz) && (aim_err_y <= fire_dz);
    const bool ammo_ok = (max_shots_ > 0) && (shots_fired_ < max_shots_);
    const bool cooldown_ok =
        ((this->now() - last_shot_time_).seconds() * 1000.0 >= static_cast<double>(shot_cooldown_ms_));

    if (in_fire_zone && aim_ready && ammo_ok && cooldown_ok && target_confirmed)
    {
      fire_hold_count_++;
      if (fire_hold_count_ >= fire_hold_frames_)
      {
        std_msgs::msg::UInt8 shotdata;
        shotdata.data = 1;
        pub_LaserShot_Command->publish(shotdata);
        shotdata.data = 0;
        pub_LaserShot_Command->publish(shotdata);

        shots_fired_++;
        last_shot_time_ = this->now();
        fire_hold_count_ = 0; // 开火后重新累计，避免连发浪费弹药

        // 上报累计发射数，供 path_executor 判停（中途径停点打满 N 发再继续）
        std_msgs::msg::Int32 countmsg;
        countmsg.data = shots_fired_;
        pub_shot_count->publish(countmsg);

        std::cout << "pub_LaserShot_Command  [" << shots_fired_ << "/" << max_shots_ << "]"
                  << "  dz=" << static_cast<int>(fire_dz)
                  << "  v=" << vx << "/" << vy
                  << "  realV=" << real_speed_0 << "/" << real_speed_1
                  << std::endl;
      }
    }
    else
    {
      fire_hold_count_ = 0;
    }
  }

  // // /// 物资点播报（新模型只有蓝/红靶两类，此分支保留以备后续扩展）
  else if ( 2 < temp_msg.data[2] )
  {
    // 检测到物资点，发布语音播报
    std_msgs::msg::UInt8 voice_msg;
    voice_msg.data = static_cast<uint8_t>(temp_msg.data[2]); // 使用物体类别作为语音ID

    if(last_voice_msg.data != voice_msg.data){
      pub_voice_switch->publish(voice_msg);
      last_voice_msg.data =voice_msg.data;
      int tempdata= voice_msg.data;
      std::cout << "voice_msg.data = " << tempdata <<std::endl;
    }

  }

  else
  { // 未检测到目标（含长宽比不合格的长方形目标）
    // 扫描确认计数"衰减"而非清零：视觉偶发漏一两帧时不会把已积累的确认进度全部抹掉，
    // 但持续看不到目标仍会很快归零，确保"必须连续看到目标才解锁射击"。
    if (target_seen_count_ > 0)
    {
      target_seen_count_--;
    }

    if (isTracking())
    {
      // 跟踪中丢帧：累计到阈值才放弃，避免单帧漏检导致跟踪中断
      lost_target_count_++;
      if (lost_target_count_ >= lost_frames_to_rescan_)
      {
        lost_target_count_ = 0;
        if (scan_continuous_)
        {
          // 持续扫描模式：目标脱离视野 → 立刻重新开始扫描
          std::cout << "==> 目标脱离视野，重新开始扫描" << std::endl;
          startScan(true);
        }
        else
        {
          setAimState(AIM_IDLE); // 回到空闲：不自动扫描，等循迹下一个点位的扫描请求
          std::cout << "==> 目标丢失，回到空闲（等下一个点位请求）" << std::endl;
        }
      }
    }

    std_msgs::msg::UInt8 shotdata;
    shotdata.data = 0;
    pub_LaserShot_Command->publish(shotdata);


    std_msgs::msg::UInt8 voice_msg;
    voice_msg.data = 111;
    pub_voice_switch->publish(voice_msg); 

  }

}

// 水平轴刻度钳位（360° 量程，留边界余量避免回卷）
int turn_on_robot::clampYawTick(int tick) const
{
  if (tick < gimbal_yaw_tick_min_)
  {
    return gimbal_yaw_tick_min_;
  }
  if (tick > gimbal_yaw_tick_max_)
  {
    return gimbal_yaw_tick_max_;
  }
  return tick;
}

// 竖直轴刻度钳位（下位机强制 950~2400）
int turn_on_robot::clampPitchTick(int tick) const
{
  if (tick < gimbal_pitch_tick_min_)
  {
    return gimbal_pitch_tick_min_;
  }
  if (tick > gimbal_pitch_tick_max_)
  {
    return gimbal_pitch_tick_max_;
  }
  return tick;
}

// 当前扫描点对应的水平角(相对车体前方，右为正)，8 个方向均匀分布在 [min, max]
double turn_on_robot::scanYawDeg() const
{
  if (scan_yaw_dirs_ <= 1)
  {
    return scan_yaw_min_deg_;
  }
  const double step = (scan_yaw_max_deg_ - scan_yaw_min_deg_) / (scan_yaw_dirs_ - 1);
  return scan_yaw_min_deg_ + step * scan_yaw_idx_;
}

// 竖直层的数量
int turn_on_robot::pitchLevelCount() const
{
  return scan_pitch_levels_deg_.empty() ? 1 : static_cast<int>(scan_pitch_levels_deg_.size());
}

// 当前竖直层对应的角度(相对水平面，上为正)，直接取自 scan_pitch_levels_deg_ 列表
double turn_on_robot::scanPitchDeg() const
{
  if (scan_pitch_levels_deg_.empty())
  {
    return 0.0;
  }
  const int idx = std::min(scan_pitch_idx_, pitchLevelCount() - 1);
  return scan_pitch_levels_deg_[idx];
}

// 巡逻(扫描)速度 = 云台最大转速 x 巡逻比例
int turn_on_robot::patrolSpeed() const
{
  return static_cast<int>(gimbal_max_speed_ * patrol_speed_ratio_);
}

// 长宽比过滤：只接受接近正方形(1:1)的检测框，长方形的描边目标返回 false。
// 视觉消息不足 5 个字段或宽高无效时不做过滤（与旧版视觉节点保持兼容）。
bool turn_on_robot::aspectRatioOk(const std_msgs::msg::Int32MultiArray &msg) const
{
  if (msg.data.size() < 5)
  {
    return true;
  }
  const int box_w = msg.data[3];
  const int box_h = msg.data[4];
  if (box_w <= 0 || box_h <= 0)
  {
    return true;
  }
  const double aspect = static_cast<double>(box_w) / static_cast<double>(box_h);
  return (aspect >= aspect_min_) && (aspect <= aspect_max_);
}

// PD 跟踪速度：
//   abs_err   —— 滤波后偏差的绝对值(像素)
//   d_abs_err —— 偏差绝对值相对上一帧的变化量，>0 表示目标正在远离画面中心
//   eff       —— 死区外的有效偏差；为 0 表示目标已在死区内
// 偏差越大追得越快；目标远离(d>0)时加速追，靠近(d<0)时减速形成阻尼，抑制超调振荡。
// 目标进入死区后速度直接置 0 —— 这是消除抖动最关键的一步。
int turn_on_robot::trackSpeedFor(double abs_err, double d_abs_err, double eff) const
{
  if (eff == 0.0)
  {
    return 0; // 已在死区内：完全不动作，彻底消除小幅来回摆动
  }

  double kp = track_kp_;
  double kd = track_kd_;
  if (abs_err < static_cast<double>(fine_zone_px_))
  {
    // 精调档：降低比例、增强阻尼，避免在中心附近来回冲
    kp *= fine_kp_scale_;
    kd *= fine_kd_scale_;
  }

  double sp = kp * abs_err + kd * d_abs_err;
  if (sp < 100.0)
  {
    sp = 100.0; // 电机最小速度，保证能起步
  }

  const double limit = static_cast<double>(patrolSpeed()) * track_speed_max_ratio_;
  if (sp > limit)
  {
    sp = limit;
  }
  return static_cast<int>(sp);
}

// 动态开火死区：用目标框宽估算距离 —— 框越大说明目标越近、视差越大，死区随之放宽。
// 框宽在 [box_w_far_, box_w_near_] 内线性映射到 [fire_dz_far_px_, fire_dz_near_px_]。
int turn_on_robot::fireDeadzonePx(double box_w) const
{
  if (!fire_dz_dynamic_)
  {
    return fire_deadzone_px_;
  }

  const double span = box_w_near_ - box_w_far_;
  if (span <= 0.0)
  {
    return fire_deadzone_px_;
  }

  double t = (box_w - box_w_far_) / span;
  if (t < 0.0)
  {
    t = 0.0; // 比"最远标定点"还远
  }
  if (t > 1.0)
  {
    t = 1.0; // 比"最近标定点"还近
  }

  return static_cast<int>(fire_dz_far_px_ + t * (fire_dz_near_px_ - fire_dz_far_px_));
}

// 把当前方位角度转换成刻度指令下发给云台(扫描转速 = 最大转速 x 比例)
void turn_on_robot::applyScanTarget()
{
  const int yaw_tick = gimbal_center_tick_ +
                       static_cast<int>(std::lround(scanYawDeg() * yaw_ticks_per_deg_ * yaw_deg_sign_));
  const int pitch_tick = gimbal_center_tick_ +
                         static_cast<int>(std::lround(scanPitchDeg() * pitch_ticks_per_deg_ * pitch_deg_sign_));

  moveBaseControl.Position_1 = clampYawTick(yaw_tick);
  moveBaseControl.Position_0 = clampPitchTick(pitch_tick);

  const int scan_speed = patrolSpeed();
  moveBaseControl.Speed_0 = scan_speed;
  moveBaseControl.Speed_1 = scan_speed;
}

// 全向扫描推进：一趟扫到端点后反向，共扫满 scan_passes_ 趟即结束。
// "先左转一圈、再右转一圈" 就是这里的第 1 趟 + 第 2 趟。
// 趟数上限同时充当"扫描一定会结束"的保证 —— 循迹不会被扫描永久卡住。
void turn_on_robot::advanceScanIndex()
{
  const int next_yaw = scan_yaw_idx_ + scan_yaw_dir_;
  if (next_yaw >= 0 && next_yaw < scan_yaw_dirs_)
  {
    scan_yaw_idx_ = next_yaw; // 同一趟继续横扫
    return;
  }

  // 一趟横扫结束
  scan_pass_done_++;

  // 每扫完一层的一轮（scan_passes_ 趟）就切到下一层
  if (scan_pass_done_ % scan_passes_ == 0)
  {
    scan_pitch_idx_ = (scan_pitch_idx_ + 1) % pitchLevelCount();
  }

  if (scan_pass_done_ >= scan_passes_ * pitchLevelCount())
  {
    scan_finished_ = true; // 所有趟数走完 → 本轮全向扫描结束
    return;
  }

  // 反向，开始下一趟
  scan_yaw_dir_ = -scan_yaw_dir_;
  scan_yaw_idx_ = (scan_yaw_dir_ > 0) ? 0 : (scan_yaw_dirs_ - 1);
}

// ================= 自瞄状态机的三个基础设施函数 =================

// 切换状态并立即上报，让循迹尽快知道"可以走了"
void turn_on_robot::setAimState(int s)
{
  if (aim_state_ == s)
  {
    return;
  }
  aim_state_ = s;
  publishAimStatus();
  std::cout << "==> 自瞄状态 -> "
            << (s == AIM_TRACK ? "TRACK(边走边跟随)"
                               : (s == AIM_SCAN ? "SCAN(点位扫描)" : "IDLE(空闲)"))
            << std::endl;
}

void turn_on_robot::publishAimStatus()
{
  std_msgs::msg::Int32 st;
  st.data = aim_state_;
  pub_aim_status->publish(st);
}

// 启动一次全向扫描（左一趟 + 右一趟）。
// continuous=true 时扫完一轮会自动从头再来，直到确认目标
// —— 用于"行进中持续扫描直到找到目标"。
void turn_on_robot::startScan(bool continuous)
{
  scan_continuous_ = continuous;
  scan_yaw_idx_ = 0;
  scan_yaw_dir_ = 1;
  scan_pitch_idx_ = 0;
  scan_pass_done_ = 0;
  scan_finished_ = false;
  last_scan_step_time_ = this->now();
  applyScanTarget(); // 立即下发第一个扫描点
  setAimState(AIM_SCAN);
}

// 扫描请求 /nav/scan_request（UInt8）：
//   2 及以上 -> 持续扫描：一轮扫完自动重来；锁定后转跟随；目标脱离视野再自动重开
//   1        -> 单次扫描：扫完两趟就回 IDLE（循迹到达点位时用这个）
//   0        -> 停止扫描，回到 IDLE
//   已在 AIM_TRACK 时一律忽略，不打断跟随。
void turn_on_robot::callback_scan_request(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (aim_state_ == AIM_TRACK)
  {
    return; // 已锁定，不打断跟随
  }

  if (msg->data == 0)
  {
    scan_continuous_ = false;
    setAimState(AIM_IDLE);
    std::cout << "==> 收到停止扫描请求" << std::endl;
    return;
  }

  if (aim_state_ == AIM_SCAN)
  {
    // 已经在扫：只更新模式，绝不重置进度 ——
    // 否则外部反复发请求会把扫描永远拽回起点附近。
    scan_continuous_ = (msg->data >= 2);
    return;
  }

  startScan(msg->data >= 2);
}

void turn_on_robot::CaremaMontorControl()
{
  // ===== 状态一：边走边跟随（AIM_TRACK）=====
  // 位置完全由 callback_offset_center 的视觉伺服修正，这里不介入。
  if (aim_state_ == AIM_TRACK)
  {
    // 视觉静默保护：视觉节点挂了/相机断了会完全收不到回调，强制退出跟踪避免卡死
    const double silent_ms = (this->now() - last_vision_time_).seconds() * 1000.0;
    if (silent_ms > static_cast<double>(vision_timeout_ms_))
    {
      lost_target_count_ = 0;
      if (scan_continuous_)
      {
        startScan(true); // 视觉彻底静默，同样视为"脱离视野"，重新开始扫描
      }
      else
      {
        setAimState(AIM_IDLE);
      }
    }
    return;
  }

  // ===== 状态二：空闲（AIM_IDLE）=====
  // 行进中且未锁定 —— 云台保持不动、不主动扫描。
  // 扫描只能由循迹的 /nav/scan_request 触发，对应"到达预设点位才扫一次"。
  if (aim_state_ != AIM_SCAN)
  {
    return;
  }

  // ===== 状态三：点位全向扫描（AIM_SCAN，左一趟 + 右一趟）=====
  const rclcpp::Time now = this->now();
  const double elapsed_ms = (now - last_scan_step_time_).seconds() * 1000.0;

  if (elapsed_ms >= static_cast<double>(scan_dwell_ms_))
  {
    const bool arrived =
        std::abs(curYuntai_feedback_data.Position_1 - moveBaseControl.Position_1) <= scan_arrive_tol_ &&
        std::abs(curYuntai_feedback_data.Position_0 - moveBaseControl.Position_0) <= scan_arrive_tol_;
    const bool timeout = elapsed_ms >= static_cast<double>(scan_dwell_ms_) * 3.0;

    if (arrived || timeout)
    {
      advanceScanIndex();
      last_scan_step_time_ = now;

      if (scan_finished_)
      {
        scan_pass_done_ = 0;
        scan_finished_ = false;
        if (scan_continuous_)
        {
          // 持续扫描模式：立刻从头再来一轮，不停下来
          scan_yaw_idx_ = 0;
          scan_yaw_dir_ = 1;
          scan_pitch_idx_ = 0;
          last_scan_step_time_ = this->now();
          std::cout << "==> 一轮扫描结束未发现目标，继续扫描" << std::endl;
        }
        else
        {
          // 单次扫描：回空闲，并上报状态让循迹继续前进
          setAimState(AIM_IDLE);
          std::cout << "==> 全向扫描结束，未发现目标" << std::endl;
          return;
        }
      }
    }
  }

  applyScanTarget();
}



void turn_on_robot::callback_camemo_feedback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 6)
  {
    curYuntai_feedback_data.Position_0 = msg->data[0];
    curYuntai_feedback_data.Position_1 = msg->data[1];
    curYuntai_feedback_data.Speed_0 = msg->data[2];
    curYuntai_feedback_data.Speed_1 = msg->data[3];
    curYuntai_feedback_data.Load_0 = msg->data[4];
    curYuntai_feedback_data.Load_1 = msg->data[5];
  }
}
int turn_on_robot::compute_comp_pixel_0(double tar_dis)
{
  // 输入范围检查
  if (tar_dis < 0.0)
    tar_dis = 0.0;
  if (tar_dis > 5.0)
    tar_dis = 5.0;

  // 计算索引并查表
  int index = static_cast<int>(tar_dis / 0.25);
  return lookup_table[index]; // 使用从文件加载的值
}

// 从文件加载查表值（每行一个数据）
bool turn_on_robot::loadLookupTable(const std::string &filePath)
{
  std::ifstream file(filePath);
  if (!file.is_open())
  {
    std::cerr << "无法打开文件: " << filePath << std::endl;
    return false;
  }

  lookup_table.clear(); // 假设lookup_table是类成员vector
  std::string line;
  int lineNum = 0;

  // 逐行读取带符号整数
  while (std::getline(file, line))
  {
    ++lineNum;
    // 忽略空行
    if (line.empty())
      continue;

    try
    {
      // 处理带符号整数（支持正负号）
      size_t pos = 0;
      int value = std::stoi(line, &pos);

      // 验证整行是否被正确解析
      if (pos != line.length())
      {
        throw std::invalid_argument("行中包含非数字字符");
      }

      lookup_table.push_back(value);
    }
    catch (const std::invalid_argument &e)
    {
      std::cerr << "第" << lineNum << "行格式错误: " << line << std::endl;
      return false;
    }
    catch (const std::out_of_range &e)
    {
      std::cerr << "第" << lineNum << "行数值超出范围: " << line << std::endl;
      return false;
    }
  }
  return true;
}





turn_on_robot::turn_on_robot(std::string name) : Node(name)
{
  RCLCPP_INFO(this->get_logger(), "创建节点成功..........");

  pub_LaserShot_Command = this->create_publisher<std_msgs::msg::UInt8>("LaserShot_Command", 10);
  pub_CameMo_Conl = this->create_publisher<std_msgs::msg::Int32MultiArray>("CameMo_Conl", 10);
  pub_voice_switch = this->create_publisher<std_msgs::msg::UInt8>("voice_switch", 10);
  pub_shot_count = this->create_publisher<std_msgs::msg::Int32>("/shoot_count", 10);
  pub_aim_status = this->create_publisher<std_msgs::msg::Int32>("/aim/status", 10);

  // Set the velocity control command callback function
  // 速度控制命令订阅回调函数设置
  sub_offset_center = this->create_subscription<std_msgs::msg::Int32MultiArray>("offset_center", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&turn_on_robot::callback_offset_center, this, std::placeholders::_1));
  sub_camemo_feedback = this->create_subscription<std_msgs::msg::Int32MultiArray>("camemo_feedback", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&turn_on_robot::callback_camemo_feedback, this, std::placeholders::_1));
  // 扫描请求用默认(可靠) QoS —— best_effort 订阅无法匹配 path_executor 默认的可靠发布者
  sub_scan_request = this->create_subscription<std_msgs::msg::UInt8>("/nav/scan_request", 10, std::bind(&turn_on_robot::callback_scan_request, this, std::placeholders::_1));


  last_voice_msg.data = 0;

  RCLCPP_INFO(this->get_logger(), "Data ready");

  // 瞄准扫描参数
  this->declare_parameter("scan_yaw_dirs", 8);			// 水平横扫点数
  this->declare_parameter("scan_yaw_min_deg", -90.0);	// 水平最小角(度)
  this->declare_parameter("scan_yaw_max_deg", 90.0);	// 水平最大角(度)
  // 竖直角度列表：只做水平扫描，竖直轴固定在 0°(水平)。如需多层扫描可改成 {15.0, 0.0, -10.0}
  this->declare_parameter("scan_pitch_levels",
                          std::vector<double>{0.0});
  this->declare_parameter("yaw_deg_sign", 1.0);			// 水平角度正方向->刻度(反了改-1)
  this->declare_parameter("pitch_deg_sign", 1.0);		// 竖直角度正方向->刻度(反了改-1)
  this->declare_parameter("gimbal_center_tick", 2047);	// 零位刻度
  this->declare_parameter("yaw_ticks_per_deg", 11.378);	  // 水平刻度/度 = 4096/360
  this->declare_parameter("pitch_ticks_per_deg", 11.378); // 竖直刻度/度 = 4096/360
  this->declare_parameter("gimbal_yaw_tick_min", 100);	  // 水平轴刻度下限(留边界余量)
  this->declare_parameter("gimbal_yaw_tick_max", 3995);	  // 水平轴刻度上限
  this->declare_parameter("gimbal_pitch_tick_min", 950);	  // 竖直轴刻度下限(下位机钳位)
  this->declare_parameter("gimbal_pitch_tick_max", 2400); // 竖直轴刻度上限(下位机钳位)
  this->declare_parameter("gimbal_max_speed", 3700);		  // 云台最大转速
  this->declare_parameter("patrol_speed_ratio", 0.5);	  // 巡逻(扫描)速度占最大转速比例
  this->declare_parameter("scan_dwell_ms", 300);		  // 每点最短驻留(ms)
  this->declare_parameter("scan_arrive_tol", 30);		  // 到位容差(刻度)
  this->declare_parameter("scan_passes", 2);			  // 全向扫描趟数(1左+1右=2)
  this->declare_parameter("lost_frames_to_rescan", 60);	  // 跟踪中丢帧回扫描阈值
  this->declare_parameter("vision_timeout_ms", 1000);	  // 视觉静默超时(ms)
  this->declare_parameter("ema_alpha", 0.3);			  // 视觉一阶低通滤波系数
  this->declare_parameter("track_deadzone_px", 12);		  // 跟踪死区(像素)
  this->declare_parameter("fine_zone_px", 20);			  // 进入精调档的偏差阈值(像素)
  this->declare_parameter("fine_kp_scale", 0.5);		  // 精调档 Kp 缩放
  this->declare_parameter("fine_kd_scale", 2.0);		  // 精调档 Kd 放大
  this->declare_parameter("track_kp", 8.0);				  // PD 比例增益
  this->declare_parameter("track_kd", 3.0);				  // PD 微分增益(阻尼)
  this->declare_parameter("track_speed_max_ratio", 0.6);  // 跟踪速度上限占巡逻速度比例
  this->declare_parameter("fire_deadzone_px", 8);		  // 固定开火死区(像素)
  this->declare_parameter("fire_dz_dynamic", true);		  // 启用动态开火框(按距离)
  this->declare_parameter("fire_dz_far_px", 6.0);		  // 远距离开火死区(像素)
  this->declare_parameter("fire_dz_near_px", 18.0);		  // 近距离开火死区(像素)
  this->declare_parameter("box_w_far", 30.0);			  // 远处目标框宽(标定值)
  this->declare_parameter("box_w_near", 120.0);			  // 近处目标框宽(标定值)
  this->declare_parameter("fire_real_speed_max", 20);	  // 实际速度静止阈值
  this->declare_parameter("fire_speed_max", 50);		  // 开火时指令速度上限
  this->declare_parameter("fire_hold_frames", 5);		  // 连续满足多少帧才开火
  this->declare_parameter("fire_use_image_stable", true);   // 开火稳定判据：true=画面稳定(移动中可开火)
  this->declare_parameter("fire_stable_px_per_frame", 1.5); // 画面稳定阈值(像素/帧)
  this->declare_parameter("predict_gain", 0.5);			  // 目标移动预判强度(帧，0=关闭)
  this->declare_parameter("laser_offset_x", 0.0);		  // 激光落点水平偏移(标定值,像素)
  this->declare_parameter("laser_offset_y", 0.0);		  // 激光落点竖直偏移(标定值,像素)
  this->declare_parameter("max_shots", 10);				  // 最大发射次数(<=0 不限)
  this->declare_parameter("shot_cooldown_ms", 500);		  // 两发之间的最小间隔(ms)
  this->declare_parameter("aspect_min", 0.75);			  // 目标框宽高比下限(只跟踪近似1:1)
  this->declare_parameter("aspect_max", 1.35);			  // 目标框宽高比上限
  this->declare_parameter("target_confirm_frames", 8);	  // 连续识别多少帧才算"扫描确认"(之后才允许射击)

  this->get_parameter("scan_yaw_dirs", scan_yaw_dirs_);
  this->get_parameter("scan_yaw_min_deg", scan_yaw_min_deg_);
  this->get_parameter("scan_yaw_max_deg", scan_yaw_max_deg_);
  this->get_parameter("scan_pitch_levels", scan_pitch_levels_deg_);
  this->get_parameter("yaw_deg_sign", yaw_deg_sign_);
  this->get_parameter("pitch_deg_sign", pitch_deg_sign_);
  this->get_parameter("gimbal_center_tick", gimbal_center_tick_);
  this->get_parameter("yaw_ticks_per_deg", yaw_ticks_per_deg_);
  this->get_parameter("pitch_ticks_per_deg", pitch_ticks_per_deg_);
  this->get_parameter("gimbal_yaw_tick_min", gimbal_yaw_tick_min_);
  this->get_parameter("gimbal_yaw_tick_max", gimbal_yaw_tick_max_);
  this->get_parameter("gimbal_pitch_tick_min", gimbal_pitch_tick_min_);
  this->get_parameter("gimbal_pitch_tick_max", gimbal_pitch_tick_max_);
  this->get_parameter("gimbal_max_speed", gimbal_max_speed_);
  this->get_parameter("patrol_speed_ratio", patrol_speed_ratio_);
  this->get_parameter("scan_dwell_ms", scan_dwell_ms_);
  this->get_parameter("scan_arrive_tol", scan_arrive_tol_);
  this->get_parameter("scan_passes", scan_passes_);
  if (scan_passes_ < 1)
  {
    scan_passes_ = 1; // 至少扫一趟，否则永远扫不完
  }
  this->get_parameter("lost_frames_to_rescan", lost_frames_to_rescan_);
  this->get_parameter("vision_timeout_ms", vision_timeout_ms_);
  this->get_parameter("ema_alpha", ema_alpha_);
  this->get_parameter("track_deadzone_px", track_deadzone_px_);
  this->get_parameter("fine_zone_px", fine_zone_px_);
  this->get_parameter("fine_kp_scale", fine_kp_scale_);
  this->get_parameter("fine_kd_scale", fine_kd_scale_);
  this->get_parameter("track_kp", track_kp_);
  this->get_parameter("track_kd", track_kd_);
  this->get_parameter("track_speed_max_ratio", track_speed_max_ratio_);
  this->get_parameter("fire_deadzone_px", fire_deadzone_px_);
  this->get_parameter("fire_dz_dynamic", fire_dz_dynamic_);
  this->get_parameter("fire_dz_far_px", fire_dz_far_px_);
  this->get_parameter("fire_dz_near_px", fire_dz_near_px_);
  this->get_parameter("box_w_far", box_w_far_);
  this->get_parameter("box_w_near", box_w_near_);
  this->get_parameter("fire_real_speed_max", fire_real_speed_max_);
  this->get_parameter("fire_speed_max", fire_speed_max_);
  this->get_parameter("fire_hold_frames", fire_hold_frames_);
  this->get_parameter("fire_use_image_stable", fire_use_image_stable_);
  this->get_parameter("fire_stable_px_per_frame", fire_stable_px_per_frame_);
  this->get_parameter("predict_gain", predict_gain_);
  this->get_parameter("laser_offset_x", laser_offset_x_);
  this->get_parameter("laser_offset_y", laser_offset_y_);
  this->get_parameter("max_shots", max_shots_);
  this->get_parameter("shot_cooldown_ms", shot_cooldown_ms_);
  this->get_parameter("aspect_min", aspect_min_);
  this->get_parameter("aspect_max", aspect_max_);
  this->get_parameter("target_confirm_frames", target_confirm_frames_);
  if (target_confirm_frames_ < 1)
  {
    target_confirm_frames_ = 1; // 至少要看到 1 帧
  }

  // 弹药硬上限：全程只有 AIM_MAX_SHOTS 发，任何参数都无法突破
  // （传 0 或负数、传超过上限的值，一律按上限处理）
  if (max_shots_ <= 0 || max_shots_ > AIM_MAX_SHOTS)
  {
    RCLCPP_WARN(this->get_logger(),
                "max_shots=%d 超出 1~%d 范围，已强制为 %d 发",
                max_shots_, AIM_MAX_SHOTS, AIM_MAX_SHOTS);
    max_shots_ = AIM_MAX_SHOTS;
  }

  // 状态初始化：开机处于空闲(IDLE)，不主动扫描，等循迹的扫描请求
  aim_state_ = AIM_IDLE;
  scan_yaw_idx_ = 0;
  scan_yaw_dir_ = 1;
  scan_pitch_idx_ = 0;
  scan_pass_done_ = 0;
  scan_finished_ = false;
  lost_target_count_ = 0;
  last_scan_step_time_ = this->now();
  last_vision_time_ = this->now();

  // 跟踪状态初始化
  dx_filt_ = 0.0;
  dy_filt_ = 0.0;
  abs_x_last_ = 0.0;
  abs_y_last_ = 0.0;
  dx_prev_filt_ = 0.0;
  dy_prev_filt_ = 0.0;
  filt_inited_ = false;
  target_seen_count_ = 0;
  fire_cleared_ = false;
  fire_hold_count_ = 0;
  shots_fired_ = 0;
  last_shot_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
              "瞄准: %d点 x %d层 | 巡逻 %d | 跟踪上限 %d | 跟踪死区 %dpx | Kp %.1f/Kd %.1f | 滤波 %.2f",
              scan_yaw_dirs_, pitchLevelCount(), patrolSpeed(),
              static_cast<int>(patrolSpeed() * track_speed_max_ratio_),
              track_deadzone_px_, track_kp_, track_kd_, ema_alpha_);

  RCLCPP_INFO(this->get_logger(),
              "射击: %s开火框 %.0f~%.0fpx(标定框宽 %.0f~%.0f) | 稳定判据 %s(画面 %.2fpx/帧, 电机 realV<%d) | 保持 %d 帧 | 预判 %.2f | 全程弹药 %d 发/冷却 %dms | 扫描确认 %d 帧",
              fire_dz_dynamic_ ? "动态" : "固定",
              fire_dz_far_px_, fire_dz_near_px_, box_w_far_, box_w_near_,
              fire_use_image_stable_ ? "画面稳定" : "电机静止",
              fire_stable_px_per_frame_, fire_real_speed_max_,
              fire_hold_frames_, predict_gain_,
              max_shots_, shot_cooldown_ms_, target_confirm_frames_);

  RCLCPP_INFO(this->get_logger(),
              "自瞄: 状态机 IDLE/SCAN/TRACK | 点位全向扫描 %d 趟 | 扫描速度 %d | 循迹请求 /nav/scan_request | 状态上报 /aim/status",
              scan_passes_, patrolSpeed());


  std::cout << FRED("Copyright©2016-2020 dzrknn_dataprocess. All rights reserved ") << std::endl;

  std::string savePath_Table = "/home/duzhong/dzrobot/src/dzrknn_dataprocess/offcenter_table.txt";
  loadLookupTable(savePath_Table);

}

turn_on_robot::~turn_on_robot()
{
}

void turn_on_robot::Control()
{
  // ROS2 time handling
  // rclcpp::Time current_time, last_time;
  // last_time = this->get_clock()->now();

  rclcpp::Rate rate(100); // Loop rate of 50Hz
  while (rclcpp::ok())
  {
    rclcpp::spin_some(shared_from_this());

    CaremaMontorControl();

    // 发布云台控制命令到dzactuator
    std_msgs::msg::Int32MultiArray camemo_cmd;
    camemo_cmd.data.resize(4);
    camemo_cmd.data[0] = moveBaseControl.Position_0;
    camemo_cmd.data[1] = moveBaseControl.Position_1;
    camemo_cmd.data[2] = moveBaseControl.Speed_0;
    camemo_cmd.data[3] = moveBaseControl.Speed_1;
    pub_CameMo_Conl->publish(camemo_cmd);

    // 自瞄状态上报：状态变化时立即发（循迹 10ms 内就能知道）；
    // 未变化时每 50 个循环(≈0.5s)补发一次保活 —— 既让后启动的 path_executor 能读到，
    // 又避免 100Hz 刷屏（否则 ros2 topic echo /aim/status 根本没法看）。
    if (aim_state_ != aim_status_last_ || ++aim_status_tick_ >= 50)
    {
      aim_status_last_ = aim_state_;
      aim_status_tick_ = 0;
      publishAimStatus();
    }

    rate.sleep(); // Sleep to maintain rate of 50Hz
  }
}
