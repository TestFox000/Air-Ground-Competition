#include "dzrknn_dataprocess.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
  // ★ 长度守卫(code_review C6)：异常帧可能只带 1~2 个字段，而下文直接取 data[0..4]。
  //   std::vector 的 operator[] 不做边界检查，越界属未定义行为(读栈垃圾 -> 靶心跳变/崩溃)。
  //   这里要求至少 5 个字段；不足则丢弃该帧并 5s 节流告警(不刷屏)。
  if (temp_msg.data.size() < 5)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "offset_center 字段不足(%zu < 5)，已丢弃该帧",
                         temp_msg.data.size());
    return;
  }
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
      shots_on_target_ = 0;   // ★P0-2 重新锁定 = 新目标，单目标连发计数归零
      cls_candidate_ = 0;     // ★P1-3 新目标 = 类别重新确认
      cls_streak_ = 0;
      locked_cls_ = 0;
      setAimState(AIM_TRACK);
      std::cout << "==> 发现目标，进入边走边跟随" << std::endl;
    }
    lost_target_count_ = 0; // 检测到就清零丢帧计数
    lost_since_valid_ = false;   // ★t42 见到了 ⇒ 去抖计时作废（【连续】未见由此保证）

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

    // ===== 车体转动前馈（惯性凝视稳定）=====
    // 用 IMU 的偏航角速度 ωz 换算成云台水平轴刻度增量，提前抵消车体转动；
    // 否则车一转弯，瞄准点就跟着车体一起转，目标会持续往画面一侧漂。
    // 增益 gyro_ff_gain_ = 0 时完全关闭（默认），行为与旧版完全一致。
    double dt = (this->now() - last_track_time_).seconds();
    last_track_time_ = this->now();
    if (dt <= 0.0 || dt > 0.5)
    {
      dt = 0.0333; // 异常值兜底（约等于 30Hz 视觉帧间隔）
    }
    double ff_x = 0.0;
    const bool imu_fresh =
        (this->now() - last_imu_time_).seconds() * 1000.0 < static_cast<double>(imu_ff_timeout_ms_);
    if (gyro_ff_gain_ != 0.0 && imu_fresh)
    {
      // rad/s -> 度/s -> 刻度/s -> 本帧增量；负号表示"反向补偿车体转动"
      ff_x = -imu_omega_z_ * 57.2957795 * yaw_ticks_per_deg_ * dt * gyro_ff_gain_ * gyro_ff_sign_;
    }

    if (eff_y != 0.0)
    {
      // ★俯仰修正方向必须与扫描路径(applyScanTarget)一致 —— 两者都过 pitch_deg_sign_。
      //   坐标系约定：dy > 0 表示目标在画面【下方】-> 需要【低头】-> 角度增量为负。
      //   旧版直接写 +eff_y/2，方向与扫描路径正好相反：
      //   一旦进入 TRACK，云台就被往错误方向推，靶被顶到画面边缘再也出不来 ——
      //   这正是"目标始终贴在画面下沿、调参无效"的直接原因。
      const int pitch_inc = static_cast<int>(std::lround(
          -(eff_y / 2.0 + vy * predict_gain_) * pitch_deg_sign_));
      // ★★ 修复（2026-10-03，实车故障）：跟踪俯仰改用【机械限位】而不是【扫描区间】。
      //   故障现象：云台只能左右跟靶，俯仰完全跟不上，总在"追着目标头顶看"。
      //   根因：clampPitchToScan 的允许区间 = offset ± margin。本车实配
      //     scan_pitch_offset_deg = 6.0、scan_pitch_levels = [0.0]、margin = 6.0
      //     → 区间 = [6+0-6, 6+0+6] = [0°, 12°]，换算刻度仅 [2047, 2183]。
      //   也就是说【跟踪时云台被锁死在水平到微抬头这一小段】：
      //     · 地面上的靶需要低头约 40°，但 lo=0° 根本低不下去 → 永远看不到；
      //     · 目标稍高一点又超出 12°，也追不上。
      //   于是水平轴还能动（区间是 ±90°，够宽），俯仰轴却几乎被焊死。
      //   修复思路：跟踪的语义是"把靶送进画面中心"，俯仰必须能覆盖整个可视范围，
      //   所以跟踪走 clampPitchTick（950~2400，约 -96°~+31°，下位机机械限位）；
      //   扫描模式仍保留 clampPitchToScan —— 那边要的就是"只在扫描层之间走"。
      moveBaseControl.Position_0 = clampPitchTick(
          curYuntai_feedback_data.Position_0 + pitch_inc);
    }
    // 注意：水平轴即使 eff_x == 0（目标已在死区中心）也要处理 ——
    // 车体仍在转动时前馈量不为零，漏掉会把转弯中的目标甩出画面。
    const int inc_x = static_cast<int>(std::lround(eff_x / 2.0 + vx * predict_gain_ + ff_x));
    if (eff_x != 0.0 || inc_x != 0)
    {
      moveBaseControl.Position_1 = clampYawToScan(     // ★P0-1 限制在扫描水平范围内
          curYuntai_feedback_data.Position_1 + inc_x);
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

    // ===== ★新版本补丁：两键开火（默认关闭，use_external_fire_gate=false 时行为与旧版一致）=====
    //   旧版"画面静止"判据对**移动轨道靶**几乎不可能满足（靶 4cm/s 一直在走），
    //   所以移动靶模式下改由任务层的两把钥匙决定：
    //     /aim/fire_gate（MissionTask：已到站/停稳/FIRE 阶段）
    //     /aim/fire_ready（AimTask：目标确认 + **预测落点**在开火框内 + 冷却/预算允许）
    //   注意：AimTask 的 ready 已包含"预测落点在框内"，所以这里不再叠加 aim_stable，
    //   否则等于又要求画面静止 —— 那个坑正是本次要修的。
    const bool external_fire_ok = use_external_fire_gate_ && external_gate_ && external_ready_;
    const bool aim_ready_effective = use_external_fire_gate_ ? external_fire_ok : aim_ready;

    // 开火零点取"激光实际中心"：激光与摄像头不同轴时，画面几何中心并非命中点
    // ★提前量：lead_apply_gain_ > 0 时把 /aim/lead_x 叠加到误差上 ——
    //   让云台**物理上瞄向目标的未来位置**（补偿视觉流水线 0.26s + 伺服滞后）。
    //   0 = 不施加（旧行为）。★现场整定建议：先只开两键（gain=0）确认能开火，
    //   再逐步加大 gain（0.3 → 0.6 → 1.0）看命中分布；AimTask 侧同时有
    //   `aim.lead_time_s` 决定"提前量算多大"，两个旋钮别同时乱调。
    const double lead_applied = lead_apply_gain_ * lead_px_;
    const double aim_err_x = std::abs(dx_filt_ + lead_applied - laser_offset_x_);
    const double aim_err_y = std::abs(dy_filt_ - laser_offset_y_);
    const bool in_fire_zone = (aim_err_x <= fire_dz) && (aim_err_y <= fire_dz);
    // ===== ★P1-3 类别迟滞推进 =====
    // 单帧的类别抖动不该改变开火决策。这里维护一个"已确认类别"：
    // 只有连续 cls_switch_confirm_frames_ 帧都指向另一个类别，才真正改锁。
    const int raw_cls = temp_msg.data[2];
    if (raw_cls == locked_cls_)
    {
      cls_streak_ = 0;          // 回到已确认类别 → 候选作废
      cls_candidate_ = raw_cls;
    }
    else if (raw_cls == cls_candidate_)
    {
      if (++cls_streak_ >= cls_switch_confirm_frames_)
      {
        locked_cls_ = raw_cls;  // 新类别稳定成立，改锁
        cls_streak_ = 0;
      }
    }
    else
    {
      cls_candidate_ = raw_cls; // 换了个新候选，重新计数
      cls_streak_ = 1;
    }

    // ★P0-4 仅攻击蓝色靶：确认类别为蓝(cls==1)才允许开火
    //   （红靶仍会被跟踪/显示，但不浪费弹药）
    const bool cls_fire_ok = (!fire_only_blue_) || (locked_cls_ == 1);
  // ★t42（④）类别原因诊断：区分【没看到目标】与【看到了但不允许打】。t41 实车 E4* 停站
  //   30s 内 Python 侧 `类别未确认` 4 次 / `类别不允许(fire_only_blue)` 1 次，而 C++ 侧
  //   只报状态不报原因 ⇒ 现场判断不了是视觉没看见，还是看见了但不允许。
  if (isTracking() && !cls_fire_ok)
  {
    const double cls_diag_ms = (this->now() - cls_diag_last_).seconds() * 1000.0;
    if (cls_diag_ms >= 1000.0)   // 1s 限流，别刷屏
    {
      cls_diag_last_ = this->now();
      std::cout << "==> 诊断(不放行开火)：已锁定但类别不允许 —— locked_cls=" << locked_cls_
                << "（1=蓝 2=红）cls_candidate=" << cls_candidate_
                << " raw_cls=" << static_cast<int>(raw_cls)
                << " fire_only_blue=" << (fire_only_blue_ ? 1 : 0) << " ⇒ "
                << (locked_cls_ == 0 ? "类别未确认(还没连续 N 帧同一类别)"
                                     : "类别不允许(fire_only_blue)")
                << std::endl;
    }
  }
    // ★P0-2 单目标连发上限：防止在同一个靶上把全程弹药一次性倾泻
    const bool per_target_ok =
        (max_shots_per_target_ <= 0) || (shots_on_target_ < max_shots_per_target_);
    const bool ammo_ok = (max_shots_ > 0) && (shots_fired_ < max_shots_) &&
                         per_target_ok && cls_fire_ok;
    const bool cooldown_ok =
        ((this->now() - last_shot_time_).seconds() * 1000.0 >= static_cast<double>(shot_cooldown_ms_));

    if (in_fire_zone && aim_ready_effective && ammo_ok && cooldown_ok && target_confirmed)
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
        shots_on_target_++;   // ★P0-2 单目标计数
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
      // ★t42 去抖滞回（①）：第一次未见时起表，本拍算【连续未见】时长
      if (!lost_since_valid_)
      {
        lost_since_ = this->now();
        lost_since_valid_ = true;
      }
      const double lost_ms = lost_since_valid_
                                 ? (this->now() - lost_since_).seconds() * 1000.0
                                 : 0.0;
      const int need_frames = std::min(std::max(1, lost_frames_to_rescan_),
                                       std::max(1, rescan_min_frames_));
      const bool rescan_debounce_ok =
          (lost_target_count_ >= need_frames) &&
          (lost_ms >= static_cast<double>(std::max(1, rescan_debounce_ms_)));
      if (rescan_debounce_ok)
      {
        lost_target_count_ = 0;
        locked_cls_ = 0;   // ★P1-3 目标丢了，类别重新确认
        cls_streak_ = 0;
        cls_candidate_ = 0;
        if (scan_continuous_)
        {
          // ★t42（②）：目标脱离视野后【接续扫掠】，不再重跑 1500ms 归位段
          //   （t41 实车：30s 内 31 次归位仅 1 次成功 ⇒ 归位段把扫掠覆盖率吃光）
          std::cout << "==> 目标脱离视野（连续 " << need_frames << " 帧 / "
                    << static_cast<int>(lost_ms) << " ms 未见）→ 接续扫掠（不重归位）"
                    << std::endl;
          startScan(true, /*home=*/false);
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

// ★P0-1 把水平轴限制在【扫描范围】内，而不是硬件极限(100~3995)。
//   实测故障：车体偏航时跟踪会一路把云台推到 3988(≈170.6°)，远超 ±90° 扫描范围，
//   最后撞在机械限位上堵转，表现为"云台定住不动"。
int turn_on_robot::clampYawToScan(int tick) const
{
  const int t1 = clampYawTick(gimbal_center_tick_ + static_cast<int>(std::lround(
      scan_yaw_min_deg_ * yaw_ticks_per_deg_ * yaw_deg_sign_)));
  const int t2 = clampYawTick(gimbal_center_tick_ + static_cast<int>(std::lround(
      scan_yaw_max_deg_ * yaw_ticks_per_deg_ * yaw_deg_sign_)));
  const int lo = std::min(t1, t2);
  const int hi = std::max(t1, t2);
  return std::max(lo, std::min(hi, tick));
}

// ★P0-1 竖直轴同理：限制在扫描俯仰范围(层角度 + 整体偏移)内
int turn_on_robot::clampPitchToScan(int tick) const
{
  double lo_deg = scan_pitch_offset_deg_;
  double hi_deg = scan_pitch_offset_deg_;
  if (!scan_pitch_levels_deg_.empty())
  {
    lo_deg += *std::min_element(scan_pitch_levels_deg_.begin(), scan_pitch_levels_deg_.end());
    hi_deg += *std::max_element(scan_pitch_levels_deg_.begin(), scan_pitch_levels_deg_.end());
  }
  // ★关键：在扫描俯仰范围之外再留出跟踪修正余量。
  //   踩过的坑：scan_pitch_levels 只剩一层时(如 [0.0]，即"只做水平扫描")，
  //   min==max -> lo_deg == hi_deg -> 允许区间宽度为 0，
  //   于是视觉伺服的每一个俯仰修正都被钳成同一个值 —— 云台俯仰彻底僵死。
  //   表现就是：靶贴在画面下沿，scan_pitch_offset_deg 从 -8 改到 -40 都毫无变化。
  lo_deg -= tracking_pitch_margin_deg_;
  hi_deg += tracking_pitch_margin_deg_;

  const int t1 = clampPitchTick(gimbal_center_tick_ + static_cast<int>(std::lround(
      lo_deg * pitch_ticks_per_deg_ * pitch_deg_sign_)));
  const int t2 = clampPitchTick(gimbal_center_tick_ + static_cast<int>(std::lround(
      hi_deg * pitch_ticks_per_deg_ * pitch_deg_sign_)));
  const int lo = std::min(t1, t2);
  const int hi = std::max(t1, t2);
  return std::max(lo, std::min(hi, tick));
}


// 当前扫描目标对应的水平角(相对车体前方，右为正)。
// 连续匀速往复扫描：目标点就是当前方向上的那一端，云台以限定的巡逻速度
// 匀速扫过去，到位后由 advanceScanIndex() 反向 —— 不再逐点步进停留。
double turn_on_robot::scanYawDeg() const
{
  return (scan_yaw_dir_ > 0) ? scan_yaw_max_deg_ : scan_yaw_min_deg_;
}

// 竖直层的数量
int turn_on_robot::pitchLevelCount() const
{
  return scan_pitch_levels_deg_.empty() ? 1 : static_cast<int>(scan_pitch_levels_deg_.size());
}

// 当前竖直层对应的角度(相对水平面，上为正) = 层列表角度 + 整体俯仰偏移。
// scan_pitch_offset_deg_ 兜底默认 +10.0（t52，用户 2026-10-14 指令"水平扫描向下压约 10°"；
//   ★2026-10-15 现场实测：+ 才向下压、- 实际向上抬，符号约定已更正）。真源是 robot.yaml。
double turn_on_robot::scanPitchDeg() const
{
  double base = 0.0;
  if (!scan_pitch_levels_deg_.empty())
  {
    const int idx = std::min(scan_pitch_idx_, pitchLevelCount() - 1);
    base = scan_pitch_levels_deg_[idx];
  }
  return base + scan_pitch_offset_deg_;
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

  // P2-2 跟踪速度上限：优先用显式配置的绝对刻度值；未配置(<=0)才按巡逻速度派生，
  //      避免"降扫描速度把跟踪也一起拖慢"。
  const double limit = (track_speed_max_ > 0)
                           ? static_cast<double>(track_speed_max_)
                           : static_cast<double>(patrolSpeed()) * track_speed_max_ratio_;
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

// 连续往复扫描推进：到位即反向。一趟 = 从一端匀速扫到另一端。
// scan_passes_ 趟(默认 2 = 左一趟 + 右一趟)算一轮；连续模式下扫完一轮自动重来。
void turn_on_robot::advanceScanIndex()
{
  scan_yaw_dir_ = -scan_yaw_dir_; // 到达端点，立即反向
  scan_pass_done_++;

  // 每扫满 scan_passes_ 趟换一层
  if (scan_pass_done_ % scan_passes_ == 0)
  {
    scan_pitch_idx_ = (scan_pitch_idx_ + 1) % pitchLevelCount();
  }

  if (scan_pass_done_ >= scan_passes_ * pitchLevelCount())
  {
    scan_finished_ = true; // 一轮扫完
    // ★t25：连续扫描轮数记账 —— 扫满 scan_continuous_cycles_ 轮仍无目标就回 IDLE 归位。
    //   为什么必须设上限（用户口径）：旧版 scan_continuous_ 无轮数上限 ⇒ 云台会一直往复扫下去，
    //   在"靶不在视野/已打完"的场合白耗电、白磨损、并让现场误以为"云台在乱转"。
    ++scan_cycles_done_;
  }
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
//
// ★★ 卡 t25（用户口径）：**进入扫描前先归位到"正前方 + 水平"**，归位到位（或超时）后才真正开扫。
//   为什么：旧版 startScan() 直接 applyScanTarget()，即"从当前任意角度直接朝第一个端点扫"——
//   若上一轮 TRACK 结束时云台指在侧向（甚至 ±150° 端），第一个横扫点会立刻"反向"，
//   扫描行程与耗时都不可预测。归位段把起点钉在已知姿态（yaw=0、pitch=0）。
//   归位由**本节点自己**下发（与 IDLE 归位同一套 Position 语义），不依赖外部指令。
// ★t42（②）home=false = 重扫接续：不重跑归位段、保留扫掠进度、重置轮数预算，直接进扫描相。
//   为什么：t41 实车 30s 内触发 31 次归位段、仅 1 次到位 ⇒ 云台在【归位→丢靶→再归位】
//   里空转、扫掠覆盖率≈0；现场表现即用户说的【到场时云台没有开始扫描】。
//   轮数预算必须重置：否则 t25 的 3 轮上限会立刻把状态打回 IDLE ⇒ 又走一遍归位段（改不动）。
void turn_on_robot::startScan(bool continuous, bool home)
{
  scan_continuous_ = continuous;
  if (!home)
  {
    scan_cycles_done_ = 0;      // 接续扫掠：给满 3 轮预算，避免立刻回落 IDLE 再归位
    scan_finished_ = false;
    last_scan_step_time_ = this->now();
    std::cout << "==> 重扫接续（跳过归位段）：从 yaw 点 #" << scan_yaw_idx_
              << " 继续（轮数预算已重置）" << std::endl;
    enterScanPhase();
    return;
  }
  scan_yaw_idx_ = 0;
  scan_yaw_dir_ = 1;
  scan_pitch_idx_ = 0;
  scan_pass_done_ = 0;
  scan_cycles_done_ = 0;              // ★t25：新的连续扫描序列，轮数从 0 起
  scan_finished_ = false;
  last_scan_step_time_ = this->now();

  // ★t25：先归位，不立刻开扫；归位段在 CaremaMontorControl() 里跑（aim_state_ 仍是 IDLE，
  //   于是 IDLE 分支的"每帧归位"正好就是归位段的执行体，二者共用一套逻辑）。
  scan_homing_ = true;
  scan_home_deadline_ = this->now() + rclcpp::Duration::from_seconds(
      std::max(1, scan_home_timeout_ms_) / 1000.0);
  std::cout << "==> 进入扫描前先归位（正前方 + 水平），超时 " << scan_home_timeout_ms_
            << " ms" << std::endl;
}

// ★t25：归位段的一拍。返回 true = 归位完成（到位或超时），调用方随后 enterScanPhase()。
// 判据：水平/俯仰反馈刻度与"零位 + 0°"的距离 ≤ scan_arrive_tol_（同"到位"口径）；
// 超时兜底用 scan_home_timeout_ms_ —— 电机卡住/反馈缺失时也**绝不**卡在归位段。
bool turn_on_robot::scanHomingStep()
{
  if (!scan_homing_)
  {
    return false;
  }
  const double yaw_home_tick = static_cast<double>(gimbal_center_tick_);
  const double pitch_home_tick = static_cast<double>(gimbal_center_tick_);
  const double d_yaw = std::fabs(static_cast<double>(curYuntai_feedback_data.Position_1) - yaw_home_tick);
  const double d_pitch = std::fabs(static_cast<double>(curYuntai_feedback_data.Position_0) - pitch_home_tick);
  const bool arrived = (d_yaw <= static_cast<double>(scan_arrive_tol_)) &&
                       (d_pitch <= static_cast<double>(scan_arrive_tol_));
  const bool timeout = (this->now() >= scan_home_deadline_);
  if (arrived || timeout)
  {
    std::cout << "==> 归位段结束（" << (arrived ? "已到位" : "超时兜底")
              << "）：yaw偏差 " << d_yaw << " tick / pitch偏差 " << d_pitch
              << " tick ⇒ 开始扫描" << std::endl;
    scan_homing_ = false;
    return true;
  }
  // 尚未到位：本拍仍由 IDLE 归位逻辑下发目标（回正 + 水平），不做别的事
  return false;
}

// ★t25：真正进入扫描阶段（下发第一个横扫点并切状态）。
void turn_on_robot::enterScanPhase()
{
  scan_homing_ = false;
  last_scan_step_time_ = this->now();
  applyScanTarget(); // 立即下发第一个扫描点（yaw = 端点、pitch = 基准）
  setAimState(AIM_SCAN);
}

// 扫描请求 /nav/scan_request（UInt8）：
//   2 及以上 -> 持续扫描：一轮扫完自动重来，**最多 scan_continuous_cycles_ 轮**（t25，默认 3），
//               到达上限仍无目标 ⇒ 回 IDLE 归位；锁定后转跟随；目标脱离视野再自动重开
//   1        -> 单次扫描：扫完 scan_passes_ 趟就回 IDLE（循迹到达点位时用这个）★t25 语义未变
//   0        -> 停止扫描，回到 IDLE
//   已在 AIM_TRACK 时一律忽略，不打断跟随。
//   ★t25：无论 1 还是 2，**进入扫描前都先走归位段**（正前方 + 水平，超时兜底）。
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
        // ★t42（②）：视觉静默同样只【接续扫掠】（不重归位）
        std::cout << "==> 视觉静默 " << static_cast<int>(silent_ms)
                  << " ms → 接续扫掠（不重归位）" << std::endl;
        startScan(true, /*home=*/false);
      }
      else
      {
        setAimState(AIM_IDLE);
      }
    }
    return;
  }

  // ===== 状态二：空闲（AIM_IDLE）=====
  // 主动归位到"正前方 + 扫描基准俯仰"，而不是什么都不做。
  //
  // ★为什么必须归位（踩过的坑）：
  //   旧版在 IDLE 时直接 return，于是云台【冻结在最后一次的任意姿态】上。
  //   后果是：改了 scan_pitch_offset_deg 却看不出任何变化 —— 因为云台压根没回基准位，
  //   它停在上一次扫描/跟踪结束时的那个角度上。这也是"调参数无效"的一半原因。
  //   归位后：① 改参数立刻可见 ② 下次扫描/锁定从已知姿态起步，收敛更快。
  //
  //   每帧持续下发是安全的：下位机是位置控制，到位后自然停住，不会抖。
  if (aim_state_ != AIM_SCAN)
  {
    // ★★ 卡 t25：**进入扫描前的归位段**。
    //   归位段期间 aim_state_ 仍是 IDLE ⇒ 就靠本分支的"每帧归位"当执行体（目标 yaw=行程中点=0°、
    //   pitch=基准=0°），与用户要求的"先复位到正前 + 水平"完全一致；归位完成（到位或超时）后
    //   由 scanHomingStep() 返回 true，这里立刻转进真正的扫描阶段。
    if (scan_homing_ && scanHomingStep())
    {
      enterScanPhase();
      return;                              // 本拍已切到 SCAN，下一拍起走扫描分支
    }
    const double yaw_mid_deg = (scan_yaw_min_deg_ + scan_yaw_max_deg_) / 2.0;
    moveBaseControl.Position_1 = clampYawToScan(gimbal_center_tick_ +
        static_cast<int>(std::lround(yaw_mid_deg * yaw_ticks_per_deg_ * yaw_deg_sign_)));
    moveBaseControl.Position_0 = clampPitchToScan(gimbal_center_tick_ +
        static_cast<int>(std::lround(scanPitchDeg() * pitch_ticks_per_deg_ * pitch_deg_sign_)));

    const int home_speed = std::max(1, static_cast<int>(patrolSpeed() * idle_homing_speed_ratio_));
    moveBaseControl.Speed_0 = home_speed;
    moveBaseControl.Speed_1 = home_speed;
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
    // 超时兜底：必须大于"以巡逻速度匀速扫完整个水平行程"所需时间，
    // 否则连续扫描会在半路被误判超时而提前反向，破坏匀速往复。
    const double span_ticks =
        std::fabs(scan_yaw_max_deg_ - scan_yaw_min_deg_) * yaw_ticks_per_deg_;
    const double traverse_ms =
        span_ticks / std::max(1.0, static_cast<double>(patrolSpeed())) * 1000.0;
    const double timeout_ms = std::max<double>(static_cast<double>(scan_dwell_ms_) * 3.0,
                                               traverse_ms * 1.5);
    const bool timeout = elapsed_ms >= timeout_ms;

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
          // ★t25：连续扫描**轮数上限**（用户口径：最多 3 轮，找不到就回 IDLE 归位，不无限扫）。
          //   scan_continuous_cycles_ <= 0 表示"不限"（= 旧行为，保留可回退）。
          if (scan_continuous_cycles_ > 0 && scan_cycles_done_ >= scan_continuous_cycles_)
          {
            setAimState(AIM_IDLE);   // 回 IDLE ⇒ 下一拍起由 IDLE 分支每帧归位到正前+水平
            std::cout << "==> 连续扫描已达上限 " << scan_continuous_cycles_
                      << " 轮仍无目标 ⇒ 回空闲归位（不再无限扫描）" << std::endl;
            return;
          }
          // 持续扫描模式：立刻从头再来一轮，不停下来
          scan_yaw_idx_ = 0;
          scan_yaw_dir_ = 1;
          scan_pitch_idx_ = 0;
          last_scan_step_time_ = this->now();
          // ★t42（③）每轮扫掠一条进度日志（轮次 + 扫掠角度）：现场一眼可判【云台在扫】
          //   （t41 之前只有一句轮次、没有角度，【在扫 / 在归位】分不清）
          std::cout << "==> 扫描进度：完成第 " << scan_cycles_done_ << " 轮，开始第 "
                    << (scan_cycles_done_ + 1) << " / "
                    << (scan_continuous_cycles_ > 0 ? scan_continuous_cycles_ : 0)
                    << " 轮（0=不限）；扫掠范围 yaw [" << scan_yaw_min_deg_ << ", "
                    << scan_yaw_max_deg_ << "]° · " << scan_yaw_dirs_
                    << " 点 · pitch " << scan_pitch_offset_deg_ << "°" << std::endl;
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

// 车体偏航角速度采集 —— 用于云台的"惯性凝视稳定"前馈。
// /imu_data 由 dzactuator 发布（已做零偏校正 + 滤波，队列深度 1，延迟最低）。
void turn_on_robot::callback_shoot_reset(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data == 0)
  {
    return;
  }
  shots_fired_ = 0;
  shots_on_target_ = 0;
  fire_hold_count_ = 0;
  std_msgs::msg::Int32 countmsg;
  countmsg.data = 0;
  pub_shot_count->publish(countmsg);
  std::cout << "==> ★ 射击计数已重置：shots_fired_=0，可用弹药恢复为 "
            << max_shots_ << " 发" << std::endl;
}

void turn_on_robot::callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_omega_z_ = msg->angular_velocity.z; // rad/s，绕车体 Z 轴的偏航角速度
  last_imu_time_ = this->now();
}

// ================= ★新版本补丁：两键开火 + 提前量的三个回调 =================
// 设计原则：回调只**更新状态**，不做决策 —— 决策集中在 CaremaMontorControl 的开火判定里，
// 这样"什么时候开火"只有一个地方可读、可测、可回退（use_external_fire_gate_ = false 即旧行为）。
void turn_on_robot::callback_fire_gate(const std_msgs::msg::UInt8::SharedPtr msg)
{
  const bool on = (msg->data != 0);
  if (on != external_gate_)
  {
    std::cout << (on ? "★ 收到【开火放行】/aim/fire_gate=1（任务级：已到站停稳）"
                     : "★ 收到【开火收回】/aim/fire_gate=0")
              << std::endl;
  }
  external_gate_ = on;
}

void turn_on_robot::callback_fire_ready(const std_msgs::msg::UInt8::SharedPtr msg)
{
  const bool on = (msg->data != 0);
  if (on != external_ready_)
  {
    std::cout << (on ? "★ 收到【瞄准合格】/aim/fire_ready=1（预测落点在开火框内）"
                     : "★ 收到【瞄准未合格】/aim/fire_ready=0")
              << std::endl;
  }
  external_ready_ = on;
}

void turn_on_robot::callback_lead_x(const std_msgs::msg::Float32::SharedPtr msg)
{
  // 提前量（px，正 = 目标向右）。范围守卫：异常值不许污染伺服（现场标定可能发出离谱数）。
  double v = static_cast<double>(msg->data);
  if (!std::isfinite(v) || std::abs(v) > 400.0)
  {
    v = 0.0;
  }
  lead_px_ = v;
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
  // ★ 射击计数重置：把 shots_fired_ / shots_on_target_ 归零并重新发布 /shoot_count=0。
  //   为什么必须有它：shots_fired_ 在头文件里明确写着"全程累计，不重置"，
  //   一旦打满 max_shots_(10) 就永远不再开火，只能重启节点 —— 调试时极难受。
  //   有了这个通道，网页上点一下就能把弹药"加满"继续测。
  sub_shoot_reset = this->create_subscription<std_msgs::msg::UInt8>(
      "/shoot_reset", 10, std::bind(&turn_on_robot::callback_shoot_reset, this, std::placeholders::_1));
  pub_aim_status = this->create_publisher<std_msgs::msg::Int32>("/aim/status", 10);

  // Set the velocity control command callback function
  // 速度控制命令订阅回调函数设置
  sub_offset_center = this->create_subscription<std_msgs::msg::Int32MultiArray>("offset_center", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&turn_on_robot::callback_offset_center, this, std::placeholders::_1));
  sub_camemo_feedback = this->create_subscription<std_msgs::msg::Int32MultiArray>("camemo_feedback", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&turn_on_robot::callback_camemo_feedback, this, std::placeholders::_1));
  // 扫描请求用默认(可靠) QoS —— best_effort 订阅无法匹配 path_executor 默认的可靠发布者
  sub_scan_request = this->create_subscription<std_msgs::msg::UInt8>("/nav/scan_request", 10, std::bind(&turn_on_robot::callback_scan_request, this, std::placeholders::_1));
  // ★浏览器遥控：手动接管云台（1=接管，0=释放）
  //   用 lambda 而不是成员函数，省掉头文件里的一处声明。
  sub_manual_gimbal = this->create_subscription<std_msgs::msg::UInt8>(
      "nav/manual_gimbal", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        const bool on = (msg->data != 0);
        if (on == manual_override_) {
          return;
        }
        manual_override_ = on;
        std::cout << (on ? "★ 收到【云台手动接管】—— 停止下发云台位置指令，交给遥控器"
                         : "★ 收到【云台手动释放】—— 恢复云台自动控制")
                  << std::endl;
      });
  // /imu_data 由 dzactuator 发布（已滤波 + 零偏校正，队列深度 1，延迟最低），用于车体转动前馈
  sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("/imu_data", 1, std::bind(&turn_on_robot::callback_imu, this, std::placeholders::_1));

  // ================= ★新版本补丁：两键开火 + 提前量 =================
  // 队列深度 1 + BEST_EFFORT：只要最新值（放行门是"当前状态"而非事件流，
  // 积压旧值会让云台按过期许可开火 —— 那正是要避免的事）。
  sub_fire_gate = this->create_subscription<std_msgs::msg::UInt8>(
      "/aim/fire_gate", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&turn_on_robot::callback_fire_gate, this, std::placeholders::_1));
  sub_fire_ready = this->create_subscription<std_msgs::msg::UInt8>(
      "/aim/fire_ready", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&turn_on_robot::callback_fire_ready, this, std::placeholders::_1));
  sub_lead_x = this->create_subscription<std_msgs::msg::Float32>(
      "/aim/lead_x", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&turn_on_robot::callback_lead_x, this, std::placeholders::_1));


  last_voice_msg.data = 0;

  RCLCPP_INFO(this->get_logger(), "Data ready");

  // 瞄准扫描参数
  this->declare_parameter("scan_yaw_dirs", 8);			// 水平横扫点数
  this->declare_parameter("scan_yaw_min_deg", -160.0);	// 水平最小角(度)
  this->declare_parameter("scan_yaw_max_deg", 160.0);	// 水平最大角(度)
  // 竖直角度列表：只做水平扫描，竖直轴固定在 0°(水平)。如需多层扫描可改成 {15.0, 0.0, -10.0}

  this->declare_parameter("scan_pitch_levels",
                          std::vector<double>{0.0});
  this->declare_parameter("yaw_deg_sign", 1.0);			// 水平角度正方向->刻度(反了改-1)
  this->declare_parameter("pitch_deg_sign", 1.0);		// 竖直角度正方向->刻度(反了改-1)
  this->declare_parameter("tracking_pitch_margin_deg", 12.0); // ★跟踪俯仰修正余量(度)
  this->declare_parameter("idle_homing_speed_ratio", 0.5);    // ★空闲归位速度比例
  this->declare_parameter("gimbal_center_tick", 2047);	// 零位刻度
  this->declare_parameter("yaw_ticks_per_deg", 11.378);	  // 水平刻度/度 = 4096/360
  this->declare_parameter("pitch_ticks_per_deg", 11.378); // 竖直刻度/度 = 4096/360
  this->declare_parameter("gimbal_yaw_tick_min", 100);	  // 水平轴刻度下限(留边界余量)
  this->declare_parameter("gimbal_yaw_tick_max", 3995);	  // 水平轴刻度上限
  this->declare_parameter("gimbal_pitch_tick_min", 950);	  // 竖直轴刻度下限(下位机钳位)
  this->declare_parameter("gimbal_pitch_tick_max", 2400); // 竖直轴刻度上限(下位机钳位)
  this->declare_parameter("gimbal_max_speed", 3700);		  // 云台最大转速
  this->declare_parameter("scan_pitch_offset_deg", +10.0); // 俯仰整体偏移(度，★实测：+向下压、-向上抬；t52 与 robot.yaml 单一真源对齐)
  this->declare_parameter("max_shots_per_target", 3);  // P0-2 单目标连发上限(<0 不限)
  this->declare_parameter("fire_only_blue", true);     // P0-4 仅攻击蓝色靶
  this->declare_parameter("track_speed_max", 0);       // P2-2 tracking cap (ticks/s, 0=derive)
  this->declare_parameter("patrol_speed_ratio", 0.3);	  // 巡逻(扫描)速度占最大转速比例
  this->declare_parameter("scan_dwell_ms", 300);		  // 每点最短驻留(ms)
  this->declare_parameter("scan_arrive_tol", 30);		  // 到位容差(刻度)
  this->declare_parameter("scan_passes", 3);			  // 全向扫描趟数(1左+1右=2)
  // ★卡 t25：连续扫描轮数上限（默认 3，用户口径"最多 3 循环"）与进入扫描前的归位超时
  this->declare_parameter("scan_continuous_cycles", 3);  // 连续扫描最多几轮；<=0 = 不限(旧行为)
  this->declare_parameter("scan_home_timeout_ms", 1500); // 进扫描前归位段超时(ms)：卡住也要进扫描
  this->declare_parameter("lost_frames_to_rescan", 60);
  this->declare_parameter("rescan_min_frames", 10);    // ★t42 去抖：连续未见帧数下限
  this->declare_parameter("rescan_debounce_ms", 400);  // ★t42 去抖：连续未见时长下限(ms)	  // 跟踪中丢帧回扫描阈值
  this->declare_parameter("vision_timeout_ms", 1000);	  // 视觉静默超时(ms)
  this->declare_parameter("ema_alpha", 0.3);			  // 视觉一阶低通滤波系数
  this->declare_parameter("track_deadzone_px", 2);		  // 跟踪死区(像素)
  this->declare_parameter("fine_zone_px", 20);			  // 进入精调档的偏差阈值(像素)
  this->declare_parameter("fine_kp_scale", 0.5);		  // 精调档 Kp 缩放
  this->declare_parameter("fine_kd_scale", 2.0);		  // 精调档 Kd 放大
  this->declare_parameter("track_kp", 8.0);				  // PD 比例增益
  this->declare_parameter("track_kd", 3.0);				  // PD 微分增益(阻尼)
  this->declare_parameter("track_speed_max_ratio", 0.6);  // 跟踪速度上限占巡逻速度比例
  this->declare_parameter("fire_deadzone_px", 8);		  // 固定开火死区(像素)
  this->declare_parameter("fire_dz_dynamic", true);		  // 启用动态开火框(按距离)
  this->declare_parameter("fire_dz_far_px", 3.5);		  // 远距离开火死区(像素)
  this->declare_parameter("fire_dz_near_px", 3.5);		  // 近距离开火死区(像素)
  this->declare_parameter("box_w_far", 30.0);			  // 远处目标框宽(标定值)
  this->declare_parameter("box_w_near", 120.0);			  // 近处目标框宽(标定值)
  this->declare_parameter("fire_real_speed_max", 20);	  // 实际速度静止阈值
  this->declare_parameter("fire_speed_max", 50);		  // 开火时指令速度上限
  this->declare_parameter("fire_hold_frames", 5);		  // 连续满足多少帧才开火
  this->declare_parameter("fire_use_image_stable", true);   // 开火稳定判据：true=画面稳定(移动中可开火)
  // ★新版本补丁：两键开火 + 提前量（默认关闭 → 不改旧行为；配置里开）
  this->declare_parameter("use_external_fire_gate", false); // true=开火由 /aim/fire_gate + /aim/fire_ready 决定
  this->declare_parameter("lead_apply_gain", 0.0);          // 0=不施加提前量；1=完全施加 /aim/lead_x
  this->declare_parameter("fire_stable_px_per_frame", 1.5); // 画面稳定阈值(像素/帧)
  this->declare_parameter("predict_gain", 0.5);			  // 目标移动预判强度(帧，0=关闭)
  this->declare_parameter("laser_offset_x", 0.0);		  // 激光落点水平偏移(标定值,像素)
  this->declare_parameter("laser_offset_y", 0.0);		  // 激光落点竖直偏移(标定值,像素)
  this->declare_parameter("max_shots", 10);				  // 最大发射次数(<=0 不限)
  this->declare_parameter("shot_cooldown_ms", 500);		  // 两发之间的最小间隔(ms)
  this->declare_parameter("aspect_min", 0.75);			  // 目标框宽高比下限(只跟踪近似1:1)
  this->declare_parameter("aspect_max", 1.35);			  // 目标框宽高比上限
  this->declare_parameter("target_confirm_frames", 4);	  // 连续识别多少帧才算"扫描确认"(之后才允许射击)
  this->declare_parameter("cls_switch_confirm_frames", 5); // ★P1-3 类别改锁需连续多少帧
  this->declare_parameter("loop_hz_track", 100);	  // ★流畅度 跟踪时主循环频率(Hz)
  this->declare_parameter("loop_hz_idle", 50);	  // ★流畅度 扫描/空闲时主循环频率(Hz)
  // ===== 车体转动前馈（惯性凝视稳定）=====
  // 车体转动时云台若不反向补偿，瞄准点会跟着车体一起转，目标会往画面一侧持续漂。
  // 增益默认 0（关闭）。标定好方向后开到 0.6~1.0。
  this->declare_parameter("gyro_ff_gain", 0.0);		  // 前馈增益：0=关闭，1.0=完全补偿
  this->declare_parameter("gyro_ff_sign", 1.0);		  // 补偿方向：反了就改 -1.0
  this->declare_parameter("imu_ff_timeout_ms", 200);	  // IMU 超过这么久没数据就暂停前馈

  this->get_parameter("scan_yaw_dirs", scan_yaw_dirs_);
  this->get_parameter("scan_yaw_min_deg", scan_yaw_min_deg_);
  this->get_parameter("scan_yaw_max_deg", scan_yaw_max_deg_);
  this->get_parameter("scan_pitch_levels", scan_pitch_levels_deg_);
  // scan_pitch_offset_deg 的 declare/get 配对由 healthcheck 静态检查兜底（见 readme 第34条）。
  this->get_parameter("scan_pitch_offset_deg", scan_pitch_offset_deg_);
  this->get_parameter("yaw_deg_sign", yaw_deg_sign_);
  this->get_parameter("pitch_deg_sign", pitch_deg_sign_);
  this->get_parameter("tracking_pitch_margin_deg", tracking_pitch_margin_deg_);
  this->get_parameter("idle_homing_speed_ratio", idle_homing_speed_ratio_);
  this->get_parameter("gimbal_center_tick", gimbal_center_tick_);
  this->get_parameter("yaw_ticks_per_deg", yaw_ticks_per_deg_);
  this->get_parameter("pitch_ticks_per_deg", pitch_ticks_per_deg_);
  this->get_parameter("gimbal_yaw_tick_min", gimbal_yaw_tick_min_);
  this->get_parameter("gimbal_yaw_tick_max", gimbal_yaw_tick_max_);
  this->get_parameter("gimbal_pitch_tick_min", gimbal_pitch_tick_min_);
  this->get_parameter("gimbal_pitch_tick_max", gimbal_pitch_tick_max_);
  this->get_parameter("gimbal_max_speed", gimbal_max_speed_);
  this->get_parameter("patrol_speed_ratio", patrol_speed_ratio_);
  this->get_parameter("max_shots_per_target", max_shots_per_target_);
  this->get_parameter("fire_only_blue", fire_only_blue_);
  this->get_parameter("track_speed_max", track_speed_max_);
  this->get_parameter("scan_dwell_ms", scan_dwell_ms_);
  this->get_parameter("scan_arrive_tol", scan_arrive_tol_);
  this->get_parameter("scan_passes", scan_passes_);
  if (scan_passes_ < 1)
  {
    scan_passes_ = 1; // 至少扫一趟，否则永远扫不完
  }
  // ★卡 t25：连续扫描轮数上限 + 进扫描前归位超时（<=0 的轮数 = 不限，保持旧行为可回退）
  this->get_parameter("scan_continuous_cycles", scan_continuous_cycles_);
  this->get_parameter("scan_home_timeout_ms", scan_home_timeout_ms_);
  if (scan_home_timeout_ms_ < 1)
  {
    scan_home_timeout_ms_ = 1; // 至少 1ms，避免"0 超时"导致归位段完全不生效
  }
  this->get_parameter("lost_frames_to_rescan", lost_frames_to_rescan_);
  this->get_parameter("rescan_min_frames", rescan_min_frames_);
  this->get_parameter("rescan_debounce_ms", rescan_debounce_ms_);
  if (rescan_min_frames_ < 1)
  {
    rescan_min_frames_ = 1;
  }
  if (rescan_debounce_ms_ < 1)
  {
    rescan_debounce_ms_ = 1;
  }
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
  // ★新版本补丁：两键开火 + 提前量
  this->get_parameter("use_external_fire_gate", use_external_fire_gate_);
  this->get_parameter("lead_apply_gain", lead_apply_gain_);
  if (lead_apply_gain_ < 0.0) { lead_apply_gain_ = 0.0; }   // 负增益无物理意义，夹到 0
  if (lead_apply_gain_ > 2.0) { lead_apply_gain_ = 2.0; }   // 上限 2 倍，防现场手滑
  this->get_parameter("fire_stable_px_per_frame", fire_stable_px_per_frame_);
  this->get_parameter("predict_gain", predict_gain_);
  this->get_parameter("laser_offset_x", laser_offset_x_);
  this->get_parameter("laser_offset_y", laser_offset_y_);
  this->get_parameter("max_shots", max_shots_);
  this->get_parameter("shot_cooldown_ms", shot_cooldown_ms_);
  this->get_parameter("aspect_min", aspect_min_);
  this->get_parameter("aspect_max", aspect_max_);
  this->get_parameter("target_confirm_frames", target_confirm_frames_);
  this->get_parameter("cls_switch_confirm_frames", cls_switch_confirm_frames_);
  this->get_parameter("loop_hz_track", loop_hz_track_);
  this->get_parameter("loop_hz_idle", loop_hz_idle_);
  this->get_parameter("gyro_ff_gain", gyro_ff_gain_);
  this->get_parameter("gyro_ff_sign", gyro_ff_sign_);
  this->get_parameter("imu_ff_timeout_ms", imu_ff_timeout_ms_);
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
  // ★★ 2026-10-11（用户指令「云台的激活设置在启动命令之后」）：改回开机 IDLE ——
  //    上电即扫会让云台在起栈后、发车命令前就开始左右往复（标定时无意义还扰人）。
  //    激活时机 = path_executor 在任务开始（发车命令）时发 /nav/scan_request=2；
  //    云台节点先于循迹起（auto_start_delay=10s 更在其后），订阅早已建立，无丢消息风险。
  //    任务结束（FINISH）path_executor 会发 scan_request=1（终点全向扫一次）后自然回 IDLE。
  //    （历史：曾"上电即扫"防 scan_request=2 丢失后云台卡死不动；现激活/停止均由循迹显式下发，
  //     不再需要上电兜底。临时想彻底冻住云台：/nav/manual_gimbal=1 遥控接管。）
  aim_state_ = AIM_IDLE;
  scan_continuous_ = false;
  scan_yaw_idx_ = 0;
  scan_yaw_dir_ = 1;
  scan_pitch_idx_ = 0;
  scan_pass_done_ = 0;
  scan_finished_ = false;
  lost_target_count_ = 0;
  last_scan_step_time_ = this->now();
  last_vision_time_ = this->now();

  // ★★ 必须显式初始化，否则一进入跟踪就崩溃 ★★
  //    rclcpp::Time 的**默认构造函数**用的是 RCL_SYSTEM_TIME(2)，
  //    而 this->now() 走节点时钟 RCL_ROS_TIME(1)。两者相减会抛
  //      "can't subtract times with different time sources [1 != 2]"
  //    并被 terminate 掉(SIGABRT)。以前旧模型几乎认不出目标、很少进 TRACK，
  //    所以这个坑一直没暴露；换成灵敏模型后一锁定目标就必崩。
  last_track_time_ = this->now(); // 跟踪前馈 dt 的上一次时刻(第 102 行使用)
  last_imu_time_ = this->now();   // IMU 新鲜度判断(第 110 行使用)

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
              "瞄准: 连续往复 %.0f°~%.0f° | 巡逻 %d(最大速度的%.0f%%) | 俯仰偏移 %+.1f° | 跟踪上限 %d | 跟踪死区 %dpx | Kp %.1f/Kd %.1f | 滤波 %.2f",
              scan_yaw_min_deg_, scan_yaw_max_deg_, patrolSpeed(),
              patrol_speed_ratio_ * 100.0, scan_pitch_offset_deg_,
              (track_speed_max_ > 0 ? track_speed_max_ : static_cast<int>(patrolSpeed() * track_speed_max_ratio_)),
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
              "流畅度: 主循环 TRACK %dHz / 其余 %dHz | 类别改锁需连续 %d 帧(抗抖动)",
              loop_hz_track_, loop_hz_idle_, cls_switch_confirm_frames_);

  RCLCPP_INFO(this->get_logger(),
              "凝视稳定: 车体转动前馈 %s (增益 %.2f, 方向 %+.0f, IMU超时 %dms) | 输入源 /imu_data",
              (gyro_ff_gain_ != 0.0 ? "已启用" : "已关闭"),
              gyro_ff_gain_, gyro_ff_sign_, imu_ff_timeout_ms_);

  RCLCPP_INFO(this->get_logger(),
              "自瞄: 状态机 IDLE/SCAN/TRACK | 点位全向扫描 %d 趟 | 扫描速度 %d | 循迹请求 /nav/scan_request | 状态上报 /aim/status",
              scan_passes_, patrolSpeed());

  // ★卡 t25：扫描升级口径必须在开机横幅里可见（"静默失败"是本项目最大的坑）——
  //   进扫描先归位 + 连续扫描轮数上限 + 扫描范围/速度。
  RCLCPP_INFO(this->get_logger(),
              "扫描(t25): 巡航 %+.0f..%+.0f° 速度比 %.2f (=%d tick/s) 俯仰偏置 %+.1f° | "
              "进入扫描先归位到正前+水平(超时 %dms) | 连续扫描最多 %d 轮"
              "(<=0=不限) | 找到目标即转 TRACK",
              scan_yaw_min_deg_, scan_yaw_max_deg_, patrol_speed_ratio_, patrolSpeed(),
              scan_pitch_offset_deg_, scan_home_timeout_ms_, scan_continuous_cycles_);

  // ★新版本补丁：两键开火 + 提前量（开机横幅必须能一眼看清是否生效 —— "静默失败"是本项目最大的坑）
  RCLCPP_INFO(this->get_logger(),
              "开火权限(★补丁): 两键开火 %s | 提前量施加增益 %.2f (输入 /aim/lead_x) | 订阅 /aim/fire_gate + /aim/fire_ready",
              (use_external_fire_gate_ ? "已启用（/aim/fire_gate + /aim/fire_ready 同时为 1 才开火）"
                                       : "已关闭（沿用旧的画面静止判据）"),
              lead_apply_gain_);


  std::cout << FRED("Copyright©2016-2020 dzrknn_dataprocess. All rights reserved ") << std::endl;

  // 查表文件放在包安装目录(share/dzrknn_dataprocess/)下，不再写死源码绝对路径
  std::string savePath_Table;
  try
  {
    savePath_Table = ament_index_cpp::get_package_share_directory("dzrknn_dataprocess") +
                     "/offcenter_table.txt";
  }
  catch (const std::exception &)
  {
    savePath_Table = "/home/duzhong/dzrobot/src/dzrknn_dataprocess/offcenter_table.txt";
    RCLCPP_WARN(this->get_logger(),
                "dzrknn_dataprocess 未安装，回退到源码路径: %s", savePath_Table.c_str());
  }
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

  // ★流畅度：主循环频率自适应 —— 跟踪保精度(100Hz)，扫描/空闲省算力(50Hz)。
  //   注意：丢帧计数(target_seen_count_/lost_target_count_)都在【视觉回调】里按
  //   30Hz 视觉帧自增，与本循环频率无关，所以降频不会改变它们的时间含义。
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
    // ★手动接管期间不下发位置指令，避免与遥控器抢控制权。
    //   只挡位置，不影响 LaserShot_Command / aim_status 等其它发布。
    if (!manual_override_)
    {
      pub_CameMo_Conl->publish(camemo_cmd);
    }

    // 自瞄状态上报：状态变化时立即发（循迹 10ms 内就能知道）；
    // 未变化时每 0.5s 补发一次保活 —— 既让后启动的 path_executor 能读到，
    // 又避免刷屏（否则 ros2 topic echo /aim/status 根本没法看）。
    // 状态未变化时按【时间】保活(0.5s)，而不是按循环次数 ——
    // 否则主循环降频后保活周期会跟着变长。
    if (aim_state_ != aim_status_last_ ||
        (this->now() - aim_status_last_time_).seconds() >= 0.5)
    {
      aim_status_last_ = aim_state_;
      aim_status_last_time_ = this->now();
      publishAimStatus();
    }

    // 按当前状态选频率：TRACK 要控制精度，其余省算力
    rclcpp::Rate(aim_state_ == AIM_TRACK ? loop_hz_track_ : loop_hz_idle_).sleep();
  }
}
