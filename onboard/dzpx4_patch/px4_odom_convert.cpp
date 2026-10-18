#include "px4_odom_convert.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include <tf2/exceptions.h>

namespace {

double wrapPi(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

// Yaw (rad, NED convention) extracted from a PX4-convention quaternion
// (w, x, y, z), rotation from FRD body to NED world.
double yawFromQuat(const Eigen::Quaterniond & q)
{
  const double siny = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny, cosy);
}

Eigen::Quaterniond quatFromYaw(double yaw)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

}  // namespace


Px4OdomConvert::Px4OdomConvert()
: Node("px4_odom_convert")
{
  source_mode_          = this->declare_parameter<std::string>("source_mode", "odom");
  odom_topic_           = this->declare_parameter<std::string>("odom_topic", "/Odometry");
  localization_topic_   = this->declare_parameter<std::string>("localization_topic", "/localization");
  tf_map_frame_         = this->declare_parameter<std::string>("tf_map_frame", "map");
  tf_lidar_frame_       = this->declare_parameter<std::string>("tf_lidar_frame", "lidar");
  z_offset_m_           = this->declare_parameter<double>("z_offset_m", 0.0);
  loop_rate_hz_         = this->declare_parameter<double>("loop_rate_hz", 50.0);

  R_enu_slam_ <<
    -1.0, 0.0, 0.0,
     0.0,-1.0, 0.0,
     0.0, 0.0, 1.0;

  t_enu_slam_ <<
    0.0,
    0.0,
    0.0;

  // ---- 防双实例①启动拒绝（2026-10-17 双栈并行实锤后的加固） ----
  // VIO 话题已有发布者 = 另一个 px4_odom_convert 在跑（10-16 实测手动栈
  // 与 autostart 栈并行 93 秒，EKF 被两路测量撕扯出 reset 风暴）。
  // 拒绝启动，由 main 捕获后退出；先 kill 旧实例再启动。
  const size_t existing_vio_pubs =
    this->count_publishers("/fmu/in/vehicle_visual_odometry");
  if (existing_vio_pubs > 0) {
    RCLCPP_FATAL(
      this->get_logger(),
      "DUPLICATE INSTANCE: /fmu/in/vehicle_visual_odometry already has %zu "
      "publisher(s) — another px4_odom_convert is running. Refusing to "
      "start; kill it first (pgrep -af px4_odom_convert).",
      existing_vio_pubs);
    throw std::runtime_error("duplicate px4_odom_convert instance");
  }

  vio_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
    "/fmu/in/vehicle_visual_odometry",
    rclcpp::SensorDataQoS());

  enable_auto_align_ =
    this->declare_parameter<bool>("enable_auto_align", true);
  align_yaw_to_ekf_ =
    this->declare_parameter<bool>("align_yaw_to_ekf", false);

  // jump guard v2 三级防护参数（2026-10-17 修复，默认值即可用）
  slam_insane_m_ =
    this->declare_parameter<double>("slam_insane_m", 2.0);
  slam_recover_n_ =
    this->declare_parameter<int>("slam_recover_n", 86);
  slam_recover_m_ =
    this->declare_parameter<double>("slam_recover_m", 0.1);
  slam_recover_yaw_rad_ =
    this->declare_parameter<double>("slam_recover_yaw_rad", 0.1);
  bump_min_interval_s_ =
    this->declare_parameter<double>("bump_min_interval_s", 10.0);
  ekf_storm_window_s_ =
    this->declare_parameter<double>("ekf_storm_window_s", 60.0);
  ekf_storm_max_resets_ =
    this->declare_parameter<int>("ekf_storm_max_resets", 5);

  // 这两个订阅是**安全件**，不是自动对齐功能的附件，必须无条件创建：
  //   sub_status_    -> fcu_armed_，armed 时 jump guard 永不接受大位移
  //   sub_local_pos_ -> ekf_pos_ned_.z()，z 换源（隔离 SLAM z 的 5.56/8.9m 漂移）
  // 原先两者都裹在 `if (enable_auto_align_)` 里：把 enable_auto_align 设成
  // false（一个听名字只关乎"要不要对齐航向"的参数）会连带静默删掉 armed
  // 门控与高度隔离两层防护，而 fcu_armed_ 会永远停在初始值 false。
  sub_local_pos_ =
    this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position",
      rclcpp::SensorDataQoS(),
      std::bind(
        &Px4OdomConvert::onEkfLocalPosition,
        this,
        std::placeholders::_1));

  // 解锁状态监测：armed 时 jump guard 永不接受大位移（物理上不可能）
  sub_status_ =
    this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status",
      rclcpp::SensorDataQoS(),
      std::bind(
        &Px4OdomConvert::onFcuStatus,
        this,
        std::placeholders::_1));

  if (enable_auto_align_) {
    if (align_yaw_to_ekf_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Auto-align: SLAM yaw rotated once (~1s average) to match EKF "
        "heading, then frozen; reset_counter bumped once");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Auto-align: SLAM pose published unchanged; EKF re-anchored to "
        "the SLAM frame once via reset_counter bump");
    }
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Auto-align disabled: fixed ENU rotation only "
      "(armed gate + z-source isolation remain active)");
  }

  if (source_mode_ == "tf") {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(this->get_logger(), "Mode: tf  [%s -> %s]",
      tf_map_frame_.c_str(), tf_lidar_frame_.c_str());
  } else {
    const std::string topic =
      (source_mode_ == "localization") ? localization_topic_ : odom_topic_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      topic,
      rclcpp::SensorDataQoS(),
      std::bind(&Px4OdomConvert::odomCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Mode: %s  topic: %s",
      source_mode_.c_str(), topic.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Publish PX4 visual odom: /fmu/in/vehicle_visual_odometry");
  if (source_mode_ == "tf") {
    RCLCPP_INFO(this->get_logger(), "TF lookup rate: %.1f Hz  z_offset: %.3f m",
      loop_rate_hz_, z_offset_m_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Input-driven publishing (no frame filling)  z_offset: %.3f m",
      z_offset_m_);
  }
  RCLCPP_INFO(this->get_logger(),
    "Resilience layer v2: jump_guard(hold %.2fm / insane-lock %.1fm / confirm %d frames, "
    "armed=never-accept) + bump_throttle(%.0fs) + ekf_storm_breaker(%d resets/%.0fs) + z-from-EKF",
    jump_hold_m_, slam_insane_m_, jump_confirm_n_,
    bump_min_interval_s_, ekf_storm_max_resets_, ekf_storm_window_s_);

  // 定位质量健康行（v2.1）：3s 周期输出 [px4_odom_convert_health] 摘要，
  // 供机上 localization_watchdog.py 感知定位质量（补齐 watchdog 零监控缺口）。
  health_timer_ = this->create_wall_timer(
    std::chrono::seconds(3),
    std::bind(&Px4OdomConvert::onHealthTimer, this));

  // ---- 防双实例②运行期监测（10-17 双栈并行实锤后的加固） ----
  // 每 10s 查一次 VIO / SLAM 话题发布者数：
  //   VIO >1    → 又起了一个 px4_odom_convert（启动拒绝被绕过/竞态）；
  //   SLAM >1   → 双 FAST-LIO 栈在并行（10-16 实测双栈同发 /localization
  //               406 秒，交错流即 26~585m 跳变的直接来源）。
  // 只打 ERROR 告警、不自动退出：armed 下跳变由 L1 insane 熔断兜底，而
  // 自动退出可能造成飞行中 VIO 断链（比双实例更危险）；留给看门狗/人工
  // kill 多余实例（pgrep -af 'fastlio_mapping|px4_odom_convert'）。
  dup_check_timer_ = this->create_wall_timer(
    std::chrono::seconds(10),
    [this]() {
      const size_t vio_pubs =
        this->count_publishers("/fmu/in/vehicle_visual_odometry");
      if (vio_pubs > 1) {
        RCLCPP_ERROR(
          this->get_logger(),
          "DUPLICATE INSTANCE: %zu publishers on "
          "/fmu/in/vehicle_visual_odometry — two px4_odom_convert are "
          "fighting over the EKF. Kill one NOW (pgrep -af "
          "px4_odom_convert).",
          vio_pubs);
      }
      const std::string slam_topic =
        (source_mode_ == "localization") ? localization_topic_ : odom_topic_;
      const size_t slam_pubs = this->count_publishers(slam_topic);
      if (slam_pubs > 1) {
        RCLCPP_ERROR(
          this->get_logger(),
          "DUAL SLAM SOURCE: %zu publishers on %s — two FAST-LIO stacks "
          "are interleaving /localization (frames jump between two "
          "independent poses). Kill one stack NOW (pgrep -af "
          "fastlio_mapping).",
          slam_pubs, slam_topic.c_str());
      }
    });
}

uint64_t Px4OdomConvert::nowUs()
{
  return static_cast<uint64_t>(this->now().nanoseconds() / 1000);
}

double Px4OdomConvert::steadyNowS()
{
  // this->now() 走的是节点时钟，默认 RCL_SYSTEM_TIME = 系统墙钟（除非
  // use_sim_time）。墙上时钟会跳：NTP 校时、手工改时间、机载无 NTP 时
  // 开机后第一次同步，都可能往回拨。一旦往回拨，所有 `now_s - t0 > 窗口`
  // 形式的判据就永远为假 ——
  //   * reset_times_ 永不裁剪  -> ekf_storm_ 永久冻结重锚（L3 锁死）
  //   * bump_ok 永不为真       -> L2 重锚节流永久不放行
  // 两者叠加会让 jump guard 在飞行中彻底不可恢复，正是它要防的那种事故。
  // steady_clock 保证单调，代价是数值不再对应真实时刻 —— 所以黑匣子/健康行
  // 里给人看的 t= 仍然用墙钟，只有"算时长"的地方用本函数。
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(
    clock::now().time_since_epoch()).count();
}

void Px4OdomConvert::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!odom_ready_)
    {
        odom_ready_count_++;

        if (odom_ready_count_ >= 20)
        {
            odom_ready_ = true;

            RCLCPP_INFO(
              this->get_logger(),
              "Odometry initialized, start publishing PX4 VIO"
            );
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
              this->get_logger(),
              *this->get_clock(),
              1000,
              "Waiting odom initialization: %d/20",
              odom_ready_count_
            );
        }
    }

    if (odom_ready_)
    {
        publishVehicleVisualOdometry(*msg);
    }
}

void Px4OdomConvert::onFcuStatus(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  // 2 = ARMING_STATE_ARMED，4 = ARMING_STATE_ARMED_ERROR。
  // 只看 2 会漏掉 ARMED_ERROR：那是"已解锁但处于故障态"，电机仍可能转，
  // 而 fcu_armed_ 会读成 false，于是"armed 时永不接受大位移"这层防护
  // 恰好在飞机状态最不好的时候被关掉。判据要按"电机是否可能转"来写。
  fcu_armed_ = (msg->arming_state == 2 || msg->arming_state == 4);
}

void Px4OdomConvert::onEkfLocalPosition(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  const bool valid =
    msg->xy_valid && msg->z_valid &&
    std::isfinite(msg->x) && std::isfinite(msg->y) &&
    std::isfinite(msg->z) && std::isfinite(msg->heading);

  if (valid) {
    ekf_pos_ned_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
    // 一旦拿到过有效 EKF z 就永久记住：z 换源的参照不该因为随后某一帧
    // xy_valid/z_valid 短暂为假而消失（见 publishVehicleVisualOdometry）。
    ever_had_ekf_z_ = true;
    ekf_heading_ = msg->heading;

    if (has_reset_counters_ &&
      (msg->xy_reset_counter != last_xy_reset_counter_ ||
       msg->heading_reset_counter != last_heading_reset_counter_))
    {
      // ---- L3 EKF reset 风暴熔断（2026-10-17 修复）----
      // 实测（node_odom.log 10-17 上午）：EKF xy_reset 每秒 +1、计数涨满 255
      // 饱和循环，旧版只打 WARN 无任何动作，风暴持续 14 小时无人察觉。
      // 滑动窗内 reset 超过阈值 -> ekf_storm_ 置位，jump guard 冻结一切
      // accept/bump（见 publishVehicleVisualOdometry），hold 输出保 EKF。
      const double now_s = steadyNowS();   // 单调：见 steadyNowS() 注释
      reset_times_.push_back(now_s);
      while (!reset_times_.empty() &&
             now_s - reset_times_.front() > ekf_storm_window_s_)
      {
        reset_times_.pop_front();
      }
      ++total_ekf_resets_;

      if (!ekf_storm_ &&
        static_cast<int>(reset_times_.size()) > ekf_storm_max_resets_)
      {
        ekf_storm_ = true;
        ++total_storm_on_;
        last_l3_storm_on_t_ = now_s;
        RCLCPP_ERROR(
          this->get_logger(),
          "EKF RESET STORM: %zu resets within %.0fs window — freezing "
          "re-anchor/bump and holding output until storm clears",
          reset_times_.size(), ekf_storm_window_s_);
        writeBlackbox(
          "L3_STORM_ON", "OK->STORM", 0.0, 0.0,
          ekf_pos_ned_, ekf_pos_ned_);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "EKF frame reset detected (xy_reset=%u heading_reset=%u, %zu in "
          "%.0fs window)%s",
          msg->xy_reset_counter, msg->heading_reset_counter,
          reset_times_.size(), ekf_storm_window_s_,
          ekf_storm_ ? " [STORM: accept/bump frozen]" : "");
      }

      // If the reset hits while we are still averaging the initial
      // alignment, the average is meaningless — start over.
      if (!aligned_) {
        align_sample_count_ = 0;
        align_ekf_heading_sum_ = 0.0;
        align_slam_yaw_sum_ = 0.0;
      }
    }

    // 风暴解除检查：窗口滑过、reset 计数回落到阈值内 -> 恢复正常防护
    if (ekf_storm_) {
      const double now_s = steadyNowS();   // 单调：见 steadyNowS() 注释
      while (!reset_times_.empty() &&
             now_s - reset_times_.front() > ekf_storm_window_s_)
      {
        reset_times_.pop_front();
      }
      if (static_cast<int>(reset_times_.size()) <= ekf_storm_max_resets_) {
        ekf_storm_ = false;
        last_l3_storm_clear_t_ = now_s;
        RCLCPP_INFO(
          this->get_logger(),
          "EKF reset storm cleared (window now %zu) — resuming normal "
          "jump-guard protection",
          reset_times_.size());
        writeBlackbox(
          "L3_STORM_CLEAR", "STORM->OK", 0.0, 0.0,
          ekf_pos_ned_, ekf_pos_ned_);
      }
    }

    last_xy_reset_counter_ = msg->xy_reset_counter;
    last_heading_reset_counter_ = msg->heading_reset_counter;
    has_reset_counters_ = true;
  }

  ekf_pos_valid_ = valid;
}

void Px4OdomConvert::writeBlackbox(
  const char * event, const char * trans,
  double dpos, double dyaw,
  const Eigen::Vector3d & from_pos, const Eigen::Vector3d & to_pos)
{
  // ---- 事件级 blackbox（v2.1）：4 行固定结构，事后只看本记录即可归因 ----
  //   1) event/t/trans/armed/aligned：状态转换与解锁状态
  //   2) jump：触发帧跳变距离 + 方向（NED 分量）+ 起止位姿
  //   3) ekf：reset 计数快照（ev_reset / 60s 窗 / 累计）+ 风暴标志
  //   4) slam_q：近 3s 质量窗口（跳变 max/mean、L1 恢复进度、协方差迹峰值）
  // 记录时点：状态已按本事件更新（ev_reset 为 bump 后新值），from 为旧
  // 锚点、to 为本帧 SLAM 原始位姿。
  const double t = this->now().seconds();
  std::cout << std::fixed << std::setprecision(3)
            << "[px4_odom_convert_blackbox] event=" << event
            << " t=" << t
            << " trans=" << trans
            << " armed=" << (fcu_armed_ ? 1 : 0)
            << " aligned=" << (aligned_ ? 1 : 0)
            << std::endl;
  std::cout << "[px4_odom_convert_blackbox]   jump"
            << " dpos_m=" << dpos
            << " dyaw_deg=" << dyaw * 180.0 / M_PI
            << " delta_ned=[" << to_pos.x() - from_pos.x() << ", "
            << to_pos.y() - from_pos.y() << ", " << to_pos.z() - from_pos.z()
            << "]"
            << " from_ned=[" << from_pos.x() << ", " << from_pos.y() << ", "
            << from_pos.z() << "]"
            << " to_ned=[" << to_pos.x() << ", " << to_pos.y() << ", "
            << to_pos.z() << "]"
            << std::endl;
  std::cout << "[px4_odom_convert_blackbox]   ekf"
            << " ev_reset=" << static_cast<unsigned>(ev_reset_counter_)
            << " resets_60s=" << reset_times_.size()
            << " resets_total=" << total_ekf_resets_
            << " storm=" << (ekf_storm_ ? 1 : 0)
            << " ekf_valid=" << (ekf_pos_valid_ ? 1 : 0)
            << " xy_cnt=" << static_cast<unsigned>(last_xy_reset_counter_)
            << " hdg_cnt=" << static_cast<unsigned>(last_heading_reset_counter_)
            << std::endl;
  std::cout << "[px4_odom_convert_blackbox]   slam_q"
            << " win_max_m=" << win_jump_max_m_
            << " win_mean_m=" << (win_slam_frames_ > 1
              ? win_jump_sum_m_ / (win_slam_frames_ - 1) : 0.0)
            << " win_frames=" << win_slam_frames_
            << " stable_now=" << slam_recover_count_ << "/" << slam_recover_n_
            << " cov_max=" << win_cov_max_
            << std::endl;
}

void Px4OdomConvert::onHealthTimer()
{
  // ---- 定位质量健康行（v2.1，3s 周期）：watchdog 的唯一常态数据源 ----
  // key=value 只增不改；行停更 = 节点死/SLAM 断流（watchdog 据此告警）。
  // 前缀不含既有解析关键字（'jump guard'/'position delta'），互不干扰。
  // t= 打的是墙钟（方便和黑匣子、其他日志对时），但**年龄必须用单调钟**：
  // last_l1_trigger_t_ / last_bump_time_ / last_l3_storm_on_t_ 现在都由
  // steadyNowS() 写入，拿墙钟去减单调钟会得到毫无意义的数字。
  const double t = this->now().seconds();
  const double t_mono = steadyNowS();
  const double win_s = 3.0;
  const char * guard =
    slam_bad_ ? "L1_LOCK" : (last_gate_holding_ ? "HOLD" : "OK");
  const double l1_age =
    last_l1_trigger_t_ < 0.0 ? -1.0 : t_mono - last_l1_trigger_t_;
  const double bump_age = has_last_bump_ ? t_mono - last_bump_time_ : -1.0;
  const double storm_age =
    last_l3_storm_on_t_ < 0.0 ? -1.0 : t_mono - last_l3_storm_on_t_;

  std::cout << std::fixed << std::setprecision(3)
            << "[px4_odom_convert_health] t=" << t
            << " guard=" << guard << (ekf_storm_ ? "+STORM" : "")
            << " armed=" << (fcu_armed_ ? 1 : 0)
            << " aligned=" << (aligned_ ? 1 : 0)
            << " ekf_valid=" << (ekf_pos_valid_ ? 1 : 0)
            << " slam_frames=" << win_slam_frames_
            << " slam_hz=" << (win_slam_frames_ > 0
              ? static_cast<double>(win_slam_frames_) / win_s : 0.0)
            << " hold_ratio=" << (win_slam_frames_ > 0
              ? static_cast<double>(win_hold_frames_) / win_slam_frames_
              : 0.0)
            << " jump_max_m=" << win_jump_max_m_
            << " jump_mean_m=" << (win_slam_frames_ > 1
              ? win_jump_sum_m_ / (win_slam_frames_ - 1) : 0.0)
            << " cov_max=" << win_cov_max_
            << " l1_lock=" << (slam_bad_ ? 1 : 0)
            << " l1_age_s=" << l1_age
            << " l1_total=" << total_l1_triggers_
            // 新增三项（健康行约定 key=value 只增不改）：
            //   nan_drop 非零 = 上游 FAST-LIO 在吐非有限位姿，比跳变更早的告警
            //   z_src=0  = 还没有有效 EKF z，高度隔离未生效，飞控在吃会漂的 SLAM z
            //   armed    = jump guard 是否处于"永不接受大位移"模式
            << " nan_drop=" << total_nonfinite_drops_
            << " z_src=" << (ever_had_ekf_z_ ? 1 : 0)
            << " armed=" << (fcu_armed_ ? 1 : 0)
            << " bump_age_s=" << bump_age
            << " bump_total=" << total_bumps_
            << " storm=" << (ekf_storm_ ? 1 : 0)
            << " storm_age_s=" << storm_age
            << " ekf_reset_60s=" << reset_times_.size()
            << " ekf_reset_total=" << total_ekf_resets_
            << " ev_reset=" << static_cast<unsigned>(ev_reset_counter_)
            << std::endl;

  // 窗口统计复位：下一个 3s 窗口重新累计
  win_slam_frames_ = 0;
  win_hold_frames_ = 0;
  win_stable_frames_ = 0;
  win_jump_sum_m_ = 0.0;
  win_jump_max_m_ = 0.0;
  win_cov_max_ = 0.0;
}

void Px4OdomConvert::computeAlignment(
  double slam_yaw_ned,
  double ekf_heading)
{
  yaw_offset_ = wrapPi(ekf_heading - slam_yaw_ned);

  aligned_ = true;
  ev_reset_counter_ = 1;  // 对齐完成：bump 0->1，EKF 重锚到 SLAM 帧
  last_bump_time_ = this->now().seconds();  // 记入 L2 节流窗口
  has_last_bump_ = true;

  RCLCPP_INFO(
    this->get_logger(),
    "Aligned SLAM->EKF: yaw_offset=%.1f deg; reset_counter bumped 0->1, "
    "EKF will re-anchor to the SLAM pose once",
    yaw_offset_ * 180.0 / M_PI);
}

void Px4OdomConvert::publishVehicleVisualOdometry(
  const nav_msgs::msg::Odometry & odom)
{
  namespace ft = px4_ros_com::frame_transforms;

  px4_msgs::msg::VehicleOdometry vio{};


  const auto & p = odom.pose.pose.position;
  const auto & q = odom.pose.pose.orientation;

  // ===== SLAM 位姿有限性门（必须在碰任何 guard 状态之前）=====
  // jump guard 的两级判据都是 `gate_dp > 阈值` 的形式，而 C++ 里
  // NaN > x 恒为 false。一帧 NaN 因此既不算 L1 疯跳、也不算 L2 大位移，
  // 会一路落到最后的 else 分支被写进 gate_prev_pos_ / gate_out_pos_；
  // 此后每次 norm() 都是 NaN，两级比较永远为假 —— 实测过 37~62m / 9.11m
  // 跳变的防护就此静默失效，而 vio.quality 仍然写着 100。
  // 四元数全零是 geometry_msgs/Quaternion 的默认值（消息只填了一半），
  // toRotationMatrix() 会得到零矩阵，再把零范数 q 发给 PX4。
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
    ++total_nonfinite_drops_;
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "SLAM pose non-finite (x=%.3f y=%.3f z=%.3f) - frame dropped, "
      "jump-guard state untouched (drops=%d)",
      p.x, p.y, p.z, total_nonfinite_drops_);
    return;
  }
  const double q_norm =
    std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!std::isfinite(q_norm) || q_norm < 0.5) {
    ++total_nonfinite_drops_;
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "SLAM quaternion degenerate (norm=%.4f) - frame dropped (drops=%d)",
      q_norm, total_nonfinite_drops_);
    return;
  }

  // SLAM 位姿协方差迹（FAST-LIO 填充时有效）：3s 窗口峰值，质量代理指标
  // （v2.1 健康行 cov_max / blackbox slam_q 数据源）
  const double slam_cov_trace =
    odom.pose.covariance[0] + odom.pose.covariance[7] +
    odom.pose.covariance[14];
  if (std::isfinite(slam_cov_trace) && slam_cov_trace > win_cov_max_) {
    win_cov_max_ = slam_cov_trace;
  }

  vio.timestamp = nowUs();
  // header.stamp.sec 是 int32：未填或为负时 static_cast<uint64_t> 会回绕成
  // 天文数字，PX4 看到的 timestamp_sample 乱序 —— 轻则整帧被拒，重则按
  // 错误的延迟做时间补偿。拿不到有效 stamp 就退回本机接收时刻，
  // 至少保证单调（代价是丢失延迟补偿，故告警而不是静默）。
  if (odom.header.stamp.sec > 0) {
    vio.timestamp_sample =
      static_cast<uint64_t>(odom.header.stamp.sec) * 1000000ULL +
      static_cast<uint64_t>(odom.header.stamp.nanosec) / 1000ULL;
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "SLAM header.stamp unset (sec=%d nsec=%u) - falling back to receive "
      "time for timestamp_sample; latency compensation degraded",
      odom.header.stamp.sec, odom.header.stamp.nanosec);
    vio.timestamp_sample = vio.timestamp;
  }

  vio.pose_frame =
    px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

  vio.velocity_frame =
    px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;

  Eigen::Vector3d position_slam(
    p.x,
    p.y,
    p.z - z_offset_m_
  );

  Eigen::Vector3d position_enu =
    R_enu_slam_ * position_slam
    + t_enu_slam_;

  Eigen::Quaterniond q_slam(
    q.w,
    q.x,
    q.y,
    q.z
  );
  // Eigen 的 toRotationMatrix() **不会**替你归一化：范数偏离 1 时得到的
  // 不是旋转矩阵（含缩放/剪切），姿态误差会直接进 EKF。上面已保证
  // q_norm >= 0.5，归一化不会除零。
  q_slam.normalize();

  Eigen::Matrix3d R_slam_body =
    q_slam.toRotationMatrix();

  Eigen::Matrix3d R_enu_body =
    R_enu_slam_ * R_slam_body;

  Eigen::Quaterniond q_enu(
    R_enu_body
  );

  Eigen::Quaterniond q_px4 =
    ft::ros_to_px4_orientation(q_enu);

  Eigen::Vector3d position_ned =
    ft::enu_to_ned_local_frame(
        position_enu
    );

  if (enable_auto_align_) {
    const double slam_yaw_ned = yawFromQuat(q_px4);

    if (!aligned_ && ekf_pos_valid_) {
      align_sample_count_++;

      if (align_yaw_to_ekf_) {
        if (align_sample_count_ == 1) {
          align_ekf_heading_ref_ = ekf_heading_;
          align_slam_yaw_ref_ = slam_yaw_ned;
        }
        align_ekf_heading_sum_ +=
          wrapPi(ekf_heading_ - align_ekf_heading_ref_);
        align_slam_yaw_sum_ +=
          wrapPi(slam_yaw_ned - align_slam_yaw_ref_);

        if (align_sample_count_ >= 100) {
          const double slam_yaw_avg = wrapPi(
            align_slam_yaw_ref_ +
            align_slam_yaw_sum_ / static_cast<double>(align_sample_count_));
          const double ekf_heading_avg = wrapPi(
            align_ekf_heading_ref_ +
            align_ekf_heading_sum_ / static_cast<double>(align_sample_count_));

          computeAlignment(slam_yaw_avg, ekf_heading_avg);
          ++total_bumps_;
          writeBlackbox(
            "ALIGN_BUMP", "UNALIGNED->ALIGNED", 0.0, yaw_offset_,
            position_ned, position_ned);
        }
      } else if (align_sample_count_ >= 30) {
        aligned_ = true;
        ev_reset_counter_ = 1;  // 对齐完成（SLAM 直通模式）：bump 0->1 让 EKF 重锚
        last_bump_time_ = this->now().seconds();  // 记入 L2 节流窗口
        has_last_bump_ = true;
        ++total_bumps_;
        writeBlackbox(
          "ALIGN_BUMP", "UNALIGNED->ALIGNED", 0.0, 0.0,
          position_ned, position_ned);
        RCLCPP_INFO(
          this->get_logger(),
          "Publishing SLAM pose unchanged; reset_counter bumped 0->1, "
          "EKF re-anchors to the SLAM frame once");
      }
    }

    if (aligned_) {
      const Eigen::Matrix3d Rz =
        Eigen::AngleAxisd(yaw_offset_, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();

      position_ned = Rz * position_ned;
      q_px4 = quatFromYaw(yaw_offset_) * q_px4;
    }
  }

  // ===== z 轴换源（2026-10-16 二次实飞实测落地）=====
  // 两天两个场地 SLAM 的 z 分别漂 5.56m / 8.9m（雷达 z 观测随环境劣化），
  // 而 EKF 的 z 由气压计主导、悬停高度始终正常。这里用 EKF 反馈 z 替换
  // SLAM z：EV z 与 EKF z 零残差（对 z 不提供信息），等效于关闭 EV-z
  // 融合——高度从此完全由气压计决定，SLAM z 漂移不再影响飞控。
  // 判据用 ever_had_ekf_z_ 而不是 ekf_pos_valid_：后者是"最新一帧 EKF 是否
  // 有效"的单帧标志（每帧被覆盖），开机后还没收到 EKF、或任何一帧
  // xy_valid/z_valid 短暂为假，都会让它为假——于是 SLAM z（两天两场实测
  // 漂 5.56m / 8.9m）顺着 vio.position[2] 直接进飞控，上面那段"SLAM z 漂移
  // 不再影响飞控"的注释在这些路径上是假的。ekf_pos_ned_ 只在 valid 时更新，
  // 因此持有的一直是最后一个好 z。
  if (ever_had_ekf_z_) {
    position_ned.z() = ekf_pos_ned_.z();
  } else {
    // 降级状态必须可见：此刻高度隔离没有生效，飞控在吃会漂的 SLAM z。
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "z-source: no valid EKF z yet, publishing SLAM z (%.2f m) which is "
      "known to drift - altitude isolation NOT active",
      position_ned.z());
  }

  // ===== jump guard v2：SLAM 跟丢/错配的三级防护（2026-10-17 修复） =====
  // 旧版缺陷（node_odom.log 10-16~10-17 实锤）：①SLAM 持续疯跳（ICP 摇摆，
  // 单帧 26~585m）时"连续 8 帧即接受"把疯跳当真实位移放行；②每次接受都
  // bump reset_counter，EKF 被每秒重锚（xy_reset 每秒+1，累计 3764 次）。
  // 三级防护：
  //   L1 insane 熔断 —— 单帧位移 > slam_insane_m_（默认 2m，43Hz 下等效
  //      86m/s，物理上限约 40 倍）判定 SLAM 数据无效：hold 到底、绝不
  //      bump；恢复需 SLAM 相邻帧稳定 slam_recover_n_ 帧（约 2s）。原位
  //      恢复直接接管；新位置恢复且 disarmed/非风暴/节流允许时一次 bump
  //      平滑重锚（86 帧稳定 >> 8 帧确认）；armed/风暴/节流窗口内保持
  //      熔断 hold，条件满足的瞬间自动重锚。
  //   L2 bump 节流 —— bump reset_counter 的最小间隔为 bump_min_interval_s_
  //      （默认 10s）；窗口内 gate_prev 与 gate_out 均保持不动，后续帧
  //      持续 hold/confirm 循环复查窗口，EKF 看到的始终是稳定测量。
  //   L3 EKF reset 风暴熔断 —— ekf_storm_ 置位期间（60s 窗 >5 次 reset）
  //      一切 accept/bump 冻结，hold 输出保 EKF，风暴缓解自动恢复。
  //   常规路径 —— 小位移放行；大位移先 hold（重发上一次放行位姿 + 放大
  //      方差）；disarmed 时连续 jump_confirm_n_ 帧仍大 -> 真实大位移
  //      （被搬运/换场），走节流 bump 重锚。armed 时永不接受。
  const double now_s = steadyNowS();   // 单调：见 steadyNowS() 注释
  bool gate_holding = false;

  // SLAM 原始相邻帧跟踪（不受 hold 替换影响）：L1 恢复判定的数据源
  bool raw_stable = false;
  if (slam_has_prev_raw_) {
    const double raw_dp = (position_ned - slam_prev_raw_pos_).norm();
    const double raw_dy =
      std::fabs(wrapPi(yawFromQuat(q_px4) - yawFromQuat(slam_prev_raw_quat_)));
    raw_stable =
      raw_dp < slam_recover_m_ && raw_dy < slam_recover_yaw_rad_;
    // 3s 质量窗口累计（v2.1 健康行 / blackbox slam_q 数据源）
    win_jump_sum_m_ += raw_dp;
    if (raw_dp > win_jump_max_m_) {
      win_jump_max_m_ = raw_dp;
    }
    if (raw_stable) {
      ++win_stable_frames_;
    }
  }
  ++win_slam_frames_;
  slam_prev_raw_pos_ = position_ned;
  slam_prev_raw_quat_ = q_px4;
  slam_has_prev_raw_ = true;

  if (!gate_has_prev_) {
    gate_prev_pos_ = position_ned;
    gate_prev_quat_ = q_px4;
    gate_out_pos_ = position_ned;
    gate_out_quat_ = q_px4;
    gate_has_prev_ = true;
  } else if (slam_bad_) {
    // ---- L1 熔断中：只认"SLAM 自身相邻帧稳定"，不看与旧位姿的距离 ----
    if (raw_stable) {
      ++slam_recover_count_;
    } else {
      slam_recover_count_ = 0;
    }

    if (slam_recover_count_ >= slam_recover_n_) {
      const double d_out = (position_ned - gate_out_pos_).norm();
      // 放行条件（86 帧稳定确认 >> 常规 8 帧确认，无需再走 confirm 路径）
      const bool bump_ok =
        !has_last_bump_ || (now_s - last_bump_time_) >= bump_min_interval_s_;
      if (d_out < jump_hold_m_) {
        // 原位恢复（疯跳回到原点）：直接接管恢复直通
        slam_bad_ = false;
        slam_recover_count_ = 0;
        gate_hold_count_ = 0;
        writeBlackbox(
          "L1_RECOVER_INPLACE", "L1_LOCK->OK", d_out,
          std::fabs(wrapPi(yawFromQuat(q_px4) - yawFromQuat(gate_out_quat_))),
          gate_out_pos_, position_ned);
        gate_prev_pos_ = position_ned;
        gate_prev_quat_ = q_px4;
        gate_out_pos_ = position_ned;
        gate_out_quat_ = q_px4;
        RCLCPP_WARN(
          this->get_logger(),
          "jump guard: SLAM recovered in place (%.2f m from held pose), "
          "passthrough resumed",
          d_out);
      } else if (!fcu_armed_ && !ekf_storm_ && bump_ok) {
        // 新位置恢复（真搬运完成 / 重定位到新锚点）且放行条件满足：
        // 一次 bump 平滑重锚（86 帧稳定远强于常规 8 帧确认）。
        slam_bad_ = false;
        slam_recover_count_ = 0;
        gate_hold_count_ = 0;
        ++ev_reset_counter_;
        last_bump_time_ = now_s;
        has_last_bump_ = true;
        ++total_bumps_;
        writeBlackbox(
          "L1_RECOVER_REANCHOR", "L1_LOCK->REANCHOR", d_out,
          std::fabs(wrapPi(yawFromQuat(q_px4) - yawFromQuat(gate_out_quat_))),
          gate_out_pos_, position_ned);
        gate_prev_pos_ = position_ned;
        gate_prev_quat_ = q_px4;
        gate_out_pos_ = position_ned;
        gate_out_quat_ = q_px4;
        RCLCPP_WARN(
          this->get_logger(),
          "jump guard: SLAM stable at new anchor %.2f m away — accepted, "
          "reset_counter=%u (re-anchor)",
          d_out, static_cast<unsigned>(ev_reset_counter_));
      }
      // 其余情况（armed / EKF 风暴 / 节流窗口内）：保持熔断 hold。
      // slam_recover_count_ 不清零：raw_stable 持续时每帧复查放行条件，
      // 条件满足的瞬间（降落 / 风暴解除 / 节流窗口滑过）自动平滑重锚。
    }

    if (slam_bad_) {
      // 仍处熔断（恢复未达标或放行条件不满足）：hold 上一次放行位姿
      gate_holding = true;
      position_ned = gate_out_pos_;
      q_px4 = gate_out_quat_;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "jump guard: SLAM INSANE — output locked to last good pose "
        "(stable %d/%d%s%s%s)",
        slam_recover_count_, slam_recover_n_,
        fcu_armed_ ? ", armed=等待降落" : "",
        ekf_storm_ ? ", storm=冻结重锚" : "",
        (!fcu_armed_ && !ekf_storm_ && slam_recover_count_ >= slam_recover_n_)
          ? ", bump=节流等待" : "");
    }
  } else {
    const double gate_dp = (position_ned - gate_prev_pos_).norm();
    const double gate_dy =
      std::fabs(wrapPi(yawFromQuat(q_px4) - yawFromQuat(gate_prev_quat_)));

    if (gate_dp > slam_insane_m_ || gate_dy > 2.0 * jump_yaw_rad_) {
      // ---- L1 触发：单帧位移物理不可能 -> 熔断，绝不接受/绝不 bump ----
      slam_bad_ = true;
      slam_recover_count_ = 0;
      gate_hold_count_ = 0;
      ++total_l1_triggers_;
      last_l1_trigger_t_ = now_s;
      l2_throttling_ = false;
      writeBlackbox(
        "L1_TRIGGER", "OK->L1_LOCK", gate_dp, gate_dy,
        gate_prev_pos_, position_ned);
      gate_holding = true;
      position_ned = gate_out_pos_;
      q_px4 = gate_out_quat_;
      RCLCPP_ERROR(
        this->get_logger(),
        "jump guard: SLAM INSANE jump (dpos=%.2f m, dyaw=%.0f deg) — "
        "locking output, no accept, no bump",
        gate_dp, gate_dy * 180.0 / M_PI);
    } else if (gate_dp > jump_hold_m_ || gate_dy > jump_yaw_rad_) {
      ++gate_hold_count_;
      // armed 时永不接受：无人机物理上不可能 0.2s 内移动数米，armed 大
      // 位移 100% 是 SLAM 跟丢——hold 到底保 EKF。
      // EKF 风暴期间（L3）同样冻结 accept，防止火上浇油。
      const bool confirmed =
        !fcu_armed_ && !ekf_storm_ && gate_hold_count_ >= jump_confirm_n_;
      if (confirmed) {
        // ---- L2 bump 节流 ----
        // 窗口内 gate_prev 保持不动：后续帧继续走 hold/confirm 循环，每
        // 0.2s 复查一次节流窗口，窗口过后才真正 bump+放行。若前移
        // gate_prev，SLAM 稳定后位置会经"正常放行分支"无 bump 跳变。
        const bool bump_ok =
          !has_last_bump_ || (now_s - last_bump_time_) >= bump_min_interval_s_;
        gate_hold_count_ = 0;
        if (bump_ok) {
          ++ev_reset_counter_;           // EKF 重锚到新位姿（uint8 自动回绕）
          last_bump_time_ = now_s;
          has_last_bump_ = true;
          ++total_bumps_;
          l2_throttling_ = false;
          writeBlackbox(
            "L2_ACCEPT", "HOLD->ACCEPT", gate_dp, gate_dy,
            gate_out_pos_, position_ned);
          gate_prev_pos_ = position_ned;   // 判定基准同步
          gate_prev_quat_ = q_px4;
          gate_out_pos_ = position_ned;  // 真正放行：输出同步到新位置
          gate_out_quat_ = q_px4;
          RCLCPP_WARN(
            this->get_logger(),
            "jump guard: large displacement accepted (dpos=%.2f m, "
            "dyaw=%.0f deg); reset_counter=%u (throttle >=%.0fs)",
            gate_dp, gate_dy * 180.0 / M_PI,
            static_cast<unsigned>(ev_reset_counter_), bump_min_interval_s_);
        } else {
          // 节流窗口内：输出保持稳定，窗口过后由下一轮 confirm 放行
          // blackbox 只记节流期首帧（每 8 帧复查一次，避免刷屏）
          if (!l2_throttling_) {
            l2_throttling_ = true;
            writeBlackbox(
              "L2_THROTTLED", "HOLD->HOLD_THROTTLE", gate_dp, gate_dy,
              gate_out_pos_, position_ned);
          }
          gate_holding = true;
          position_ned = gate_out_pos_;
          q_px4 = gate_out_quat_;
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "jump guard: displacement confirmed (dpos=%.2f m) but bump "
            "throttled — holding output %.0fs more",
            gate_dp,
            bump_min_interval_s_ - (now_s - last_bump_time_));
        }
      } else {
        gate_holding = true;
        position_ned = gate_out_pos_;
        q_px4 = gate_out_quat_;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "jump guard: holding previous pose (dpos=%.2f m, dyaw=%.0f deg, "
          "frame %d%s%s)",
          gate_dp, gate_dy * 180.0 / M_PI, gate_hold_count_,
          fcu_armed_ ? ", armed=永不接受" : "",
          ekf_storm_ ? ", storm=冻结重锚" : "");
      }
    } else {
      gate_hold_count_ = 0;
      l2_throttling_ = false;
      gate_prev_pos_ = position_ned;
      gate_prev_quat_ = q_px4;
      gate_out_pos_ = position_ned;
      gate_out_quat_ = q_px4;
    }
  }

  // v2.1 窗口统计：hold 帧计数 + 供健康行读取的最近一帧 guard 状态
  if (gate_holding) {
    ++win_hold_frames_;
  }
  last_gate_holding_ = gate_holding;

  vio.position[0] =
    static_cast<float>(position_ned.x());

  vio.position[1] =
    static_cast<float>(position_ned.y());

  vio.position[2] =
    static_cast<float>(position_ned.z());

  vio.velocity[0] = nan;
  vio.velocity[1] = nan;
  vio.velocity[2] = nan;

  vio.q[0] =
    static_cast<float>(q_px4.w());

  vio.q[1] =
    static_cast<float>(q_px4.x());

  vio.q[2] =
    static_cast<float>(q_px4.y());

  vio.q[3] =
    static_cast<float>(q_px4.z());

  vio.angular_velocity[0] = nan;
  vio.angular_velocity[1] = nan;
  vio.angular_velocity[2] = nan;

  // hold 期间位置不可信：方差放大两个数量级，EKF 会大幅降低该测量权重
  const float gate_pos_var = gate_holding ? 100.0f : 0.01f;
  const float gate_quat_var = gate_holding ? 100.0f : 0.01f;

  vio.position_variance[0] = gate_pos_var;
  vio.position_variance[1] = gate_pos_var;
  vio.position_variance[2] = gate_pos_var;

  vio.orientation_variance[0] = gate_quat_var;
  vio.orientation_variance[1] = gate_quat_var;
  vio.orientation_variance[2] = gate_quat_var;

  vio.velocity_variance[0] = nan;
  vio.velocity_variance[1] = nan;
  vio.velocity_variance[2] = nan;

  // reset_counter 变化即"EV 帧变化"：对齐完成 0->1（重锚 SLAM 帧），之后
  // jump guard 每次接受真实大位移再 +1（重锚新位姿）。其余时间恒定，
  // 不触发 EKF reset。hold 期间保持不变（EKF 视角=同一帧的静止测量）。
  vio.reset_counter = ev_reset_counter_;

  // hold 期间同时降低 quality，保证 EKF2_EV_NOISE_MD=0（用 quality 决定
  // 可信度）时 hold 仍然生效。正常帧 100；hold 时 1（PX4 允许最低值）。
  vio.quality = gate_holding ? 1 : 100;

  const Eigen::Vector3f current_position_ned(
    vio.position[0], vio.position[1], vio.position[2]);

  if (has_previous_published_position_) {
    // 43Hz 每帧一条 std::endl —— endl 会 flush，这是热路径上的阻塞点：
    // stdout 一旦接到管道/文件而下游卡住（盘满、没人收的 tee、串口反压），
    // 回调就堵在这里，VIO 停发 -> PX4 EV 超时 -> 触发失效保护。
    // 节流到 ~2Hz 且改用 '\n'（不 flush）。previous_published_position_ned_
    // 仍每帧更新，所以 dx/dy/dz 依旧是单帧位移，语义没变。
    // localization_watchdog 只解析 [px4_odom_convert_health] 行，不依赖本行。
    if (now_s - last_pos_log_t_ >= 0.5) {
      last_pos_log_t_ = now_s;
      const Eigen::Vector3f delta_position =
        current_position_ned - previous_published_position_ned_;

      std::cout << std::fixed << std::setprecision(4)
                << "[px4_odom_convert] position delta NED [m]: "
                << "dx=" << delta_position.x() << "  "
                << "dy=" << delta_position.y() << "  "
                << "dz=" << delta_position.z()
                << '\n';
    }
  } else {
    std::cout << std::fixed << std::setprecision(4)
              << "[px4_odom_convert] first position NED [m]: "
              << "x=" << current_position_ned.x() << "  "
              << "y=" << current_position_ned.y() << "  "
              << "z=" << current_position_ned.z()
              << std::endl;
  }

  previous_published_position_ned_ = current_position_ned;
  has_previous_published_position_ = true;

  vio_pub_->publish(vio);
}

void Px4OdomConvert::lookupTfAndPublish()
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      tf_map_frame_, tf_lidar_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "TF lookup [%s->%s] failed: %s",
      tf_map_frame_.c_str(), tf_lidar_frame_.c_str(), ex.what());
    return;
  }

  if (has_last_tf_stamp_ && tf_stamped.header.stamp == last_tf_stamp_) {
    return;
  }
  last_tf_stamp_ = tf_stamped.header.stamp;
  has_last_tf_stamp_ = true;

  nav_msgs::msg::Odometry odom{};
  odom.header.stamp = tf_stamped.header.stamp;
  odom.header.frame_id = tf_stamped.header.frame_id;
  odom.child_frame_id = tf_stamped.child_frame_id;
  odom.pose.pose.position.x = tf_stamped.transform.translation.x;
  odom.pose.pose.position.y = tf_stamped.transform.translation.y;
  odom.pose.pose.position.z = tf_stamped.transform.translation.z;
  odom.pose.pose.orientation = tf_stamped.transform.rotation;
  // velocity fields remain zero — PX4 EKF2 is configured to not fuse velocity

  publishVehicleVisualOdometry(odom);
}

void Px4OdomConvert::run()
{
  if (source_mode_ != "tf") {
    rclcpp::spin(shared_from_this());
    return;
  }

  rclcpp::Rate rate(loop_rate_hz_);

  while (rclcpp::ok()) {
    rclcpp::spin_some(shared_from_this());
    lookupTfAndPublish();
    rate.sleep();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 防双实例①：构造抛异常（VIO 话题已有发布者）时拒绝启动、干净退出
  try {
    auto node = std::make_shared<Px4OdomConvert>();
    node->run();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("px4_odom_convert"),
      "startup refused: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
