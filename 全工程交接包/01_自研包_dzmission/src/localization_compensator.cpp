#include "dzmission/localization_compensator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace dzmission
{

// ★ 节点版本戳：launch 文件加载的是 colcon **安装副本**（install/ 或
//   site-packages），源码改了但忘记 colcon build 时，跑起来的仍是旧代码
//   —— 这个坑一定会踩。把版本号打进关键日志，一眼就能看出跑的是哪一版。
//   （学员代码的教训："a stale colcon install silently runs old code"）
constexpr const char *kNodeVersion = "v5-dr-imu-yaw-anchor";

namespace
{
double normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

LocalizationCompensator::LocalizationCompensator(const rclcpp::NodeOptions &options)
: Node("localization_compensator", options)
{
  loadParams();

  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();
  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();
  auto fix_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      std::bind(&LocalizationCompensator::imuCallback, this, std::placeholders::_1));
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_qos,
      std::bind(&LocalizationCompensator::odomCallback, this, std::placeholders::_1));
  sub_amcl_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_topic_, fix_qos,
      std::bind(&LocalizationCompensator::amclCallback, this, std::placeholders::_1));
  sub_anchor_ = this->create_subscription<std_msgs::msg::String>(
      anchor_topic_, 10,
      std::bind(&LocalizationCompensator::anchorCallback, this, std::placeholders::_1));

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 20);
  pub_healthy_ = this->create_publisher<std_msgs::msg::UInt8>("localization/healthy", 5);
  pub_yaw_deg_ = this->create_publisher<std_msgs::msg::Float32>("localization/yaw_deg", 5);
  pub_bias_ = this->create_publisher<std_msgs::msg::Float32>("localization/gyro_bias_z", 5);
  pub_step_cm_ = this->create_publisher<std_msgs::msg::Float32>("localization/dr_step_cm", 5);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  last_imu_time_ = this->now();
  last_imu_rx_time_ = this->now();
  last_odom_time_ = this->now();
  last_fix_time_ = this->now();
  last_pub_time_ = this->now();
  first_data_time_ = this->now();
  last_step_time_ = this->now();

  // 启动锚定：把位置设到初始格心、偏航设为初始地图偏航
  if (anchor_at_startup_)
  {
    // 格心世界坐标（米）。约定：col A..N 对应 x 递增，row 1..11 对应 y 递增
    dr_x_m_ = (initial_col_ + 0.5) * block_size_m_;
    dr_y_m_ = (initial_row_ + 0.5) * block_size_m_;
    yaw_offset_rad_ = initial_map_yaw_deg_ * M_PI / 180.0;
    yaw_offset_valid_ = true;
    map_yaw_ = initial_map_yaw_deg_;
    anchor_valid_ = false;
  }

  // 看门狗：IMU 一旦停止发布，imuCallback 不会再被调用，
  // 如果只在回调里更新 healthy_，状态会永久冻结在最后一次的"健康"上，
  // 任务层就无法察觉定位失效。这里独立定时评估并发布。
  watchdog_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LocalizationCompensator::watchdogCallback, this));

  RCLCPP_INFO(this->get_logger(),
              "[Localization %s] imu=%s odom=%s anchor=%s fix=%s(|%d) "
              "tf=%d map->odom=%d",
              kNodeVersion, imu_topic_.c_str(), odom_topic_.c_str(),
              anchor_topic_.c_str(), amcl_topic_.c_str(),
              static_cast<int>(enable_absolute_fix_),
              static_cast<int>(publish_tf_), static_cast<int>(publish_map_to_odom_));
  RCLCPP_INFO(this->get_logger(),
              "[Localization] 位置来源=航位推算(距离沿IMU偏航积分)，"
              "启动锚定 col=%d row=%d yaw=%.1fdeg",
              initial_col_, initial_row_, initial_map_yaw_deg_);
}

LocalizationCompensator::~LocalizationCompensator() = default;

void LocalizationCompensator::loadParams()
{
  const std::string p = "loc.";

  declare_parameter(p + "imu_topic", imu_topic_);
  declare_parameter(p + "odom_topic", odom_topic_);
  declare_parameter(p + "amcl_topic", amcl_topic_);
  declare_parameter(p + "imu_timeout_ms", imu_timeout_ms_);
  declare_parameter(p + "odom_timeout_ms", odom_timeout_ms_);
  declare_parameter(p + "zupt_linear_thresh", zupt_linear_thresh_);
  declare_parameter(p + "zupt_gyro_thresh", zupt_gyro_thresh_);
  declare_parameter(p + "zupt_min_duration_ms", zupt_min_duration_ms_);
  declare_parameter(p + "wheel_sigma_v", wheel_sigma_v_);
  declare_parameter(p + "gyro_sigma_z", gyro_sigma_z_);
  declare_parameter(p + "enable_absolute_fix", enable_absolute_fix_);
  declare_parameter(p + "fix_sigma_pos", fix_sigma_pos_);
  declare_parameter(p + "fix_sigma_yaw", fix_sigma_yaw_);
  declare_parameter(p + "fix_min_interval_ms", fix_min_interval_ms_);
  declare_parameter(p + "fix_max_jump_m", fix_max_jump_m_);
  declare_parameter(p + "publish_tf", publish_tf_);
  declare_parameter(p + "publish_map_to_odom", publish_map_to_odom_);
  declare_parameter(p + "odom_frame", odom_frame_);
  declare_parameter(p + "base_frame", base_frame_);
  declare_parameter(p + "map_frame", map_frame_);

  // ★ 航位推算参数
  declare_parameter(p + "anchor_topic", anchor_topic_);
  declare_parameter(p + "dr_min_step_m", dr_min_step_m_);
  declare_parameter(p + "dr_max_step_m", dr_max_step_m_);
  declare_parameter(p + "initial_map_yaw_deg", initial_map_yaw_deg_);
  declare_parameter(p + "initial_col", initial_col_);
  declare_parameter(p + "initial_row", initial_row_);
  declare_parameter(p + "anchor_at_startup", anchor_at_startup_);
  declare_parameter(p + "block_size_m", block_size_m_);
  declare_parameter(p + "anchor_prefix", anchor_prefix_);

  get_parameter(p + "imu_topic", imu_topic_);
  get_parameter(p + "odom_topic", odom_topic_);
  get_parameter(p + "amcl_topic", amcl_topic_);
  get_parameter(p + "imu_timeout_ms", imu_timeout_ms_);
  get_parameter(p + "odom_timeout_ms", odom_timeout_ms_);
  get_parameter(p + "zupt_linear_thresh", zupt_linear_thresh_);
  get_parameter(p + "zupt_gyro_thresh", zupt_gyro_thresh_);
  get_parameter(p + "zupt_min_duration_ms", zupt_min_duration_ms_);
  get_parameter(p + "wheel_sigma_v", wheel_sigma_v_);
  get_parameter(p + "gyro_sigma_z", gyro_sigma_z_);
  get_parameter(p + "enable_absolute_fix", enable_absolute_fix_);
  get_parameter(p + "fix_sigma_pos", fix_sigma_pos_);
  get_parameter(p + "fix_sigma_yaw", fix_sigma_yaw_);
  get_parameter(p + "fix_min_interval_ms", fix_min_interval_ms_);
  get_parameter(p + "fix_max_jump_m", fix_max_jump_m_);
  get_parameter(p + "publish_tf", publish_tf_);
  get_parameter(p + "publish_map_to_odom", publish_map_to_odom_);
  get_parameter(p + "odom_frame", odom_frame_);
  get_parameter(p + "base_frame", base_frame_);
  get_parameter(p + "map_frame", map_frame_);

  get_parameter(p + "anchor_topic", anchor_topic_);
  get_parameter(p + "dr_min_step_m", dr_min_step_m_);
  get_parameter(p + "dr_max_step_m", dr_max_step_m_);
  get_parameter(p + "initial_map_yaw_deg", initial_map_yaw_deg_);
  get_parameter(p + "initial_col", initial_col_);
  get_parameter(p + "initial_row", initial_row_);
  get_parameter(p + "anchor_at_startup", anchor_at_startup_);
  get_parameter(p + "block_size_m", block_size_m_);
  get_parameter(p + "anchor_prefix", anchor_prefix_);
}

// -----------------------------------------------------------------------------
void LocalizationCompensator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const rclcpp::Time stamp = this->now();

  std::lock_guard<std::mutex> lk(mtx_);

  if (!eskf_.initialized())
  {
    eskf_.initializeFromImu(*msg);
    eskf_.tryFinishInit(9.80665);
    have_imu_ = true;
    last_imu_time_ = stamp;
    return;
  }

  if (!have_imu_)
  {
    last_imu_time_ = stamp;
    have_imu_ = true;
    return;
  }

  double dt = (stamp - last_imu_time_).seconds();
  last_imu_time_ = stamp;
  if (dt <= 0.0 || dt > 0.5)
  {
    // 时间跳变（首次收到 / 丢包），跳过本次预测
    return;
  }

  eskf_.predict(*msg, dt);

  // ---- ★ 航向：IMU 相对偏航 + 标定偏移 -> 地图偏航 ----
  // IMU 的绝对读数没有意义（安装朝向未知），但**相对变化**准确。
  // 所以记下第一帧作为零点，之后只用相对量；再加上 yaw_offset_rad_
  // 把它映射到地图系。yaw_offset_rad_ 由启动锚定或 /nav/anchor 指令标定。
  {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                      msg->orientation.z, msg->orientation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw_raw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_raw);
    (void)roll;
    (void)pitch;
    if (!have_imu_yaw_)
    {
      imu_yaw_bias_ = yaw_raw;
      have_imu_yaw_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "[Localization] IMU 偏航零点标定 %.2f deg",
                  yaw_raw * 180.0 / M_PI);
    }
    imu_yaw_rel_ = normalizeAngle(yaw_raw - imu_yaw_bias_);
    if (yaw_offset_valid_)
    {
      map_yaw_ = normalizeAngle(imu_yaw_rel_ + yaw_offset_rad_) * 180.0 / M_PI;
    }
  }

  // ---- ZUPT：连续静止时把速度钉住，防止 IMU 零偏积分产生的假位移 ----
  // 注意：ZUPT 只作用于 ESKF 的速度/零偏估计。**位置不再来自 ESKF**，
  // 而是由 odomCallback 里的"距离沿地图偏航积分"得到，所以 IMU 零偏
  // 不会像以前那样直接积分成假位移。
  const double speed = eskf_.velocity().head<2>().norm();
  const double gyro_z = std::fabs(msg->angular_velocity.z - eskf_.gyroBiasZ());
  if (speed < zupt_linear_thresh_ && gyro_z < zupt_gyro_thresh_)
  {
    if (stationary_since_ < 0.0)
    {
      stationary_since_ = stamp.seconds();
    }
    else if ((stamp.seconds() - stationary_since_) * 1000.0 > zupt_min_duration_ms_)
    {
      eskf_.zeroVelocityUpdate(zupt_linear_thresh_ * 0.5);
      eskf_.updateGyroZ(msg->angular_velocity.z, 0.005);
    }
  }
  else
  {
    stationary_since_ = -1.0;
  }

  // ---- 健康判定 ----
  last_imu_rx_time_ = stamp;
  healthy_ = evaluateHealth();

  publishEstimate();
}

// -----------------------------------------------------------------------------
// ★ 锚点标定：/nav/anchor  "anchor:COL,ROW,HEADING_DEG"
//   语义 = 车**此刻**位于 COL,ROW 格心、朝向 HEADING_DEG（度，地图系，
//          0 = 沿 +x；90 = 沿 +y）。
//   作用：
//     1. 把航位推算位置吸附到该格心（消除累积漂移）
//     2. 把 IMU 偏航到地图偏航的偏移量重新标定为 HEADING_DEG
//   这等价于学员代码里的 'reset:N10,y'，但用格坐标 + 角度显式给出，
//   不需要在车端维护一个"朝向代号 -> 角度"的映射表。
// -----------------------------------------------------------------------------
void LocalizationCompensator::anchorCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);

  const std::string data = msg->data;
  if (data.rfind(anchor_prefix_, 0) != 0)
  {
    return;   // 不是锚点指令，忽略（同一话题可能复用）
  }

  const std::string body = data.substr(anchor_prefix_.size());
  // 解析 COL,ROW,HEADING
  const size_t c1 = body.find(',');
  const size_t c2 = (c1 == std::string::npos) ? std::string::npos
                                              : body.find(',', c1 + 1);
  if (c1 == std::string::npos || c2 == std::string::npos)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "[Localization] 锚点指令格式错误 '%s'，应为 "
                 "'anchor:COL,ROW,HEADING_DEG'",
                 data.c_str());
    return;
  }

  const std::string col_s = body.substr(0, c1);
  const std::string row_s = body.substr(c1 + 1, c2 - c1 - 1);
  const std::string head_s = body.substr(c2 + 1);

  int col = 0;
  int row = 0;
  double head_deg = 0.0;
  try
  {
    if (col_s.size() != 1)
    {
      throw std::invalid_argument("col 必须是单个字母 A..N");
    }
    col = static_cast<int>(std::toupper(col_s[0]) - 'A');
    row = std::stoi(row_s) - 1;
    head_deg = std::stod(head_s);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "[Localization] 锚点解析失败：%s", e.what());
    return;
  }

  if (col < 0 || col > 13 || row < 0 || row > 10)
  {
    RCLCPP_ERROR(this->get_logger(), "[Localization] 锚点格坐标越界 col=%d row=%d",
                 col, row);
    return;
  }
  if (!have_imu_yaw_)
  {
    RCLCPP_WARN(this->get_logger(),
                "[Localization] 锚点被忽略：尚未收到 IMU 偏航");
    return;
  }

  // 1) 位置吸附到格心
  const double new_x = (col + 0.5) * block_size_m_;
  const double new_y = (row + 0.5) * block_size_m_;
  const double dx = new_x - dr_x_m_;
  const double dy = new_y - dr_y_m_;
  dr_x_m_ = new_x;
  dr_y_m_ = new_y;

  // 2) 偏航偏移重新标定
  const double target_rad = head_deg * M_PI / 180.0;
  const double old_map = map_yaw_;
  yaw_offset_rad_ = normalizeAngle(target_rad - imu_yaw_rel_);
  yaw_offset_valid_ = true;
  map_yaw_ = head_deg;

  // 3) map->odom 重锚，避免 TF 跳变
  anchor_x_ += dx;
  anchor_y_ += dy;
  anchor_valid_ = true;

  RCLCPP_INFO(this->get_logger(),
              "[Localization %s] 锚定 %c%d 朝向 %.1fdeg：位置修正 (%.3f, %.3f) m，"
              "地图偏航 %.1f -> %.1f deg",
              kNodeVersion, 'A' + col, row + 1, head_deg, dx, dy,
              old_map, map_yaw_);
}

// -----------------------------------------------------------------------------
// 供任务层在"到达某格心"时调用，把位置吸附到格心（限制漂移累积）。
// 这就是学员代码的"每个转角做位姿快照"在现代 C++ 侧的等价物。
// -----------------------------------------------------------------------------
void LocalizationCompensator::snapToCell(int col, int row)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (col < 0 || col > 13 || row < 0 || row > 10)
  {
    return;
  }
  const double new_x = (col + 0.5) * block_size_m_;
  const double new_y = (row + 0.5) * block_size_m_;
  const double dx = new_x - dr_x_m_;
  const double dy = new_y - dr_y_m_;
  dr_x_m_ = new_x;
  dr_y_m_ = new_y;
  anchor_x_ += dx;
  anchor_y_ += dy;
  anchor_valid_ = true;
  RCLCPP_INFO(this->get_logger(),
              "[Localization] 转角快照 -> %c%d，修正 (%.3f, %.3f) m",
              'A' + col, row + 1, dx, dy);
}

// -----------------------------------------------------------------------------
// 健康评估：IMU / 里程计 是否新鲜 + 状态是否有限
// 启动后 startup_grace_ms_ 内不判故障，给传感器与串口留出建立时间
// -----------------------------------------------------------------------------
bool LocalizationCompensator::evaluateHealth() const
{
  const rclcpp::Time now = this->now();

  if ((now - first_data_time_).seconds() * 1000.0 < startup_grace_ms_)
  {
    return true;
  }

  const bool imu_ok =
      (now - last_imu_rx_time_).seconds() * 1000.0 < imu_timeout_ms_;
  const bool odom_ok = !have_odom_ ||
                       (now - last_odom_time_).seconds() * 1000.0 < odom_timeout_ms_;
  // 状态有限性：现在检查的是**航位推算状态**（位置/偏航）与 IMU 偏航零点，
  // 因为位置已不再来自 ESKF，检查 ESKF 位置没有意义
  const bool state_ok = std::isfinite(dr_x_m_) && std::isfinite(dr_y_m_) &&
                        std::isfinite(map_yaw_) && have_imu_yaw_ &&
                        yaw_offset_valid_;

  return imu_ok && odom_ok && state_ok;
}

// -----------------------------------------------------------------------------
// 看门狗：IMU 停发时回调不再触发，必须由定时器接管状态发布与故障判定
// -----------------------------------------------------------------------------
void LocalizationCompensator::watchdogCallback()
{
  const rclcpp::Time now = this->now();

  {
    std::lock_guard<std::mutex> lk(mtx_);
    const bool was_healthy = healthy_;
    healthy_ = evaluateHealth();

    if (was_healthy && !healthy_ && eskf_.initialized())
    {
      RCLCPP_WARN(this->get_logger(),
                  "[Localization] UNHEALTHY: imu_age=%.0fms odom_age=%.0fms",
                  (now - last_imu_rx_time_).seconds() * 1000.0,
                  (now - last_odom_time_).seconds() * 1000.0);
    }

    // 数据新鲜时不在这里发布（避免与 imuCallback 的双重发布竞争）
    const bool data_fresh =
        (now - last_imu_rx_time_).seconds() * 1000.0 < imu_timeout_ms_;
    if (data_fresh && eskf_.initialized())
    {
      return;
    }
  }

  // IMU 已停发或尚未初始化：以 50Hz 持续发布当前估计与"不健康"标志
  if ((now - last_pub_time_).seconds() < 0.02)
  {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  publishEstimate();
}

// -----------------------------------------------------------------------------
void LocalizationCompensator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time stamp = this->now();

  std::lock_guard<std::mutex> lk(mtx_);

  last_odom_time_ = stamp;
  have_odom_ = true;

  // dzactuator 的 odom.twist 已经是车体系速度（linear.x 前进，angular.z 偏航率）
  wheel_vx_ = msg->twist.twist.linear.x;
  wheel_vy_ = msg->twist.twist.linear.y;
  last_w_ = msg->twist.twist.angular.z;

  // ===========================================================================
  // ★★ 位置更新：只用「相邻采样的行驶距离」，沿**地图偏航**积分 ★★
  // ---------------------------------------------------------------------------
  // 为什么不用 msg->pose：dzactuator 积分出的位姿活在一个"编码器偏航系"里，
  // 该系的旋转与物理世界不一致（实测：同一次 90° 右转，两次运行的原始轨迹
  // 偏向相反两侧，并单调外扩），所以原始位姿增量不能当世界坐标用。
  // 但**采样间距离是旋转不变量**且实测准确，所以取距离、沿 IMU 偏航积分。
  // ===========================================================================
  const double raw_x = msg->pose.pose.position.x;
  const double raw_y = msg->pose.pose.position.y;

  if (!have_prev_raw_)
  {
    have_prev_raw_ = true;
    prev_raw_x_ = raw_x;
    prev_raw_y_ = raw_y;
    return;   // 第一帧只用来建立基准，不产生位移
  }

  const double step = std::hypot(raw_x - prev_raw_x_, raw_y - prev_raw_y_);
  prev_raw_x_ = raw_x;
  prev_raw_y_ = raw_y;
  last_step_m_ = step;

  // 静止抖动滤除 + 异常跳变丢弃
  if (step >= dr_min_step_m_ && step <= dr_max_step_m_ && yaw_offset_valid_)
  {
    const double yaw_rad = map_yaw_ * M_PI / 180.0;
    dr_x_m_ += step * std::cos(yaw_rad);
    dr_y_m_ += step * std::sin(yaw_rad);
    total_travel_m_ += step;

    const double dt_step = (stamp - last_step_time_).seconds();
    last_step_v_ = (dt_step > 1e-3) ? (step / dt_step) : 0.0;

    std_msgs::msg::Float32 f;
    f.data = static_cast<float>(step * 100.0);
    pub_step_cm_->publish(f);
  }
  else if (step > dr_max_step_m_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "[Localization] 里程计单步跳变 %.3f m，已丢弃该步",
                         step);
  }
  last_step_time_ = stamp;

  // ESKF 仍然保留：它的陀螺零偏估计与 ZUPT 对航向有帮助，
  // 但**它的位置输出不再被使用**（避免把坏的位姿系吸进状态）。
  if (eskf_.initialized())
  {
    const double sigma = std::max(wheel_sigma_v_, 0.02 + 0.05 * std::fabs(wheel_vx_));
    eskf_.updateWheelVelocity(wheel_vx_, wheel_vy_, sigma);
  }
}

// -----------------------------------------------------------------------------
void LocalizationCompensator::amclCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!enable_absolute_fix_)
  {
    return;
  }

  const rclcpp::Time stamp = this->now();
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;
  const double yaw = tf2::getYaw(msg->pose.pose.orientation);

  std::lock_guard<std::mutex> lk(mtx_);

  // 频率限制
  if ((stamp - last_fix_time_).seconds() * 1000.0 < fix_min_interval_ms_)
  {
    return;
  }

  if (!eskf_.initialized())
  {
    eskf_.updateAbsolutePose(x, y, yaw, fix_sigma_pos_, fix_sigma_yaw_);
    has_fix_ = true;
    last_fix_time_ = stamp;
    anchor_x_ = 0.0;
    anchor_y_ = 0.0;
    anchor_yaw_ = 0.0;
    anchor_valid_ = true;
    return;
  }

  // 异常跳变保护：与当前估计相差过大时拒绝该次观测
  const double dx = x - dr_x_m_;
  const double dy = y - dr_y_m_;
  const double jump = std::sqrt(dx * dx + dy * dy);
  if (jump > fix_max_jump_m_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "[Localization] reject absolute fix, jump=%.2f m > %.2f m",
                         jump, fix_max_jump_m_);
    return;
  }

  // 保存修正前的估计，用于把残差吸收进 map->odom
  // （位置来自航位推算，偏航来自 IMU 相对角 + 偏移量，两者都要重锚）
  const double x_before = dr_x_m_;
  const double y_before = dr_y_m_;
  const double yaw_before = map_yaw_ * M_PI / 180.0;

  // 绝对位姿只用来**重锚**：直接设置航位推算位置与偏航偏移量，
  // 不再送进 ESKF（ESKF 的位置输出已不使用，喂进去只会浪费算力）
  dr_x_m_ = x;
  dr_y_m_ = y;
  yaw_offset_rad_ = normalizeAngle(yaw - imu_yaw_rel_);
  yaw_offset_valid_ = true;
  map_yaw_ = normalizeAngle(yaw) * 180.0 / M_PI;

  // 把 (观测 - 修正前估计) 的残差累加到 map->odom 锚点：
  //   这样 odom 系本身是平滑的，跳变只体现在 map 与 odom 的相对关系上。
  const double rx = x - x_before;
  const double ry = y - y_before;
  const double ryaw = normalizeAngle(yaw - yaw_before);
  // 残差需要表达在 map 系：观测本身就是 map 系，直接累加
  anchor_x_ += rx;
  anchor_y_ += ry;
  anchor_yaw_ = normalizeAngle(anchor_yaw_ + ryaw);
  anchor_valid_ = true;

  has_fix_ = true;
  last_fix_time_ = stamp;

  RCLCPP_DEBUG(this->get_logger(),
               "[Localization] fix applied: res=(%.3f, %.3f, %.1f deg) anchor=(%.3f, %.3f)",
               rx, ry, ryaw * 180.0 / M_PI, anchor_x_, anchor_y_);
}

// -----------------------------------------------------------------------------
void LocalizationCompensator::publishEstimate()
{
  const rclcpp::Time now = this->now();

  // ★ 位置与偏航都来自航位推算状态（不是 ESKF 的绝对状态）
  const double x = dr_x_m_;
  const double y = dr_y_m_;
  const double yaw = map_yaw_ * M_PI / 180.0;
  // 速度用最近一步的行程推算，比 ESKF 的速度更贴近这段定位方案的实际含义
  const double vx = last_step_v_ * std::cos(yaw);
  const double vy = last_step_v_ * std::sin(yaw);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = tf2::toMsg(q);

  // 协方差：航位推算的误差随行程累积，但每经过一次锚点/转角快照就被截断。
  // 这里用一个随"自上次锚定以来的行程"增长的保守模型。
  const double sigma_pos = std::min(0.30, 0.01 + 0.05 * total_travel_m_);
  odom.pose.covariance[0] = sigma_pos * sigma_pos;
  odom.pose.covariance[6] = 0.0;
  odom.pose.covariance[1] = 0.0;
  odom.pose.covariance[7] = sigma_pos * sigma_pos;
  odom.pose.covariance[14] = 1e6;   // z 不可观测
  odom.pose.covariance[21] = 1e6;   // roll
  odom.pose.covariance[28] = 1e6;   // pitch
  odom.pose.covariance[35] = 0.01;  // yaw

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = last_w_;
  odom.twist.covariance[0] = 1e-3;
  odom.twist.covariance[7] = 1e-3;
  odom.twist.covariance[35] = 1e-2;

  pub_odom_->publish(odom);

  std_msgs::msg::UInt8 h;
  h.data = healthy_ ? 1 : 0;
  pub_healthy_->publish(h);

  std_msgs::msg::Float32 yd;
  yd.data = static_cast<float>(map_yaw_);
  pub_yaw_deg_->publish(yd);

  std_msgs::msg::Float32 bias;
  bias.data = static_cast<float>(eskf_.gyroBiasZ());
  pub_bias_->publish(bias);

  if (publish_tf_)
  {
    publishTransform(x, y, yaw);
  }

  last_pub_time_ = now;
}

void LocalizationCompensator::publishTransform(double x, double y, double yaw)
{
  const rclcpp::Time now = this->now();

  // odom -> base_link：直接用滤波后的位姿（odom 系与滤波输出一致）
  if (publish_map_to_odom_ && anchor_valid_)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = map_frame_;
    t.child_frame_id = odom_frame_;

    // map->odom 由锚点决定：x_odom = x_map + anchor
    t.transform.translation.x = anchor_x_;
    t.transform.translation.y = anchor_y_;
    t.transform.translation.z = 0.0;
    tf2::Quaternion aq;
    aq.setRPY(0.0, 0.0, anchor_yaw_);
    t.transform.rotation = tf2::toMsg(aq);
    tf_broadcaster_->sendTransform(t);
  }

  geometry_msgs::msg::TransformStamped ob;
  ob.header.stamp = now;
  ob.header.frame_id = odom_frame_;
  ob.child_frame_id = base_frame_;
  ob.transform.translation.x = x - anchor_x_;
  ob.transform.translation.y = y - anchor_y_;
  ob.transform.translation.z = 0.0;
  tf2::Quaternion oq;
  oq.setRPY(0.0, 0.0, normalizeAngle(yaw - anchor_yaw_));
  ob.transform.rotation = tf2::toMsg(oq);
  tf_broadcaster_->sendTransform(ob);
}

}  // namespace dzmission
