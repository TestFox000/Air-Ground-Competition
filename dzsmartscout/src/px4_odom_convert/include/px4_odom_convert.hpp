#ifndef PX4_ODOM_CONVERT__PX4_ODOM_CONVERT_HPP_
#define PX4_ODOM_CONVERT__PX4_ODOM_CONVERT_HPP_
// ============================================================================
// px4_odom_convert 增强版 v2（air_ground 韧性定位体系，2026-10-17）
//
// 与官方原版的差异（全部来自实飞实测证据，见注释）：
//   1) auto-align：SLAM 位姿直通 + EV reset_counter 一次重锚（团队实验版）
//   2) jump guard：三级防护（2026-10-17 日志回溯修复，详见 cpp 内注释）：
//      L1 insane 熔断 —— 单帧位移超过物理上限（默认 2m/帧）判定 SLAM 数据
//         无效，输出锁定在最后可信位姿，恢复需 SLAM 相邻帧稳定约 2s；
//      L2 bump 节流 —— 真实大位移接受的 reset_counter bump 最小间隔 10s，
//         窗口内只跟踪不放行（旧版每秒多次 bump 直接造成 EKF reset 风暴）；
//      L3 EKF reset 风暴熔断 —— 60s 滑动窗内 EKF reset > 5 次即冻结一切
//         accept/bump，hold 保 EKF，风暴缓解后自动恢复。
//      实测背景（node_odom.log 10-16~10-17）：FAST-LIO ICP 摇摆时
//      /localization 持续输出 26~585m 跳变，旧版"连续 8 帧即接受"把疯跳
//      当真实位移放行且每次都 bump，EKF xy_reset 每秒 +1（累计 3764 次）。
//   3) armed 门控：解锁状态下跳变永不接受（物理不可能 = 100% 跟丢）
//   4) z 换源：高度用 EKF 反馈 z（气压计主导），SLAM z（漂 5.56/8.9m）绝缘
//   5) 可观测性（v2.1，2026-10-17）：事件级 blackbox（[px4_odom_convert_blackbox]
//      4 行完整上下文）+ 3s 周期健康行（[px4_odom_convert_health]），机上
//      localization_watchdog.py 据此感知定位质量——补齐"EKF 每秒 reset 时
//      watchdog 仍报健康"的监控缺口。决策路径与既有日志行格式零改动。
//
// 部署：tools/patch_deploy.py 一键完成（备份机上原版 → 推本文件 → colcon）。
// 机上对应备份：px4_odom_convert.cpp.bak_20261016 / .bak_v2_20261016。
// ============================================================================

#include <string>
#include <cmath>
#include <deque>
#include <memory>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_ros_com/frame_transforms.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

class Px4OdomConvert : public rclcpp::Node
{
public:
  Px4OdomConvert();
  void run();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void onFcuStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  void onEkfLocalPosition(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  void publishVehicleVisualOdometry(
    const nav_msgs::msg::Odometry & odom);

  void computeAlignment(double slam_yaw_ned, double ekf_heading);

  void lookupTfAndPublish();
  uint64_t nowUs();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    sub_local_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vio_pub_;

  // ---- 防双实例（2026-10-17 双栈并行实锤后的加固） ----
  // 实测（10-16 21:39:53~21:46:38）：手动栈与 autostart 栈并行，双
  // px4_odom_convert 同发 VIO 93 秒、双 FAST-LIO 同发 /localization 406
  // 秒，EKF 被两路测量撕扯 → reset 风暴。防护分两级：
  //   ① 启动拒绝：构造时若 VIO 话题已有发布者（另一个实例在跑）直接
  //      抛异常拒绝启动（main 捕获后退出）；
  //   ② 运行期监测：10s 定时器查 VIO/SLAM 话题发布者数，>1 打 ERROR
  //      告警（含双 SLAM 源检测），不自动退出——armed 下双实例跳变由
  //      L1 熔断兜底，而自动退出可能造成飞行中 VIO 断链（更危险），
  //      交给看门狗/人工 kill。
  rclcpp::TimerBase::SharedPtr dup_check_timer_;

  // Used only by TF mode to avoid publishing the same TF sample repeatedly.
  builtin_interfaces::msg::Time last_tf_stamp_{};
  bool has_last_tf_stamp_{false};

  std::string source_mode_;
  std::string odom_topic_;
  std::string localization_topic_;
  std::string tf_map_frame_;
  std::string tf_lidar_frame_;
  double z_offset_m_;
  double loop_rate_hz_;

  int odom_ready_count_{0};
  bool odom_ready_{false};

  float nan = std::numeric_limits<float>::quiet_NaN();

  bool has_previous_published_position_{false};
  Eigen::Vector3f previous_published_position_ned_{Eigen::Vector3f::Zero()};

  Eigen::Matrix3d R_enu_slam_;
  Eigen::Vector3d t_enu_slam_;

  // ---- one-shot alignment to EKF local NED frame ----
  // With align_yaw_to_ekf=false (default for SLAM-only flight): the SLAM pose
  // is published unchanged (fixed ENU->NED rotation only) and the EKF is
  // re-anchored to it once via a single reset_counter bump. The EKF frame
  // then coincides with the SLAM map frame, which is what px4_path replay
  // and rviz overlay expect.
  // With align_yaw_to_ekf=true: the SLAM yaw is rotated once (averaged over
  // ~1s, then frozen) to match the EKF heading convention; used when the EKF
  // heading source (mag/GPS) must stay authoritative.
  bool enable_auto_align_{true};
  bool align_yaw_to_ekf_{false};

  bool aligned_{false};
  double yaw_offset_{0.0};

  int align_sample_count_{0};
  double align_ekf_heading_ref_{0.0};
  double align_slam_yaw_ref_{0.0};
  double align_ekf_heading_sum_{0.0};
  double align_slam_yaw_sum_{0.0};

  bool ekf_pos_valid_{false};
  Eigen::Vector3d ekf_pos_ned_{Eigen::Vector3d::Zero()};
  double ekf_heading_{0.0};

  bool has_reset_counters_{false};
  uint8_t last_xy_reset_counter_{0};
  uint8_t last_heading_reset_counter_{0};

  // ---- jump guard v2：SLAM 跟丢/错配的三级防护（2026-10-17 修复） ----
  // 2026-10-16 试飞实测：FAST-LIO 在被搬运/剧烈晃动时跟丢重捕，/localization
  // 单步输出 37~62m 跳变、z 漂 5m；直通策略让 EKF 瞬间被打崩、解锁坐标漂走。
  // 2026-10-17 日志回溯发现旧版两处放大器：①"连续 8 帧即接受"在 SLAM 持续
  // 疯跳（ICP 摇摆输出 26~585m/帧）时把疯跳当真实位移放行；②每次接受都
  // bump EV reset_counter，EKF 被每秒重锚（xy_reset 每秒 +1，累计 3764 次，
  // 计数 255 饱和循环），悬停被反复撕扯。三级修复见hpp 头注释。
  double jump_hold_m_{0.5};     // 单步位置跳变阈值（米）：超出先 hold 观察
  double jump_yaw_rad_{0.8};   // 单步 yaw 跳变阈值（弧度，~46°）
  int jump_confirm_n_{8};       // 连续多少帧大位移才判定"真实大位移"（43Hz 下约 0.2s）

  // L1 insane 熔断：单帧位移超过物理可能（43Hz 下 2m/帧 ≈ 86m/s）判定 SLAM
  // 数据无效。实测疯跳 26~585m 远超任何真实运动；熔断后 hold 到底、绝不
  // bump，恢复需 SLAM 相邻帧稳定 slam_recover_n_ 帧（约 2s）。
  double slam_insane_m_{2.0};      // 单帧位置熔断门限（米）
  int slam_recover_n_{86};          // 恢复所需连续稳定帧数（43Hz 下约 2s）
  double slam_recover_m_{0.1};      // 相邻帧稳定判定的位移门限（米）
  double slam_recover_yaw_rad_{0.1}; // 相邻帧稳定判定的 yaw 门限（弧度）

  // L2 bump 节流：真实大位移接受后 bump reset_counter 的最小间隔。
  double bump_min_interval_s_{10.0};

  // L3 EKF reset 风暴熔断：滑动窗内 EKF reset 超过阈值即冻结一切 accept/bump。
  double ekf_storm_window_s_{60.0};
  int ekf_storm_max_resets_{5};

  bool gate_has_prev_{false};
  int gate_hold_count_{0};
  Eigen::Vector3d gate_prev_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond gate_prev_quat_{Eigen::Quaterniond::Identity()};

  // gate_out_*：hold 期间实际输出的位姿（上一次真正放行的位姿）。
  // 与 gate_prev_*（判定基准）同步更新（正常放行 / 真正 bump 放行时）；
  // 节流窗口内两者均保持不动，保证 EKF 看到的始终是稳定测量。
  Eigen::Vector3d gate_out_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond gate_out_quat_{Eigen::Quaterniond::Identity()};

  // L1 状态：SLAM 原始相邻帧跟踪（不经过 hold 替换），用于熔断后的恢复判定
  bool slam_bad_{false};
  int slam_recover_count_{0};
  bool slam_has_prev_raw_{false};
  Eigen::Vector3d slam_prev_raw_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond slam_prev_raw_quat_{Eigen::Quaterniond::Identity()};

  // L2 状态：上次 bump 时刻（秒），对齐完成的首次 bump 同样记录
  bool has_last_bump_{false};
  double last_bump_time_{0.0};

  // L3 状态：EKF reset 时间戳滑动窗 + 风暴标志
  std::deque<double> reset_times_;
  bool ekf_storm_{false};

  // 飞控解锁状态（2026-10-16）：armed 时 jump 永不接受——无人机物理上
  // 不可能在 0.2s 内移动数米，armed 大位移 100% 是 SLAM 跟丢，hold 到底
  // 保 EKF；仅地面待命（disarmed）允许"被搬运"式重锚。
  bool fcu_armed_{false};

  // 我们维护的 EV reset_counter：对齐完成时 bump 到 1（EKF 重锚到 SLAM 帧），
  // 之后每次"接受真实大位移"再 +1（EKF 重锚到新位姿），其余时间保持不变。
  uint8_t ev_reset_counter_{0};

  // ---- 定位质量可观测性 v2.1（2026-10-17：blackbox 增强 + 健康行 + watchdog）----
  // 缺口（真机日志回溯实锤）：①机上 watchdog 对定位质量零监控——EKF 每秒
  // reset 风暴持续 14 小时无人察觉；②事件日志过薄（每次 2-3 行）无法事后
  // 归因。本节为"只增不改"的观测层：jump guard 决策路径与既有日志行完全
  // 不动，新增两类结构化输出（均走 std::cout，与 position delta 同一流）：
  //   [px4_odom_convert_blackbox] 事件 4 行完整上下文（事后归因唯一依据）：
  //     1) event/t/trans/armed/aligned        2) jump：跳变距离+方向（NED 分量
  //     + 起止位姿）                          3) ekf：reset 计数/风暴/ev_reset 快照
  //     4) slam_q：近 3s 质量窗口（跳变 max/mean、稳定进度、协方差迹峰值）
  //   [px4_odom_convert_health] 3s 周期健康行（key=value，字段只增不改），
  //     watchdog（localization_watchdog.py）的唯一常态数据源；行停更即
  //     "节点死/SLAM 断流"，watchdog 据此告警。
  // 兼容性：新前缀不含 'jump guard'/'position delta'/'Resilience layer'
  // 子串，机上既有日志解析（patch_deploy.py 验证步骤等）不受影响。
  rclcpp::TimerBase::SharedPtr health_timer_;

  // 3s 窗口统计（健康行与 blackbox slam_q 的数据源；每条健康行输出后清零）
  int win_slam_frames_{0};
  int win_hold_frames_{0};
  int win_stable_frames_{0};
  double win_jump_sum_m_{0.0};
  double win_jump_max_m_{0.0};
  double win_cov_max_{0.0};

  // 事件计数与时间戳（健康行"近期是否触发 L1/L2/L3"数据源；-1 = 从未）
  bool last_gate_holding_{false};
  bool l2_throttling_{false};
  int total_l1_triggers_{0};
  int total_bumps_{0};
  int total_storm_on_{0};
  long total_ekf_resets_{0};
  double last_l1_trigger_t_{-1.0};
  double last_l3_storm_on_t_{-1.0};
  double last_l3_storm_clear_t_{-1.0};

  void onHealthTimer();
  void writeBlackbox(
    const char * event, const char * trans,
    double dpos, double dyaw,
    const Eigen::Vector3d & from_pos, const Eigen::Vector3d & to_pos);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // PX4_ODOM_CONVERT__PX4_ODOM_CONVERT_HPP_
