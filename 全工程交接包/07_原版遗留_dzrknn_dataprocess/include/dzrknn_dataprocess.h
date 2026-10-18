#ifndef DZRKNN_DATAPROCESS_H
#define DZRKNN_DATAPROCESS_H

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

/* FOREGROUND */
#define RST "\x1B[0m"
#define KRED "\x1B[31m"
#define KGRN "\x1B[32m"
#define KYEL "\x1B[33m"
#define KBLU "\x1B[34m"
#define KMAG "\x1B[35m"
#define KCYN "\x1B[36m"
#define KWHT "\x1B[37m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST

#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST

typedef struct sMartcarControl
{
  double TargetAngle;
  int SpeedDirection;
  int TargetSpeed;
  int LaserGunSwitch;
  int Position_0;
  int Position_1;
  int Speed_0;
  int Speed_1;
  int Time_0;
  int Time_1;
  int Fun;
  int Orin_ID;
  int Set_ID_Num;
  int VoiceSwitch;
} sMartcarControl;

typedef struct curYuntai_feedback
{
  int Position_0;
  int Position_1;
  int Speed_0;
  int Speed_1;
  int Load_0;
  int Load_1;
} curYuntai_feedback;

class turn_on_robot : public rclcpp::Node
{
public:
  explicit turn_on_robot(const std::string &name);
  ~turn_on_robot() override;

  void Control();

private:
  // ===== 回调 =====
  void callback_offset_center(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void callback_stop_point_signal(const std_msgs::msg::UInt8::SharedPtr msg);
  void callback_HpAndHitmsg(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void callback_camemo_feedback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

  // ===== 核心控制 =====
  void CaremaMontorControl();
  double CaremaSpeedControl(int target_pose, int current_pose);
  int clampTick(int tick) const;
  int compute_comp_pixel_0(double tar_dis);
  bool loadLookupTable(const std::string &filePath);

  // ===== 扫描辅助 =====
  double scanYawDeg() const;
  double scanPitchDeg() const;
  int pitchLevelCount() const;
  void applyScanTarget();
  void advanceScanIndex();

  int computeTrackingSpeed(int target, int current) const;
  void resetTrackingSmoothing();
  // 是否俯仰轴
  int computeTrackingSpeed(int target, int current, bool is_pitch) const;

  // ===== 自适应追踪参数 =====
  int fast_error_threshold_px_{40}; // 超过此误差进入"快速模式"
  double fast_alpha_{0.85};         // 快速模式下的 EMA 系数(更跟手)
  double fast_step_gain_{0.7};      // 快速模式下的步进增益
  int fast_max_step_{80};           // 快速模式下的单帧最大步进
  double fast_motor_kp_{50.0};      // 快速模式下的比例增益

  // ===== 追踪平滑与死区 =====
  double vision_cx_filtered_{0.0};  // EMA 后的水平像素误差
  double vision_cy_filtered_{0.0};  // EMA 后的垂直像素误差
  bool vision_smooth_ready_{false}; // 首帧直接赋值
  int vision_deadband_px_{3};       // 视觉死区(px)
  int position_deadband_ticks_{5};  // 位置死区(tick)
  double vision_smooth_alpha_{0.4}; // EMA 系数(0~1, 越小越平滑)
  double vision_step_gain_{0.3};    // tick/px 步进增益
  int max_step_per_frame_{20};      // 每帧位置最大增量(tick)
  int min_tracking_speed_{0};       // 追踪最小速度(0=不强制)

  // ===== 俯仰轴独立参数（固定高度靶）=====
  double pitch_motor_kp_{5.0};            // 俯仰比例增益（远低于 yaw 的 motor_kp）
  int pitch_max_speed_{200};              // 俯仰最大转速
  double pitch_step_gain_{0.1};           // 每像素俯仰步进(tick/px)
  int pitch_deadband_px_{8};              // 俯仰视觉死区(px)
  int pitch_position_deadband_ticks_{15}; // 俯仰位置死区(tick)
  int pitch_max_step_per_frame_{3};       // 每帧俯仰最大增量(tick)
  int pitch_scan_speed_{100};             // 俯仰扫描速度(可低到几乎不动)

  // ===== ROS 接口 =====
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_LaserShot_Command;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_CameMo_Conl;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_voice_switch;

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_offset_center;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_stop_point_singal;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_HpAndHitmsg;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_camemo_feedback;

  // ===== 运行状态 =====
  sMartcarControl moveBaseControl{};
  curYuntai_feedback curYuntai_feedback_data{};
  std_msgs::msg::UInt8 last_voice_msg;
  bool find_center{false};

  // 丢帧 / 心跳
  int lost_target_count_{0};
  int lost_frames_to_rescan_{60};
  int vision_timeout_ms_{1000};
  rclcpp::Time last_vision_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_step_time_{0, 0, RCL_ROS_TIME};

  // 激光状态机
  bool laser_active_{false};
  rclcpp::Time laser_start_time_{0, 0, RCL_ROS_TIME};
  int laser_duration_ms_{100};
  int fire_tolerance_px_{10};

  // ===== 新增：开火防抖与冷却 =====
  int in_tolerance_count_{0};   // 连续命中容差帧计数
  int fire_debounce_frames_{3}; // 需要连续 N 帧在容差内才开火
  int fire_cooldown_ms_{500};   // 两次开火最小间隔(ms)
  rclcpp::Time last_fire_time_{0, 0, RCL_ROS_TIME};

  // 扫描状态
  int scan_yaw_idx_{0};
  int scan_yaw_dir_{1};
  int scan_pitch_idx_{0};

  // ===== 参数 =====
  double motor_kp_{1000.0};
  double motor_ki_{0.0};
  double motor_kd_{0.0};
  double pose_kp_{1000.0};

  int scan_yaw_dirs_{8};
  double scan_yaw_min_deg_{-90.0};
  double scan_yaw_max_deg_{90.0};
  std::vector<double> scan_pitch_levels_deg_{30.0, 0.0, -10.0};
  double yaw_deg_sign_{1.0};
  double pitch_deg_sign_{1.0};
  int gimbal_center_tick_{2047};
  double yaw_ticks_per_deg_{11.111};
  double pitch_ticks_per_deg_{11.111};
  int gimbal_tick_min_{1000};
  int gimbal_tick_max_{3000};
  int gimbal_max_speed_{3700};
  double scan_speed_ratio_{0.3};
  int scan_dwell_ms_{300};
  int scan_arrive_tol_{30};

  // ===== 查表 =====
  std::vector<int> lookup_table;

  // ===== 其它 =====
  int stop_point_signal_msg_{0};
  int being_shot_data_{0};
  bool being_shot_{false};
};

#endif // DZRKNN_DATAPROCESS_H