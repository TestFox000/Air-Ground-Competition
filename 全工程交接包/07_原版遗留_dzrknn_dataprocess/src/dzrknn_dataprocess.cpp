#include "dzrknn_dataprocess.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<turn_on_robot>("dzrknn_dataprocess");
  node->Control();
  rclcpp::shutdown();
  return 0;
}

// =============================================================
// 构造函数
// =============================================================
turn_on_robot::turn_on_robot(const std::string &name) : Node(name)
{
  RCLCPP_INFO(this->get_logger(), "创建节点成功..........");

  // ---- 发布者 ----
  pub_LaserShot_Command = this->create_publisher<std_msgs::msg::UInt8>("LaserShot_Command", 10);
  pub_CameMo_Conl = this->create_publisher<std_msgs::msg::Int32MultiArray>("CameMo_Conl", 10);
  pub_voice_switch = this->create_publisher<std_msgs::msg::UInt8>("voice_switch", 10);

  // ---- 订阅者 ----
  sub_offset_center = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "offset_center", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&turn_on_robot::callback_offset_center, this, std::placeholders::_1));

  sub_stop_point_singal = this->create_subscription<std_msgs::msg::UInt8>(
      "/move_base/stop_signal", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&turn_on_robot::callback_stop_point_signal, this, std::placeholders::_1));

  sub_camemo_feedback = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "camemo_feedback", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&turn_on_robot::callback_camemo_feedback, this, std::placeholders::_1));

  sub_HpAndHitmsg = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "HpAndHit", rclcpp::QoS(10),
      std::bind(&turn_on_robot::callback_HpAndHitmsg, this, std::placeholders::_1));

  // ---- 参数声明 ----
  this->declare_parameter("motor_kp", 1000.0);
  this->declare_parameter("motor_ki", 0.0);
  this->declare_parameter("motor_kd", 0.0);
  this->declare_parameter("pose_kp", 1000.0);

  this->declare_parameter("scan_yaw_dirs", 8);
  this->declare_parameter("scan_yaw_min_deg", -90.0);
  this->declare_parameter("scan_yaw_max_deg", 90.0);
  this->declare_parameter("scan_pitch_levels", std::vector<double>{30.0, 0.0, -10.0});
  this->declare_parameter("yaw_deg_sign", 1.0);
  this->declare_parameter("pitch_deg_sign", 1.0);
  this->declare_parameter("gimbal_center_tick", 2047);
  this->declare_parameter("yaw_ticks_per_deg", 11.111);
  this->declare_parameter("pitch_ticks_per_deg", 11.111);
  this->declare_parameter("gimbal_tick_min", 1000);
  this->declare_parameter("gimbal_tick_max", 3000);
  this->declare_parameter("gimbal_max_speed", 3700);
  this->declare_parameter("scan_speed_ratio", 0.3);
  this->declare_parameter("scan_dwell_ms", 300);
  this->declare_parameter("scan_arrive_tol", 30);
  this->declare_parameter("lost_frames_to_rescan", 60);
  this->declare_parameter("vision_timeout_ms", 1000);
  this->declare_parameter("fire_tolerance_px", 10);
  this->declare_parameter("laser_duration_ms", 100);

  this->declare_parameter("vision_deadband_px", 3);
  this->declare_parameter("position_deadband_ticks", 5);
  this->declare_parameter("vision_smooth_alpha", 0.4);
  this->declare_parameter("vision_step_gain", 0.3);
  this->declare_parameter("max_step_per_frame", 20);
  this->declare_parameter("min_tracking_speed", 0);

  this->declare_parameter("pitch_motor_kp", 5.0);
  this->declare_parameter("pitch_max_speed", 200);
  this->declare_parameter("pitch_step_gain", 0.1);
  this->declare_parameter("pitch_deadband_px", 8);
  this->declare_parameter("pitch_position_deadband_ticks", 15);
  this->declare_parameter("pitch_max_step_per_frame", 3);
  this->declare_parameter("pitch_scan_speed", 100);

  // ===== 新增 =====
  this->declare_parameter("fire_debounce_frames", 3);
  this->declare_parameter("fire_cooldown_ms", 500);

  this->declare_parameter("fast_error_threshold_px", 40);
  this->declare_parameter("fast_alpha", 0.85);
  this->declare_parameter("fast_step_gain", 0.7);
  this->declare_parameter("fast_max_step", 80);
  this->declare_parameter("fast_motor_kp", 50.0);

  // ---- 参数读取 ----
  this->get_parameter("motor_kp", motor_kp_);
  this->get_parameter("motor_ki", motor_ki_);
  this->get_parameter("motor_kd", motor_kd_);
  this->get_parameter("pose_kp", pose_kp_);

  this->get_parameter("scan_yaw_dirs", scan_yaw_dirs_);
  this->get_parameter("scan_yaw_min_deg", scan_yaw_min_deg_);
  this->get_parameter("scan_yaw_max_deg", scan_yaw_max_deg_);
  this->get_parameter("scan_pitch_levels", scan_pitch_levels_deg_);
  this->get_parameter("yaw_deg_sign", yaw_deg_sign_);
  this->get_parameter("pitch_deg_sign", pitch_deg_sign_);
  this->get_parameter("gimbal_center_tick", gimbal_center_tick_);
  this->get_parameter("yaw_ticks_per_deg", yaw_ticks_per_deg_);
  this->get_parameter("pitch_ticks_per_deg", pitch_ticks_per_deg_);
  this->get_parameter("gimbal_tick_min", gimbal_tick_min_);
  this->get_parameter("gimbal_tick_max", gimbal_tick_max_);
  this->get_parameter("gimbal_max_speed", gimbal_max_speed_);
  this->get_parameter("scan_speed_ratio", scan_speed_ratio_);
  this->get_parameter("scan_dwell_ms", scan_dwell_ms_);
  this->get_parameter("scan_arrive_tol", scan_arrive_tol_);
  this->get_parameter("lost_frames_to_rescan", lost_frames_to_rescan_);
  this->get_parameter("vision_timeout_ms", vision_timeout_ms_);
  this->get_parameter("fire_tolerance_px", fire_tolerance_px_);
  this->get_parameter("laser_duration_ms", laser_duration_ms_);

  this->get_parameter("vision_deadband_px", vision_deadband_px_);
  this->get_parameter("position_deadband_ticks", position_deadband_ticks_);
  this->get_parameter("vision_smooth_alpha", vision_smooth_alpha_);
  this->get_parameter("vision_step_gain", vision_step_gain_);
  this->get_parameter("max_step_per_frame", max_step_per_frame_);
  this->get_parameter("min_tracking_speed", min_tracking_speed_);

  this->get_parameter("pitch_motor_kp", pitch_motor_kp_);
  this->get_parameter("pitch_max_speed", pitch_max_speed_);
  this->get_parameter("pitch_step_gain", pitch_step_gain_);
  this->get_parameter("pitch_deadband_px", pitch_deadband_px_);
  this->get_parameter("pitch_position_deadband_ticks", pitch_position_deadband_ticks_);
  this->get_parameter("pitch_max_step_per_frame", pitch_max_step_per_frame_);
  this->get_parameter("pitch_scan_speed", pitch_scan_speed_);
  // ===== 新增 =====
  this->get_parameter("fire_debounce_frames", fire_debounce_frames_);
  this->get_parameter("fire_cooldown_ms", fire_cooldown_ms_);

  this->get_parameter("fast_error_threshold_px", fast_error_threshold_px_);
  this->get_parameter("fast_alpha", fast_alpha_);
  this->get_parameter("fast_step_gain", fast_step_gain_);
  this->get_parameter("fast_max_step", fast_max_step_);
  this->get_parameter("fast_motor_kp", fast_motor_kp_);
  // 参数合法性校验
  if (vision_smooth_alpha_ < 0.05)
    vision_smooth_alpha_ = 0.05;
  if (vision_smooth_alpha_ > 1.0)
    vision_smooth_alpha_ = 1.0;
  if (vision_deadband_px_ < 0)
    vision_deadband_px_ = 0;
  if (position_deadband_ticks_ < 0)
    position_deadband_ticks_ = 0;
  if (max_step_per_frame_ < 1)
    max_step_per_frame_ = 1;
  if (pitch_motor_kp_ < 0.0)
    pitch_motor_kp_ = 0.0;
  if (pitch_max_speed_ < 1)
    pitch_max_speed_ = 1;
  if (pitch_step_gain_ < 0.0)
    pitch_step_gain_ = 0.0;
  if (pitch_deadband_px_ < 0)
    pitch_deadband_px_ = 0;
  if (pitch_position_deadband_ticks_ < 0)
    pitch_position_deadband_ticks_ = 0;
  if (pitch_max_step_per_frame_ < 0)
    pitch_max_step_per_frame_ = 0;
  if (pitch_scan_speed_ < 0)
    pitch_scan_speed_ = 0;
  if (pitch_max_speed_ > gimbal_max_speed_)
    pitch_max_speed_ = gimbal_max_speed_;

  if (fast_error_threshold_px_ < 0)
    fast_error_threshold_px_ = 0;
  if (fast_alpha_ < 0.05 || fast_alpha_ > 1.0)
    fast_alpha_ = 0.85;
  if (fast_step_gain_ < 0.0)
    fast_step_gain_ = 0.0;
  if (fast_max_step_ < 1)
    fast_max_step_ = 1;
  if (fast_motor_kp_ < 0.0)
    fast_motor_kp_ = 0.0;

  // ---- 状态初始化 ----
  moveBaseControl.Position_0 = gimbal_center_tick_;
  moveBaseControl.Position_1 = gimbal_center_tick_;
  moveBaseControl.Speed_0 = 0;
  moveBaseControl.Speed_1 = 0;

  curYuntai_feedback_data.Position_0 = gimbal_center_tick_;
  curYuntai_feedback_data.Position_1 = gimbal_center_tick_;
  curYuntai_feedback_data.Speed_0 = 0;
  curYuntai_feedback_data.Speed_1 = 0;
  curYuntai_feedback_data.Load_0 = 0;
  curYuntai_feedback_data.Load_1 = 0;

  find_center = false;
  lost_target_count_ = 0;
  scan_yaw_idx_ = 0;
  scan_yaw_dir_ = 1;
  scan_pitch_idx_ = 0;
  laser_active_ = false;
  last_voice_msg.data = 0;

  last_vision_time_ = this->now();
  last_scan_step_time_ = this->now();
  laser_start_time_ = this->now();
  last_fire_time_ = this->now(); // 新增
  in_tolerance_count_ = 0;       // 新增
  // ---- 加载查表 ----
  const std::string savePath_Table =
      "/home/duzhong/dzrobot/src/dzrknn_dataprocess/offcenter_table.txt";
  if (!loadLookupTable(savePath_Table))
  {
    RCLCPP_ERROR(this->get_logger(), "loadLookupTable failed: %s", savePath_Table.c_str());
  }

  RCLCPP_INFO(this->get_logger(),
              "瞄准扫描: %d 个水平点 x %d 个竖直层, 转速比例 %.2f",
              scan_yaw_dirs_, pitchLevelCount(), scan_speed_ratio_);

  std::cout << FRED("Copyright©2016-2020 dzrknn_dataprocess. All rights reserved ") << std::endl;
  std::cout << FYEL("*****dzrknn_dataprocess:parameters*******************") << std::endl;
  std::cout << FYEL("*****dzrknn_dataprocess:parameters end***************") << std::endl;
}

turn_on_robot::~turn_on_robot() = default;

// =============================================================
// 主循环
// =============================================================
void turn_on_robot::Control()
{
  rclcpp::Rate rate(50);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(shared_from_this());

    CaremaMontorControl();

    // ---- 激光状态机：维持 laser_duration_ms_ 毫秒的高电平，避免 1/0 瞬时切换 ----
    std_msgs::msg::UInt8 shot;
    if (laser_active_)
    {
      const double active_ms = (this->now() - laser_start_time_).seconds() * 1000.0;
      if (active_ms < static_cast<double>(laser_duration_ms_))
      {
        shot.data = 1;
      }
      else
      {
        shot.data = 0;
        laser_active_ = false;
      }
    }
    else
    {
      shot.data = 0;
    }
    pub_LaserShot_Command->publish(shot);

    // ---- 云台控制指令 ----
    std_msgs::msg::Int32MultiArray camemo_cmd;
    camemo_cmd.data.resize(4);
    camemo_cmd.data[0] = moveBaseControl.Position_0;
    camemo_cmd.data[1] = moveBaseControl.Position_1;
    camemo_cmd.data[2] = moveBaseControl.Speed_0;
    camemo_cmd.data[3] = moveBaseControl.Speed_1;
    pub_CameMo_Conl->publish(camemo_cmd);

    rate.sleep();
  }
}

// =============================================================
// 视觉回调
// =============================================================
void turn_on_robot::callback_offset_center(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "offset_center data size < 4, ignore");
    return;
  }

  // 视觉心跳
  last_vision_time_ = this->now();

  const int center_x = msg->data[0];
  const int center_y = msg->data[1];
  const int class_id = msg->data[2];
  // const int target_dist = msg->data[3];
  /*
    if (class_id == 1)
    {
      // // ---- 检测到靶子：进入追踪 ----
      // find_center = true;
      // lost_target_count_ = 0;

      // // 增量式视觉伺服：以当前反馈位置为基准，把目标往画面中心拉
      // moveBaseControl.Position_0 = clampTick(
      //     curYuntai_feedback_data.Position_0 + static_cast<int>(std::lround(center_y * 0.5)));
      // moveBaseControl.Position_1 = clampTick(
      //     curYuntai_feedback_data.Position_1 + static_cast<int>(std::lround(center_x * 0.5)));

      // moveBaseControl.Speed_0 = static_cast<int>(std::max(
      //     100.0, CaremaSpeedControl(moveBaseControl.Position_0, curYuntai_feedback_data.Position_0)));
      // moveBaseControl.Speed_1 = static_cast<int>(std::max(
      //     100.0, CaremaSpeedControl(moveBaseControl.Position_1, curYuntai_feedback_data.Position_1)));

      // // ---- 目标进入容差范围：触发激光 ----
      // if (std::abs(center_x) < fire_tolerance_px_ && std::abs(center_y) < fire_tolerance_px_)
      // {
      //   laser_active_ = true;
      //   laser_start_time_ = this->now();
      // }
      // ---- 开火判定：防抖 + 冷却 ----
      const bool in_tolerance =
          std::abs(center_x) <= fire_tolerance_px_ &&
          std::abs(center_y) <= fire_tolerance_px_;

      if (in_tolerance)
      {
        ++in_tolerance_count_;
      }
      else
      {
        // 目标偏离容差，防抖计数清零
        in_tolerance_count_ = 0;
      }

      // 三重条件：连续帧够 + 冷却结束 + 当前不在开火
      if (!laser_active_ && in_tolerance_count_ >= fire_debounce_frames_)
      {
        const double since_last_fire_ms =
            (this->now() - last_fire_time_).seconds() * 1000.0;

        if (since_last_fire_ms >= static_cast<double>(fire_cooldown_ms_))
        {
          laser_active_ = true;
          laser_start_time_ = this->now();
          last_fire_time_ = this->now();

          // 开火后重置防抖计数：冷却期内不会因"还瞄着"立即再开火
          in_tolerance_count_ = 0;

          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                               "FIRE! cx=%d cy=%d (debounce=%d, cooldown=%dms)",
                               center_x, center_y, fire_debounce_frames_, fire_cooldown_ms_);
        }
      }
    }
    else if (class_id > 1)
    {
      // ---- 物资点：播报语音，停止追踪 ----
      find_center = false;
      laser_active_ = false;
      in_tolerance_count_ = 0; // 新增

      if (last_voice_msg.data != static_cast<uint8_t>(class_id))
      {
        std_msgs::msg::UInt8 voice_msg;
        voice_msg.data = static_cast<uint8_t>(class_id);
        pub_voice_switch->publish(voice_msg);
        last_voice_msg.data = voice_msg.data;
        RCLCPP_INFO(this->get_logger(), "voice_msg.data = %d", static_cast<int>(voice_msg.data));
      }
    }
    else
    {
      // ---- 未检测到目标 ----
      // 丢帧：重置防抖
      in_tolerance_count_ = 0; // 新增
      if (find_center)
      {
        // 追踪中丢帧：累计到阈值才回扫描，避免单帧误检打断扫描
        lost_target_count_++;
        if (lost_target_count_ >= lost_frames_to_rescan_)
        {
          find_center = false;
          lost_target_count_ = 0;
          last_scan_step_time_ = this->now();
        }
      }

      laser_active_ = false;

      // 未检测到目标的语音只播一次
      if (last_voice_msg.data != 111)
      {
        std_msgs::msg::UInt8 voice_msg;
        voice_msg.data = 111;
        pub_voice_switch->publish(voice_msg);
        last_voice_msg.data = 111;
      }
    }
  }
  */
  if (class_id == 1)
  {
    // ---- 检测到靶子：进入追踪 ----
    find_center = true;
    lost_target_count_ = 0;

    // ===== 1. 判断当前处于哪种模式 =====
    const int abs_cx_raw = std::abs(center_x);
    const int abs_cy_raw = std::abs(center_y);
    const bool fast_mode =
        (abs_cx_raw > fast_error_threshold_px_) ||
        (abs_cy_raw > fast_error_threshold_px_);
    // 根据模式选择参数
    const double alpha = fast_mode ? fast_alpha_ : vision_smooth_alpha_;
    const double step_gain = fast_mode ? fast_step_gain_ : vision_step_gain_;
    const int max_step = fast_mode ? fast_max_step_ : max_step_per_frame_;

    // ===== 2. 视觉误差 EMA 平滑（用当前模式的 alpha） =====
    if (!vision_smooth_ready_)
    {
      vision_cx_filtered_ = static_cast<double>(center_x);
      vision_cy_filtered_ = static_cast<double>(center_y);
      vision_smooth_ready_ = true;
    }
    else
    {
      vision_cx_filtered_ = alpha * static_cast<double>(center_x) + (1.0 - alpha) * vision_cx_filtered_;
      vision_cy_filtered_ = alpha * static_cast<double>(center_y) + (1.0 - alpha) * vision_cy_filtered_;
    }

    // // ===== 1. 视觉误差 EMA 平滑 =====
    // if (!vision_smooth_ready_)
    // {
    //   vision_cx_filtered_ = static_cast<double>(center_x);
    //   vision_cy_filtered_ = static_cast<double>(center_y);
    //   vision_smooth_ready_ = true;
    // }
    // else
    // {
    //   vision_cx_filtered_ = vision_smooth_alpha_ * static_cast<double>(center_x) + (1.0 - vision_smooth_alpha_) * vision_cx_filtered_;
    //   vision_cy_filtered_ = vision_smooth_alpha_ * static_cast<double>(center_y) + (1.0 - vision_smooth_alpha_) * vision_cy_filtered_;
    // }
    /*
        // ===== 2. 视觉死区 =====
        double ex = vision_cx_filtered_;
        double ey = vision_cy_filtered_;
        if (std::abs(ex) < static_cast<double>(vision_deadband_px_))
          ex = 0.0;
        if (std::abs(ey) < static_cast<double>(vision_deadband_px_))
          ey = 0.0;

        // ===== 3. 计算步进并限幅 =====
        int step_pitch = static_cast<int>(std::lround(ey * vision_step_gain_));
        int step_yaw = static_cast<int>(std::lround(ex * vision_step_gain_));
        step_pitch = std::clamp(step_pitch, -max_step_per_frame_, max_step_per_frame_);
        step_yaw = std::clamp(step_yaw, -max_step_per_frame_, max_step_per_frame_);

        const int target_p0 = clampTick(curYuntai_feedback_data.Position_0 + step_pitch);
        const int target_p1 = clampTick(curYuntai_feedback_data.Position_1 + step_yaw);

        moveBaseControl.Position_0 = target_p0;
        moveBaseControl.Position_1 = target_p1;

        // ===== 4. 速度：带死区的比例控制（不再强制 100 最小速度）=====
        moveBaseControl.Speed_0 = computeTrackingSpeed(target_p0, curYuntai_feedback_data.Position_0);
        moveBaseControl.Speed_1 = computeTrackingSpeed(target_p1, curYuntai_feedback_data.Position_1);
    */
    // ===== 3. 视觉死区（俯仰用大死区，水平用小死区） =====
    double ex = vision_cx_filtered_;
    double ey = vision_cy_filtered_;
    if (std::abs(ex) < static_cast<double>(vision_deadband_px_))
      ex = 0.0;
    if (std::abs(ey) < static_cast<double>(pitch_deadband_px_))
      ey = 0.0;

    // ===== 4. 步进计算（水平 / 俯仰分开） =====
    int step_yaw = static_cast<int>(std::lround(ex * step_gain));
    int step_pitch = static_cast<int>(std::lround(ey * pitch_step_gain_));
    step_yaw = std::clamp(step_yaw, -max_step, max_step);
    step_pitch = std::clamp(step_pitch, -pitch_max_step_per_frame_, pitch_max_step_per_frame_);

    const int target_p0 = clampTick(curYuntai_feedback_data.Position_0 + step_pitch);
    const int target_p1 = clampTick(curYuntai_feedback_data.Position_1 + step_yaw);

    moveBaseControl.Position_0 = target_p0;
    moveBaseControl.Position_1 = target_p1;

    // ===== 5. 速度：水平用模式化 kp，俯仰保持低增益 =====
    const double kp_yaw = fast_mode ? fast_motor_kp_ : motor_kp_;
    // 用现有函数也行，但这里直接算更快
    moveBaseControl.Speed_1 = static_cast<int>(std::min(
        std::abs(kp_yaw * static_cast<double>(target_p1 - curYuntai_feedback_data.Position_1)),
        static_cast<double>(gimbal_max_speed_)));

    moveBaseControl.Speed_0 = computeTrackingSpeed(target_p0, curYuntai_feedback_data.Position_0, true);

    // ===== 4. 速度：两轴分别计算 =====
    moveBaseControl.Speed_0 = computeTrackingSpeed(target_p0, curYuntai_feedback_data.Position_0, true);  // 俯仰
    moveBaseControl.Speed_1 = computeTrackingSpeed(target_p1, curYuntai_feedback_data.Position_1, false); // 水平
                                                                                                          // ===== 6. 开火判定（不变） =====
    const bool in_tolerance =
        std::abs(center_x) < fire_tolerance_px_ &&
        std::abs(center_y) < fire_tolerance_px_;

    if (in_tolerance)
      ++in_tolerance_count_;
    else
      in_tolerance_count_ = 0;

    if (!laser_active_ && in_tolerance_count_ >= fire_debounce_frames_)
    {
      const double since_last_fire_ms =
          (this->now() - last_fire_time_).seconds() * 1000.0;
      if (since_last_fire_ms >= static_cast<double>(fire_cooldown_ms_))
      {
        laser_active_ = true;
        laser_start_time_ = this->now();
        last_fire_time_ = this->now();
        in_tolerance_count_ = 0;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                             "FIRE! cx=%d cy=%d fast=%d",
                             center_x, center_y, static_cast<int>(fast_mode));
      }
    }
  }
  else if (class_id > 1)
  {
    // ---- 物资点：播报语音，停止追踪 ----
    find_center = false;
    laser_active_ = false;
    in_tolerance_count_ = 0;  // 新增
    resetTrackingSmoothing(); // 新增
    if (last_voice_msg.data != static_cast<uint8_t>(class_id))
    {
      std_msgs::msg::UInt8 voice_msg;
      voice_msg.data = static_cast<uint8_t>(class_id);
      pub_voice_switch->publish(voice_msg);
      last_voice_msg.data = voice_msg.data;
      RCLCPP_INFO(this->get_logger(), "voice_msg.data = %d", static_cast<int>(voice_msg.data));
    }
  }
  else
  {
    // ---- 未检测到目标 ----
    // 丢帧：重置防抖
    in_tolerance_count_ = 0;  // 新增
    resetTrackingSmoothing(); // 新增

    if (find_center)
    {
      // 追踪中丢帧：累计到阈值才回扫描，避免单帧误检打断扫描
      lost_target_count_++;
      if (lost_target_count_ >= lost_frames_to_rescan_)
      {
        find_center = false;
        lost_target_count_ = 0;
        last_scan_step_time_ = this->now();
      }
    }

    laser_active_ = false;

    // 未检测到目标的语音只播一次
    if (last_voice_msg.data != 111)
    {
      std_msgs::msg::UInt8 voice_msg;
      voice_msg.data = 111;
      pub_voice_switch->publish(voice_msg);
      last_voice_msg.data = 111;
    }
  }
}
// =============================================================
// 云台扫描控制
// =============================================================
void turn_on_robot::CaremaMontorControl()
{
  // 追踪中：位置由 callback_offset_center 视觉伺服修正，这里只管超时保护
  if (find_center)
  {
    const double silent_ms = (this->now() - last_vision_time_).seconds() * 1000.0;
    if (silent_ms > static_cast<double>(vision_timeout_ms_))
    {
      RCLCPP_WARN(this->get_logger(), "vision timeout, back to scan");
      find_center = false;
      lost_target_count_ = 0;
      laser_active_ = false;
      in_tolerance_count_ = 0;  // 新增
      resetTrackingSmoothing(); // 新增
      last_scan_step_time_ = this->now();
    }
    return;
  }

  // 扫描模式
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
    }
  }

  applyScanTarget();
}

// =============================================================
// 速度控制（当前只用 P，避免静态变量跨轴共享）
// =============================================================
double turn_on_robot::CaremaSpeedControl(int target_pose, int current_pose)
{
  const double kp = motor_kp_;
  const double max_speed = static_cast<double>(gimbal_max_speed_);
  const double error = static_cast<double>(target_pose - current_pose);

  double speed = std::abs(kp * error);
  return std::clamp(speed, 0.0, max_speed);
}

// =============================================================
// 追踪速度计算：带死区的比例控制（无强制最小速度）
// =============================================================
// =============================================================
// 追踪速度计算：带死区的比例控制，支持水平/俯仰双轴
// =============================================================
int turn_on_robot::computeTrackingSpeed(int target, int current, bool is_pitch) const
{
  const int err = target - current;
  const int abs_err = std::abs(err);

  const int deadband = is_pitch ? pitch_position_deadband_ticks_
                                : position_deadband_ticks_;
  const double kp = is_pitch ? pitch_motor_kp_ : motor_kp_;
  const int max_speed = is_pitch ? pitch_max_speed_ : gimbal_max_speed_;

  // 位置死区：太近就不动，避免极限环
  if (abs_err <= deadband)
  {
    return 0;
  }

  // 比例控制
  double speed = std::abs(kp * static_cast<double>(err));
  speed = std::min(speed, static_cast<double>(max_speed));

  // 只有水平轴才施加最小速度（俯仰轴不要强制最小值，避免抖）
  if (!is_pitch && min_tracking_speed_ > 0 && abs_err > deadband * 4)
  {
    speed = std::max(speed, static_cast<double>(min_tracking_speed_));
  }

  return static_cast<int>(std::lround(speed));
}
void turn_on_robot::resetTrackingSmoothing()
{
  vision_smooth_ready_ = false;
  vision_cx_filtered_ = 0.0;
  vision_cy_filtered_ = 0.0;
}

// =============================================================
// 扫描辅助函数
// =============================================================
int turn_on_robot::clampTick(int tick) const
{
  if (tick < gimbal_tick_min_)
    return gimbal_tick_min_;
  if (tick > gimbal_tick_max_)
    return gimbal_tick_max_;
  return tick;
}

int turn_on_robot::pitchLevelCount() const
{
  return scan_pitch_levels_deg_.empty() ? 1 : static_cast<int>(scan_pitch_levels_deg_.size());
}

double turn_on_robot::scanYawDeg() const
{
  if (scan_yaw_dirs_ <= 1)
  {
    return scan_yaw_min_deg_;
  }
  const double step = (scan_yaw_max_deg_ - scan_yaw_min_deg_) / (scan_yaw_dirs_ - 1);
  return scan_yaw_min_deg_ + step * scan_yaw_idx_;
}

double turn_on_robot::scanPitchDeg() const
{
  if (scan_pitch_levels_deg_.empty())
  {
    return 0.0;
  }
  const int idx = std::min(scan_pitch_idx_, pitchLevelCount() - 1);
  return scan_pitch_levels_deg_[idx];
}

void turn_on_robot::applyScanTarget()
{
  const int yaw_tick = gimbal_center_tick_ +
                       static_cast<int>(std::lround(scanYawDeg() * yaw_ticks_per_deg_ * yaw_deg_sign_));
  const int pitch_tick = gimbal_center_tick_ +
                         static_cast<int>(std::lround(scanPitchDeg() * pitch_ticks_per_deg_ * pitch_deg_sign_));

  moveBaseControl.Position_1 = clampTick(yaw_tick);
  moveBaseControl.Position_0 = clampTick(pitch_tick);

  const int scan_speed = static_cast<int>(gimbal_max_speed_ * scan_speed_ratio_);
  moveBaseControl.Speed_0 = scan_speed;
  moveBaseControl.Speed_1 = scan_speed;

  // 水平：正常扫描速度；俯仰：低速（甚至几乎不动）
  moveBaseControl.Speed_1 = static_cast<int>(gimbal_max_speed_ * scan_speed_ratio_);
  moveBaseControl.Speed_0 = pitch_scan_speed_;
}

void turn_on_robot::advanceScanIndex()
{
  const int next_yaw = scan_yaw_idx_ + scan_yaw_dir_;
  if (next_yaw >= 0 && next_yaw < scan_yaw_dirs_)
  {
    scan_yaw_idx_ = next_yaw;
    return;
  }

  // 本层扫完：水平反向 + 竖直切下一层
  scan_yaw_dir_ = -scan_yaw_dir_;
  scan_pitch_idx_ = (scan_pitch_idx_ + 1) % pitchLevelCount();
  scan_yaw_idx_ = (scan_yaw_dir_ > 0) ? 0 : (scan_yaw_dirs_ - 1);
}

// =============================================================
// 其它回调
// =============================================================
void turn_on_robot::callback_stop_point_signal(const std_msgs::msg::UInt8::SharedPtr msg)
{
  stop_point_signal_msg_ = static_cast<int>(msg->data);
  RCLCPP_INFO(this->get_logger(), "stop_msgs = %d", stop_point_signal_msg_);
}

void turn_on_robot::callback_camemo_feedback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 6)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "camemo_feedback data size < 6, ignore");
    return;
  }

  curYuntai_feedback_data.Position_0 = msg->data[0];
  curYuntai_feedback_data.Position_1 = msg->data[1];
  curYuntai_feedback_data.Speed_0 = msg->data[2];
  curYuntai_feedback_data.Speed_1 = msg->data[3];
  curYuntai_feedback_data.Load_0 = msg->data[4];
  curYuntai_feedback_data.Load_1 = msg->data[5];
}

void turn_on_robot::callback_HpAndHitmsg(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 3)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "HpAndHit data size < 3, ignore");
    return;
  }

  if (msg->data[2] > 0 && msg->data[2] < 5)
  {
    being_shot_data_ = msg->data[2];
    being_shot_ = true;
  }
}

// =============================================================
// 查表
// =============================================================
int turn_on_robot::compute_comp_pixel_0(double tar_dis)
{
  if (lookup_table.empty())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "lookup_table is empty, return 0");
    return 0;
  }

  tar_dis = std::clamp(tar_dis, 0.0, 5.0);
  int index = static_cast<int>(tar_dis / 0.25);
  index = std::clamp(index, 0, static_cast<int>(lookup_table.size()) - 1);
  return lookup_table[index];
}

bool turn_on_robot::loadLookupTable(const std::string &filePath)
{
  std::ifstream file(filePath);
  if (!file.is_open())
  {
    std::cerr << "无法打开文件: " << filePath << std::endl;
    return false;
  }

  lookup_table.clear();
  std::string line;
  int lineNum = 0;

  while (std::getline(file, line))
  {
    ++lineNum;
    // 处理 Windows 换行
    if (!line.empty() && line.back() == '\r')
    {
      line.pop_back();
    }
    if (line.empty())
      continue;

    try
    {
      size_t pos = 0;
      int value = std::stoi(line, &pos);
      if (pos != line.length())
      {
        throw std::invalid_argument("行中包含非数字字符");
      }
      lookup_table.push_back(value);
    }
    catch (const std::invalid_argument &)
    {
      std::cerr << "第" << lineNum << "行格式错误: " << line << std::endl;
      return false;
    }
    catch (const std::out_of_range &)
    {
      std::cerr << "第" << lineNum << "行数值超出范围: " << line << std::endl;
      return false;
    }
  }

  if (lookup_table.empty())
  {
    std::cerr << "查表数据为空: " << filePath << std::endl;
    return false;
  }

  return true;
}