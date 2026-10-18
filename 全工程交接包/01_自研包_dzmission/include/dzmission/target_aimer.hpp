#ifndef DZMISSION_TARGET_AIMER_HPP
#define DZMISSION_TARGET_AIMER_HPP

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace dzmission
{

// =============================================================================
// TargetAimer —— 云台识别标靶 + 激光发射打击
// -----------------------------------------------------------------------------
// 与 dzrknn_dataprocess 的关系（重要）：
//   二者功能重叠，不能同时控制云台，否则指令相互打架。
//   本节点提供 own_gimbal 开关：
//     own_gimbal = true  —— 本节点发布 CameMo_Conl / LaserShot_Command，
//                           启动时不要运行 dzrknn_dataprocess（或把它的
//                           own_gimbal 关掉），launch 里由参数控制。
//     own_gimbal = false —— 本节点只做监视与联锁判断，不发云台/激光指令，
//                           云台仍由 dzrknn_dataprocess 驱动。
//
// 状态机：
//   IDLE      —— 云台归中，不扫描
//   SCANNING  —— 按 yaw 栅格 + pitch 层来回扫描找靶
//   TRACKING  —— 视觉有目标且在容差外，视觉伺服把目标拉到画面中心
//   ALIGNED   —— 连续 N 帧目标在容差内，满足开火条件
//   FIRING    —— 激光高电平保持 fire_duration_ms
//   COOLDOWN  —— 冷却期内不再开火
//
// 开火联锁（全部满足才开火）：
//   1. ammo > 0            （弹量来自裁判板）
//   2. 未被击打/未处于被击退状态
//   3. 目标在画面中心容差内且连续 fire_debounce_frames 帧
//   4. 距上次开火 > fire_cooldown_ms
//   5. 车体不需要处于运动禁止以外（可配 require_stationary）
// =============================================================================
class TargetAimer
{
public:
  enum class State
  {
    kIdle = 0,
    kScanning = 1,
    kTracking = 2,
    kAligned = 3,
    kFiring = 4,
    kCooldown = 5,
  };

  struct AimResult
  {
    State state{State::kIdle};
    int center_x{0};
    int center_y{0};
    int class_id{0};
    bool target_visible{false};
    bool fire_authorized{false};
    bool fired_this_tick{false};
    double last_fire_age_s{1e9};
    int shots_fired{0};
    int gimbal_yaw_tick{2047};
    int gimbal_pitch_tick{2047};
    double range_m{0.0};
    // ★ 新增观测项
    int face{0};              // 可见面 0=不确定 1=红(友) 2=蓝(敌)
    bool laser_dot_seen{false}; // 检测框内是否有激光红点
    bool target_destroyed{false};// 靶标已击毁（血量达上限）
    int local_shots{0};       // 本地累计发射数
    int ammo_effective{0};    // 本次判定实际采用的弹量（取小）
    bool link_degraded{false};// 裁判板链路断、正在用本地记账
    // ★ 移动靶
    bool turn_around_ok{false};   // 当前是否处于"换向点"（可开火窗口）
    double track_velocity_px_s{0.0}; // 靶标横向速度估计（像素/秒，带符号）
    // ★ 方案 β（提前量射击）观测项
    bool lead_active{false};        // 本周期是否走了 β 路径
    double lead_offset_px{0.0};     // 算出的提前量（像素，带符号）
    double lead_total_ms{0.0};      // T_total（毫秒）
    int lead_fire_count{0};         // β 路径开火次数
    int lead_fallback_count{0};     // 退回 α 的次数
  };

  TargetAimer(rclcpp::Node *node, const std::string &ns);
  ~TargetAimer() = default;

  TargetAimer(const TargetAimer &) = delete;
  TargetAimer &operator=(const TargetAimer &) = delete;

  void loadParams();

  // ---- 交战请求（任务层） ----
  // target_id: 0 关闭交战；>0 进入交战并打击该类目标
  void requestEngage(uint8_t target_id);
  void ceaseFire();
  void abort();

  bool engaging() const { return engaging_.load(); }
  uint8_t engageTargetId() const { return engage_target_id_.load(); }

  // ---- 外部联锁输入 ----
  // 这三个 setter 会被与 ROS 执行器不同的线程上下文调用（CmdLink/RefereeBoard
  // 各自有工作线程，若将来换成 MultiThreadedExecutor 就会真正并发），
  // 统一用原子量，避免与 tick()/result() 的数据竞争。
  void setAmmo(int ammo) { ammo_.store(ammo); }
  void setBeingHit(bool being_hit) { being_hit_.store(being_hit); }
  void setChassisStationary(bool stationary) { chassis_stationary_.store(stationary); }
  // ★ 靶标血量（裁判板反馈）。>= 3 表示该侧已击毁（正反共用同一靶标实体）。
  void setTargetHp(int hp) { target_hp_.store(hp); }
  // ★ 裁判板累计发射数（用于"链路断后是否真有反馈"的判据）
  void setBoardShotCount(int c) { board_shot_count_.store(c); }
  // ★ 裁判板链路是否健康（决定用裁判板弹量还是本地记账）
  void setRefereeLinkOk(bool ok) { referee_link_ok_.store(ok); }

  // ---- 周期性调用（建议 50Hz） ----
  void tick();

  AimResult result() const;

  // 云台是否由本节点驱动
  bool ownsGimbal() const { return own_gimbal_; }

private:
  void visionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void feedbackCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

  // 扫描
  double scanYawDeg() const;
  double scanPitchDeg() const;
  int pitchLevelCount() const;
  void applyScanTarget();
  void advanceScanIndex();
  void centerGimbal();
  // ★ 连续慢扫模式：维护一个"当前扫描角"，逐 tick 平滑推进
  void sweepScanTarget(double now_s);
  // 由前一个方向/层推出下一个扫描角（跳档模式用）
  void computeNextScanAngle(double &yaw_deg, double &pitch_deg);

  // 追踪
  int computeTrackingSpeed(int target, int current, bool is_pitch) const;
  int clampTick(int tick) const;

  void publishGimbal();
  void publishLaser(bool on);

  rclcpp::Node *node_{nullptr};

  // ---- 参数 ----
  bool own_gimbal_{false};
  std::string vision_topic_{"offset_center"};
  std::string gimbal_feedback_topic_{"camemo_feedback"};
  std::string gimbal_cmd_topic_{"CameMo_Conl"};
  std::string laser_topic_{"LaserShot_Command"};

  int gimbal_center_tick_{2047};
  int gimbal_tick_min_{1000};
  int gimbal_tick_max_{3000};
  int gimbal_max_speed_{3700};
  double yaw_ticks_per_deg_{11.111};
  double pitch_ticks_per_deg_{11.111};
  double yaw_deg_sign_{1.0};
  double pitch_deg_sign_{1.0};

  // 扫描
  int scan_yaw_dirs_{8};
  double scan_yaw_min_deg_{-90.0};
  double scan_yaw_max_deg_{90.0};
  std::vector<double> scan_pitch_levels_deg_{30.0, 0.0, -10.0};
  int scan_dwell_ms_{300};
  int scan_arrive_tol_{30};
  int scan_speed_{400};
  int pitch_scan_speed_{100};

  // ---- ★ 扫描模式 ----
  //   "step"  = 跳档式：跳到下一个预置角度，停 dwell 等视觉（原行为，扫得快）
  //   "sweep" = 连续慢扫：以恒速连续扫，端点停顿（学员两版的共同做法）
  //             依据：学员注释"sweep outran the detection latency" ——
  //             扫描速度必须与视觉流水线延迟匹配，否则云台扫过目标时
  //             检测帧还没回来，永远扫不到。
  std::string scan_mode_{"step"};
  // sweep 模式的角速度（度/秒）。上限建议：
  //   检测帧率 15fps -> 帧周期 66ms；流水线 4 帧 = 267ms
  //   要让目标在视野内至少留 2 帧，单帧转过的角度应 < FOV/2 / 2
  //   假设水平 FOV 65° -> 单帧 < 16° -> 角速度 < 240°/s
  //   学员实测可用值约 2.9°/s（很慢），本实现默认 12°/s 作为折中
  double scan_sweep_rate_dps_{12.0};
  // sweep 模式在换向点的停顿时间（让视觉追上）
  int scan_hold_ms_{200};
  // sweep 模式是否也遍历 pitch 层
  bool scan_sweep_multi_pitch_{true};

  // ---- ★ 丢帧宽限：目标刚消失时"保持不动"而不是立刻重新扫描 ----
  //   依据（学员两版都有，注释解释了原因）：
  //     "Grace period: if the target was seen recently, HOLD position and
  //      wait for the (slow) image pipeline instead of resuming the sweep.
  //      This is why the gimbal used to scan forever past a nearby target."
  //   量级推导：流水线 4 帧 × 66ms(15fps) = 267ms，留一倍余量 -> 600ms
  int lost_grace_ms_{600};

  // ---- ★ 开火保持窗口（与"对准容差"分开）----
  //   对准容差 fire_tolerance_px_ 决定"何时进入开火态"，
  //   保持窗口 fire_hold_px_ 决定"开火态下漂到多远仍继续开火"。
  //   分开的理由：只有一个阈值时，目标在阈值边缘抖动会导致
  //   "开火↔停火"反复切换，而全局限 10 发弹量，浪费不起。
  //   学员取值：对准 5px / 保持 15px。
  int fire_hold_px_{15};

  // 视觉伺服
  double vision_smooth_alpha_{0.4};
  double fast_alpha_{0.85};
  int fast_error_threshold_px_{40};
  double vision_step_gain_{0.3};
  double fast_step_gain_{0.7};
  int max_step_per_frame_{20};
  int fast_max_step_{80};
  int vision_deadband_px_{3};
  int pitch_deadband_px_{8};
  double pitch_step_gain_{0.1};
  int pitch_max_step_per_frame_{3};
  int vision_timeout_ms_{500};
  int lost_frames_to_rescan_{20};

  // 电机
  double motor_kp_{1000.0};
  double fast_motor_kp_{50.0};
  double pitch_motor_kp_{5.0};
  int pitch_max_speed_{200};
  int position_deadband_ticks_{5};
  int pitch_position_deadband_ticks_{15};

  // 开火
  int fire_tolerance_px_{10};
  int fire_debounce_frames_{3};
  // ★ 开火冷却。默认 3000ms 而非 500ms —— 依据规则：
  //   "命中后靶标 1 秒后自动恢复；任一侧累计 3 次则整体击毁"。
  //   500ms 会在靶标尚未恢复时打出无效命中，白白消耗 10 发弹量。
  int fire_cooldown_ms_{3000};
  int fire_duration_ms_{100};
  bool require_stationary_{false};
  int min_ammo_to_fire_{1};
  // 超过该距离不打击（射程/裁判规则限制）。
  // 距离由视觉侧给出：目前 rknn_pt 的 /offset_center 只发 [cx, cy, class_id]，
  // 若要把该限制真正生效，需要让视觉节点输出第 4 位距离（米*100），
  // 见 doc/INTEGRATION.md「射程联锁」一节。
  double max_engage_range_m_{8.0};
  double last_target_range_m_{0.0};
  bool enable_auto_scan_{true};

  // ===========================================================================
  // ★★ 红蓝面联锁（本赛题的关键安全约束）★★
  // ---------------------------------------------------------------------------
  // 规则：靶标是**双面结构**，一面红（=友方）、一面蓝（=敌方），朝向随机。
  //       打中友方（红面）要**扣分**。
  // 车端相机只能看到**朝向自己的那一面**，所以：
  //   · 可见面 == 蓝(2)  -> 允许开火
  //   · 可见面 == 红(1)  -> **禁止开火**（那是友方）
  //   · 可见面 == 不确定 -> 由 face_unknown_policy 决定
  //
  // 为什么"不确定"默认不开火：
  //   误打友方的代价是扣分，而漏打只是少拿分。规则里
  //   "友方受击计数每 +1 扣 5 分"，比命中的收益（每 2 次有效命中 +5）更贵。
  //
  // 现场必须标定的量：det_node 的 HSV 阈值（face_min_ratio / face_red_bias）。
  // ===========================================================================
  // 可见面来源：0=不用该联锁（危险，仅台架自测）1=用（推荐）
  bool require_blue_face_{true};
  // "可见面不确定"时的策略：
  //   0 = 禁止开火（默认，最保守）
  //   1 = 允许开火（仅当无人机下发的 TARGET_LIST 明确 face==2 时）
  int face_unknown_policy_{0};
  // 连续多少帧看到同一面才采信（抗单帧误判）
  int face_debounce_frames_{2};

  // ★ 激光红点联锁（命中确认）
  //   true  = 必须在检测框内看到红点才允许开火（最可靠，因为直接确认落点）
  //   false = 不要求（台架/红点看不见时）
  //   ★ 注意：红点联锁与"是否首次开火"要区分 —— 首次开火前激光还没打出去，
  //     当然看不到红点。所以本联锁只在"已经打过至少一发、正在打同一目标"
  //     时作为**再次开火**的门槛（见 laser_dot_required_after_first_）。
  bool require_laser_dot_{true};
  bool laser_dot_required_after_first_{true};
  // 连续多少帧看到红点才采信
  int laser_dot_debounce_frames_{2};

  bool require_turn_around_{true};
  // 速度阈值（像素/秒）。靶标在画面中横向移动速度低于该值即视为"停住"。
  //   标定方法：让靶标匀速移动，看日志里的 |v|，取明显低于运动时的值。
  //   默认 25 px/s：对 640 宽画面约 0.04 画面宽/秒，足够区分"动"与"停"。
  double turn_around_v_thresh_px_s_{25.0};
  // 速度需持续低于阈值多久才采信（毫秒）。防噪声导致的单帧过零误判。
  int turn_around_dwell_ms_{120};
  // 速度估计的中值滤波窗口长度（帧数）
  //   ★ 用中值而非一阶低通：量化验证发现一阶低通（alpha=0.35）滞后约 600ms，
  //     使换向后的残余位移达 7.1cm > 靶标半径 4.5cm —— 等于没解决问题。
  //     中值对孤立噪声免疫，对单调趋势几乎无滞后。
  //   窗口越长越抗噪，但滞后越大。3 帧 ≈ 200ms（15fps），是折中值。
  int turn_around_vel_history_{3};
  // 【已废弃，保留兼容】一阶低通系数，现改用中值滤波
  double turn_around_velocity_alpha_{0.35};
  // ★ 方向反转检测（主判据）
  //   "幅值低于阈值"会被滤波器拖尾（实测滞后约 600ms，导致真换向识别不到），
  //   而"速度变号"是更稳健的特征 —— 它只需要趋势翻转，不受拖尾影响。
  // 换向时 |v| 允许达到硬阈值的多少倍（区分真换向与噪声抖动）
  double turn_around_v_soft_ratio_{3.0};
  // 换向发生后，开火窗口保持多久（毫秒）
  //   这段时间内靶标速度仍接近 0，延迟位移可忽略
  int turn_around_window_ms_{250};

  // ★ 移动靶：换向点检测（方案 α）。返回 true 表示"此刻适合开火"。
  bool evaluateTurnAround(const rclcpp::Time &now, int px, int py);

  // ===========================================================================
  // ★★ 方案 β：提前量射击（与 α 互补，不是替代）★★
  // ---------------------------------------------------------------------------
  // 详见 doc/MOVING_TARGET_PLAN.md 与 test/moving_target_beta_check.py。
  //
  // 核心：**激光与相机同轴**（Q14 确认，都在云台上）。
  //   所以命中只取决于**云台指向**；相机画面仅用于估计靶标运动。
  //   提前量 = 靶标像素速度(px/s) × T_total
  //
  // T_total 构成（漏算任何一段都会系统性打偏）：
  //     相机曝光 + 图像传输      33 ms
  //     det_node 流水线（4 帧）  267 ms
  //     云台转到瞄准点           60 ms   ← 只能实测标定
  //     开火指令 -> 出光         20 ms   ← 只能实测标定
  //     ------------------------------------
  //     合计                    380 ms
  //   ★ 只算流水线会少补偿 113ms ≈ 5.0cm（靶速 0.44m/s），
  //     而靶标直径只有 9cm —— 漏这段必然打偏。
  //
  // ★★ 正确形态是「预瞄固定点 + 穿越触发」，不是「跟踪 + 补偿」★★
  //   跟踪+补偿要求云台高带宽跟踪**移动的瞄准点**，稳态误差 = v×T_total。
  //   "预瞄固定点"让云台几乎不动，命中点由**开火时机**控制：
  //     1) 靶标沿轨道往复 -> 运动轴线固定
  //     2) 取画面中心对应的靶面点为**固定瞄准点**，云台预先转到并稳住
  //     3) 预测靶标到达该点的时刻 t_hit
  //     4) 在 t_hit - T_total 时刻开火
  //   对云台带宽要求极低，且云台不动 -> 无运动模糊，视觉估计更稳。
  //
  // ★ 射速真相（诚实说明）：
  //   冷却 3.0s 与半周期相当，所以 α 与 β 的**射速上限相同**（都被冷却卡住）。
  //   β 的真正价值：不依赖靶标恰好停在换向点（α 若靶标不停就打不了），
  //   开火窗口更连续。
  // ===========================================================================
  bool lead_enable_{true};
  // T_total 的四段（毫秒）。★ 后两段只能实测；不确定时先设 0
  double lead_exposure_ms_{33.0};
  double lead_servo_ms_{60.0};
  double lead_fire_ms_{20.0};
  // 流水线延迟由帧率与深度现算
  int lead_pipeline_frames_{4};
  double detect_rate_hz_{15.0};
  // 至少几帧有效观测才启用提前量；不足则退回 α
  int lead_min_confidence_{3};
  // ★ 兜底：连续多少发未命中就退回 α（弹量只有 10 发，必须设上限）
  int lead_max_misses_{2};
  // 云台"已指向瞄准点"的容差（像素）；不满足则不开火（等它稳）
  int lead_aim_tolerance_px_{8};
  // 只在靶标接近画面中心时信任速度估计（透视在边缘失真）
  int lead_trust_center_px_{160};

  // 提前量射击判定：返回 true 表示"此刻开火可命中"
  bool evaluateLeadFire(const rclcpp::Time &now, int px, int py);
  // T_total（毫秒）
  double leadTotalMs() const;

  // ★ 靶标击毁检测：血量达到该值即视为该侧已毁（正反共用，整体失效）
  int target_hp_destroyed_{3};

  // ===========================================================================
  // ★★ 弹量纪律 ★★
  // ---------------------------------------------------------------------------
  // 规则：弹量共 **10 发**。裁判板会报 ammo；但链路断时车端必须自己守纪律，
  //       否则可能超打（超打部分无效，且浪费赛时）。
  //
  // 策略（face_unknown_policy 之外的另一处策略点，用 ammo_link_lost_policy 选）：
  //   0 = 保守：链路断即禁止开火（原行为）
  //   1 = 乐观：链路断后按本地记账（10 - local_shots）
  //   2 = 折中（推荐）：链路断后允许再打 ammo_link_lost_allowance 发，
  //       且**每发都要看到裁判板计数或靶标血量变化**，否则停止。
  //       理由：规则没有禁止链路断时开火，但"有效命中"只能靠裁判板确认；
  //             打满 3 次（任一侧 3 次即毁）都没有任何反馈，
  //             说明要么没命中、要么链路真的坏了，继续打没有意义。
  // ===========================================================================
  int ammo_link_lost_policy_{2};
  int ammo_link_lost_allowance_{3};
  // 本地硬上限（不依赖裁判板，防止裁判板计数异常导致超打）
  int max_local_shots_{10};

  // ---- 状态 ----
  std::atomic<bool> engaging_{false};
  std::atomic<uint8_t> engage_target_id_{0};

  // ★ 新增外部联锁输入的原子量
  std::atomic<int> target_hp_{0};
  std::atomic<int> board_shot_count_{0};
  std::atomic<bool> referee_link_ok_{false};

  // ★ 新增状态（只在 tick() 内读写，需要与 result() 互斥）
  int face_{0};                       // 当前采信的面 0/1/2
  int face_candidate_{0};             // 去抖用的候选面
  int face_seen_count_{0};            // 候选面连续帧数
  bool laser_dot_seen_{false};
  int laser_dot_count_{0};
  int local_shots_{0};                // 本地累计发射数
  int board_shot_at_link_loss_{0};    // 链路断时的裁判板计数（基准）
  int target_hp_at_link_loss_{0};     // 链路断时的血量（基准）
  bool link_degraded_{false};         // 是否处于"链路断、用本地记账"
  bool link_loss_baseline_valid_{false};

  // ★ 移动靶换向点检测状态
  int last_track_px_{0};
  bool last_track_px_valid_{false};
  rclcpp::Time last_track_time_;
  double last_track_velocity_px_s_{0.0};
  bool turn_around_velocity_ready_{false};
  bool turn_around_armed_{false};
  rclcpp::Time turn_around_dwell_start_;
  // ★ 方向反转检测状态
  double turn_around_prev_v_{0.0};              // 上一帧估计速度（判变号用）
  bool turn_around_reversal_seen_{false};       // 当前是否处于换向开火窗口
  rclcpp::Time turn_around_reversal_time_;      // 最近一次检出的换向时刻
  std::vector<double> vel_hist_;                // 帧间速度环形缓冲（中值用）
  int vel_hist_idx_{0};

  // ★ 方案 β 状态
  int lead_miss_streak_{0};        // 连续未命中发数（用于退回 α）
  bool lead_active_{false};        // 本周期是否走了 β 路径
  double lead_offset_px_{0.0};     // 本周期算出的提前量（像素，带符号）
  int lead_fire_count_{0};         // β 路径下开火次数
  int lead_fallback_count_{0};     // 退回 α 的次数（可观测性）

  mutable std::mutex mtx_;
  State state_{State::kIdle};
  int center_x_{0};
  int center_y_{0};
  int class_id_{0};
  bool target_visible_{false};

  double vision_cx_filtered_{0.0};
  double vision_cy_filtered_{0.0};
  bool vision_smooth_ready_{false};

  int in_tolerance_count_{0};
  int lost_target_count_{0};
  int scan_yaw_idx_{0};
  int scan_yaw_dir_{1};
  int scan_pitch_idx_{0};

  // ---- ★ 新增状态 ----
  // 丢帧宽限的起点（目标"刚还在"的时间戳）。用于"保持不动"判定。
  rclcpp::Time last_target_seen_time_;
  bool ever_seen_target_{false};
  // 开火态下的漂移计数（超出保持窗口则暂停开火并微调）
  int out_of_hold_count_{0};
  // sweep 模式状态
  double sweep_yaw_deg_{0.0};
  double sweep_pitch_deg_{0.0};
  int sweep_dir_{1};
  rclcpp::Time sweep_hold_until_;
  rclcpp::Time last_sweep_tick_;
  bool sweep_initialized_{false};

  rclcpp::Time last_vision_time_;
  rclcpp::Time last_scan_step_time_;
  rclcpp::Time laser_start_time_;
  rclcpp::Time last_fire_time_;
  bool laser_active_{false};
  int shots_fired_{0};

  // ★ 外部联锁输入的原子量：这些 setter 会被与 ROS 执行器不同的线程调用
  //   （CmdLink / RefereeBoard 各有工作线程），必须用原子量避免与 tick() 竞争。
  //   ⚠ 曾漏写 ammo_ 的 std::atomic<>，导致 setAmmo 里的 .store() 编译失败：
  //      "request for member 'store' in 'ammo_', which is of non-class type 'int'"
  std::atomic<int> ammo_{0};
  std::atomic<bool> being_hit_{false};
  std::atomic<bool> chassis_stationary_{true};

  // 云台命令与反馈
  int cmd_pos_yaw_{2047};
  int cmd_pos_pitch_{2047};
  int cmd_spd_yaw_{0};
  int cmd_spd_pitch_{0};
  int fb_pos_yaw_{2047};
  int fb_pos_pitch_{2047};
  rclcpp::Time last_feedback_time_;
  bool has_feedback_{false};

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_vision_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_feedback_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_gimbal_cmd_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_laser_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_aim_state_;
};

}  // namespace dzmission

#endif  // DZMISSION_TARGET_AIMER_HPP
