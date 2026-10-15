#include "rclcpp/rclcpp.hpp"
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <iostream>
#include <cmath>
#include <deque>
#include <vector>
#include <tf2_ros/transform_broadcaster.h>
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <fstream>
using namespace std;

// 宏定义

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
	double TargetAngle; // 角度量
	int SpeedDirection; // 速度符号
	int TargetSpeed;	// 速度
	int LaserGunSwitch; // 激光开关
	int Position_0;		// 云台电机0位置
	int Position_1;		// 云台电机1位置
	int Speed_0;		// 云台电机0速度
	int Speed_1;		// 云台电机1速度
	int Time_0;			// 云台电机0时间
	int Time_1;			// 云台电机1时间
	int Fun;			// 功能函数  默认：0x00 设置ID：0x01 置中0x02
	int Orin_ID;		// 使用ID
	int Set_ID_Num;		// 重置ID
	int VoiceSwitch;	// 预留位
} sMartcarControl;

typedef struct curYuntai_feedback
{
	int Position_0; // 云台电机0位置
	int Position_1; // 云台电机1位置
	int Speed_0;	// 云台电机0速度
	int Speed_1;	// 云台电机1速度
	int Load_0;		// 云台电机0时间
	int Load_1;		// 云台电机1时间
} curYuntai_feedback;

class turn_on_robot : public rclcpp::Node
{
public:
	turn_on_robot(std::string name); // Constructor //构造函数
	~turn_on_robot();				 // Destructor //析构函数
	void Control();					 // Loop control code //循环控制代码
	void CaremaMontorControl();

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_offset_center;
	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_camemo_feedback;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_scan_request; // 循迹: 到达点位，请求全向扫描
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_manual_gimbal; // ★浏览器遥控: 手动接管开关
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_shoot_reset;   // ★射击计数重置 (/shoot_reset)

	// ================= ★新版本补丁（2026-10-07）：两键开火 + 提前量 =================
	// 为什么需要（旧版两个硬缺口，见 docs/survey_gimbal_fire.md）：
	//   ① 放行门是**死线**：path_executor 一直在发 /move_base/stop_signal，
	//      但 C++ 侧**零订阅者** —— "允许开火"从未真正生效，云台按自己的判据开火；
	//   ② 预判只有 0.5 帧（≈16ms）而链路延迟 0.5~0.9s，且开火要求"画面静止 ≤1.5px/帧"
	//      —— 移动轨道靶几乎打不出去（车上日志正是"确认→允许射击→马上丢失"循环）。
	// 本补丁把开火权改成**两把钥匙**（默认关闭，保持旧行为）：
	//   · /aim/fire_gate  ← MissionTask：已到站、已停稳、进入 FIRE 阶段
	//   · /aim/fire_ready ← AimTask：目标确认、**预测落点**在开火框内、冷却/弹药预算允许
	//   两者都为 1 才开火；同时用 /aim/lead_x（px）做**延迟补偿提前量**（增益可调）。
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_fire_gate;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_fire_ready;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_lead_x;
	void callback_fire_gate(const std_msgs::msg::UInt8::SharedPtr msg);
	void callback_fire_ready(const std_msgs::msg::UInt8::SharedPtr msg);
	void callback_lead_x(const std_msgs::msg::Float32::SharedPtr msg);

	bool use_external_fire_gate_ = false; // true = 开火改由"两键"决定（移动靶模式）
	double lead_apply_gain_ = 0.0;        // 0 = 不施加提前量（旧行为）；1 = 完全施加 /aim/lead_x
	bool external_gate_ = false;          // /aim/fire_gate 当前值
	bool external_ready_ = false;         // /aim/fire_ready 当前值
	double lead_px_ = 0.0;                // /aim/lead_x 当前值（px，正 = 目标向右）

	// ★云台手动接管标志：1 = 停止下发 CameMo_Conl，把云台控制权让给遥控器。
	//   为什么必须让云台闭嘴：本节点主循环以 50~100Hz 持续发 CameMo_Conl，
	//   遥控器也发同一个话题 —— 两边不同频，云台会疯狂来回抽。
	//   注意：只停"位置下发"，开火与状态上报不受影响。
	bool manual_override_ = false;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;         // 车体角速度（前馈凝视稳定用）

    void callback_offset_center(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
	void callback_camemo_feedback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
	void callback_scan_request(const std_msgs::msg::UInt8::SharedPtr msg);
	void callback_shoot_reset(const std_msgs::msg::UInt8::SharedPtr msg);
	void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_LaserShot_Command;
	rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_CameMo_Conl;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_voice_switch;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_shot_count; // 累计发射数，发布到 /shoot_count
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_aim_status; // 自瞄状态: 0/1/2，发布到 /aim/status

	// ================= 自瞄状态机 =================
	// 循迹始终是最高优先级：本状态机不会阻塞循迹。
	//   循迹侧对"等待扫描"有硬超时；云台侧扫描有趟数上限 —— 双重兜底。
	//   AIM_IDLE  : 未锁定、未扫描 —— 云台保持不动（行进中的默认状态，不主动扫）
	//   AIM_SCAN  : 收到循迹"到达点位"请求后的全向扫描（第1趟向一端 + 第2趟反向扫回）
	//   AIM_TRACK : 已确认目标，边走边跟随（位置由视觉伺服持续修正）
	enum AimState { AIM_IDLE = 0, AIM_SCAN = 1, AIM_TRACK = 2 };
	int aim_state_ = AIM_IDLE;
	int aim_status_last_ = -1; // 上次已上报的状态（-1 = 还没发过）
	int aim_status_tick_ = 0;  // (已废弃) 原按循环计数的保活，改为按时间
	rclcpp::Time aim_status_last_time_{0, 0, RCL_ROS_TIME}; // 上次上报状态的时间

	int lost_target_count_ = 0; // 连续丢帧计数

	// ===== ★P1-3 目标类别迟滞 =====
	// 视觉偶尔会把蓝靶的某一帧判成红靶或"无目标"。若直接采信当帧类别，
	// 在 fire_only_blue 模式下会频繁掐断开火，表现为"明明瞄上了却打不出去"。
	// 做法：锁一个"已确认类别"，只有连续 cls_switch_confirm_frames_ 帧
	// 都是另一个类别，才改锁 —— 单帧抖动被完全吸收。
	int locked_cls_ = 0;            // 已确认类别(0=无 1=蓝 2=红)
	int cls_candidate_ = 0;         // 待确认的新类别
	int cls_streak_ = 0;            // 新类别连续出现帧数
	int cls_switch_confirm_frames_ = 5;

	// ===== ★流畅度：自适应主循环频率 =====
	// 跟踪时需要 100Hz 保控制精度；扫描/空闲时 50Hz 完全够用（扫描就是匀速转），
	// 降频能明显省 CPU，把算力让给 RKNN 推理。
	int loop_hz_track_ = 100;
	int loop_hz_idle_ = 50;

	void setAimState(int s); // 切换状态并立即上报 /aim/status
	void publishAimStatus(); // 发布当前自瞄状态
	bool isTracking() const { return aim_state_ == AIM_TRACK; }

	sMartcarControl moveBaseControl;
	curYuntai_feedback curYuntai_feedback_data; //需要一个回调函数获取当前反馈值

	// ======================= 瞄准扫描参数 =======================
	// 扫描方式：连续匀速左右往复（不再逐点步进停留）。
	//   目标点始终取当前方向上的端点，云台以 patrol_speed_ratio x 最大转速 匀速扫过去，
	//   到达端点后立即反向，如此往复，直到确认目标。
	// 竖直层角度列表(相对水平面，上为正)，默认依次为 上15° -> 水平0° -> 下10°
	std::vector<double> scan_pitch_levels_deg_;
	int scan_yaw_dirs_ = 8;				// (保留兼容) 旧逐点模式的点数，连续扫描下不再参与计算
	double scan_yaw_min_deg_ = -90.0;	// 水平最小角(相对车体前方，右为正)
	double scan_yaw_max_deg_ = 90.0;	// 水平最大角
	double scan_pitch_offset_deg_ = -10.0; // 俯仰整体偏移(度，负 = 向下压；t52 与 robot.yaml 对齐)
	double yaw_deg_sign_ = 1.0;			// 水平角度正方向->刻度增减(反了就改-1)
	double pitch_deg_sign_ = 1.0;		// 竖直角度正方向->刻度增减(反了就改-1)
	int gimbal_center_tick_ = 2047;		// 零位(正前方/水平)对应刻度
	double yaw_ticks_per_deg_ = 11.378;	  // 水平 刻度/度 = 4096/360 (12位编码器/360°)
	double pitch_ticks_per_deg_ = 11.378; // 竖直 刻度/度 = 4096/360
	int gimbal_yaw_tick_min_ = 100;       // 水平轴刻度下限(360°量程，留边界余量)
	int gimbal_yaw_tick_max_ = 3995;      // 水平轴刻度上限
	int gimbal_pitch_tick_min_ = 950;     // 竖直轴刻度下限(下位机强制钳位 950)
	int gimbal_pitch_tick_max_ = 2400;    // 竖直轴刻度上限(下位机强制钳位 2400)
	int gimbal_max_speed_ = 3700;	  // 云台最大转速
	double patrol_speed_ratio_ = 0.35; // 巡逻(扫描)速度 = 最大转速 x 该比例
	int scan_dwell_ms_ = 300;		  // 每个点最短驻留时间(ms)
	int scan_arrive_tol_ = 30;		  // 认为云台"已到位"的刻度容差
	int scan_passes_ = 2;			  // 全向扫描趟数：1 趟"左" + 1 趟"右" = 2
	// ===== ★卡 t25：进入扫描前的"归位段" + 连续扫描轮数上限 =====
	// 用户口径：①进入扫描先复位到正前+水平，再开始巡航；②连续扫描最多 3 轮，
	//   3 轮仍无目标 → 自动回 IDLE 归位（不再无限扫）；③找到目标立即 TRACK（既有链路不变）。
	int scan_continuous_cycles_ = 3;   // 连续扫描最多几轮（一轮 = scan_passes_ 趟）；<=0 = 不限（旧行为）
	int scan_home_timeout_ms_ = 1500;  // 归位段超时(ms)：电机卡住/到位判据不满足时也要进扫描（绝不永远不进）
	rclcpp::Time scan_home_deadline_;  // 归位段截止时刻（startScan 时按上面超时设置）
	bool scan_homing_ = false;         // true = 当前处于"进扫描前先归位"阶段（此时 aim_state_ 仍是 IDLE）
	int scan_cycles_done_ = 0;         // 本次连续扫描已完成的轮数（用于 3 轮上限）
	int lost_frames_to_rescan_ = 60;  // 跟踪中连续丢帧超过该值才放弃追踪回到扫描
  // ★t42（t41 台账 §3 主因 H1）丢靶去抖滞回：重启扫掠前必须【连续未见 >= rescan_min_frames_
  //   帧】且【持续 >= rescan_debounce_ms_】；旧行为（单帧即重扫）可把两参数设 1 / 0 回退。
  int rescan_min_frames_ = 10;      // ★t42 连续未见帧数下限（10 帧 @33fps ~= 300ms）
  int rescan_debounce_ms_ = 400;    // ★t42 连续未见时长下限（ms，>=300ms 口径）
  rclcpp::Time lost_since_;         // ★t42 本轮【第一次未见】的时刻（见到即作废）
  bool lost_since_valid_ = false;   // ★t42 lost_since_ 是否有效
  rclcpp::Time cls_diag_last_ = rclcpp::Time(0, 0, RCL_ROS_TIME);  // ★t42 类别诊断限流
	int vision_timeout_ms_ = 1000;	  // 追踪中多久收不到视觉结果就强制回扫描(ms)
	// ===== 视觉滤波与消抖 =====
	double ema_alpha_ = 0.3;	  // 一阶低通滤波系数(0~1)，越小越平滑但延迟越大
	int track_deadzone_px_ = 12;  // 跟踪死区：滤波后偏差小于此值不再动位置(消抖核心)
	int fine_zone_px_ = 20;		  // 偏差小于此值进入"精调"档(降Kp、加Kd)
	double fine_kp_scale_ = 0.5;  // 精调档 Kp 缩放系数
	double fine_kd_scale_ = 2.0;  // 精调档 Kd 放大系数
	// ===== PD 跟踪控制 =====
	double track_kp_ = 8.0;				 // 比例增益(原 track_speed_per_px，已调低)
	double track_kd_ = 3.0;				 // 微分增益(阻尼，抑制超调振荡)
	double track_speed_max_ratio_ = 0.6; // 跟踪速度上限 = 巡逻速度 x 该比例
	// P2-2 tracking speed cap decoupled: if >0 use this absolute tick value,
	//      else derive from patrolSpeed()*ratio (old behavior).
	int track_speed_max_ = 0;	// ===== 开火判定：绝对静止 + 动态框 + 弹药管理 =====
	int fire_deadzone_px_ = 8;		// 固定开火死区(未启用动态框时使用)
	bool fire_dz_dynamic_ = true;	// 启用动态开火框：目标越近视差越大，框越宽
	double fire_dz_far_px_ = 6.0;	// 远距离(约35cm)开火死区
	double fire_dz_near_px_ = 18.0; // 近距离(约10cm)开火死区
	double box_w_far_ = 30.0;		// 远处目标框宽(标定值，越小越远)
	double box_w_near_ = 120.0;		// 近处目标框宽(标定值)
	// ===== 开火稳定判据（两种模式二选一）=====
	bool fire_use_image_stable_ = true;		// true=画面稳定判据(方案B，移动中也能开火)；false=旧的电机静止判据
	double fire_stable_px_per_frame_ = 1.5; // 画面稳定阈值：滤波后偏差的单帧变化量上限(像素/帧)，越小越严
	// 以下三项仅在 fire_use_image_stable_ = false（电机静止判据）时生效
	int fire_real_speed_max_ = 20;	// 下位机回传实际速度 < 该值才算"绝对静止"
	int fire_speed_max_ = 50;		// 下发指令速度上限(辅助判据)
	int fire_hold_frames_ = 5;		// 连续满足条件多少帧才真正开火
	// ===== 预判补偿 =====
	double predict_gain_ = 0.5;		// 按目标移动速度提前的帧数(0=关闭预判)
	// ===== 车体转动前馈（惯性凝视稳定）=====
	// 车体转动时云台若不反向补偿，瞄准点会跟着车体一起转，目标会持续往画面一侧漂。
	// 用 /imu_data 的偏航角速度提前抵消，视觉伺服只负责收尾。
	double gyro_ff_gain_ = 0.0;				  // 前馈增益：0=关闭，1.0=完全补偿车体转动
	double gyro_ff_sign_ = 1.0;				  // 补偿方向：实测反了就改 -1.0
	int imu_ff_timeout_ms_ = 200;			  // IMU 超过这么久没数据就暂停前馈
	// 前馈运行时状态
	double imu_omega_z_ = 0.0;				  // 最近一次车体偏航角速度(rad/s)
	rclcpp::Time last_imu_time_;			  // 最近一次收到 IMU 的时刻
	rclcpp::Time last_track_time_;			  // 上一次视觉回调时刻(用于计算前馈 dt)
	// ===== 激光同轴偏差补偿 =====
	// 激光与摄像头不同轴时，激光落点相对画面中心有固定偏移。
	// 开火判定应以"激光实际落点"为零点，而不是画面几何中心。
	double laser_offset_x_ = 0.0;	// 激光落点相对画面中心的水平偏移(像素，标定值)
	double laser_offset_y_ = 0.0;	// 竖直偏移(像素)
	// ===== 弹药管理：全程固定 10 发，用光即止 =====
	static constexpr int AIM_MAX_SHOTS = 10; // 全程弹药硬上限，任何参数都无法突破
	int max_shots_ = AIM_MAX_SHOTS;			 // 实际可用发数(只能调到更小，不能超过 10)
	int max_shots_per_target_ = 3;  // ★P0-2 单目标最多连发数，防止全程弹药倾泻在一个靶上
	int shots_on_target_ = 0;       // 当前目标已连发数
	bool fire_only_blue_ = true;    // ★P0-4 仅攻击蓝色靶(cls==1)，忽略红色靶(cls==2)
	int shot_cooldown_ms_ = 500;			 // 两发之间的最小间隔(ms)
	// ===== 目标筛选：只跟踪长宽比近似 1:1 的目标 =====
	double aspect_min_ = 0.75;		   // 框宽/框高 下限，超出范围视为长方形不跟踪
	double aspect_max_ = 1.35;		   // 框宽/框高 上限
	// 扫描运行时状态
	int scan_yaw_idx_ = 0;	  // 当前水平点索引
	int scan_yaw_dir_ = 1;	  // 水平横扫方向(+1 向右扫 / -1 向左扫)
	int scan_pitch_idx_ = 0;

	// ★跟踪俯仰修正余量(度)。clampPitchToScan 会在扫描俯仰范围外再留出这么多度，
	//   供视觉伺服修正使用。原因：scan_pitch_levels 只有一层时(如 [0.0])，
	//   min==max -> 允许区间宽度为 0 -> 伺服的俯仰修正全被钳掉 -> 云台僵死。
	//   默认 12°，足够覆盖常见的瞄准偏差，又不会让云台撞上下限位。
	double tracking_pitch_margin_deg_ = 12.0;

	// ★空闲归位速度比例（相对巡逻速度）。IDLE 时云台主动回到"正前方+基准俯仰"，
	//   避免冻结在最后一次的任意姿态上，导致"改了俯仰参数却看不出变化"。
	double idle_homing_speed_ratio_ = 0.5;
	int scan_pass_done_ = 0;  // 本轮全向扫描已完成的横扫趟数
	bool scan_finished_ = false; // 本轮全向扫描是否已扫满 scan_passes_ 趟
	bool scan_continuous_ = true; // 持续扫描模式：一轮扫完自动重来，直到确认目标；目标丢失后也自动重开扫描
	                              // ★ 默认 true：开机即持续往复扫描，不依赖外部 /nav/scan_request 的时序
	rclcpp::Time last_scan_step_time_; // 上一次推进扫描点的时刻
	rclcpp::Time last_vision_time_;	   // 上一次收到视觉结果(offset_center)的时刻
	// 跟踪运行时状态
	double dx_filt_ = 0.0;	  // 水平偏差的滤波值
	double dy_filt_ = 0.0;	  // 竖直偏差的滤波值
	bool filt_inited_ = false; // 滤波器是否已用首帧初始化
	double abs_x_last_ = 0.0; // 上一帧水平绝对偏差(用于微分项)
	double abs_y_last_ = 0.0; // 上一帧竖直绝对偏差
	double dx_prev_filt_ = 0.0; // 上一帧滤波值(预判用)
	double dy_prev_filt_ = 0.0;
	// ===== 扫描确认（射击解锁）=====
	int target_seen_count_ = 0;		// 本次跟踪中连续识别到目标的帧数
	int target_confirm_frames_ = 8; // 连续识别多少帧才算"扫描确认有目标"
	bool fire_cleared_ = false;		// 本次跟踪是否已解锁射击(仅用于打印一次)
	// ===== 开火执行 =====
	int fire_hold_count_ = 0; // 开火条件连续满足的帧计数
	int shots_fired_ = 0;	  // 已发射次数(全程累计，不重置)
	rclcpp::Time last_shot_time_; // 上一次开火时刻(冷却用)

	int clampYawTick(int tick) const;
	int clampPitchTick(int tick) const;
	int clampYawToScan(int tick) const;   // ★P0-1 钳位到"扫描范围"，而非硬件极限
	int clampPitchToScan(int tick) const;
	double scanYawDeg() const;
	double scanPitchDeg() const;
	int pitchLevelCount() const;
	int patrolSpeed() const;
	bool aspectRatioOk(const std_msgs::msg::Int32MultiArray &msg) const;
	void applyScanTarget();
	void advanceScanIndex();
	void startScan(bool continuous, bool home = true);   // ★t42 重扫传 home=false：不重跑归位段 // 启动全向扫描；continuous=true 时扫完自动重来
	bool scanHomingStep();           // ★t25：进扫描前的归位段；返回 true = 归位完成，可以开扫
	void enterScanPhase();           // ★t25：真正进入扫描阶段（下发第一个横扫点）
	int trackSpeedFor(double abs_err, double d_abs_err, double eff) const;
	int fireDeadzonePx(double box_w) const;
	// ===================== 瞄准扫描参数结束 =====================

	// 视差补偿查表（预留：当前用固定 laser_offset_x/y，改动态补偿时接 compute_comp_pixel_0）
	int compute_comp_pixel_0(double tar_dis);
	bool loadLookupTable(const std::string& filePath);
	std::vector<int> lookup_table;
	std_msgs::msg::UInt8 last_voice_msg;

};


