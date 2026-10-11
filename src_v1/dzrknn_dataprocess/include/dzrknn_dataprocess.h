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

    void callback_offset_center(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
	void callback_camemo_feedback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
	void callback_scan_request(const std_msgs::msg::UInt8::SharedPtr msg);

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
	int aim_status_tick_ = 0;  // 状态未变化时的保活计数

	int lost_target_count_ = 0; // 连续丢帧计数

	void setAimState(int s); // 切换状态并立即上报 /aim/status
	void publishAimStatus(); // 发布当前自瞄状态
	bool isTracking() const { return aim_state_ == AIM_TRACK; }

	sMartcarControl moveBaseControl;
	curYuntai_feedback curYuntai_feedback_data; //需要一个回调函数获取当前反馈值

	// ======================= 瞄准扫描参数 =======================
	// 扫描路径：竖直分若干层，每层做一次完整的水平左右横扫，层间横扫方向交替，整体呈 Z 字形
	// 竖直层角度列表(相对水平面，上为正)，默认依次为 上15° -> 水平0° -> 下10°
	std::vector<double> scan_pitch_levels_deg_;
	int scan_yaw_dirs_ = 8;				// 水平横扫的点数
	double scan_yaw_min_deg_ = -90.0;	// 水平最小角(相对车体前方，右为正)
	double scan_yaw_max_deg_ = 90.0;	// 水平最大角
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
	double patrol_speed_ratio_ = 0.5; // 巡逻(扫描)速度 = 最大转速 x 该比例
	int scan_dwell_ms_ = 300;		  // 每个点最短驻留时间(ms)
	int scan_arrive_tol_ = 30;		  // 认为云台"已到位"的刻度容差
	int scan_passes_ = 2;			  // 全向扫描趟数：1 趟"左" + 1 趟"右" = 2
	int lost_frames_to_rescan_ = 60;  // 跟踪中连续丢帧超过该值才放弃追踪回到扫描
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
	// ===== 开火判定：绝对静止 + 动态框 + 弹药管理 =====
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
	// ===== 激光同轴偏差补偿 =====
	// 激光与摄像头不同轴时，激光落点相对画面中心有固定偏移。
	// 开火判定应以"激光实际落点"为零点，而不是画面几何中心。
	double laser_offset_x_ = 0.0;	// 激光落点相对画面中心的水平偏移(像素，标定值)
	double laser_offset_y_ = 0.0;	// 竖直偏移(像素)
	// ===== 弹药管理：全程固定 10 发，用光即止 =====
	static constexpr int AIM_MAX_SHOTS = 10; // 全程弹药硬上限，任何参数都无法突破
	int max_shots_ = AIM_MAX_SHOTS;			 // 实际可用发数(只能调到更小，不能超过 10)
	int shot_cooldown_ms_ = 500;			 // 两发之间的最小间隔(ms)
	// ===== 目标筛选：只跟踪长宽比近似 1:1 的目标 =====
	double aspect_min_ = 0.75;		   // 框宽/框高 下限，超出范围视为长方形不跟踪
	double aspect_max_ = 1.35;		   // 框宽/框高 上限
	// 扫描运行时状态
	int scan_yaw_idx_ = 0;	  // 当前水平点索引
	int scan_yaw_dir_ = 1;	  // 水平横扫方向(+1 向右扫 / -1 向左扫)
	int scan_pitch_idx_ = 0;  // 当前竖直层索引
	int scan_pass_done_ = 0;  // 本轮全向扫描已完成的横扫趟数
	bool scan_finished_ = false; // 本轮全向扫描是否已扫满 scan_passes_ 趟
	bool scan_continuous_ = false; // 持续扫描模式：一轮扫完自动重来，直到确认目标；目标丢失后也自动重开扫描
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
	double scanYawDeg() const;
	double scanPitchDeg() const;
	int pitchLevelCount() const;
	int patrolSpeed() const;
	bool aspectRatioOk(const std_msgs::msg::Int32MultiArray &msg) const;
	void applyScanTarget();
	void advanceScanIndex();
	void startScan(bool continuous); // 启动全向扫描；continuous=true 时扫完自动重来
	int trackSpeedFor(double abs_err, double d_abs_err, double eff) const;
	int fireDeadzonePx(double box_w) const;
	// ===================== 瞄准扫描参数结束 =====================

	// 视差补偿查表（预留：当前用固定 laser_offset_x/y，改动态补偿时接 compute_comp_pixel_0）
	int compute_comp_pixel_0(double tar_dis);
	bool loadLookupTable(const std::string& filePath);
	std::vector<int> lookup_table;
	std_msgs::msg::UInt8 last_voice_msg;

};


