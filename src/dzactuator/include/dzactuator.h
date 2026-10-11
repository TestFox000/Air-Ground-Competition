#ifndef __dzactuator_H_
#define __dzactuator_H_
#include "rclcpp/rclcpp.hpp"
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <iostream>
#include <cmath>
#include "serial/serial.h"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include <deque>
#include <tf2_ros/transform_broadcaster.h>
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/int32.hpp>
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include "tf2/LinearMath/Transform.h"
// #include "tf2/LinearMath/Quaternion.h"

// std_msgs/msg/Float32
// nav_msgs/msg/Odometry

#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

// template<typename T>
// T clamp(T value, T min, T max) {
//     return value < min ? min : (value > max ? max : value);
// }

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

#define SEND_DATA_CHECK 1	 // Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK 0	 // Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER 0X7B	 // Frame head //帧头
#define FRAME_TAIL 0X7D		 // Frame tail //帧尾
#define RECEIVE_DATA_SIZE 34 // The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE 11	 // The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
// #define PI 				  3.1415926f //PI //圆周率

#define CARL 0.15 // 轴距
#define CARW 0.157 // 轮距(m)：2026-10-11 用户卷尺实测 0.157（旧 0.11 与实测不符；本常量当前无代码引用，ticksPer2PI=3236 与 0.157 自洽）

extern sensor_msgs::msg::Imu BMI088; // External variables, IMU topic data //外部变量，IMU话题数据

// Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
// 协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
										 0, 1e-3, 0, 0, 0, 0,
										 0, 0, 1e6, 0, 0, 0,
										 0, 0, 0, 1e6, 0, 0,
										 0, 0, 0, 0, 1e6, 0,
										 0, 0, 0, 0, 0, 1e3};

const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
										  0, 1e-3, 1e-9, 0, 0, 0,
										  0, 0, 1e6, 0, 0, 0,
										  0, 0, 0, 1e6, 0, 0,
										  0, 0, 0, 0, 1e6, 0,
										  0, 0, 0, 0, 0, 1e-9};

// 对于vyaw值1e-1 相对信任程度较高
const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
										  0, 1e-3, 0, 0, 0, 0,
										  0, 0, 1e-3, 0, 0, 0,
										  0, 0, 0, 1e6, 0, 0,
										  0, 0, 0, 0, 1e6, 0,
										  0, 0, 0, 0, 0, 1e-1};

const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
										   0, 1e-3, 1e-9, 0, 0, 0,
										   0, 0, 1e6, 0, 0, 0,
										   0, 0, 0, 1e6, 0, 0,
										   0, 0, 0, 0, 1e6, 0,
										   0, 0, 0, 0, 0, 1e-9};

// Data structure for position
// 位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
} Vel_Pos_Data;

// Data structure for speed
// 速度数据结构体
typedef struct __Vel_Speed_Data_
{
	float Left;
	float Right;
} Vel_Speed_Data;

// IMU data structure
// IMU数据结构体
typedef struct __BMI088_DATA_
{
	short accele_x_data;
	short accele_y_data;
	short accele_z_data;
	short gyros_x_data;
	short gyros_y_data;
	short gyros_z_data;

} BMI088_DATA;

// The structure of the ROS to send data to the down machine
// ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_
{
	uint8_t tx[SEND_DATA_SIZE];
	float X_speed;
	float Y_speed;
	float Z_speed;
	unsigned char Frame_Tail;
} SEND_DATA;

// The structure in which the lower computer sends data to the ROS
// 下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_
{
	uint8_t rx[RECEIVE_DATA_SIZE];
	uint8_t Flag_Stop;
	unsigned char Frame_Header;
	float X_speed;
	float Y_speed;
	float Z_speed;
	float Power_Voltage;
	unsigned char Frame_Tail;
} RECEIVE_DATA;

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
	int VoiceSwitch;		// 预留位
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

// The robot chassis class uses constructors to initialize data, publish topics, etc
// 机器人底盘类，使用构造函数初始化数据和发布话题等
class turn_on_robot : public rclcpp::Node
{
public:
	turn_on_robot(std::string name); // Constructor //构造函数
	~turn_on_robot();				 // Destructor //析构函数
	void Control();					 // Loop control code //循环控制代码
	serial::Serial Stm32_Serial;	 // Declare a serial object //声明串口对象

	// ros::NodeHandle n;			// Create a ROS node handle //创建ROS节点句柄
	rclcpp::Time _Now, _Last_Time; // Time dependent, used for integration to find displacement (mileage) //时间相关，用于积分求位移(里程)
	float Sampling_Time;		   // Sampling time, used for integration to find displacement (mileage) //采样时间，用于积分求位移(里程)

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_movebase_angle;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_logicontrol;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr  sub_voice_switch;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr  sub_laser_switch;  // /laser_switch：激光常亮开关(0关/1开)
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_vehicle_state;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_vehicle_Drivestate;
	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_CameMo_Conl;
	// The speed topic subscribes to the callback function
	// 速度话题订阅回调函数
	void callback_cmd_vel_angle(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
	void callback_movebase_angle(const geometry_msgs::msg::Twist::SharedPtr msg);
	void callback_logi_control(const geometry_msgs::msg::Twist::SharedPtr msg);
    void callback_voice_switch(const std_msgs::msg::UInt8::SharedPtr msg);
    void callback_laser_switch(const std_msgs::msg::UInt8::SharedPtr msg);
	void callback_vehicle_state(const std_msgs::msg::UInt8::SharedPtr msg);
    void callback_vehicle_Drivestate(const std_msgs::msg::UInt8::SharedPtr msg);
	void callback_CameMo_Conl(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
	void callback_stopCMD(const std_msgs::msg::UInt8::SharedPtr msg);

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_odom_msg_valid;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_imu_msg_valid;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr Battery_Percentage_pub;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_diff_filter;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_navi_cmd;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_current_vel;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr wakeup_ack_pub_;
	rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_camemo;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_stopCMD;

	void Publish_Odom();			   // Pub the speedometer topic //发布里程计话题
	void Publish_ImuSensor();		   // Pub the IMU sensor topic //发布IMU传感器话题
	void Publish_ImuSensor_filter();   // Pub the IMU sensor topic of BMI088 //发布BMI088的IMU传感器话题
	void Publish_Voltage();			   // Pub the power supply voltage topic //发布电源电压话题
	void Publish_Battery_Percentage(); // Pub the Battery Percentage topic //发布电量百分比话题
	void Publish_CameMo();			   // Pub the camera motor feedback topic //发布云台电机反馈话题
	// 从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
	// Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
	bool Get_Sensor_Data_New();
	int detect_toggle(uint8_t new_value);

	unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode); // BBC check function //BBC校验函数
	short IMU_Trans(uint8_t Data_High, uint8_t Data_Low);					 // IMU data conversion read //IMU数据转化读取
	float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);					 // Odometer data is converted to read //里程计数据转化读取
	void sendCarInfoKernel();

	double normalizeAngle(double angle);

	void imu_bias_correction(const sensor_msgs::msg::Imu &raw_imu,
							 float &gx, float &gy, float &gz,
							 float &ax, float &ay, float &az);
	string serial_port_name, robot_frame_id, gyro_frame_id, odom_frame_id; // Define the related variables //定义相关变量

	RECEIVE_DATA Receive_Data; // The serial port receives the data structure //串口接收数据结构体
	SEND_DATA Send_Data;	   // The serial port sends the data structure //串口发送数据结构体

	Vel_Pos_Data Robot_Pos;	  // The position of the robot //机器人的位置
	Vel_Speed_Data Robot_Vel; // The speed of the robot //机器人的速度
	BMI088_DATA BMI088_Data;  // IMU data //IMU数据
	float Power_voltage;	  // Power supply voltage //电源电压


	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	sMartcarControl moveBaseControl;

	curYuntai_feedback curYuntai_feedback_data;

	bool montion_flag = false; // 小车是否在运动

	// ros::Publisher Battery_Percentage_pub; std_msgs::Float32
	double last_Battery_Percentage;
	int count_A, count_B, count_C;

	double Power_max, Power_min;

	std_msgs::msg::Float32 Battery_Percentage_msgs;

public:
	int calibrate_lineSpeed;
	int serial_baud_rate; // Serial communication baud rate //串口通信波特率
	double ticksPerMeter;
	double ticksPer2PI;
	double linear_Speed;
	double ThetaSpeed;
	double leftDistance;
	double rightDistance;

	// ==================== 全局变量 ====================
	bool bias_calibrated = false;			  // 零偏是否完成
	const int BIAS_CALIBRATION_SAMPLES = 200; // 校正样本数量

	float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
	float accel_bias_x = 0, accel_bias_y = 0, accel_bias_z = 0;
	int control_mode; // 1:非平行驾驶  2：平行驾驶
	int last_control_mode;
	int control_mode_count = 0;
	void switchvoice();
	std::vector<int> voice_nums;
    int lastvoice_num_1;
	int voice_numID =0;
	rclcpp::Time change_start_time;
	bool can_changevoice =true;
	
	int current_mode_ = 0; // 0:未定, 1:DWA, 2:Pursuit

	rclcpp::Time last_dwa_msg_time_;
	rclcpp::Time last_pursuit_msg_time_;
	rclcpp::Time last_pursuit_high_time_;

	geometry_msgs::msg::Twist::SharedPtr last_pursuit_msg_;
	geometry_msgs::msg::Twist::SharedPtr last_dwb_msg_; 

	rclcpp::TimerBase::SharedPtr mode_timer_;

	// 参数（可根据频率调整）
	const double ACTIVITY_TIMEOUT = 0.5;      // 秒：某源超过此时间没消息则视为停
	const double PURSUIT_LOW_TIMEOUT = 3.0;   // 秒：pursuit低于0.5超过此时间才从DWA→Pursuit
	const double CMD_WATCHDOG_S = 1.0;        // 秒：超过此时间没有任何速度指令就强制停车
	void modeTimerCallback();
	void checkCommandWatchdog();              // 指令超时看门狗：上位机掉线后自动停车，防止小车失控
	// 是否上次是 high (>=0.5)
	bool pursuit_prev_high_ = false;
	// 记录从 high -> low 的那一刻（fall time）
	rclcpp::Time pursuit_fell_time_;
	int stop_cmd =0;
	int stop_cmd_count =0;

	std::deque<float> speed_history_;
	std::deque<float> angle_history_;

	int smooth_window_speed =20;   // 平滑窗口大小
	int smooth_window_angle =5;
	float smoothValue(std::deque<float>& history, float new_value, int window);
	float smoothValueAdaptive(std::deque<float>& history,float new_value ,int window);
};

#endif
