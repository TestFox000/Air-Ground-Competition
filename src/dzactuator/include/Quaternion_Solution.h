
#ifndef __QUATERNION_SOLUTION_H_
#define __QUATERNION_SOLUTION_H_
#include "dzactuator.h"
#include "geometry_msgs/msg/quaternion.hpp"

// ==================== 核心函数 ====================
float InvSqrt(float number);
geometry_msgs::msg::Quaternion Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az);

// ==================== 滤波函数 ====================
/**
 * 独立的IMU数据滤波函数 (不包含四元数解算)
 * @param gx,gy,gz 角速度 (输入输出)
 * @param ax,ay,az 线性加速度 (输入输出)
 */
void filter_imu_data(float &gx, float &gy, float &gz, float &ax, float &ay, float &az);

// ==================== 滤波器参数变量 (外部可直接修改) ====================
extern float IMU_SAMPLING_FREQ;         // IMU采样频率 (Hz)
extern float GYRO_CUTOFF_FREQ;          // 角速度截止频率 (Hz)
extern bool MOTION_ADAPTIVE_FILTER;     // 运动状态自适应滤波开关
extern bool ENABLE_SMOOTH_PREFILTER;    // 滑动平均预滤波开关
extern double GYRO_NOISE;               // 角速度噪声 (rad/s)
extern double ACCEL_NOISE;              // 加速度噪声 (m/s²)

#endif


