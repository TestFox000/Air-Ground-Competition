
#include "Quaternion_Solution.h"
#include <vector>
#include <cmath>

#define M_PI 3.14159265358979323846

// ==================== 滤波器参数配置 ====================
// 可调节的滤波器参数 - 直接修改变量即可
float IMU_SAMPLING_FREQ = 50.0f;         // IMU采样频率 (Hz) - 可动态调整
float GYRO_CUTOFF_FREQ = 10.0f;          // 角速度截止频率 (Hz) - 基于官方建议调整
bool MOTION_ADAPTIVE_FILTER = true;      // 是否启用运动状态自适应滤波
bool ENABLE_SMOOTH_PREFILTER = true;    // 预滤波开关 (默认关闭)

// 基于Allan方差的噪声参数
double GYRO_NOISE = 1.044e-3;            // 角速度噪声 (rad/s)
double ACCEL_NOISE = 2.214e-2;           // 加速度噪声 (m/s²)

// 滑动平均预滤波参数
#define SMOOTH_FILTER_NUM 10              // 滑动平均窗口大小 (基于官方建议)

// ==================== 滤波器状态变量 ====================
// 卡尔曼滤波器状态 (加速度)
static double acc_x_est = 0, acc_y_est = 0, acc_z_est = 0;
static double P_acc_x = 1, P_acc_y = 1, P_acc_z = 1;

// Butterworth滤波器缓冲区 (角速度)
static std::vector<double> gyr_x_buf(3, 0), gyr_y_buf(3, 0), gyr_z_buf(3, 0);
static std::vector<double> gyr_x_y_buf(2, 0), gyr_y_y_buf(2, 0), gyr_z_y_buf(2, 0);

// 滑动平均预滤波缓冲区 (角速度)
static std::vector<float> smooth_gx_buf(SMOOTH_FILTER_NUM, 0);
static std::vector<float> smooth_gy_buf(SMOOTH_FILTER_NUM, 0);
static std::vector<float> smooth_gz_buf(SMOOTH_FILTER_NUM, 0);

// 滤波器系数缓存
static double butter_b0 = 0, butter_b1 = 0, butter_b2 = 0;
static double butter_a1 = 0, butter_a2 = 0;



// ==================== 滤波器实现函数 ====================
/**
 * 优化的卡尔曼滤波器 (基于Allan方差参数)
 */
 double kalman_filter_optimized(double z, double &x_est_prev, double &P_prev, bool is_motion) {
  // Allan 给出的加速度噪声标准差 (m/s²)，取平方为方差
  double R_base = ACCEL_NOISE * ACCEL_NOISE;

  // 过程噪声 Q 通常比观测噪声小几个量级
  // 建议范围：R_base * 0.001 ~ R_base * 0.05
  double Q = R_base * 0.01;

  // 自适应 R（观测噪声方差）
  double R = R_base;
  if (MOTION_ADAPTIVE_FILTER) {
      if (is_motion) {
          R = R_base * 5.0;    // 运动时更信任预测
      } else {
          R = R_base * 0.5;    // 静止时更信任观测
      }
  }

  // -------- 标准卡尔曼滤波 --------
  // 预测
  double x_pred = x_est_prev;
  double P_pred = P_prev + Q;

  // 更新
  double K = P_pred / (P_pred + R);
  double x_est = x_pred + K * (z - x_pred);
  double P = (1.0 - K) * P_pred;

  // 保存状态
  x_est_prev = x_est;
  P_prev = P;

  return x_est;
}

/**
 * 计算二阶Butterworth低通滤波器系数
 */
void calculate_butterworth_coefficients() {
    // 每次都重新计算，确保参数变化时能自动更新
    // 标准二阶Butterworth低通滤波器设计
    double w = tan(M_PI * GYRO_CUTOFF_FREQ / IMU_SAMPLING_FREQ);
    double k1 = sqrt(2.0) * w;
    double k2 = w * w;
    double a = k2 + k1 + 1.0;
    
    // 计算滤波器系数
    butter_b0 = k2 / a;
    butter_b1 = 2.0 * butter_b0;
    butter_b2 = butter_b0;
    butter_a1 = (2.0 * (k2 - 1.0)) / a;
    butter_a2 = (k2 - k1 + 1.0) / a;
}

/**
 * 修正的二阶Butterworth低通滤波器
 */
double butterworth_filter_optimized(double x_new, std::vector<double> &x_buf, std::vector<double> &y_buf) {
    // 计算滤波器系数
    calculate_butterworth_coefficients();
    
    // 更新输入缓冲区
    x_buf[2] = x_buf[1];
    x_buf[1] = x_buf[0];
    x_buf[0] = x_new;
    
    // 滤波器方程
    double y_new = butter_b0 * x_buf[0] + butter_b1 * x_buf[1] + butter_b2 * x_buf[2] 
                   - butter_a1 * y_buf[0] - butter_a2 * y_buf[1];
    
    // 更新输出缓冲区
    y_buf[1] = y_buf[0];
    y_buf[0] = y_new;
    
    return y_new;
}

/**
 * 滑动平均预滤波 (去除尖峰噪声) - 基于官方建议
 * @param input 输入值
 * @param buffer 滑动窗口缓冲区
 * @return 滤波后的值
 */
float smooth_prefilter(float input, std::vector<float> &buffer) {
    // 更新缓冲区 (移位)
    for (int i = SMOOTH_FILTER_NUM - 1; i >= 1; i--) {
        buffer[i] = buffer[i-1];
    }
    buffer[0] = input;
    
    // 计算滑动平均
    float sum = 0;
    for (int i = 0; i < SMOOTH_FILTER_NUM; i++) {
        sum += buffer[i];
    }
    
    return sum / (float)SMOOTH_FILTER_NUM;
}

/**
 * 检测运动状态 (简单的阈值检测)
 */
bool detect_motion(float gx, float gy, float gz, float ax, float ay, float az) {
    const float GYRO_THRESHOLD = 0.1f;  // rad/s
    const float ACCEL_THRESHOLD = 1.0f; // m/s²
    
    float gyro_mag = sqrt(gx*gx + gy*gy + gz*gz);
    float accel_mag = sqrt(ax*ax + ay*ay + az*az);
    float gravity_error = fabs(accel_mag - 9.81f);
    
    return (gyro_mag > GYRO_THRESHOLD) || (gravity_error > ACCEL_THRESHOLD);
}

/**
 * 处理IMU数据的完整滤波函数
 */
void filter_imu_data(float &gx, float &gy, float &gz, float &ax, float &ay, float &az) {
    // 检测运动状态
    bool is_motion = detect_motion(gx, gy, gz, ax, ay, az);
    
    // ==================== 角速度滤波处理 ====================
    // 第一步：滑动平均预滤波 (去除尖峰) - 当前注释掉不启用
    
    if (ENABLE_SMOOTH_PREFILTER) {
        gx = smooth_prefilter(gx, smooth_gx_buf);
        gy = smooth_prefilter(gy, smooth_gy_buf);
        gz = smooth_prefilter(gz, smooth_gz_buf);
    }
    
    
    // 第二步：二阶Butterworth低通滤波 (去除高频谐波)
    gx = butterworth_filter_optimized(gx, gyr_x_buf, gyr_x_y_buf);
    gy = butterworth_filter_optimized(gy, gyr_y_buf, gyr_y_y_buf);
    gz = butterworth_filter_optimized(gz, gyr_z_buf, gyr_z_y_buf);
    
    // ==================== 加速度滤波处理 ====================
    // 对加速度数据进行卡尔曼滤波
    ax = kalman_filter_optimized(ax, acc_x_est, P_acc_x, is_motion);
    ay = kalman_filter_optimized(ay, acc_y_est, P_acc_y, is_motion);
    az = kalman_filter_optimized(az, acc_z_est, P_acc_z, is_motion);
}

/**************************************
Date: May 31, 2020
Function: 平方根倒数 求四元数用到
***************************************/
float InvSqrt(float number)
{
  volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;
    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );

  return y;
}
/**************************************
Date: May 31, 2020
Function: 四元数解算
***************************************/
volatile float twoKp = 1.0f;     // 2 * proportional gain (Kp)
volatile float twoKi = 0.0f;     // 2 * integral gain (Ki) - 禁用积分校正测试
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
geometry_msgs::msg::Quaternion Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // 首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;      
    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    //误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    // 计算并应用积分反馈（如果启用）
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / 100);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / 100);
      integralFBz += twoKi * halfez * (1.0f / 100);
      gx += integralFBx;        // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f;       // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }
    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / 100));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / 100));
  gz *= (0.5f * (1.0f / 100));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx); 
  // Normalise quaternion
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  
  // 创建并返回四元数
  geometry_msgs::msg::Quaternion orientation;
  orientation.w = q0;
  orientation.x = q1;
  orientation.y = q2;
  orientation.z = q3;
  
  return orientation;
}
