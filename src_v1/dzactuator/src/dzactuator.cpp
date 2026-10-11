#include "dzactuator.h"
#include "Quaternion_Solution.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <chrono>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>

// 定义BOOST_BIND_GLOBAL_PLACEHOLDERS以消除警告
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/bind.hpp>

sensor_msgs::msg::Imu BMI088; // Instantiate an IMU object //实例化IMU对象

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<turn_on_robot>("dzactuator");
  // rclcpp::spin(node); // 使用rclcpp::spin而不是直接调用Control
  node->Control();
  rclcpp::shutdown();
  return 0;
}

/**************************************
Date: January 28, 2021
Function: Data conversion function
功能: 数据转换函数
***************************************/
short turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  short transition_16;
  transition_16 = 0;
  transition_16 |= Data_High << 8;
  transition_16 |= Data_Low;
  return transition_16;
}
float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  float data_return;
  short transition_16;
  transition_16 = 0;
  transition_16 |= Data_High << 8; // Get the high 8 bits of data   //获取数据的高8位
  transition_16 |= Data_Low;       // Get the lowest 8 bits of data //获取数据的低8位
  data_return = transition_16;     //(transition_16 / 1000)+(transition_16 % 1000)*0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
  return data_return;
}

/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void turn_on_robot::callback_cmd_vel_angle(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{

  last_dwa_msg_time_ = this->get_clock()->now(); // 记录收到时间
  last_dwb_msg_ = std::make_shared<geometry_msgs::msg::Twist>(*twist_aux);

  // 只有当当前模式 == DWA 时才执行原有控制逻辑
  if (current_mode_ != 1)
    return;
  
  // printf("dwb\n");
  if (control_mode == 1) // control_mode ： 1:非平行驾驶  2：平行驾驶
  {                      // printf("callback_cmd_vel_angle\n");
    float v = twist_aux->linear.x;
    float w = twist_aux->angular.z;
    // v = smoothValueAdaptive(speed_history_, v, smooth_window_speed);
    moveBaseControl.TargetSpeed = abs(clamp(v * 23 / 0.33, -255.0, 255.0));
    if (!v == 0)
    {
      moveBaseControl.TargetAngle = round(atan(CARL * w / v) * 57.3);
    }
    else
    {
      moveBaseControl.TargetAngle = 0;
    }
    moveBaseControl.TargetAngle += 60;

    // std::cout << " msg->linear.x = " << moveBaseControl.TargetSpeed << " msg->angular.z =  " <<  moveBaseControl.TargetAngle <<std::endl;

    // printf("linear X = %.2f,gngularZ = %.2f,%d,angle = %d\n",msg->linear.x,msg->angular.z,
    // abs(moveBaseControl.TargetSpeed),abs(moveBaseControl.TargetAngle));
    if (moveBaseControl.TargetAngle < 25)
    {
      moveBaseControl.TargetAngle = 25;
    }

    if (moveBaseControl.TargetAngle > 95)
    {
      moveBaseControl.TargetAngle = 95;
    }
    // linear speed
    if (v > 0)
      moveBaseControl.SpeedDirection = 0x00;
    else if (v < 0)
      moveBaseControl.SpeedDirection = 0x01;
    else if (v == 0)
      moveBaseControl.SpeedDirection = 0x00;
  }
}

void turn_on_robot::callback_movebase_angle(const geometry_msgs::msg::Twist::SharedPtr msg)
{

  last_pursuit_msg_time_ = this->get_clock()->now();
  last_pursuit_msg_ = std::make_shared<geometry_msgs::msg::Twist>(*msg);

  // 🔹 当 linear.y >= 0.5 时，持续更新高值时间
  if (msg->linear.y >= 0.5)
    last_pursuit_high_time_ = last_pursuit_msg_time_;

  // 仅在 Pursuit 模式时执行控制逻辑
  if (current_mode_ != 2)
    return;
  // printf("pursuit\n");

  if (control_mode == 2)
  {
    control_mode_count++;
    // 2s未接受到方向盘数据，切换到自动模式
    if (control_mode_count > 40)
    {
      control_mode_count = 0;
      control_mode = 1;
      last_control_mode = control_mode;

      std_msgs::msg::UInt8 navi_cmd;
      navi_cmd.data = 1;
      pub_navi_cmd->publish(navi_cmd);
    }
  }

  if (control_mode == 1) // control_mode ： 1:非平行驾驶  2：平行驾驶
  {
    // printf("callback_movebase_angle\n");
    float v = msg->linear.x;
    float w = msg->angular.z;
    v = smoothValueAdaptive(speed_history_, v, smooth_window_speed);
    // printf("v=%.2f,msg->linear.x=%.2f\n",v,msg->linear.x);
    moveBaseControl.TargetSpeed = abs(clamp(v * 23 / 0.33, -255.0, 255.0));
    moveBaseControl.TargetAngle = w;
    moveBaseControl.TargetAngle += 60;

    // printf("linear X = %.2d\n",moveBaseControl.TargetSpeed);
    if (moveBaseControl.TargetAngle < 25)
    {
      moveBaseControl.TargetAngle = 25;
    }
    if (moveBaseControl.TargetAngle > 95)
    {
      moveBaseControl.TargetAngle = 95;
    }
    // linear speed
    if (v > 0)
      moveBaseControl.SpeedDirection = 0x00;
    else if (v < 0)
      moveBaseControl.SpeedDirection = 0x01;
    else if (v == 0)
      moveBaseControl.SpeedDirection = 0x00;

    // current_vel_msg.linear.y 档位 1:P 2：R  3：N  4：D1  5：D2  6：D3  7：D4  8：D5 9：D
    // current_vel_msg.linear.z 驾驶模式 1：非平行驾驶模式 ，2 平行驾驶模式
    geometry_msgs::msg::Twist current_vel_msg = *msg;
    if (msg->linear.x > 0.05)
      current_vel_msg.linear.y = 9.0;
    else if (msg->linear.x < -0.05)
      current_vel_msg.linear.y = 2.0;
    else
      current_vel_msg.linear.y = 3.0;

    current_vel_msg.linear.z = 1.0;
    pub_current_vel->publish(current_vel_msg);
  }

}

void turn_on_robot::callback_logi_control(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  control_mode_count = 0;
  // printf("callback_movebase_angle\n");
  float v = msg->linear.x;
  float w = msg->angular.z;
  float z = msg->linear.z;
  if (z > 1.0)
  {
    control_mode = 2; // control_mode ： 1:非平行驾驶  2：平行驾驶
  }
  else
  {
    control_mode = 1;
  }

  if (last_control_mode == 2 && control_mode == 1)
  {
    std_msgs::msg::UInt8 navi_cmd;
    navi_cmd.data = 1;
    pub_navi_cmd->publish(navi_cmd);
    voice_numID =1;
  }
  last_control_mode = control_mode;
  v = smoothValueAdaptive(speed_history_, v, smooth_window_speed);
  moveBaseControl.TargetSpeed = abs(clamp(v * 23 / 0.33, -255.0, 255.0));
  moveBaseControl.TargetAngle = w;
  moveBaseControl.TargetAngle += 60;

  // printf("linear X = %.2d\n",moveBaseControl.TargetSpeed);
  if (moveBaseControl.TargetAngle < 25)
  {
    moveBaseControl.TargetAngle = 25;
  }
  if (moveBaseControl.TargetAngle > 95)
  {
    moveBaseControl.TargetAngle = 95;
  }
  // linear speed
  if (v > 0)
    moveBaseControl.SpeedDirection = 0x00;
  else if (v < 0)
    moveBaseControl.SpeedDirection = 0x01;
  else if (v == 0)
    moveBaseControl.SpeedDirection = 0x00;

  // current_vel_msg.linear.y 档位 1:P 2：R  3：N  4：D1  5：D2  6：D3  7：D4  8：D5 9：D
  // current_vel_msg.linear.z  驾驶模式 1：非平行驾驶模式 ，2 平行驾驶模式
  geometry_msgs::msg::Twist current_vel_msg = *msg;
  pub_current_vel->publish(current_vel_msg);
  voice_numID =15;
}

void turn_on_robot::callback_voice_switch(const std_msgs::msg::UInt8::SharedPtr msg)
{
  moveBaseControl.VoiceSwitch = static_cast<int>(msg->data);
  int tempdata = moveBaseControl.VoiceSwitch;
  std::cout << "moveBaseControl.VoiceSwitch = " << tempdata << std::endl;
}
void turn_on_robot::callback_vehicle_state(const std_msgs::msg::UInt8::SharedPtr msg)
{
  voice_nums[0] = msg->data;
  // printf("callback_vehicle_state %d\n",voice_nums[0]);
}

void turn_on_robot::callback_vehicle_Drivestate(const std_msgs::msg::UInt8::SharedPtr msg)
{
  voice_nums[1] = msg->data;
  // printf("callback_vehicle_Drivestate %d\n",voice_nums[1]);
}

void turn_on_robot::callback_CameMo_Conl(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  // 检查消息数据长度是否足够
  if (msg->data.size() >= 4)
  {
    moveBaseControl.Position_0 = msg->data[0];
    moveBaseControl.Position_1 = msg->data[1];
    moveBaseControl.Speed_0 = msg->data[2];
    moveBaseControl.Speed_1 = msg->data[3];
  }
  // 时间全部设为0
  moveBaseControl.Time_0 = 0;
  moveBaseControl.Time_1 = 0;
}

void turn_on_robot::callback_stopCMD(const std_msgs::msg::UInt8::SharedPtr msg)
{
    stop_cmd = msg->data;
    stop_cmd_count =0;

    // printf("stop_cmd received =%d!!!\n\n",stop_cmd);
}

void turn_on_robot::switchvoice()
{
    
  if (lastvoice_num_1 != voice_nums[0])
  {
    lastvoice_num_1 = voice_nums[0];
    voice_numID = lastvoice_num_1;
    can_changevoice =false;    
  }

  rclcpp::Time now = this->now();
  double elapsed = (now - change_start_time).seconds();
  
  if (elapsed>2.0)//至少经过2s才能播报下一个语音
  {
    can_changevoice =true;    
  }

  if (can_changevoice)
  {
    moveBaseControl.VoiceSwitch = voice_numID;    
    change_start_time = this->now();
    // std::cout<< "can_changevoice" << moveBaseControl.VoiceSwitch <<std::endl;
  }

  
}

float turn_on_robot::smoothValue(std::deque<float>& history, float new_value, int window)
{
  history.push_back(new_value);

  if (history.size() > window)
    history.pop_front();

  float sum = 0.0;
  for (auto& v : history)
    sum += v;

  return sum / history.size();
}

float turn_on_robot::smoothValueAdaptive(std::deque<float>& history,
                                         float new_value,
                                         int window)
{
    int temp_window = window;
    int max_window  = window + 15;

    float speed = fabs(new_value);

    // 动态决定平均窗口
    if(speed < 0.3)
        temp_window = window;
    else if(speed < 0.5)
        temp_window = window + 5;
    else if(speed < 1.0)
        temp_window = window + 10;
    else
        temp_window = window + 15;

    // 存入新值
    history.push_back(new_value);

    // 只控制最大缓存长度
    while(history.size() > max_window)
        history.pop_front();

    // 从后往前取最近 temp_window 个值求平均
    float sum = 0.0f;
    int start_index = std::max(0, (int)history.size() - temp_window); // 最近 temp_window 个元素的起始下标
    int count = 0;

    for(int i = start_index; i < history.size(); ++i)
    {
        sum += history[i];
        count++;
    }

    return sum / count;
}


void turn_on_robot::sendCarInfoKernel()
{

  if(stop_cmd ==1 && stop_cmd_count < 10){
    moveBaseControl.TargetSpeed =0;
    stop_cmd_count++;
  }
  // 数据平滑
  // moveBaseControl.TargetSpeed =
  //     smoothValueAdaptive(speed_history_, moveBaseControl.TargetSpeed, smooth_window_speed);
  // printf("moveBaseControl.TargetSpeed =%.2f\n");
  moveBaseControl.TargetAngle =
      smoothValue(angle_history_, moveBaseControl.TargetAngle, smooth_window_angle);


  unsigned char buf[23] = {0};
  buf[0] = 0xa5; // hdr1
  buf[1] = 0x5a; // hdr2
  buf[2] = 20;   // len - 数据长度 22 - 3 = 19字节
  // 角度 TargetAngle 转换成整形后放入 buf[3] 和 buf[4]
  int16_t angle = static_cast<int16_t>(moveBaseControl.TargetAngle * 10); // 假设角度以0.1度为单位
  buf[3] = (angle >> 8) & 0xFF;                                           // 高字节
  buf[4] = angle & 0xFF;                                                  // 低字节
  // 速度符号 SpeedDirection 放入 buf[5]
  buf[5] = static_cast<unsigned char>(moveBaseControl.SpeedDirection);
  // printf("moveBaseControl.TargetSpeed = %d\n",moveBaseControl.TargetSpeed);
  // 目标速度 TargetSpeed 放入 buf[6]（占1字节）
  buf[6] = static_cast<unsigned char>(moveBaseControl.TargetSpeed);

  // 备用
  buf[7] = 0x00;
  // 云台电机0位置 Position_O 放入 buf[8] 和 buf[9]
  buf[8] = (moveBaseControl.Position_0 >> 8) & 0xFF; // 高字节
  buf[9] = moveBaseControl.Position_0 & 0xFF;        // 低字节
  // 云台电机1位置 Position_1 放入 buf[10] 和 buf[11]
  buf[10] = (moveBaseControl.Position_1 >> 8) & 0xFF; // 高字节
  buf[11] = moveBaseControl.Position_1 & 0xFF;        // 低字节
  // 云台电机0速度 Speed_0 放入 buf[12] 和 buf[13]
  buf[12] = (moveBaseControl.Speed_0 >> 8) & 0xFF; // 高字节
  buf[13] = moveBaseControl.Speed_0 & 0xFF;        // 低字节
  // 云台电机1速度 Speed_1 放入 buf[14] 和 buf[15]
  buf[14] = (moveBaseControl.Speed_1 >> 8) & 0xFF; // 高字节
  buf[15] = moveBaseControl.Speed_1 & 0xFF;        // 低字节
  // 云台电机0时间 Time_0 放入 buf[16]
  buf[16] = static_cast<unsigned char>(moveBaseControl.Time_0);
  // 云台电机1时间 Time_1 放入 buf[17]
  buf[17] = static_cast<unsigned char>(moveBaseControl.Time_1);
  // 功能模式
  buf[18] = static_cast<unsigned char>(moveBaseControl.Fun);
  // 使用ID
  buf[19] = static_cast<unsigned char>(moveBaseControl.Orin_ID);
  // 重置ID
  buf[20] = static_cast<unsigned char>(moveBaseControl.Set_ID_Num);
  // std::cout << " moveBaseControl.Position_0  =" << moveBaseControl.Position_0 << " moveBaseControl.Position_1 = " << moveBaseControl.Position_1 << std::endl;

  // 预留位 Reservel
  buf[21] = static_cast<unsigned char>(moveBaseControl.VoiceSwitch);
  // 校验和
  unsigned char sum = 0;
  for (int i = 2; i < 22; ++i) // 从 buf[2] 到 buf[18] 计算校验和
    sum += buf[i];
  buf[22] = sum;

  try
  {
    Stm32_Serial.write(buf, 23); // 发送22字节的数据
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to send data through serial port"); // 如果发送数据失败，打印错误信息
  }
}

void turn_on_robot::imu_bias_correction(const sensor_msgs::msg::Imu &raw_imu,
                                        float &gx, float &gy, float &gz,
                                        float &ax, float &ay, float &az)
{
  static int bias_sample_count = 0;
  static float gyro_bias_sum_x = 0, gyro_bias_sum_y = 0, gyro_bias_sum_z = 0;
  static float accel_bias_sum_x = 0, accel_bias_sum_y = 0, accel_bias_sum_z = 0;

  if (!bias_calibrated)
  {
    // 累积零偏
    gyro_bias_sum_x += raw_imu.angular_velocity.x;
    gyro_bias_sum_y += raw_imu.angular_velocity.y;
    gyro_bias_sum_z += raw_imu.angular_velocity.z;
    accel_bias_sum_x += raw_imu.linear_acceleration.x;
    accel_bias_sum_y += raw_imu.linear_acceleration.y;
    accel_bias_sum_z += raw_imu.linear_acceleration.z;

    bias_sample_count++;

    // 样本够了就计算零偏
    if (bias_sample_count >= BIAS_CALIBRATION_SAMPLES)
    {
      gyro_bias_x = gyro_bias_sum_x / BIAS_CALIBRATION_SAMPLES;
      gyro_bias_y = gyro_bias_sum_y / BIAS_CALIBRATION_SAMPLES;
      gyro_bias_z = gyro_bias_sum_z / BIAS_CALIBRATION_SAMPLES;
      accel_bias_x = accel_bias_sum_x / BIAS_CALIBRATION_SAMPLES;
      accel_bias_y = accel_bias_sum_y / BIAS_CALIBRATION_SAMPLES;
      accel_bias_z = (accel_bias_sum_z / BIAS_CALIBRATION_SAMPLES) - 9.81f; // Z轴减重力

      bias_calibrated = true;
    }

    // 零偏未完成时输出原始数据（或0）
    gx = gy = gz = 0;
    ax = ay = az = 0;
  }
  else
  {
    // 零偏已完成，进行矫正
    gx = raw_imu.angular_velocity.x - gyro_bias_x;
    gy = raw_imu.angular_velocity.y - gyro_bias_y;
    gz = raw_imu.angular_velocity.z - gyro_bias_z;

    ax = raw_imu.linear_acceleration.x - accel_bias_x;
    ay = raw_imu.linear_acceleration.y - accel_bias_y;
    az = raw_imu.linear_acceleration.z - accel_bias_z;
  }
}

void turn_on_robot::Publish_ImuSensor_filter()
{
  sensor_msgs::msg::Imu Imu_Filter_Data_Pub;
  Imu_Filter_Data_Pub.header.stamp = this->now();
  Imu_Filter_Data_Pub.header.frame_id = gyro_frame_id;

  // 调用零偏校正函数
  float gx, gy, gz, ax, ay, az;
  imu_bias_correction(BMI088, gx, gy, gz, ax, ay, az);

  if (bias_calibrated)
  {
    // ==================== 校正完成后的正常处理 ====================
    // 零偏矫正
    float gx = BMI088.angular_velocity.x - gyro_bias_x;
    float gy = BMI088.angular_velocity.y - gyro_bias_y;
    float gz = BMI088.angular_velocity.z - gyro_bias_z;
    float ax = BMI088.linear_acceleration.x - accel_bias_x;
    float ay = BMI088.linear_acceleration.y - accel_bias_y;
    float az = BMI088.linear_acceleration.z - accel_bias_z;

    // 应用滤波
    filter_imu_data(gx, gy, gz, ax, ay, az);

    // 使用校正+滤波后的数据E
    Imu_Filter_Data_Pub.angular_velocity.x = gx;
    Imu_Filter_Data_Pub.angular_velocity.y = gy;
    Imu_Filter_Data_Pub.angular_velocity.z = gz;

    Imu_Filter_Data_Pub.linear_acceleration.x = ax;
    Imu_Filter_Data_Pub.linear_acceleration.y = ay;
    Imu_Filter_Data_Pub.linear_acceleration.z = az;

    Imu_Filter_Data_Pub.orientation = Quaternion_Solution(gx, gy, gz, ax, ay, az);

    if (montion_flag) // 运动状态
    {
      // ----------------- angular_velocity_covariance (rad/s)^2 -----------------
      Imu_Filter_Data_Pub.angular_velocity_covariance[0] = 1e6;
      Imu_Filter_Data_Pub.angular_velocity_covariance[4] = 1e6;
      Imu_Filter_Data_Pub.angular_velocity_covariance[8] = 1e-6;

      Imu_Filter_Data_Pub.linear_acceleration_covariance[0] = 1e-6;
      Imu_Filter_Data_Pub.linear_acceleration_covariance[4] = 1e-6;
      Imu_Filter_Data_Pub.linear_acceleration_covariance[8] = 1e6;

      // ----------------- orientation_covariance -----------------
      // 保留原来的设置，roll/pitch可信，yaw不可信
      Imu_Filter_Data_Pub.orientation_covariance[0] = 1e6;
      Imu_Filter_Data_Pub.orientation_covariance[4] = 1e6;
      Imu_Filter_Data_Pub.orientation_covariance[8] = 1e-6;
    }
    else // 静止状态
    {
      Imu_Filter_Data_Pub.angular_velocity_covariance[0] = 1e6;
      Imu_Filter_Data_Pub.angular_velocity_covariance[4] = 1e6;
      Imu_Filter_Data_Pub.angular_velocity_covariance[8] = 1e6;

      Imu_Filter_Data_Pub.linear_acceleration_covariance[0] = 1e6;
      Imu_Filter_Data_Pub.linear_acceleration_covariance[4] = 1e6;
      Imu_Filter_Data_Pub.linear_acceleration_covariance[8] = 1e6;
      Imu_Filter_Data_Pub.orientation_covariance[0] = 1e6;
      Imu_Filter_Data_Pub.orientation_covariance[4] = 1e6;
      Imu_Filter_Data_Pub.orientation_covariance[8] = 1e6;
    }
    pub_diff_filter->publish(Imu_Filter_Data_Pub);
  }

}

/**************************************
Date: January 28, 2021
Function: Publish the IMU data topic
功能: 发布IMU数据话题
***************************************/
void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::msg::Imu Imu_Data_Pub;
  Imu_Data_Pub.header.stamp = this->now();
  Imu_Data_Pub.header.frame_id = gyro_frame_id;

  Imu_Data_Pub.angular_velocity.x = BMI088.angular_velocity.x;
  Imu_Data_Pub.angular_velocity.y = BMI088.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = BMI088.angular_velocity.z;

  Imu_Data_Pub.linear_acceleration.x = BMI088.linear_acceleration.x;
  Imu_Data_Pub.linear_acceleration.y = BMI088.linear_acceleration.y;
  Imu_Data_Pub.linear_acceleration.z = BMI088.linear_acceleration.z;

  // 使用原始数据计算四元数
  Imu_Data_Pub.orientation = Quaternion_Solution(BMI088.angular_velocity.x, BMI088.angular_velocity.y, BMI088.angular_velocity.z,
                                                 BMI088.linear_acceleration.x, BMI088.linear_acceleration.y, BMI088.linear_acceleration.z);

  // 根据 Allan 标定结果设置协方差矩阵
  if (montion_flag) // 运动状态
  {
    // 姿态协方差 - 运动时 z 轴（偏航角）更可信
    Imu_Data_Pub.orientation_covariance[0] = 1e-2; // roll 方差
    Imu_Data_Pub.orientation_covariance[4] = 1e-2; // pitch 方差
    Imu_Data_Pub.orientation_covariance[8] = 1e-6; // yaw 方差（更可信）

    // 角速度协方差 - 使用 Allan 标定结果
    Imu_Data_Pub.angular_velocity_covariance[0] = pow(1.0650280329456384e-03, 2); // x轴
    Imu_Data_Pub.angular_velocity_covariance[4] = pow(1.2158208736495688e-03, 2); // y轴
    Imu_Data_Pub.angular_velocity_covariance[8] = pow(8.5126141040866936e-04, 2); // z轴

    // 加速度协方差 - 使用 Allan 标定结果
    Imu_Data_Pub.linear_acceleration_covariance[0] = pow(1.3023515631060851e-02, 2); // x轴
    Imu_Data_Pub.linear_acceleration_covariance[4] = pow(1.3648427512323884e-02, 2); // y轴
    Imu_Data_Pub.linear_acceleration_covariance[8] = pow(3.9736805938079857e-02, 2); // z轴
  }
  else // 静止状态
  {
    // 静止时姿态不太可信
    Imu_Data_Pub.orientation_covariance[0] = 1e-1;
    Imu_Data_Pub.orientation_covariance[4] = 1e-1;
    Imu_Data_Pub.orientation_covariance[8] = 1e-1;

    // 角速度协方差保持一致
    Imu_Data_Pub.angular_velocity_covariance[0] = pow(1.0650280329456384e-03, 2);
    Imu_Data_Pub.angular_velocity_covariance[4] = pow(1.2158208736495688e-03, 2);
    Imu_Data_Pub.angular_velocity_covariance[8] = pow(8.5126141040866936e-04, 2);

    // 加速度协方差
    Imu_Data_Pub.linear_acceleration_covariance[0] = pow(1.3023515631060851e-02, 2);
    Imu_Data_Pub.linear_acceleration_covariance[4] = pow(1.3648427512323884e-02, 2);
    Imu_Data_Pub.linear_acceleration_covariance[8] = pow(3.9736805938079857e-02, 2);
  }

  imu_publisher->publish(Imu_Data_Pub);
  std_msgs::msg::UInt8 imu_msg_valid;
  imu_msg_valid.data = 1;
  pub_imu_msg_valid->publish(imu_msg_valid);
}

double getYaw(geometry_msgs::msg::PoseStamped pose)
{
  return tf2::getYaw(pose.pose.orientation);
}

/**************************************
Date: January 28, 2021
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/
void turn_on_robot::Publish_Odom()
{
  // Convert the Z-axis rotation Angle into a quaternion for expression
  // 将Z轴转角转换为四元数进行表达
  if ((std::isnan(Robot_Pos.X)) || (std::isnan(Robot_Pos.Y)) || (std::isnan(Robot_Pos.Z)))
  {
    RCLCPP_INFO(this->get_logger(), "Data no effect!");
    return;
  }

  tf2::Quaternion odom_quat;
  odom_quat.setRPY(0, 0, Robot_Pos.Z); // Convert the Z-axis rotation to a quaternion

  // Create odometry message
  nav_msgs::msg::Odometry odom;                 // Instance the odometer topic data
  odom.header.stamp = this->get_clock()->now(); // ROS2 time
  odom.header.frame_id = odom_frame_id;         // Odometer TF parent coordinates
  odom.pose.pose.position.x = Robot_Pos.X;      // Position
  odom.pose.pose.position.y = Robot_Pos.Y;
  odom.pose.pose.position.z = 0.2;
  odom.pose.pose.orientation = tf2::toMsg(odom_quat); // Quaternion converted by Z-axis rotation

  odom.child_frame_id = robot_frame_id;     // Odometer TF subcoordinates
  odom.twist.twist.linear.x = linear_Speed * 50; // Speed in the X direction
  odom.twist.twist.linear.y = 0.0;          // Speed in the Y direction
  odom.twist.twist.angular.z = ThetaSpeed * 50;  // Angular velocity around the Z axis

  // Two types of covariance matrices used for robot pose and twist covariance
  if (Robot_Vel.Left == 0 && Robot_Vel.Right == 0)
    // If the velocity is zero, consider encoder data as more reliable
    std::memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
        std::memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
  else
    // If velocity is non-zero, consider IMU data as more reliable
    std::memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
        std::memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));

  // Publish the odometry message
  odom_publisher->publish(odom);

  std_msgs::msg::UInt8 odom_msg_valid;
  odom_msg_valid.data = 1;
  pub_odom_msg_valid->publish(odom_msg_valid);

  // Publish the transform (TF2 equivalent)
  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = this->get_clock()->now();
  odom_trans.header.frame_id = odom_frame_id;
  odom_trans.child_frame_id = robot_frame_id;

  odom_trans.transform.translation.x = Robot_Pos.X;
  odom_trans.transform.translation.y = Robot_Pos.Y;
  odom_trans.transform.translation.z = 0.10;
  odom_trans.transform.rotation = tf2::toMsg(odom_quat);

  // Publish the transform using tf2
  // tf_broadcaster_->sendTransform(odom_trans);
}

/**************************************
Date: January 28, 2021
Function: Publish voltage-related information
功能: 发布电压相关信息
***************************************/
void turn_on_robot::Publish_Voltage()
{
  std_msgs::msg::Float32 voltage_msgs;
  voltage_msgs.data = Power_voltage;        // The power supply voltage is obtained //电源供电的电压获取
  voltage_publisher->publish(voltage_msgs); // Post the power supply voltage topic unit: V, volt //发布电源电压话题单位：V、伏特
  // printf("voltage_publisher->publish=%.5f\n",Power_voltage);
}

/**************************************
Date: February 16, 2021
Function: Publish Battery_Percentage information
功能: 发布电量百分比信息
***************************************/
double v;
bool flag = false;
int N = 10;
int Index = 0;
std::vector<double> xx(N, 0);
void turn_on_robot::Publish_Battery_Percentage()
{
  v = Power_voltage;
  // v = 13;

  float x;

  if (v >= Power_max)
  {
    x = 1;
  }
  else if (v > Power_min)
  {
    x = (v - Power_min) / (Power_max - Power_min);
  }
  else if (v <= Power_min)
  {
    x = 0;
  }

  xx[Index] = x;
  ++Index;
  Index %= N;

  // std::time_t nowtime = std::time(NULL) - starttime; // 获取时间戳
  // out_txt_file.open("power_voltage.txt", ios::out | ios::trunc);
  // out_txt_file << fixed;
  // out_txt_file << nowtime;
  // out_txt_file << " ";
  // out_txt_file << setprecision(4) << v;
  // out_txt_file << endl;
  // std::cout << "nowtime= " << nowtime << std::endl;
  if (count_A < 100)
    count_A++;

  float temp = 0;
  for (int i = 0; i < N; ++i)
    temp += xx[i];

  double cur_Battery_Percentage;
  cur_Battery_Percentage = temp / N; // 设置数据字段
  if (cur_Battery_Percentage > 1)
    cur_Battery_Percentage = 1;
  // printf("cur_Battery_Percentage=%.2f,--count_A=%d\n",cur_Battery_Percentage,count_A);
  // 保证电量百分比稳定下降，上升时只有连续比历史值高100次，才上升
  if (count_A > 30)
  {
    if (cur_Battery_Percentage > last_Battery_Percentage)
    {
      count_B++;
      if (count_B > 100)
      {
        last_Battery_Percentage = cur_Battery_Percentage;
        count_B = 0;
        count_C = 0;
      }
    }
    else
      count_B = 0;

    if (count_B > 200)
      count_B = 0;
    // 下降时低10次就下降
    if (cur_Battery_Percentage < last_Battery_Percentage)
    {
      count_C++;
      if (count_C > 10)
      {
        last_Battery_Percentage = cur_Battery_Percentage;
        count_B = 0;
        count_C = 0;
      }
    }
    else
      count_C = 0;
  }
  else if (last_Battery_Percentage < cur_Battery_Percentage)
  {
    last_Battery_Percentage = cur_Battery_Percentage;
  }

  Battery_Percentage_msgs.data = last_Battery_Percentage;

  Battery_Percentage_pub->publish(Battery_Percentage_msgs); // 发布电池电量百分比
  // printf("last_Battery_Percentage=%f\n",last_Battery_Percentage);

  // std::cout << "电压" << v << "==>"
  //           << "电量百分比： " << Battery_Percentage_msgs.data << std::endl;
}

/**************************************
Date: January 28, 2021
Function: Publish the camera motor feedback topic, Contains position, speed, and load data for both motors
功能: 发布云台电机反馈话题，包含两个电机的位置、速度和负载数据
***************************************/
void turn_on_robot::Publish_CameMo()
{
  std_msgs::msg::Int32MultiArray camemo_msg;
  
  // 设置数据布局
  camemo_msg.layout.dim.resize(1);
  camemo_msg.layout.dim[0].label = "camemo_data";
  camemo_msg.layout.dim[0].size = 6;  // 6个数据：Position_0, Position_1, Speed_0, Speed_1, Load_0, Load_1
  camemo_msg.layout.dim[0].stride = 6;
  camemo_msg.layout.data_offset = 0;
  
  // 填充数据：位置、速度、负载
  camemo_msg.data.resize(6);
  camemo_msg.data[0] = curYuntai_feedback_data.Position_0; // 云台电机0位置
  camemo_msg.data[1] = curYuntai_feedback_data.Position_1; // 云台电机1位置
  camemo_msg.data[2] = curYuntai_feedback_data.Speed_0;   // 云台电机0速度
  camemo_msg.data[3] = curYuntai_feedback_data.Speed_1;   // 云台电机1速度
  camemo_msg.data[4] = curYuntai_feedback_data.Load_0;    // 云台电机0负载
  camemo_msg.data[5] = curYuntai_feedback_data.Load_1;    // 云台电机1负载
  
  // 发布云台电机反馈数据
  pub_camemo->publish(camemo_msg);
}



/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BCC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/

unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
  unsigned char check_sum = 0, k;

  if (mode == 0) // Receive data mode //接收数据模式
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Receive_Data.rx[k]; // By bit or by bit //按位异或
    }
  }
  if (mode == 1) // Send data mode //发送数据模式
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Send_Data.tx[k]; // By bit or by bit //按位异或
    }
  }
  return check_sum; // Returns the bitwise XOR result //返回按位异或结果
}

/**************************************
Date: November 18, 2021
Function: Read and verify the data sent by the lower computer frame by frame through the serial port, and then convert the data into international units
功能: 通过串口读取并逐帧校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/
bool turn_on_robot::Get_Sensor_Data_New()
{
  // printf("Get_Sensor_Data_New success 1 \n");

  uint8_t head_Receive_Data[1] = {0}; // 临时变量，保存下位机数据
  while (1)
  {
    Stm32_Serial.read(head_Receive_Data, 1); // 通过串口读取下位机发送的数据
    if (head_Receive_Data[0] == 0x7b)
      break;
  }

  // printf("1\n");
  // if (head_Receive_Data[0] != 0x7b)
  //   return false;

  uint8_t len_Receive_Data[1] = {0};      // 临时变量，保存下位机数据
  Stm32_Serial.read(len_Receive_Data, 1); // 通过串口读取下位机发送的数据

  if (len_Receive_Data[0] != 0x1f)
    return false;

  // 修复有符号/无符号比较问题
  short transition_16 = 0;
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE - 2] = {0};        // 临时变量，保存下位机数据
  Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr)); // 通过串口读取下位机发送的数据

  Receive_Data.rx[0] = head_Receive_Data[0];
  Receive_Data.rx[1] = len_Receive_Data[0];
  for (size_t j = 0; j < sizeof(Receive_Data_Pr); j++)
  {
    Receive_Data.rx[j + 2] = Receive_Data_Pr[j];
  }

  // for (int kk = 0; kk < sizeof(Receive_Data_Pr); kk++)
  // {
  //   printf("[%d]=%02x,", kk, Receive_Data_Pr[kk]);
  // }
  // printf(" \n");
  // for (int kk = 0; kk < sizeof(Receive_Data.rx); kk++)
  // {
  //   printf("[%d]=%02x,", kk, Receive_Data.rx[kk]);
  // }
  // printf(" -----------------------------\n");

  // 校验和验证
  // if (Receive_Data.rx[33] != Check_Sum(33, READ_DATA_CHECK))
  // {
  //   printf("Check_Sum false  ----------\n");
  //   return false; // 校验和错误
  // }

  // 开始解析各个数据字段

  // 解析左轮和右轮速度

  Robot_Vel.Left = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);  // 左轮速度
  Robot_Vel.Right = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]); // 右轮速度

  // 解析电池电压
  transition_16 = 0;
  transition_16 |= Receive_Data.rx[6] << 8;
  transition_16 |= Receive_Data.rx[7];
  Power_voltage = transition_16 / 1000.0; // 单位转换为伏特

  // 解析IMU加速度数据
  BMI088_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);   // X轴加速度
  BMI088_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]); // Y轴加速度
  BMI088_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]); // Z轴加速度

  BMI088_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]); // Z轴加速度

  // 解析陀螺仪数据
  BMI088_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]); // X轴角速度
  BMI088_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]); // Y轴角速度
  BMI088_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]); // Z轴角速度

  BMI088_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]); // Y轴角速度
  BMI088_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]); // Z轴角速度

  // 加速度和角速度的单位转换
  BMI088.linear_acceleration.x = BMI088_Data.accele_x_data * 24 / 32768.0f * 9.8;
  BMI088.linear_acceleration.y = BMI088_Data.accele_y_data * 24 / 32768.0f * 9.8;
  BMI088.linear_acceleration.z = BMI088_Data.accele_z_data * 24 / 32768.0f * 9.8;

  BMI088.angular_velocity.x = BMI088_Data.gyros_x_data * 1000 / 32768.0f / 57.2957795f;
  BMI088.angular_velocity.y = BMI088_Data.gyros_y_data * 1000 / 32768.0f / 57.2957795f;
  BMI088.angular_velocity.z = BMI088_Data.gyros_z_data * 1000 / 32768.0f / 57.2957795f;

  // 解析位置、速度和负载数据
  // 位置 1
  curYuntai_feedback_data.Position_0 = Odom_Trans(Receive_Data.rx[20], Receive_Data.rx[21]);
  // 速度 1
  curYuntai_feedback_data.Speed_0 = Odom_Trans(Receive_Data.rx[22], Receive_Data.rx[23]);
  // 负载 1
  curYuntai_feedback_data.Load_0 = Odom_Trans(Receive_Data.rx[24], Receive_Data.rx[25]);
  // printf("Position 1: %d, Speed 1: %d, Load 1: %d\n", curYuntai_feedback_data.Position_0, curYuntai_feedback_data.Speed_0, curYuntai_feedback_data.Load_0);

  // 位置 2
  curYuntai_feedback_data.Position_1 = Odom_Trans(Receive_Data.rx[26], Receive_Data.rx[27]);
  // 速度 2
  curYuntai_feedback_data.Speed_1 = Odom_Trans(Receive_Data.rx[28], Receive_Data.rx[29]);
  // 负载 2
  curYuntai_feedback_data.Load_1 = Odom_Trans(Receive_Data.rx[30], Receive_Data.rx[31]);
  // printf("Position 2: %d, Speed 2: %d, Load 2: %d\n", curYuntai_feedback_data.Position_1, curYuntai_feedback_data.Speed_1, curYuntai_feedback_data.Load_1);

  // 处理预留位
  // Receive_Data.Flag_Reserved = Receive_Data.rx[32]; // 预留位
  // 处理预留位
  uint8_t rx32 = Receive_Data.rx[32];
  // std::cout << "Receive_Data.rx[32] = "
  //           << int(Receive_Data.rx[32])
  //           << std::endl;

  int flage_voice = detect_toggle(rx32);

  if (flage_voice == 1)
  {
      std::cout << "Trigger! rx32 changed to: "
                << int(rx32) << std::endl;
    std_msgs::msg::Int32 wakeup_ack_data;
    wakeup_ack_data.data = 1;
    wakeup_ack_pub_->publish(wakeup_ack_data);
  }

  // 最后返回true，表示数据解析成功
  return true;
}

int turn_on_robot::detect_toggle(uint8_t new_value)
{
    static uint8_t last_value = 0;   // 上一帧的值
    int flag = 0;

    if (new_value != last_value)
    {
        flag = 1;              // 有跳变
        last_value = new_value; // 更新历史值
    }
    else
    {
        flag = 0;              // 无跳变
    }

    return flag;
}

/**
 * @description: The function normalizes angle values.
 * @param {double} angle
 * @return {double}  angle
 * @author: Senerity
 */
double turn_on_robot::normalizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

turn_on_robot::turn_on_robot(std::string name) : Node(name)
{
  RCLCPP_INFO(this->get_logger(), "创建节点成功..........");

  // Clear the data
  // 清空数据
  linear_Speed = 0;
  ThetaSpeed = 0;
  ticksPerMeter = 0;
  ticksPer2PI = 0;
  leftDistance = 0;
  rightDistance = 0;
  calibrate_lineSpeed = 0;
  last_Battery_Percentage = 0;
  count_B = 0;
  count_A = 0;
  count_C = 0;

  Power_max = 12;
  Power_min = 10;
  control_mode = 1;
  last_control_mode = 1;

  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&BMI088_Data, 0, sizeof(BMI088_Data));
  memset(&moveBaseControl, 0, sizeof(sMartcarControl));
  moveBaseControl.Position_0 = 2047;
  moveBaseControl.Position_1 = 2047;
  moveBaseControl.Speed_0 = 200;
  moveBaseControl.Speed_1 = 200;
  moveBaseControl.Time_0 = 0;
  moveBaseControl.Time_1 = 0;

  voice_nums.resize(2);
  voice_nums[0] = 0;
  voice_nums[1] = 0;
  lastvoice_num_1 = 0;


  // 声明参数并设置默认值
  this->declare_parameter("serial_port_name", "/dev/ttyUSB0");
  this->declare_parameter("serial_baud_rate", 115200);
  this->declare_parameter("odom_frame_id", "odom");
  this->declare_parameter("robot_frame_id", "base_link");
  this->declare_parameter("gyro_frame_id", "imu_link");
  this->declare_parameter("calibrate_lineSpeed", 1);
  this->declare_parameter("ticksPerMeter", 1000.0);
  this->declare_parameter("ticksPer2PI", 3200.0);
  this->declare_parameter("smooth_window_speed", 20);
  this->declare_parameter("smooth_window_angle", 5);

  // 获取参数值
  this->get_parameter("serial_port_name", serial_port_name);
  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);
  this->get_parameter("calibrate_lineSpeed", calibrate_lineSpeed);
  this->get_parameter("ticksPerMeter", ticksPerMeter);
  this->get_parameter("ticksPer2PI", ticksPer2PI);
  this->get_parameter("smooth_window_speed", smooth_window_speed);
  this->get_parameter("smooth_window_angle", smooth_window_angle);



  voltage_publisher = this->create_publisher<std_msgs::msg::Float32>("PowerVoltage", 10);            // Create a battery-voltage topic publisher //创建电池电压话题发布者
  Battery_Percentage_pub = this->create_publisher<std_msgs::msg::Float32>("Battery_Percentage", 10); // Create a battery-voltage topic publisher //创建电池电量百分比话题发布者
  odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);                      // Create the odometer topic publisher //创建里程计话题发布者
  imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("raw", 20);                          // Create an IMU topic publisher //创建IMU话题发布者
  pub_imu_msg_valid = this->create_publisher<std_msgs::msg::UInt8>("imu_msg_valid", 10);
  pub_odom_msg_valid = this->create_publisher<std_msgs::msg::UInt8>("odom_msg_valid", 10);
  pub_diff_filter = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 1);
  pub_navi_cmd = this->create_publisher<std_msgs::msg::UInt8>("navi_cmd", 10);
  pub_current_vel = this->create_publisher<geometry_msgs::msg::Twist>("current_vel_state", 10);
  wakeup_ack_pub_ = this->create_publisher<std_msgs::msg::Int32>("wakeup_signal", 10);
  pub_camemo = this->create_publisher<std_msgs::msg::Int32MultiArray>("camemo_feedback", 10);

  // Set the velocity control command callback function
  // 速度控制命令订阅回调函数设置
  Cmd_Vel_Sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(), std::bind(&turn_on_robot::callback_cmd_vel_angle, this, std::placeholders::_1));
  sub_movebase_angle = this->create_subscription<geometry_msgs::msg::Twist>("pursuitAngle", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&turn_on_robot::callback_movebase_angle, this, std::placeholders::_1));
  sub_logicontrol = this->create_subscription<geometry_msgs::msg::Twist>("logicontrol", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&turn_on_robot::callback_logi_control, this, std::placeholders::_1));
  sub_voice_switch = this->create_subscription<std_msgs::msg::UInt8>("voice_switch", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&turn_on_robot::callback_voice_switch, this, std::placeholders::_1));
  sub_vehicle_state = this->create_subscription<std_msgs::msg::UInt8>("vehicle_state", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&turn_on_robot::callback_vehicle_state, this, std::placeholders::_1));
  sub_vehicle_Drivestate = this->create_subscription<std_msgs::msg::UInt8>("vehicle_Drivestate", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&turn_on_robot::callback_vehicle_Drivestate, this, std::placeholders::_1));
  sub_CameMo_Conl = this->create_subscription<std_msgs::msg::Int32MultiArray>("CameMo_Conl", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&turn_on_robot::callback_CameMo_Conl, this, std::placeholders::_1));
  sub_stopCMD = this->create_subscription<std_msgs::msg::UInt8>("StopCmd", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&turn_on_robot::callback_stopCMD, this, std::placeholders::_1));

  
  // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  
  RCLCPP_INFO(this->get_logger(), "Data ready");

  std::cout << FRED("Copyright©2016-2020 dzactuator. All rights reserved ") << std::endl;
  std::cout << FYEL("*****dzactuator:parameters*******************") << std::endl;
  std::cout << FGRN("serial_port_name:") << serial_port_name << std::endl;
  std::cout << FGRN("serial_baud_rate:") << serial_baud_rate << std::endl;
  std::cout << FGRN("calibrate_lineSpeed:") << calibrate_lineSpeed << std::endl;
  std::cout << FGRN("ticksPerMeter:") << ticksPerMeter << std::endl;
  std::cout << FGRN("ticksPer2PI:") << ticksPer2PI << std::endl;
  std::cout << FGRN("smooth_window_speed:") << smooth_window_speed << std::endl;
  std::cout << FGRN("smooth_window_angle:") << smooth_window_angle << std::endl;
  std::cout << FYEL("*****dzactuator:parameters end***************") << std::endl;

  // RCLCPP_ERROR_STREAM(this->get_logger(), "dzactuator can not open serial port,Please check the serial port cable! "); // If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  try
  {
    // Attempts to initialize and open the serial port //尝试初始化与开启串口
    Stm32_Serial.setPort(serial_port_name);                     // Select the serial port number to enable //选择要开启的串口号
    Stm32_Serial.setBaudrate(serial_baud_rate);                 // Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(30); // Timeout //超时等待
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open(); // Open the serial port //开启串口
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "dzactuator can not open serial port,Please check the serial port cable! "); // If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if (Stm32_Serial.isOpen())
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "dzactuator serial port opened"); // Serial port opened successfully //串口开启成功提示
  }

  // boost::thread Control_Thread;
  // Control_Thread = boost::thread(boost::bind(&turn_on_robot::Control, this));
  // printf("绑定线程函数\n");

  
  mode_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 10Hz
    std::bind(&turn_on_robot::modeTimerCallback, this)
  );
  if (last_pursuit_high_time_.nanoseconds() == 0)
  last_pursuit_high_time_ = this->get_clock()->now();

}

turn_on_robot::~turn_on_robot()
{
  // Sends the stop motion command to the lower machine before the turn_on_robot object ends
  // 对象turn_on_robot结束前向下位机发送停止运动命令
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;
  Send_Data.tx[2] = 0;

  // The target velocity of the X-axis of the robot //机器人X轴的目标线速度
  Send_Data.tx[4] = 0;
  Send_Data.tx[3] = 0;

  // The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;

  // The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度
  Send_Data.tx[8] = 0;
  Send_Data.tx[7] = 0;
  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data.tx[10] = FRAME_TAIL;
  try
  {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); // Send data to the serial port //向串口发数据
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to send data through serial port"); // If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
  }
  Stm32_Serial.close();                                    // Close the serial port //关闭串口
  RCLCPP_INFO_STREAM(this->get_logger(), "Shutting down"); // Prompt message //提示信息
}

void turn_on_robot::Control()
{
  // Initialize position to zero
  Robot_Pos.X = 0.0;
  Robot_Pos.Y = 0.0;
  Robot_Pos.Z = 0.0;
  leftDistance = 0;
  rightDistance = 0;
  change_start_time= this->now();

  // ROS2 time handling
  // rclcpp::Time current_time, last_time;
  // last_time = this->get_clock()->now();

  rclcpp::Rate rate(100); // Loop rate of 50Hz
  while (rclcpp::ok())
  {
    rclcpp::spin_some(shared_from_this());

    if (true == Get_Sensor_Data_New()) // Read and verify data from the lower computer
    {
      leftDistance += Robot_Vel.Left;
      rightDistance += Robot_Vel.Right;
      // // Calculate speed for left and right wheels
      double leftSpeed = Robot_Vel.Left / ticksPerMeter;
      double rightSpeed = Robot_Vel.Right / ticksPerMeter;

      linear_Speed = (leftSpeed + rightSpeed) / 2.0;
      ThetaSpeed = (Robot_Vel.Right - Robot_Vel.Left) * (2 * M_PI) / ticksPer2PI;

      if (linear_Speed > 0.00001 || linear_Speed < -0.000001)
      {
        montion_flag = true;
        Robot_Pos.Z += ThetaSpeed;                      // Update angular displacement (Z-axis)
        Robot_Pos.Z = normalizeAngle(Robot_Pos.Z);      // Normalize angle
        Robot_Pos.X += linear_Speed * cos(Robot_Pos.Z); // Calculate X displacement
        Robot_Pos.Y += linear_Speed * sin(Robot_Pos.Z); // Calculate Y displacement

        //         Robot_Pos.Z = normalizeAngle(Robot_Pos.Z); // Normalize angle
      }
      else
      {
        montion_flag = false;
      }
      // std::cout << "Robot_Pos.X " << Robot_Pos.X << "Robot_Pos.Y " << Robot_Pos.Y << "Robot_Pos.Z " << Robot_Pos.Z << std::endl;

      Publish_Odom(); // Publish odometry
      Publish_CameMo(); // Publish camera motor feedback

      if (calibrate_lineSpeed == 1)
      {
        RCLCPP_INFO(this->get_logger(),
                    "left=%.2f, right=%.2f, x=%.2f, y=%.2f, th=%.2f, linear_Speed=%f, leftDistance=%.2f, rightDistance=%.2f, Power_voltage=%.2f",
                    Robot_Vel.Left, Robot_Vel.Right, Robot_Pos.X, Robot_Pos.Y, Robot_Pos.Z, linear_Speed, leftDistance, rightDistance, Power_voltage);
      }

      // Publish IMU data
      Publish_ImuSensor();
      Publish_ImuSensor_filter();
      Publish_Voltage();
      Publish_Battery_Percentage();

      switchvoice();
      // Control camera and send info to the kernel
      sendCarInfoKernel();
    }
    rate.sleep(); // Sleep to maintain rate of 50Hz
  }
}


void turn_on_robot::modeTimerCallback()
{
  rclcpp::Time now = this->get_clock()->now();

  bool dwa_active = (last_dwa_msg_time_.nanoseconds() > 0) &&
                    ((now - last_dwa_msg_time_).seconds() < ACTIVITY_TIMEOUT);
  bool pursuit_active = (last_pursuit_msg_time_.nanoseconds() > 0) &&
                        ((now - last_pursuit_msg_time_).seconds() < ACTIVITY_TIMEOUT);

  // 状态打印
  // RCLCPP_INFO_THROTTLE(
  //     this->get_logger(), *this->get_clock(), 1000,
  //     "[STATUS] dwa:%s  pursuit:%s  mode:%s",
  //     dwa_active ? "ON" : "OFF",
  //     pursuit_active ? "ON" : "OFF",
  //     (current_mode_ == 1 ? "DWA" : (current_mode_ == 2 ? "Pursuit" : "Unknown")));


  if (!dwa_active && !pursuit_active){
    return;
  }
  
  // 单源活跃逻辑保持原样
  if (!dwa_active && pursuit_active)
  {
    if (current_mode_ != 2)
    {
      current_mode_ = 2;
      // RCLCPP_INFO(this->get_logger(), "[MODE] → Pursuit (only pursuit active)");
    }
    return;
  }

  if (dwa_active && !pursuit_active)
  {
    if (current_mode_ != 1)
    {
      current_mode_ = 1;
      // RCLCPP_INFO(this->get_logger(), "[MODE] → DWA (only dwa active)");
    }
    return;
  }

  // 两者都活跃
  double pursuit_y = last_pursuit_msg_ ? last_pursuit_msg_->linear.y : 0.0;

  // Pursuit >=0.5 → DWA
  if (pursuit_y >= 0.5)
  {
    if (current_mode_ != 1)
    {
      current_mode_ = 1;
      // RCLCPP_INFO(this->get_logger(), "[MODE] → DWA (pursuit_y >= 0.5)");
    }
    return;
  }

  // Pursuit <0.5 超过3秒 且Pursuit速度大于 dwb→ Pursuit
  if (current_mode_ == 1 && last_pursuit_high_time_.nanoseconds() > 0  && last_pursuit_msg_->linear.x > last_dwb_msg_->linear.x )
  {
    // last_pursuit_msg_
    // last_dwb_msg_

    double dt = (now - last_pursuit_high_time_).seconds();
    if (dt >= PURSUIT_LOW_TIMEOUT)
    {
      current_mode_ = 2;
      // RCLCPP_INFO(this->get_logger(),
      //             "[MODE] → Pursuit (pursuit_y < 0.5 for %.1fs >= %.1fs)",
      //             dt, PURSUIT_LOW_TIMEOUT);
    }
  }
}