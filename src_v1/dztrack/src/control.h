
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <sstream>
#include <iostream>
#include <fstream>
#include <Eigen//Dense>
#include "Eigen/Eigen"

// ros2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/utils.h"

#include <tf2_ros/transform_listener.h>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/convert.h>
#include <nav2_util/robot_utils.hpp>
#include <rclcpp/time.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#define STOP_DIS 0.13
#define RST "\x1B[0m"
#define KRED "\x1B[31m"
#define KGRN "\x1B[32m"
#define KYEL "\x1B[33m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST

class control : public rclcpp::Node
{
public:

  control(std::string name);

  virtual ~control();

private:
  bool getRobotPose(geometry_msgs::msg::PoseStamped &global_pose);
  double distance(const geometry_msgs::msg::PoseStamped &p1, const geometry_msgs::msg::PoseStamped &p2);
  // tf2_ros::Buffer &tf_;

public:
  void doplanner();
  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_wait;
  std::string robot_base_frame_, global_frame_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_, action_goal_pub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_current_pose;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr angle_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr backcar_road_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_vehicle_state;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr pub_parkingOccupation;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_vehicle_Drivestate;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr all_plan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav2_goal_pub_;


  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_trafficStatus;
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_parkingSpaceStatusArray;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lasersScan;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_saveMapping;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_stopCMD;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl_pose;

  void callback_amcl_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void callback_trafficStatus(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void callback_lasersScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void callback_saveMapping(const std_msgs::msg::UInt8::SharedPtr msg);
  
  bool makePlan(const geometry_msgs::msg::PoseStamped &start, std::vector<geometry_msgs::msg::PoseStamped> &plan);
  void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> &path);
  void pub_stopP_marks();

  void map_to_baselink(Eigen::Vector3d p1, Eigen::Vector3d *p2);

  bool amcl_pose_flag;
  bool saveMapping;
  geometry_msgs::msg::PoseStamped last_pose;
  std::string savePath;
  FILE *fp;

  //
  int maxstopIndex;
  int LaststopIndex;
  double pursuitSpeed;
  float pubtargetSpeed;
  double target_min_dis;
  double far_min_dis;
  double reduct_ratio;

  std_msgs::msg::UInt8 vehicle_state_data;
  std_msgs::msg::UInt8 vehicle_Drivestate_data;


  geometry_msgs::msg::PoseStamped last_stop_pose;
  std::vector<geometry_msgs::msg::PoseStamped> stopPoints;
  // std::vector<geometry_msgs::msg::PoseStamped> forwardPoints;
  FILE *stopPointfp;
  std::string saveStopPointPath;
  geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose_msg;

  double angle_Kp;
  double diffangle_Kp;
  int NearestPoint_index_i;

  double dis_y;

  double NearestPoint_yaw;

  geometry_msgs::msg::PoseStamped currentstoppoint;
  sensor_msgs::msg::LaserScan currentScan;
  sensor_msgs::msg::PointCloud2 scancloud;
  laser_geometry::LaserProjection laserprojector;
  char recvScanFlag;
  int foundObstacleNum, nofoundObstacleNum;
  volatile int foundObstacleFlag;

  int pub_Num = 0;
  int minIndex = 0;
  int Last_lastIndex=0;
  int LastIndex=0;
  int maxIndex;
  double minDis;

  std_msgs::msg::UInt8 saveMappingMsg;
  std_msgs::msg::UInt8MultiArray trafficLightArray;

  void loadmap();

  int loadStopPoint();
  // int loadforwardPoint();

  int detectReachedGoal(const geometry_msgs::msg::PoseStamped &currentPose, geometry_msgs::msg::PoseStamped &stopPose);
  float sLineControl(geometry_msgs::msg::PoseStamped current_position, std::vector<geometry_msgs::msg::PoseStamped> temp_plan, float *pubSpeed);
  float foundNearestPoint(geometry_msgs::msg::PoseStamped current_position, std::vector<geometry_msgs::msg::PoseStamped> temp_plan);
  float lineDirection(float xsecond, float ysecond);
  float foundTargetAngle(float targetDis, float speed, geometry_msgs::msg::PoseStamped current_position, std::vector<geometry_msgs::msg::PoseStamped> temp_plan, float *pubSpeed);
  void obstacleDetect(std::vector<geometry_msgs::msg::PoseStamped> temp_plan);
  double getDis(geometry_msgs::msg::PoseStamped currentpose, geometry_msgs::msg::PoseStamped roadpose);
  //sub for multi_robor  stop_CMD

  std::vector<geometry_msgs::msg::PoseStamped> roadPoints;
  bool cout_hglocation;

  double obstacle_x_max_dis;
  double obstacle_y_max_dis;
  int vehicle_tar_orientation; // 0:直行，1左转，2右转
  double minDis_param;

  double save_mapping_point_dis;


  
};

#endif