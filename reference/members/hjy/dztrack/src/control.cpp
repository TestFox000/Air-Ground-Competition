
#include "control.h"

control::control(std::string name) : Node(name)
// control::control()
{

    // get some parameters that will be global to the move base node
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    LastIndex = 0;
    maxIndex = 0;
    minDis = 10000;
    target_min_dis = 0.5;
    Last_lastIndex = -999;
    pursuitSpeed = 0.3;
    far_min_dis = 0.2;
    reduct_ratio = 1.1;
    pubtargetSpeed = 0.6;
    recvScanFlag = 0;
    saveMappingMsg.data = 0;
    foundObstacleFlag = 0;
    minDis_param =0.2;

    amcl_pose_flag = false;
    cout_hglocation = false;
    vehicle_state_data.data = 0;
    vehicle_Drivestate_data.data = 0;

    currentstoppoint.pose.position.x = 8888;
    currentstoppoint.pose.position.y = 8888;
    currentstoppoint.pose.position.z = 0;
    currentstoppoint.pose.orientation.x = 0;
    currentstoppoint.pose.orientation.y = 0;
    currentstoppoint.pose.orientation.z = 0;
    currentstoppoint.pose.orientation.w = 1;

    saveMapping = false;

    robot_base_frame_ = "base_link";
    global_frame_ = "map";
    obstacle_x_max_dis = 0.8;
    obstacle_y_max_dis = 0.15;

    angle_Kp = 1;

    declare_parameter("angle_Kp", angle_Kp);
    get_parameter("angle_Kp", angle_Kp);
    std::cout << FGRN("angle_Kp: ") << angle_Kp << std::endl;

    diffangle_Kp = 0.14;
    declare_parameter("diffangle_Kp", diffangle_Kp);
    get_parameter("diffangle_Kp", diffangle_Kp);
    std::cout << FGRN("diffangle_Kp: ") << diffangle_Kp << std::endl;

    minDis_param = 0.2;
    declare_parameter("minDis_param", minDis_param);
    get_parameter("minDis_param", minDis_param);
    std::cout << FGRN("minDis_param: ") << minDis_param << std::endl;
    save_mapping_point_dis = 0.01;
    declare_parameter("save_mapping_point_dis", save_mapping_point_dis);
    get_parameter("save_mapping_point_dis", save_mapping_point_dis);
    std::cout << FGRN("save_mapping_point_dis: ") << save_mapping_point_dis << std::endl;

    savePath = "/home/duzhong/dzrobot/src/naviPointPath/mapping.txt";
    saveStopPointPath = "/home/duzhong/dzrobot/src/naviPointPath/stopmapping.txt";


    declare_parameter("saveMapping", saveMapping);
    declare_parameter("savePath", savePath);
    declare_parameter("saveStopPointPath", saveStopPointPath);
    declare_parameter("robot_base_frame", robot_base_frame_);
    declare_parameter("global_frame", global_frame_);
    declare_parameter("pursuitSpeed", pursuitSpeed);
    declare_parameter("far_min_dis", far_min_dis);
    declare_parameter("reduct_ratio", reduct_ratio);
    declare_parameter("target_min_dis", target_min_dis);
    declare_parameter("cout_hglocation", cout_hglocation);
    declare_parameter("obstacle_x_max_dis", obstacle_x_max_dis);
    declare_parameter("obstacle_y_max_dis", obstacle_y_max_dis);

    get_parameter("saveMapping", saveMapping);
    get_parameter("savePath", savePath);
    get_parameter("saveStopPointPath", saveStopPointPath);
    get_parameter("robot_base_frame", robot_base_frame_);
    get_parameter("global_frame", global_frame_);
    get_parameter("pursuitSpeed", pursuitSpeed);
    get_parameter("far_min_dis", far_min_dis);
    get_parameter("reduct_ratio", reduct_ratio);
    get_parameter("target_min_dis", target_min_dis);
    get_parameter("cout_hglocation", cout_hglocation);
    get_parameter("obstacle_x_max_dis", obstacle_x_max_dis);
    get_parameter("obstacle_y_max_dis", obstacle_y_max_dis);

    std::cout << FRED("Copyright©2016-2020 duzhong robot. All rights reserved ") << std::endl;
    std::cout << FYEL("*****dztrack:parameters*******************") << std::endl;
    std::cout << FGRN("saveMapping: ") << saveMapping << std::endl;
    std::cout << FGRN("savePath: ") << savePath << std::endl;
    std::cout << FGRN("saveStopPointPath: ") << saveStopPointPath << std::endl;
    std::cout << FGRN("robot_base_frame: ") << robot_base_frame_ << std::endl;
    std::cout << FGRN("global_frame: ") << global_frame_ << std::endl;
    std::cout << FGRN("pursuitSpeed: ") << pursuitSpeed << std::endl;
    std::cout << FGRN("far_min_dis: ") << far_min_dis << std::endl;
    std::cout << FGRN("reduct_ratio: ") << reduct_ratio << std::endl;
    std::cout << FGRN("target_min_dis: ") << target_min_dis << std::endl;

    std::cout << FYEL("*****dztrack:parameters end***************") << std::endl;

    // pub
    plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("dzplan", 1);
    all_plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("dzallplan", 1);
    vis_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10);
    pub_current_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
    pub_vehicle_state = this->create_publisher<std_msgs::msg::UInt8>("vehicle_state", 5);
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_dztrack", 1);
    angle_pub = this->create_publisher<geometry_msgs::msg::Twist>("pursuitAngle", 1);
    pub_vehicle_Drivestate = this->create_publisher<std_msgs::msg::UInt8>("vehicle_Drivestate", 5);
    nav2_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 5);



    // sub
    sub_trafficStatus = this->create_subscription<std_msgs::msg::UInt8MultiArray>("trafficLightArray", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(), std::bind(&control::callback_trafficStatus, this, std::placeholders::_1));
    sub_lasersScan = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(), std::bind(&control::callback_lasersScan, this, std::placeholders::_1));
    sub_saveMapping = this->create_subscription<std_msgs::msg::UInt8>("savemapping", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(), std::bind(&control::callback_saveMapping, this, std::placeholders::_1));
    // // sub for multi_robor  stop_CMD/
    sub_amcl_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(), std::bind(&control::callback_amcl_pose, this, std::placeholders::_1));
    // robotaxi
    // sub_passenger_waypoints = this->create_subscription<nav_msgs::msg::Path>("passenger_waypoints", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(), std::bind(&control::callback_passenger_waypoints, this, std::placeholders::_1));
    // sub_navi_cmd = this->create_subscription<std_msgs::msg::UInt8>("navi_cmd", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(), std::bind(&control::callback_navi_cmd, this, std::placeholders::_1));
    // sub_avp_parking_Num = this->create_subscription<std_msgs::msg::UInt8>("avp_parking_Num", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(), std::bind(&control::callback_avp_parking_Num, this, std::placeholders::_1));

    if (saveMapping)
    {
        printf("control--> saveMapping = true\n");
    }
    else if (!saveMapping)
    {
        printf("control--> saveMapping = false\n");
    }
    // printf("success-2\n");
    loadmap();
    // printf("dzglobalplanner world size %d\n", ret);

    loadStopPoint();

    doplanner();
}
control::~control()
{
}

void control::callback_amcl_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    amcl_pose_msg = *msg;
    amcl_pose_flag = true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr current_pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>);

void control::callback_lasersScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    currentScan = *msg;
    sensor_msgs::msg::PointCloud2 scan_cloud;
    laser_geometry::LaserProjection projector_;
    projector_.projectLaser(*msg, scan_cloud);

    pcl::fromROSMsg(scan_cloud, *current_pcl_ptr);
    recvScanFlag = 1;
}

void control::callback_saveMapping(const std_msgs::msg::UInt8::SharedPtr msg)
{
    saveMappingMsg = *msg;
    if (saveMappingMsg.data == 1)
    {
        saveMapping = true;
    }
    else if (saveMappingMsg.data == 2)
    {
        saveMapping = false;
    }
    if (saveMapping)
    {
        printf("control callback_saveMapping--> saveMapping = true\n");
    }
    else if (!saveMapping)
    {
        printf("control callback_saveMapping--> saveMapping = false\n");
    }
}

void control::callback_trafficStatus(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    trafficLightArray = *msg;
}

int control::loadStopPoint()
{
    stopPoints.clear();
    std::ifstream fin(saveStopPointPath.c_str());
    std::string line, temp;
    float tempheading = 0.0;
    // int lineNum = 0;
    while (getline(fin, line))
    {
        temp = "";
        geometry_msgs::msg::PoseStamped rp;
        int strnum = 0;
        for (unsigned int i = 0; i < line.size(); ++i)
        {
            if (line[i] == ',')
            {
                std::stringstream stream;
                stream << temp;
                switch (strnum)
                {
                case 0:
                    stream >> rp.pose.position.x;
                    stream.str("");
                    temp = "";
                    break;
                case 1:
                    stream >> rp.pose.position.y;
                    stream.str("");
                    temp = "";
                    break;
                case 2:
                    stream >> tempheading;
                    rp.pose.position.z = 0.0;
                    stream.str("");
                    temp = "";
                    break;
                case 3:
                    stream >> rp.pose.orientation.x;
                    stream.str("");
                    temp = "";
                    break;
                case 4:
                    stream >> rp.pose.orientation.y;
                    stream.str("");
                    temp = "";
                    break;
                case 5:
                    stream >> rp.pose.orientation.z;
                    stream.str("");
                    temp = "";
                    break;
                case 6:
                    stream >> rp.pose.orientation.w;
                    stream.str("");
                    temp = "";
                    break;
                default:
                    stream.str("");
                    temp = "";
                    break;
                }
                strnum++;
            }
            else
            {
                temp += line[i];
            }
        }
        // lineNum++;
        // rp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, tempheading);
        // tf2::Quaternion myQuaternion;
        // myQuaternion.setRPY(0.0, 0.0, tempheading);
        // rp.pose.orientation.x = myQuaternion.x();
        // rp.pose.orientation.y = myQuaternion.y();
        // rp.pose.orientation.z = myQuaternion.z();
        // rp.pose.orientation.w = myQuaternion.w();

        stopPoints.push_back(rp);
    }
    fin.close();
    maxstopIndex = (int)stopPoints.size();
    pub_stopP_marks();

    // printf("load stop point %d\n", (int)stopPoints.size());
    return stopPoints.size();
}

void control::pub_stopP_marks()
{
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    // marker.header.stamp = ros::Time();
    marker.header.stamp = this->now();

    marker.ns = "stopPoints";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // marker.pose.position.x = rp.pose.position.x;
    // marker.pose.position.y = rp.pose.position.y;
    // marker.pose.position.z = rp.pose.position.z;
    // marker.pose.orientation.x = rp.pose.orientation.x;
    // marker.pose.orientation.y = rp.pose.orientation.y;
    // marker.pose.orientation.z = rp.pose.orientation.z;
    // marker.pose.orientation.w = rp.pose.orientation.w;
    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 254;
    marker.color.g = 0;
    marker.color.b = 0.0;

    markers.markers.clear();
    for (int i = 0; i < (int)stopPoints.size(); i++)
    {
        marker.id = i;
        marker.pose.position.x = stopPoints[i].pose.position.x;
        marker.pose.position.y = stopPoints[i].pose.position.y;
        marker.pose.position.z = stopPoints[i].pose.position.z;
        marker.pose.orientation.x = stopPoints[i].pose.orientation.x;
        marker.pose.orientation.y = stopPoints[i].pose.orientation.y;
        marker.pose.orientation.z = stopPoints[i].pose.orientation.z;
        marker.pose.orientation.w = stopPoints[i].pose.orientation.w;
        markers.markers.push_back(marker);
    }
    markers.markers.push_back(marker);
    // printf("\nmarkers.markers.size()=%d\n",(int)markers.markers.size());
    vis_pub->publish(markers);
}

double getYaw(geometry_msgs::msg::PoseStamped pose)
{
    return tf2::getYaw(pose.pose.orientation);
}

double control::getDis(geometry_msgs::msg::PoseStamped currentpose, geometry_msgs::msg::PoseStamped roadpose)
{
    return sqrt((currentpose.pose.position.x - roadpose.pose.position.x) * (currentpose.pose.position.x - roadpose.pose.position.x) + (currentpose.pose.position.y - roadpose.pose.position.y) * (currentpose.pose.position.y - roadpose.pose.position.y));
}

int control::detectReachedGoal(const geometry_msgs::msg::PoseStamped &currentPose, geometry_msgs::msg::PoseStamped &stopPose)
{
    double diffDistance = getDis(currentPose, stopPose);
    // printf("diffDistance = %.2f\n",diffDistance);
    if (diffDistance < STOP_DIS)
    {
        double currentpose_yaw = tf2::getYaw(currentPose.pose.orientation);
        double targetpose_yaw = tf2::getYaw(stopPose.pose.orientation);
        double diffangle = fabs(currentpose_yaw - targetpose_yaw);
        // printf("diffangle-1 = %.2f\n",diffangle);
        if (diffangle > 3.1415926)
            diffangle = 6.283 - diffangle;
        // printf("diffangle = %.2f,diffDistance = %.2f\n",diffangle,diffDistance);
        if (diffangle > 1.0)
        {
            return 0;
        }
        else
            return 1;
    }
    else
        return 0;
}

float control::foundTargetAngle(float targetDis, float speed, geometry_msgs::msg::PoseStamped current_position, std::vector<geometry_msgs::msg::PoseStamped> temp_plan, float *pubSpeed)
{

    // float resultSpeed = speed;

    double target_dis_near = 0;
    double targetAngle = 0;
    geometry_msgs::msg::PoseStamped targetPose;
    geometry_msgs::msg::PoseStamped near_targetPose;
    double sum_dis=0;
    for (int i = NearestPoint_index_i; i < (int)temp_plan.size(); i++)
    {

        if (i == 0)
        {
            continue;
        }
        sum_dis += getDis(temp_plan[i - 1], temp_plan[i]);

        if (sum_dis >= targetDis)
        {
            targetPose = temp_plan[i];
            break;
        }
        else
        {
            targetPose = temp_plan[(int)temp_plan.size() - 1];
        }
    }

    double currentpose_yaw = tf2::getYaw(current_position.pose.orientation);
    double targetpose_yaw = tf2::getYaw(targetPose.pose.orientation);
    double diffangle_temp = (currentpose_yaw - targetpose_yaw);

    if ((fabs(diffangle_temp) < 0.2) || ((6.283 - fabs(diffangle_temp)) < 0.2))
    {
        vehicle_tar_orientation = 0;
    }
    else if ((diffangle_temp > 0.2 && diffangle_temp < 3.14) || (diffangle_temp > -6.08 && diffangle_temp < -3.14))
    {
        vehicle_tar_orientation = 2;
    }
    else
    {
        vehicle_tar_orientation = 1;
    }

    // printf("vehicle_tar_orientation= %d\n",vehicle_tar_orientation);

    double diffangle = fabs(diffangle_temp);
    if (diffangle > 3.1415926)
        diffangle = 6.283 - diffangle;

    targetDis = targetDis / (1.0 + diffangle_Kp * diffangle);


    speed = speed / (1 + reduct_ratio * diffangle); // 降速
                                                    // 1.1         0.5-1.57
    target_dis_near = targetDis;                    // 拉近预瞄点


    sum_dis = 0;
    for (int i = NearestPoint_index_i; i < (int)temp_plan.size(); i++)
    {

        if (i == 0)
        {
            continue;
        }
        sum_dis += getDis(temp_plan[i - 1], temp_plan[i]);

        if (sum_dis >= targetDis)
        {
            near_targetPose = temp_plan[i];
            break;
        }
        else
        {
            near_targetPose = temp_plan[(int)temp_plan.size() - 1];
        }
    }

    geometry_msgs::msg::PoseStamped baselink_near_target_pose;
    baselink_near_target_pose = near_targetPose;
    Eigen::Vector3d targetPose_near_point;
    Eigen::Vector3d baselink_near_target_pose_vec;
    targetPose_near_point.x() = near_targetPose.pose.position.x;
    targetPose_near_point.y() = near_targetPose.pose.position.y;
    targetPose_near_point.z() = 1;
    map_to_baselink(targetPose_near_point, &baselink_near_target_pose_vec);
    baselink_near_target_pose.pose.position.x = baselink_near_target_pose_vec.x();
    baselink_near_target_pose.pose.position.y = baselink_near_target_pose_vec.y();
    baselink_near_target_pose.pose.position.z = baselink_near_target_pose_vec.z();

    float near_alpha = lineDirection(baselink_near_target_pose.pose.position.x, baselink_near_target_pose.pose.position.y);
    float near_targetAngle = atan(angle_Kp * 0.17 * sin(near_alpha) / target_dis_near) * 57.3;

    targetAngle = near_targetAngle;
    // printf("diffangle= %f targetDis= %f speed= %f targetAngle= %f  \n",diffangle,targetDis,speed,targetAngle);

    *pubSpeed = speed;
    return targetAngle;
}

float control::foundNearestPoint(geometry_msgs::msg::PoseStamped current_position, std::vector<geometry_msgs::msg::PoseStamped> temp_plan)
{
    float minDis = 8888;
    float temp_dis = 9999;
    for (int i = 0; i < (int)temp_plan.size(); i++)
    {
        temp_dis = getDis(current_position, temp_plan[i]);
        if (temp_dis < minDis)
        {
            minDis = temp_dis;
            NearestPoint_index_i = i;
        }
    }

    NearestPoint_yaw = tf2::getYaw(temp_plan[NearestPoint_index_i].pose.orientation);
    return minDis;
}

float control::lineDirection(float xsecond, float ysecond)
{
    float alpha = 0.0;
    // printf("xsecond =%.2f,ysecond =%.2f\n",xsecond,ysecond);
    if ((xsecond > 0) && (ysecond > 0))
    {
        alpha = atan(fabs(ysecond - 0) / fabs(xsecond - 0));
    }
    else if ((xsecond > 0) && (ysecond <= 0))
    {
        //       alpha=2*M_PI-atan(fabs(ysecond-0)/fabs(xsecond-0));
        alpha = -atan(fabs(ysecond - 0) / fabs(xsecond - 0));
    }

    // add for back_car
    else if ((xsecond < 0) && (ysecond > 0))
    {
        alpha = atan(fabs(ysecond - 0) / fabs(xsecond - 0));
    }
    else if ((xsecond < 0) && (ysecond <= 0))
    {
        //       alpha=2*M_PI-atan(fabs(ysecond-0)/fabs(xsecond-0));
        alpha = -atan(fabs(ysecond - 0) / fabs(xsecond - 0));
    }

    return alpha;
}

float control::sLineControl(geometry_msgs::msg::PoseStamped current_position, std::vector<geometry_msgs::msg::PoseStamped> temp_plan, float *pubSpeed)
{
    
    float NearestDis = foundNearestPoint(current_position, temp_plan);

    // printf("NearestDis=%.2f\n",NearestDis);

    dis_y = 1 + NearestDis;

    if (dis_y > 1.35)
    {
        dis_y = 1.35;
    }

    *pubSpeed = *pubSpeed / (1.0 + 10.0 * NearestDis);

    float targetDis = target_min_dis * fabs(*pubSpeed) + far_min_dis; // 

    float targetAngle = foundTargetAngle(targetDis, *pubSpeed, current_position, temp_plan, pubSpeed);

    return targetAngle;
}


void control::obstacleDetect(std::vector<geometry_msgs::msg::PoseStamped> temp_plan)
{
    // printf("[obstacleDetect]\n");

    geometry_msgs::msg::PoseStamped targetPose;
    geometry_msgs::msg::PoseStamped baselink_local_planner_pose;
    std::string base_frame = robot_base_frame_;

    double minObstacleDis = 8888.0;
    int planSize = static_cast<int>(temp_plan.size());
    foundObstacleFlag = 0;
    // printf("[obstacleDetect--0 planSize= %d ]\n",planSize);
    // ------------------------- 0️⃣ 输入有效性检查 -------------------------
    // if (planSize <= 20 || !current_pcl_ptr || current_pcl_ptr->points.empty())
    //     return;
    // printf("[obstacleDetect--1 ]\n");
    // ------------------------- 1️⃣ 构建 KD-Tree -------------------------
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_xy(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_xy->reserve(current_pcl_ptr->points.size());

    int print_count = 0; 
    // printf("[obstacleDetect--2 ]\n");
    for (const auto& p : current_pcl_ptr->points)
    {
        // printf("[obstacleDetect--2 ]\n");
        print_count++;
        pcl::PointXYZ pt;
        // 坐标修正，与原版本保持一致
        pt.x = (-p.x) + 0.149;
        pt.y = -p.y;
        pt.z = 0.0;
        

        // 仅保留前方 ±0.5 m、前向 obstacle_x_max_dis 的点
        if (pt.x > 0.16 && pt.x < obstacle_x_max_dis &&
            pt.y > -obstacle_y_max_dis && pt.y < obstacle_y_max_dis)
        {      
            // printf("[Filtered PCL] : %d ,x=%.3f, y=%.3f\n",print_count,pt.x, pt.y); 

            pcl_xy->points.push_back(pt);
            // if (print_count < 20)
            // {
            //     printf("[Filtered PCL---] #%d: x=%.3f, y=%.3f, z=%.3f\n",
            //         print_count, pt.x, pt.y, pt.z);
            //     print_count++;
            // }
        }
    }

    if (pcl_xy->points.empty())
    {
        // printf("[obstacleDetect] Filtered PCL empty\n");
        return;
    }

    kdtree.setInputCloud(pcl_xy);

    // ------------------------- 2️⃣ 沿路径采样检测 -------------------------
    for (int i = 0; i < planSize; i += 2)
    {
        targetPose = temp_plan[i];
        targetPose.header.stamp = rclcpp::Time();
        baselink_local_planner_pose = targetPose;

        // 坐标变换：map -> base_link
        Eigen::Vector3d targetPose_point, baselink_target_pose_vec;
        targetPose_point.x() = targetPose.pose.position.x;
        targetPose_point.y() = targetPose.pose.position.y;
        targetPose_point.z() = 1.0;
        map_to_baselink(targetPose_point, &baselink_target_pose_vec);

        baselink_local_planner_pose.pose.position.x = baselink_target_pose_vec.x();
        baselink_local_planner_pose.pose.position.y = baselink_target_pose_vec.y();
        baselink_local_planner_pose.pose.position.z = baselink_target_pose_vec.z();

        // 若路径点超出检测范围则停止
        if (baselink_local_planner_pose.pose.position.x >= obstacle_x_max_dis + 0.05)
            break;

        // ------------------------- 3️⃣ KD-Tree 最近邻搜索 -------------------------
        pcl::PointXYZ searchPoint;
        searchPoint.x = baselink_local_planner_pose.pose.position.x;
        searchPoint.y = baselink_local_planner_pose.pose.position.y;
        searchPoint.z = 0.0;

        std::vector<int> pointIdx(1);
        std::vector<float> pointDist(1);

        if (kdtree.nearestKSearch(searchPoint, 1, pointIdx, pointDist) > 0)
        {
            double minDis = std::sqrt(pointDist[0]);  // 障碍点距路径点的距离

            // 若障碍点在路径线 0.2 m 内，则视为靠近轨迹
            if (minDis < minDis_param)
            {
                // 计算障碍物离车体中心的实际距离
                double obstacleDis = std::sqrt(
                    pcl_xy->points[pointIdx[0]].x * pcl_xy->points[pointIdx[0]].x +
                    pcl_xy->points[pointIdx[0]].y * pcl_xy->points[pointIdx[0]].y
                );

                if (obstacleDis < minObstacleDis)
                    minObstacleDis = obstacleDis;
            }
        }
    }

    // ------------------------- 4️⃣ 结果判断 -------------------------
    // 若障碍物距车体中心 0.06~1.0 m，则认为前方有障碍
    if (minObstacleDis > 0.06 && minObstacleDis < obstacle_x_max_dis)
        foundObstacleFlag = 1;

    // ------------------------- 5️⃣ 调试输出 -------------------------
    // printf("[obstacleDetect] PCL size=%zu  minObstacleDis=%.3f  flag=%d\n",
    //        pcl_xy->points.size(), minObstacleDis, foundObstacleFlag);
}


void control::doplanner()
{
    // printf("success-doplanner\n");
    int run_rate = 20;
    rclcpp::Node::SharedPtr node_(this); // 创建基类指针，指向子类对象this
    rclcpp::WallRate loop_rate(run_rate);
    geometry_msgs::msg::PoseStamped goal;
    std::vector<geometry_msgs::msg::PoseStamped> global_plan;
    geometry_msgs::msg::PoseStamped global_pose;
    geometry_msgs::msg::PoseStamped current_position;
    geometry_msgs::msg::Twist cmd_vel;

    rclcpp::sleep_for(std::chrono::seconds(4));
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node_);
        fflush(stdout);

        if (amcl_pose_flag)
        {
            current_position.header = amcl_pose_msg.header;
            current_position.header.frame_id = "map";
            current_position.pose.position.x = amcl_pose_msg.pose.pose.position.x;
            current_position.pose.position.y = amcl_pose_msg.pose.pose.position.y;
            current_position.pose.position.z = 0;
            current_position.pose.orientation.x = amcl_pose_msg.pose.pose.orientation.x;
            current_position.pose.orientation.y = amcl_pose_msg.pose.pose.orientation.y;
            current_position.pose.orientation.z = amcl_pose_msg.pose.pose.orientation.z;
            current_position.pose.orientation.w = amcl_pose_msg.pose.pose.orientation.w;
        }
        // saveMapping = false;

        if (saveMapping)
        {
            printf("start save mapping\n");
            if (distance(current_position, last_pose) >= save_mapping_point_dis)
            {
                // printf("current_position.pose.position.x,%4lf:\n",current_position.pose.position.x);
                fp = fopen(savePath.c_str(), "a");
                // fp = fopen("/home/duzhong/dzrobot/src/naviPointPath/mapping.txt", "a");

                fprintf(fp, "%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,\n",
                        current_position.pose.position.x,
                        current_position.pose.position.y,
                        current_position.pose.position.z,
                        current_position.pose.orientation.x,
                        current_position.pose.orientation.y,
                        current_position.pose.orientation.z,
                        current_position.pose.orientation.w);
                fclose(fp);
                last_pose = current_position;
            }
        }
        else
        {
            if (cout_hglocation)
            {
                double currentpose_yaw = tf2::getYaw(current_position.pose.orientation);
                std::cout << current_position.pose.position.x << "," << current_position.pose.position.y << ", " << currentpose_yaw << "," << std::endl;
            }
            pub_current_pose->publish(current_position);

            global_plan.clear();

            if (!makePlan(current_position, global_plan) || global_plan.empty())
            {
                // printf("error makePlan \n");
                loop_rate.sleep();
                RCLCPP_ERROR(this->get_logger(), "dztrack--- makePlan error---Failed to find a  plan ");

                continue;
            }
            else
            {
                vehicle_state_data.data = 1;
                vehicle_Drivestate_data.data = 1;

                pubtargetSpeed = pursuitSpeed;
                float tagetAngle = sLineControl(current_position, global_plan, &pubtargetSpeed);
                cmd_vel.linear.x = pubtargetSpeed;
                cmd_vel.angular.z = tagetAngle;                
            }

            foundObstacleFlag = 0;

            obstacleDetect(global_plan);

            if (foundObstacleFlag == 1)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 1.0;
                cmd_vel.angular.z = 0;
                vehicle_Drivestate_data.data = 2;
                // std::cout <<"foundObstacleFlag == 1" <<std::endl;
            }
            else{
                cmd_vel.linear.y = 0.0;
            }

            // vel_pub_->publish(cmd_vel);
            angle_pub->publish(cmd_vel);
            pub_vehicle_state->publish(vehicle_state_data);
            pub_vehicle_Drivestate->publish(vehicle_Drivestate_data);

            loop_rate.sleep();
            // //----------------end for pure pursuit----------------
        }
    }
    loop_rate.sleep();
}

double control::distance(const geometry_msgs::msg::PoseStamped &p1, const geometry_msgs::msg::PoseStamped &p2)
{
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

bool control::getRobotPose(geometry_msgs::msg::PoseStamped &global_pose)
{
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::msg::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    // robot_pose.header.stamp = ros::Time();     // latest available
    robot_pose.header.stamp = rclcpp::Time(); // latest available
    rclcpp::Time current_time = this->now();
    // double transform_tolerance_ = 0.1;
    // save time for checking tf delay later
    // get robot pose on the given costmap frame
    try
    {
        // tf2_ros::TransformReadyCallback aaa;
        // tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
        // tf_buffer_->transform(robot_pose, global_pose, "robot1/map");
        tf_buffer_->transform(robot_pose, global_pose, "map", tf2::Duration(3));
        // std::cout<< "end" << std::endl;
    }
    catch (tf2::LookupException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ConnectivityException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ExtrapolationException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    // check if global_pose time stamp is within costmap transform tolerance
    // if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    double se = current_time.seconds() - rclcpp::Time(global_pose.header.stamp).seconds();
    if (current_time.seconds() - rclcpp::Time(global_pose.header.stamp).seconds() > 1)
    {
        RCLCPP_ERROR(this->get_logger(), "Transform timeout for %s. "
                                         "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f",
                     "map",
                     current_time.seconds(), rclcpp::Time(global_pose.header.stamp).seconds(), se);
        return false;
    }
    return true;
}

void control::loadmap()
{
    roadPoints.clear();
    std::ifstream fin(savePath.c_str());
    std::string line, temp;
    while (getline(fin, line))
    {
        temp = "";
        geometry_msgs::msg::PoseStamped rp;
        int strnum = 0;
        for (unsigned int i = 0; i < line.size(); ++i)
        {
            if (line[i] == ',')
            {
                std::stringstream stream;
                stream << temp;
                switch (strnum)
                {
                case 0:
                    stream >> rp.pose.position.x;
                    stream.str("");
                    temp = "";
                    break;
                case 1:
                    stream >> rp.pose.position.y;
                    stream.str("");
                    temp = "";
                    break;
                case 2:
                    stream >> rp.pose.position.z;
                    stream.str("");
                    temp = "";
                    break;
                case 3:
                    stream >> rp.pose.orientation.x;
                    stream.str("");
                    temp = "";
                    break;
                case 4:
                    stream >> rp.pose.orientation.y;
                    stream.str("");
                    temp = "";
                    break;
                case 5:
                    stream >> rp.pose.orientation.z;
                    stream.str("");
                    temp = "";
                    break;
                case 6:
                    stream >> rp.pose.orientation.w;
                    stream.str("");
                    temp = "";
                    break;
                default:
                    stream.str("");
                    temp = "";
                    break;
                }
                strnum++;
            }
            else
            {
                temp += line[i];
            }
        }

        roadPoints.push_back(rp);
    }
    fin.close();
    maxIndex = (int)roadPoints.size();
    // return roadPoints.size();
    return;
}

bool control::makePlan(const geometry_msgs::msg::PoseStamped &start,
                       std::vector<geometry_msgs::msg::PoseStamped> &plan)
{
    // printf("minIndex 0 = %d\n",minIndex);
    int plan_front = 60;
    int plan_bear = 3;
    double Min_cartoplan_dis = 1.5;
    double Max_dis_Toplan = 0.3;
    double current_yaw = 0, roadPoint_yaw = 0;

    // printf("Got a start: %.2f, %.2f\n", start.pose.position.x, start.pose.position.y);
    plan.clear();
    std::vector<geometry_msgs::msg::PoseStamped> visualplan;
    visualplan.clear();
    minDis = 10000;
    double diffangle = 0.0;
    int j_num = 0;
    int endpoint_flag = 0;
    // printf("LastIndex = %d,roadPoints.size = %d\n",LastIndex,(int)roadPoints.size());
    pub_Num++;



    if (pub_Num >= 40)
    {
        pub_Num = 0;
        nav_msgs::msg::Path gui_path;
        gui_path.poses.resize((int)roadPoints.size());

        gui_path.header.frame_id = "map";
        gui_path.header.stamp = this->now();

        gui_path.poses.clear();
        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (int i = 0; i < (int)roadPoints.size(); i += 10)
        {
            gui_path.poses.push_back(roadPoints[i]);
        }
        all_plan_pub_->publish(gui_path);
        //     loadStopPoint();
    }

    if (minIndex > (int)roadPoints.size())
        minIndex = 0;
    // printf("minIndex1 =%d\n",minIndex);

    if (abs((int)roadPoints.size() - LastIndex) < 5)
    {
        LastIndex = 0;
        minIndex = 0;
        endpoint_flag = 1;
        // printf("success1--abs((int)roadPoints.size() - LastIndex) < 5\n");
    }
    else
        endpoint_flag = 0;
    //   maxIndex = (int)roadPoints.size();


    for (int j = minIndex; j < (int)roadPoints.size(); j++)
    {
        double diffDis = getDis(start, roadPoints[j]);
        if (diffDis < minDis)
        {
            minDis = diffDis;
            LastIndex = j;
            // printf("success-- 2");
        }

        j_num++;
        if ((j_num % plan_front) == 0 && minDis < Max_dis_Toplan)
        {
            break;
        }
    }



    if (minDis > Min_cartoplan_dis)
    {
        // printf("success-4\n");
        LastIndex = 0;
        Last_lastIndex = -999;
        // publishPlan(plan);
        // return !plan.empty();
    }
    // printf("minIndex2 =%d\n",minIndex);
    // printf("LastIndex=%d,Last_lastIndex=%d\n",LastIndex,Last_lastIndex);
    if (fabs(LastIndex - Last_lastIndex) > 50 && endpoint_flag == 0)
    {
        {
            double tem_mindis = Min_cartoplan_dis;
            double diffDis;
            int flag_findminpoint = 0;
            for (int k = 0; k < (int)roadPoints.size(); k++)
            {
                diffDis = getDis(start, roadPoints[k]);
                if (diffDis < tem_mindis)
                {
                    //// printf("diffDis=%.2f\n",diffDis);
                    // tem_mindis = diffDis;
                    current_yaw = getYaw(start),
                    roadPoint_yaw = getYaw(roadPoints[k]);
                    diffangle = fabs(current_yaw - roadPoint_yaw);

                    if (diffangle > 3.1415926)
                        diffangle = 6.283 - diffangle;
                    // printf("diffangle=%.2f\n",diffangle);
                    // printf("%d diffDis= %.2f,diffangle=%.2f,roadPoint_yaw%.2f\n",k,diffDis,diffangle,roadPoint_yaw);
                    if (diffangle < 2.57)
                    {
                        tem_mindis = diffDis;
                        LastIndex = k;
                        flag_findminpoint = 1;
                    }
                }
            }
            if (flag_findminpoint == 0)
            {
                // printf("flag_findminpoint =0\n");
                LastIndex = 0;
                Last_lastIndex = -999;
                publishPlan(plan);
                return !plan.empty();
            }
        }
    }
    // printf("LastIndex =%d,diffangle =%.2f\n",LastIndex,diffangle);
    // printf("minIndex5 =%d\n",minIndex);
    if (LastIndex >= plan_bear)
        minIndex = LastIndex - plan_bear;
    else
        minIndex = 0;



    maxIndex = LastIndex + plan_front;

    if (maxIndex >= (int)roadPoints.size())
    {
        maxIndex = (int)roadPoints.size();
    }

    rclcpp::Time plan_time = this->now();
    double sum_path_dis = 0;
    // printf("minIndex10 =%d\n",minIndex);
    for (int i = minIndex; i < maxIndex; i++)
    {
        geometry_msgs::msg::PoseStamped visualpose;
        geometry_msgs::msg::PoseStamped carCoordinatePose;
        visualpose.header.stamp = plan_time;
        visualpose.header.frame_id = "map";

        // if (minDis < Min_cartoplan_dis)
        // {
        //     double diffDistance = getDis(start, roadPoints[i]);
        //     if (diffDistance < 1.6) // 局部路径取 3米 内得点
        //     {
        //         visualpose.header.frame_id = "robot1/map";
        //         visualpose.header.stamp = this->now();
        //         visualpose.pose.position.x = roadPoints[i].pose.position.x;
        //         visualpose.pose.position.y = roadPoints[i].pose.position.y;
        //         visualpose.pose.position.z = 0.1;
        //         visualpose.pose.orientation.x = roadPoints[i].pose.orientation.x;
        //         visualpose.pose.orientation.y = roadPoints[i].pose.orientation.y;
        //         visualpose.pose.orientation.z = roadPoints[i].pose.orientation.z;
        //         visualpose.pose.orientation.w = roadPoints[i].pose.orientation.w;
        //         // if (i % 4 == 0)
        //         {
        //             plan.push_back(visualpose);
        //         }
        //         if ((int)plan.size() >= 150)
        //         {
        //             break;
        //         }
        //     }
        //     else
        //         break;
        // }
        if (minDis < Min_cartoplan_dis)
        {
            // double diffDistance = getDis(start,roadPoints[i]);
            // if(diffDistance < 9.0)
            visualpose.header.frame_id = "map";
            visualpose.header.stamp = this->now();
            visualpose.pose.position.x = roadPoints[i].pose.position.x;
            visualpose.pose.position.y = roadPoints[i].pose.position.y;
            visualpose.pose.position.z = 0.1;
            visualpose.pose.orientation.x = roadPoints[i].pose.orientation.x;
            visualpose.pose.orientation.y = roadPoints[i].pose.orientation.y;
            visualpose.pose.orientation.z = roadPoints[i].pose.orientation.z;
            visualpose.pose.orientation.w = roadPoints[i].pose.orientation.w;
            // if (i % 4 == 0)
            {
            plan.push_back(visualpose);
            }
            if (i < roadPoints.size() - 2)
            {
            sum_path_dis += getDis(roadPoints[i], roadPoints[i + 1]);
            }

            if (sum_path_dis >= 1.5)
            {
            break;
            }

            // if ((int)plan.size() >= 400)
            // {
            //   break;
            // }
        }
    }
    // maxIndex = LastIndex + plan_front;
    // if(maxIndex >= (int)roadPoints.size()){
    //     maxIndex = (int)roadPoints.size();
    // }
    if ((abs((int)roadPoints.size() - LastIndex) < 10 && (int)plan.size() < 10) || (int)plan.size() < 10)
    {
        LastIndex = 0;
        minIndex = 0;
        maxIndex = LastIndex + plan_front;
    }


    Last_lastIndex = LastIndex;
    // printf("dzglobalplanner-->find path number size=%d,LastIndex=%d,minDis=%.2f,maxIndex=%d,minIndex = %d\n",(int)plan.size(),LastIndex,minDis,maxIndex,minIndex);
    // (int)plan.size()
    // printf("(int)plan.size() =%d,%d\n",(int)plan.size(), maxIndex -minIndex);
    publishPlan(plan);

    return !plan.empty();
}

// void control::publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> &path)
// {
//     nav_msgs::msg::Path gui_path;
//     gui_path.poses.resize(path.size());

//     gui_path.header.frame_id = "map";
//     gui_path.header.stamp = this->now();

//     for (unsigned int i = 0; i < path.size(); i++)
//     {
//         gui_path.poses[i] = path[i];
//     }

//     plan_pub_->publish(gui_path);
    
    
//     geometry_msgs::msg::PoseStamped goal_msg;    
//     // goal_msg = path[path.size()-1];
//     // nav2_goal_pub_->publish(gui_path);

// }
void control::publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> &path)
{
    if (path.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Empty path received, skip publishing.");
        return;
    }

    // 1️⃣ 发布路径给 RViz 可视化
    nav_msgs::msg::Path gui_path;
    gui_path.header.frame_id = "map";
    gui_path.header.stamp = this->now();
    gui_path.poses = path;  // 简洁拷贝
    plan_pub_->publish(gui_path);

    // 2️⃣ 发布路径终点给 Nav2（触发导航）
    geometry_msgs::msg::PoseStamped goal_msg = path.back();
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = this->now();

    nav2_goal_pub_->publish(goal_msg);

    // RCLCPP_INFO(this->get_logger(),
    //             "Published path (%zu poses) and goal (x=%.2f, y=%.2f).",
    //             path.size(),
    //             goal_msg.pose.position.x,
    //             goal_msg.pose.position.y);
}
void control::map_to_baselink(Eigen::Vector3d p1, Eigen::Vector3d *p2)
{
    Eigen::Vector3d pos(0, 0, 0);
    Eigen::Matrix3d rol(Eigen::Matrix3d::Identity());
    Eigen::Quaterniond Qre;

    pos.x() = amcl_pose_msg.pose.pose.position.x;

    pos.y() = amcl_pose_msg.pose.pose.position.y;

    pos.z() = 1;

    Qre.x() = amcl_pose_msg.pose.pose.orientation.x;
    Qre.y() = amcl_pose_msg.pose.pose.orientation.y;
    Qre.z() = amcl_pose_msg.pose.pose.orientation.z;
    Qre.w() = amcl_pose_msg.pose.pose.orientation.w;

    Qre.normalize();

    rol = Qre.matrix();

    *p2 = rol.inverse() * (p1 - pos);
}
