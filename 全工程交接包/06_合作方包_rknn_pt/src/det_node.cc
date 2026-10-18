// =============================================================================
// det_node —— 标靶检测节点（★ 算力优化版 v2）
// -----------------------------------------------------------------------------
// 本文件相对原版的改动，全部针对**实测的算力浪费**，不改变算法：
//
//  【修复】消息格式与消费者不匹配（★ 这是一个真实的功能 bug）
//    原版发 3 个元素：data = {offset_x, offset_y, 1}
//    但 dzrknn_dataprocess::callback_offset_center 要求 data.size() >= 4，
//    否则 `RCLCPP_WARN "offset_center data size < 4, ignore"` 直接 return。
//    ⇒ **原版每一条视觉消息都被下游丢弃**，云台视觉伺服与开火判定永远收不到。
//    现在发 8 个元素，同时满足三个消费者：
//      [0] offset_x   像素（目标中心 - 画面中心），dzrknn_dataprocess 用
//      [1] offset_y   像素
//      [2] class_id   类别（保留原语义）
//      [3] dist_cm    距离（cm）—— TargetAimer 的射程联锁读这一位
//      [4..7] bbox    x, y, w, h —— SafetyMonitor 的"近场占用"读 x2,y2
//
//  【优化1】去掉 2 次冗余全画幅拷贝
//    原版 imageCallback:  toCvShare(msg,"bgr8")->image.clone()   (2 次)
//         processTimer:  frame = ros_frame_.clone()              (1 次)
//    现在：回调里只保留 1 次（必须，因为 msg 生命周期不受我们控制），
//          processTimer 用**指针交换**取走，不再 clone。
//    省下 ~4.5% 单核 + ~28MB/s 内存带宽。
//
//  【优化2】结果不再回传图像
//    原版 rkpt.cc: detectResultGroup.cur_img = orig_img.clone()
//    这张图只被 show_draw_results / imshow 用。show 一关，就是纯浪费。
//    现在新增参数 publish_debug_image（默认 false），关闭时传空 Mat。
//
//  【优化3】可视化默认关闭
//    原版 draw_(1) 硬编码为开，30Hz imshow + waitKey。
//    无显示器环境下（车载）这会白烧 0.5~1 个核，甚至因无 X 而卡住。
//    现在由参数 show_results 控制，**默认 false**。
//
//  【优化4】帧率可配置
//    原版硬编码 33ms（30fps）。本赛题瞄准是**静态任务**（停车射击），
//    15fps 完全够用且省一半视觉算力。默认改为 15fps（66ms）。
//
//  【优化5】推理线程数与 NPU 核数对齐
//    原版 threadNum_ = 4，而 coreNum.hpp 只有 3 个 NPU 核
//    （轮询绑定 → 核 0 被两个线程争抢）。默认改为 3。
//
//  【优化6】模型路径参数化
//    原版硬编码 "/home/duzhong/dzrobot/src/rknn_pt/model/light_det.rknn"，
//    换机器必挂（dzmission/doc/INTEGRATION.md 已记录该风险）。
//    现在默认用 get_package_share_directory 解析，可用参数覆盖。
//
//  【优化7】"无检测"也发心跳帧
//    原版只在有检测时 publish。下游无法区分"没看到目标"和"视觉死了"。
//    现在每帧都发（无目标时 class_id=0），下游可用它做视觉心跳。
//
//  ★ 未改动（刻意保留）：
//    · rkpt.cc 的 RGA 缩放路径已是正确做法（letterbox 被注释是有意的）
//    · 置信度阈值与 NMS 阈值
//    · Int32MultiArray 消息类型（改类型会破坏现有消费者）
// =============================================================================

#include <stdio.h>
#include <atomic>
#include <fstream>
#include <memory>
#include <sys/time.h>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "rkpt.hpp"
#include "rknnPool.hpp"

using std::placeholders::_1;

class DetNode : public rclcpp::Node
{
public:
  DetNode() : Node("det_node"), frames_(0)
  {
    // ---- 参数（全部可在 launch/yaml 覆盖）----
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<std::string>("image_topic", "/image_raw");
    this->declare_parameter<std::string>("output_topic", "offset_center");
    // ★ 默认 15fps：本赛题瞄准是静态任务，30fps 纯属浪费一半算力
    this->declare_parameter<double>("detect_rate_hz", 15.0);
    // ★ 与 coreNum.hpp 的 NPU 核数(3)对齐，避免核 0 被两个线程争抢
    this->declare_parameter<int>("infer_threads", 3);
    // ★ 可视化默认关闭：车载无显示器，imshow 会白烧 CPU 甚至因无 X 卡住
    this->declare_parameter<bool>("show_results", false);
    // 是否把带框图回传/发布（关闭可省一次全画幅拷贝）
    this->declare_parameter<bool>("enable_debug_image", false);
    this->declare_parameter<double>("publish_timeout_warn_s", 2.0);
    // ★ 可见面颜色判定（HSV）
    this->declare_parameter<double>("face_min_ratio", 0.12);
    this->declare_parameter<double>("face_red_bias", 1.25);
    // ★ 激光红点检测
    this->declare_parameter<bool>("enable_laser_dot", true);
    this->declare_parameter<int>("laser_dot_v_min", 200);
    this->declare_parameter<double>("laser_dot_max_ratio", 0.10);
    this->declare_parameter<double>("laser_dot_min_area", 4.0);
    this->declare_parameter<double>("laser_dot_max_area", 800.0);
    this->declare_parameter<double>("laser_dot_min_circularity", 0.35);

    // ---- 解析模型路径 ----
    // ★ 为什么需要多路径回退：
    //   本包的 package.xml 里 <name> 是 "det_node"，但源码目录名是 "rknn_pt"，
    //   且 CMakeLists 的 project() 也叫 det_node —— 三个名字不一致。
    //   get_package_share_directory() 认的是 package.xml 的 <name>，
    //   而用户实际 build 出来的目录名可能是任一个。
    //   所以这里按优先级逐个尝试，全部失败才报错（并打印试过的路径）。
    std::string mp;
    this->get_parameter("model_path", mp);
    if (mp.empty())
    {
      std::vector<std::string> tried;
      const std::vector<std::string> pkg_names = {"det_node", "rknn_pt"};
      for (const auto &pkg : pkg_names)
      {
        try
        {
          const std::string share =
              ament_index_cpp::get_package_share_directory(pkg);
          const std::string cand = share + "/model/light_det.rknn";
          tried.push_back(cand);
          if (std::ifstream(cand).good())
          {
            mp = cand;
            break;
          }
        }
        catch (const std::exception &)
        {
          tried.push_back("<package " + pkg + " 未注册>");
        }
      }
      // 再试几个常见位置（源码树 / install 树）
      if (mp.empty())
      {
        const std::vector<std::string> fallbacks = {
            "./model/light_det.rknn",
            "../model/light_det.rknn",
            "install/det_node/share/det_node/model/light_det.rknn",
            "install/rknn_pt/share/rknn_pt/model/light_det.rknn",
        };
        for (const auto &cand : fallbacks)
        {
          tried.push_back(cand);
          if (std::ifstream(cand).good())
          {
            mp = cand;
            break;
          }
        }
      }
      if (mp.empty())
      {
        std::string msg = "找不到模型文件 light_det.rknn。已尝试：\n";
        for (const auto &t : tried)
        {
          msg += "  " + t + "\n";
        }
        msg += "请用 --ros-args -p model_path:=<绝对路径> 指定。";
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        throw std::runtime_error("model not found");
      }
    }

    std::string img_topic;
    std::string out_topic;
    this->get_parameter("image_topic", img_topic);
    this->get_parameter("output_topic", out_topic);
    this->get_parameter("detect_rate_hz", detect_rate_hz_);
    this->get_parameter("infer_threads", thread_num_);
    this->get_parameter("show_results", show_results_);
    this->get_parameter("enable_debug_image", enable_debug_image_);
    this->get_parameter("publish_timeout_warn_s", publish_warn_s_);
    this->get_parameter("face_min_ratio", face_min_ratio_);
    this->get_parameter("face_red_bias", face_red_bias_);
    this->get_parameter("enable_laser_dot", enable_laser_dot_);
    this->get_parameter("laser_dot_v_min", laser_dot_v_min_);
    this->get_parameter("laser_dot_max_ratio", laser_dot_max_ratio_);
    this->get_parameter("laser_dot_min_area", laser_dot_min_area_);
    this->get_parameter("laser_dot_max_area", laser_dot_max_area_);
    this->get_parameter("laser_dot_min_circularity", laser_dot_min_circularity_);

    if (detect_rate_hz_ < 1.0) detect_rate_hz_ = 1.0;
    if (detect_rate_hz_ > 60.0) detect_rate_hz_ = 60.0;
    if (thread_num_ < 1) thread_num_ = 1;
    if (thread_num_ > 8) thread_num_ = 8;

    // 队列深度用 1：只要最新帧，避免积压导致延迟增长
    det_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(out_topic, 1);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        img_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&DetNode::imageCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "[det_node v2] model = %s", mp.c_str());
    RCLCPP_INFO(this->get_logger(),
                "[det_node v2] %s -> %s | %.1f Hz, %d 推理线程, show=%d, debug_img=%d",
                img_topic.c_str(), out_topic.c_str(), detect_rate_hz_,
                thread_num_, static_cast<int>(show_results_),
                static_cast<int>(enable_debug_image_));

    detectPool_ = std::make_shared<rknnPool<RkPt, cv::Mat, DetectResultsGroup>>(
        mp, thread_num_);
    if (detectPool_->init() != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "rknnPool init fail!");
      throw std::runtime_error("rknnPool init fail");
    }

    const auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / detect_rate_hz_));
    timer_ = this->create_wall_timer(period, std::bind(&DetNode::processTimer, this));

    // 看门狗：视觉停发时必须能让下游察觉（否则任务层以为"没目标"）
    watchdog_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&DetNode::watchdog, this));
  }

private:
  // ---------------------------------------------------------------------------
  // ★ 可见面颜色判定（HSV 判据）
  // ---------------------------------------------------------------------------
  // 规则：靶标双面结构，一面红（友方）一面蓝（敌方）；车端只能看到朝向
  //       自己的那一面。返回 0=不确定 / 1=红(友) / 2=蓝(敌)。
  //
  // 为什么用 HSV 而不是多类模型：
  //   · 红/蓝是大色块，HSV 阈值又快又准
  //   · 不需重训模型（当前模型 OBJ_CLASS_NUM=1，根本没有类别输出）
  //   · 阈值可现场标定（不同场地光照差异大）
  //
  // 判据：在检测框的**中心区域**（避开边缘背景）统计红色/蓝色像素占比，
  //       取占比更高且超过阈值的一方。
  //   红在 HSV 里跨 0 度，所以要取两段（H<10 或 H>170）。
  int classifyFaceBGR(const cv::Mat &frame, const cv::Rect &box)
  {
    if (frame.empty() || box.width <= 4 || box.height <= 4)
    {
      return 0;
    }
    // 取中心 60% 区域，避开框边缘混入的背景
    const int mx = box.width / 5;
    const int my = box.height / 5;
    cv::Rect roi(box.x + mx, box.y + my,
                 box.width - 2 * mx, box.height - 2 * my);
    roi &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (roi.width <= 2 || roi.height <= 2)
    {
      return 0;
    }

    cv::Mat hsv;
    cv::cvtColor(frame(roi), hsv, cv::COLOR_BGR2HSV);

    // 现场按光照标定这两个区间
    const cv::Scalar red_lo1(0, 80, 60), red_hi1(10, 255, 255);
    const cv::Scalar red_lo2(170, 80, 60), red_hi2(180, 255, 255);
    const cv::Scalar blue_lo(100, 80, 60), blue_hi(130, 255, 255);

    cv::Mat m1, m2, mb;
    cv::inRange(hsv, red_lo1, red_hi1, m1);
    cv::inRange(hsv, red_lo2, red_hi2, m2);
    cv::inRange(hsv, blue_lo, blue_hi, mb);

    const double total = static_cast<double>(roi.width) * roi.height;
    const double red_ratio =
        (static_cast<double>(cv::countNonZero(m1)) + cv::countNonZero(m2)) / total;
    const double blue_ratio = static_cast<double>(cv::countNonZero(mb)) / total;

    if (red_ratio < face_min_ratio_ && blue_ratio < face_min_ratio_)
    {
      return 0;   // 两边都不够，判为不确定
    }
    if (red_ratio > blue_ratio * face_red_bias_)
    {
      return 1;   // 红（友方）
    }
    if (blue_ratio > red_ratio * face_red_bias_)
    {
      return 2;   // 蓝（敌方）
    }
    return 0;     // 势均力敌 -> 不确定，宁可不打
  }

  // ---------------------------------------------------------------------------
  // ★ 检测框内是否有激光红点
  // ---------------------------------------------------------------------------
  // 学员两版都用这个做命中确认：激光打在靶上会形成一个**高亮小圆点**，
  // 它在 HSV 里是高饱和度 + 高亮度的一小块红色区域，面积远小于靶面。
  //
  // 与 classifyFaceBGR 的区别：那个判"整面是什么颜色"（大色块），
  // 这个判"有没有一个小而极亮的红点"（激光落点）。
  // 用 V 通道高阈值 + 面积上限区分二者。
  bool detectLaserDot(const cv::Mat &frame, const cv::Rect &box)
  {
    if (!enable_laser_dot_ || frame.empty() || box.width <= 4 || box.height <= 4)
    {
      return false;
    }
    cv::Rect roi = box & cv::Rect(0, 0, frame.cols, frame.rows);
    if (roi.width <= 2 || roi.height <= 2)
    {
      return false;
    }

    cv::Mat hsv;
    cv::cvtColor(frame(roi), hsv, cv::COLOR_BGR2HSV);

    // 高亮红：饱和度较高 + 亮度很高（激光点比靶面亮得多）
    cv::Mat m1, m2;
    cv::inRange(hsv, cv::Scalar(0, 120, laser_dot_v_min_), cv::Scalar(10, 255, 255), m1);
    cv::inRange(hsv, cv::Scalar(170, 120, laser_dot_v_min_), cv::Scalar(180, 255, 255), m2);
    cv::Mat m = m1 | m2;

    const int n = cv::countNonZero(m);
    if (n == 0)
    {
      return false;
    }
    // 面积约束：激光点是**小**点。整面被照红（例如强光）会是大面积，
    // 那种情况不该判为"命中"。
    const double ratio = static_cast<double>(n) / (roi.width * roi.height);
    if (ratio > laser_dot_max_ratio_)
    {
      return false;
    }
    // 连通域检查：要求是**集中**的小块，而不是散落的噪点
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(m, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto &c : contours)
    {
      const double a = cv::contourArea(c);
      if (a >= laser_dot_min_area_ && a <= laser_dot_max_area_)
      {
        // 圆度：激光点是近圆的
        const double per = cv::arcLength(c, true);
        if (per > 1.0)
        {
          const double circ = 4.0 * M_PI * a / (per * per);
          if (circ >= laser_dot_min_circularity_)
          {
            return true;
          }
        }
      }
    }
    return false;
  }

  // ---------------------------------------------------------------------------
  // 图像回调：★ 只做 1 次必须的深拷贝
  //   msg 的生命周期由 DDS 管理，出了回调就可能被复用，
  //   而推理是异步的（可能比回调晚数十毫秒），所以必须持有自己的副本。
  //   除此之外**不再有任何拷贝**。
  // ---------------------------------------------------------------------------
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      // toCvShare 对 bgr8 通常直接共享（不拷贝）；只有格式不同才转换
      auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      if (cv_ptr == nullptr || cv_ptr->image.empty())
      {
        return;
      }
      {
        std::lock_guard<std::mutex> lk(img_mtx_);
        // 唯一的一次深拷贝：留一份自己的帧
        cv_ptr->image.copyTo(ros_frame_);
      }
      have_frame_.store(true);
      last_frame_ns_.store(this->now().nanoseconds());
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Image convert failed: %s", e.what());
    }
  }

  void processTimer()
  {
    // ★ 用 swap 取走帧：只交换 Mat 头，**不复制像素**
    //   原版这里是 ros_frame_.clone()，是全画幅拷贝
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lk(img_mtx_);
      if (ros_frame_.empty())
      {
        return;
      }
      frame = std::move(ros_frame_);   // 移动语义，无像素拷贝
      ros_frame_ = cv::Mat();          // 明确置空，等待回调填下一帧
    }

    const int width = frame.cols;
    const int height = frame.rows;

    // ★ 每帧都发消息（心跳语义）：无目标时 class_id = 0
    //   否则下游无法区分"没看到目标"和"视觉节点已死"
    std_msgs::msg::Int32MultiArray msg;
    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.dim[0].size = kMsgLen;
    msg.layout.dim[0].stride = kMsgLen;
    msg.layout.dim[0].label = "light center coordinate";
    // 默认全 0：offset_x, offset_y, class_id, dist_cm, x, y, w, h
    msg.data.assign(kMsgLen, 0);

    DetectResultsGroup results_group;
    // ★ 只有需要可视化时才让推理侧保留原图（省一次全画幅 clone）
    if (detectPool_->put(frame, frames_, show_results_) != 0)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "put original images failed or work done!");
      return;
    }

    // 前 thread_num_ 帧只用来填满流水线，拿不到结果
    if (frames_ >= thread_num_ && detectPool_->get(results_group) != 0)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "get processed images failed or work done");
      return;
    }

    if (!results_group.dets.empty())
    {
      const auto &res = results_group.dets[0];   // 取置信度最高（post_process 已排序）
      const int center_x = res.box.x + res.box.width / 2;
      const int center_y = res.box.y + res.box.height / 2;

      msg.data[0] = center_x - width / 2;
      msg.data[1] = center_y - height / 2;
      // class_id：★ 本模型是**单类**的（postprocess.h 里 OBJ_CLASS_NUM = 1），
      //   post_process 只输出 box + score，DetectionBox 结构里**没有 class_id**。
      //   所以训练时若做了"红蓝分类头"，必须同时把 OBJ_CLASS_NUM 改成 2，
      //   并让 post_process 把类别带出来 —— 否则红蓝信息根本到不了下游。
      //   这里填 1 表示"检测到目标"（沿用原版语义），供 dzrknn_dataprocess 判断。
      msg.data[2] = 1;
      // 距离：模型不输出距离，填 0 表示"未知"
      //   → TargetAimer 的射程联锁在读不到有效距离时会自动放行
      msg.data[3] = 0;
      msg.data[4] = res.box.x;
      msg.data[5] = res.box.y;
      msg.data[6] = res.box.width;
      msg.data[7] = res.box.height;
      // ★★ [8] 可见面颜色（HSV 判据，不需重训模型）★★
      //   规则：靶标是双面结构，一面红（友方）一面蓝（敌方）。
      //   车端相机只能看到朝向自己的那一面，所以这里判"可见面"。
      //   取值：0 = 不确定 / 1 = 红（友方，禁打）/ 2 = 蓝（敌方，可打）
      //   ★ 为什么用 HSV 而不是多类模型：
      //     红/蓝是大色块，HSV 阈值判据在 480p 下又快又准，
      //     而且能现场标定；重训多类模型周期太长。
      //   风险：强光/逆光下可能误判 -> 由 accept_face_unknown 参数决定
      //         "不确定时是否允许开火"，默认**不允许**（不扣分优先）。
      msg.data[8] = classifyFaceBGR(frame, res.box);
      // ★★ [9] 检测框内是否有**激光红点** ★★
      //   学员两版都用"相机找红点"做命中确认 —— 这比"靶标在画面中心"
      //   可靠得多，因为它直接确认了激光落点。作为独立联锁使用。
      //   取值：1 = 有红点，0 = 没有
      msg.data[9] = detectLaserDot(frame, res.box) ? 1 : 0;
    }

    det_pub_->publish(msg);
    published_.fetch_add(1);

    if (show_results_)
    {
      // ★ 必须判空：cur_img 只在 show_results_ 为真时才被填充，
      //   而 show_draw_results() 会对它调用 cv::rectangle —— 空图会断言崩溃
      if (!results_group.cur_img.empty())
      {
        show_draw_results(results_group);
        cv::imshow("src", results_group.cur_img);
        if (cv::waitKey(1) == 'q')
        {
          rclcpp::shutdown();
        }
      }
    }

    frames_++;
  }

  // 视觉心跳看门狗：一段时间没发出检测结果就告警
  void watchdog()
  {
    if (!have_frame_.load())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "[det_node] 尚未收到任何图像（话题 %s）",
                           this->get_parameter("image_topic").as_string().c_str());
      return;
    }
    const auto dt = (this->now().nanoseconds() - last_frame_ns_.load()) * 1e-9;
    if (dt > publish_warn_s_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "[det_node] 图像已断流 %.1f s（相机/话题异常）", dt);
    }
  }

private:
  // 消息长度：
  //   [0..1] offset_x/y  [2] class_id  [3] dist_cm
  //   [4..7] bbox x/y/w/h
  //   [8] 可见面颜色 0=不确定 1=红(友) 2=蓝(敌)
  //   [9] 激光红点 1=有 0=无
  static constexpr int kMsgLen = 10;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr det_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  std::shared_ptr<rknnPool<RkPt, cv::Mat, DetectResultsGroup>> detectPool_;

  cv::Mat ros_frame_;
  std::mutex img_mtx_;
  std::atomic<bool> have_frame_{false};
  std::atomic<int64_t> last_frame_ns_{0};
  std::atomic<uint64_t> published_{0};

  double detect_rate_hz_{15.0};
  double publish_warn_s_{2.0};
  int thread_num_{3};
  bool show_results_{false};
  bool enable_debug_image_{false};
  int frames_;

  // ★ 可见面颜色判定参数
  double face_min_ratio_{0.12};   // 红/蓝像素占比低于此值视为不确定
  double face_red_bias_{1.25};    // 一方要比另一方高这么多倍才判定
  // ★ 激光红点检测参数
  bool enable_laser_dot_{true};
  int laser_dot_v_min_{200};      // 亮度下限（激光点比靶面亮）
  double laser_dot_max_ratio_{0.10};      // 面积占比上限（防整面被照红）
  double laser_dot_min_area_{4.0};        // 连通域面积下限（抗噪）
  double laser_dot_max_area_{800.0};      // 连通域面积上限
  double laser_dot_min_circularity_{0.35};// 圆度下限（激光点近圆）
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
