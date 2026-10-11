#include <stdio.h>
#include <memory>
#include <sys/time.h>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>

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
  DetNode()
      : Node("det_node"), draw_(1), threadNum_(4), frames_(0)
  {
    det_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("offset_center", 1);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 1, std::bind(&DetNode::imageCallback, this, _1));

    std::string model_name = "/home/duzhong/dzrobot/src/rknn_pt/model/light_det.rknn";

    RCLCPP_INFO(this->get_logger(), "Model path: %s", model_name.c_str());

    detectPool_ = std::make_shared<rknnPool<RkPt, cv::Mat, DetectResultsGroup>>(model_name, threadNum_);
    if (detectPool_->init() != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "rknnPool init fail!");
      throw std::runtime_error("rknnPool init fail");
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&DetNode::processTimer, this));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      std::lock_guard<std::mutex> lk(img_mtx_);
      ros_frame_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Image convert failed: %s", e.what());
    }
  }

  void processTimer()
  {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lk(img_mtx_);
      if (ros_frame_.empty())
      {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "waiting for /image_raw ... (is the camera node running?)");
        return;
      }
      frame = ros_frame_.clone();
    }

    std_msgs::msg::Int32MultiArray msg;
    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.dim[0].size = 8;
    msg.layout.dim[0].stride = 8;
    msg.layout.dim[0].label = "light center coordinate";
    // [0] centerx偏移 [1] centery偏移 [2] 类别(1=蓝靶 2=红靶 0=无目标)
    // [3] 框宽 [4] 框高 [5] 关键点x偏移 [6] 关键点y偏移 [7] 关键点置信度*100
    msg.data = {0, 0, 0, 0, 0, 0, 0, 0};

    DetectResultsGroup results_group;
    int width = frame.cols;
    int height = frame.rows;

    if (detectPool_->put(frame, frames_) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "put original images failed or work done!");
      return;
    }

    if (frames_ >= threadNum_ && detectPool_->get(results_group) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "frames > threadNum but get processed images failed or work done");
      return;
    }

    if (!results_group.cur_img.empty())
    {
      for (const auto &res : results_group.dets)
      {
        int center_x = res.box.x + res.box.width / 2;
        int center_y = res.box.y + res.box.height / 2;

        int offset_center_x = center_x - width / 2;
        int offset_center_y = center_y - height / 2;

        msg.data[0] = offset_center_x;
        msg.data[1] = offset_center_y;
        msg.data[2] = res.cls + 1;    // 1=blue target, 2=red target（0 留给"无目标"）
        msg.data[3] = res.box.width;  // 框宽，供控制端做长宽比过滤
        msg.data[4] = res.box.height; // 框高
        // 关键点(靶心)相对画面中心的偏移，精度高于框中心，备用
        msg.data[5] = (int)res.kpt_x - width / 2;
        msg.data[6] = (int)res.kpt_y - height / 2;
        msg.data[7] = (int)(res.kpt_conf * 100.f);
      }
      det_pub_->publish(msg);

      if (draw_)
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

private:
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr det_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<rknnPool<RkPt, cv::Mat, DetectResultsGroup>> detectPool_;

  cv::Mat ros_frame_;
  std::mutex img_mtx_;

  int draw_;
  int threadNum_;
  int frames_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
