#include <stdio.h>
#include <memory>
#include <sys/time.h>
#include <sys/stat.h>
#include <mutex>
#include <string>

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
  DetNode()
      : Node("det_node"), frames_(0)
  {
    det_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("offset_center", 1);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 1, std::bind(&DetNode::imageCallback, this, _1));

    // =================== 参数（在 config/robot.yaml 的 /det_node 段调整）===================
    // 以前这些全是硬编码，换个模型/想省点 CPU 都得改代码重编译，非常难用。
    this->declare_parameter("model_path", std::string(""));
    this->declare_parameter("draw", true);
    this->declare_parameter("thread_num", 4);
    this->declare_parameter("max_fps", 30.0);
    this->declare_parameter("hot_reload", true);

    const std::string cfg_model = this->get_parameter("model_path").as_string();
    draw_ = this->get_parameter("draw").as_bool();
    thread_num_ = static_cast<int>(this->get_parameter("thread_num").as_int());
    max_fps_ = this->get_parameter("max_fps").as_double();
    hot_reload_ = this->get_parameter("hot_reload").as_bool();

    if (thread_num_ < 1)
    {
      RCLCPP_WARN(this->get_logger(), "thread_num=%d 非法，回退为 1", thread_num_);
      thread_num_ = 1;
    }

    model_path_ = resolveModelPath(cfg_model);

    if (!loadPool())
    {
      throw std::runtime_error("rknnPool init fail");
    }

    // ★限帧用【定时器周期】实现，而不是在回调里卡时间戳。
    //   踩过的坑：定时器 33ms、阈值 1/30=33.3ms，每次回调都比阈值快 0.3ms，
    //   于是每隔一帧被拒，实际频率腰斩到 ~15Hz（实测 30.3 -> 15.6）。
    //   直接让周期 = 1/max_fps 就没有这种边界效应。
    //   相机本身最快 30fps，所以周期不会小于 33ms —— 再快也拿不到新画面。
    double period_ms = 1000.0 / (max_fps_ > 0.0 ? max_fps_ : 30.0);
    if (period_ms < 33.0)
    {
      period_ms = 33.0;
    }
    const int period_int = static_cast<int>(period_ms + 0.5);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(period_int),
                                     std::bind(&DetNode::processTimer, this));

    RCLCPP_INFO(this->get_logger(),
                "视觉检测: 模型=%s | 线程=%d | 推理周期=%dms(约%.0f fps) | 窗口=%s | 热加载=%s",
                model_path_.c_str(), thread_num_, period_int, 1000.0 / period_int,
                draw_ ? "开" : "关", hot_reload_ ? "开" : "关");
    if (!draw_)
    {
      // 关窗口能省掉 30Hz 的 imshow+waitKey 开销（在 ARM 板上相当可观）。
      RCLCPP_INFO(this->get_logger(), "已关闭识别画面窗口（config: draw: false）");
    }
  }

private:
  // ---------------------------------------------------------------------------
  //  模型路径解析：优先参数 > 包内 share/det_node/model/ > 源码目录回退
  // ---------------------------------------------------------------------------
  std::string resolveModelPath(const std::string &cfg_model)
  {
    if (!cfg_model.empty())
    {
      RCLCPP_INFO(this->get_logger(), "使用配置中心指定的模型: %s", cfg_model.c_str());
      return cfg_model;
    }
    try
    {
      // 优先取包安装目录下的 share/det_node/model/。
      // 这样目录改名、换用户、换机器都不会再让节点启动失败（以前改成 det_node 就崩过）。
      return ament_index_cpp::get_package_share_directory("det_node") +
             "/model/light_det.rknn";
    }
    catch (const std::exception &)
    {
      RCLCPP_WARN(this->get_logger(), "det_node 未安装，回退到源码路径");
      return "/home/duzhong/dzrobot/src/rknn_pt/model/light_det.rknn";
    }
  }

  // ---------------------------------------------------------------------------
  //  加载/重载推理池
  //  先建新的、成功了才替换旧的 —— 新模型有问题时旧模型继续工作，不会瞎掉。
  //  旧池在赋值瞬间析构，其析构函数会 drain 掉所有在途任务（rknnPool.hpp:116），
  //  且本函数只在 processTimer 里调用，此时没有其它线程在用池，所以是安全的。
  // ---------------------------------------------------------------------------
  bool loadPool()
  {
    try
    {
      auto new_pool = std::make_shared<rknnPool<RkPt, cv::Mat, DetectResultsGroup>>(
          model_path_, thread_num_);
      if (new_pool->init() != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "模型加载失败: %s（继续沿用旧模型）",
                     model_path_.c_str());
        return false;
      }
      detectPool_ = new_pool;
      frames_ = 0;
      model_mtime_ = 0;             // 让下一次 mtime 检查重新取基准
      last_mtime_check_ = now();
      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "模型加载异常: %s（继续沿用旧模型）", e.what());
      return false;
    }
  }

  // ---------------------------------------------------------------------------
  //  模型热加载：换模型只要覆盖文件即可，不必重启节点
  //  每 1 秒最多查一次 mtime，避免频繁 stat 拖累推理循环。
  // ---------------------------------------------------------------------------
  void checkHotReload()
  {
    if (!hot_reload_)
    {
      return;
    }
    const rclcpp::Time t = now();
    if ((t - last_mtime_check_).seconds() < 1.0)
    {
      return;
    }
    last_mtime_check_ = t;

    struct stat st;
    if (stat(model_path_.c_str(), &st) != 0)
    {
      return;
    }
    if (model_mtime_ == 0)
    {
      model_mtime_ = st.st_mtime;   // 首次取基准
      return;
    }
    if (st.st_mtime == model_mtime_)
    {
      return;
    }

    model_mtime_ = st.st_mtime;
    RCLCPP_WARN(this->get_logger(), "检测到模型文件已更新，正在热加载: %s",
                model_path_.c_str());
    if (loadPool())
    {
      RCLCPP_INFO(this->get_logger(), "模型热加载完成，无需重启节点");
    }
  }

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
    // 限帧已由定时器周期完成（见构造函数），这里不再二次卡时间 ——
    // 两处都卡会因为边界效应互相打架，反而把频率压低。
    checkHotReload();

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

    if (frames_ >= thread_num_ && detectPool_->get(results_group) != 0)
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
    infer_count_++;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr det_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<rknnPool<RkPt, cv::Mat, DetectResultsGroup>> detectPool_;

  cv::Mat ros_frame_;
  std::mutex img_mtx_;

  std::string model_path_;
  bool draw_;
  int thread_num_;
  double max_fps_;
  bool hot_reload_;

  time_t model_mtime_ = 0;
  rclcpp::Time last_mtime_check_{0, 0, RCL_ROS_TIME};

  int frames_;
  long infer_count_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
