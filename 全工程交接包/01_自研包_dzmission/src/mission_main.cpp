#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "dzmission/localization_compensator.hpp"
#include "dzmission/mission_manager.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 参数全部由各节点构造函数中的 declare_parameter 显式声明，
  // launch / yaml 中的同名参数会作为 override 自动生效。
  rclcpp::NodeOptions options;

  // 两个节点必须同时运行：
  //   LocalizationCompensator —— 产生 /odometry/filtered 与 /localization/healthy，
  //                              是任务层判断"自身定位是否可信"的唯一依据。
  //                              缺了它，pose_valid_ 永远为 false，任务层会一直
  //                              处于定位故障急停状态，无法导航与交战。
  //   MissionManager         —— 任务状态机 / 仲裁 / 避碰 / 云台 / 裁判板
  auto loc = std::make_shared<dzmission::LocalizationCompensator>(options);
  auto mgr = std::make_shared<dzmission::MissionManager>(options);

  // 用多线程执行器：LocalizationCompensator 要在 100Hz IMU 上做预测，
  // 不能与 50Hz 的 MissionManager 定时器互相阻塞。
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(loc);
  executor.add_node(mgr);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
