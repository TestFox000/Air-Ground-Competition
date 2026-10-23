#include "judgment.h"

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     /*产生一个waypoint的节点*/
//     auto node = std::make_shared<dzjudgment>("robot1_judgment");
//     /* 运行节点，并检测退出信号*/
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
#include "judgment.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dzjudgment>();
  node->run();
  rclcpp::shutdown();
  return 0;
}