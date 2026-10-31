#include "judgment.h"

#include "judgment.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dzjudgment>();
  node->run();
  rclcpp::shutdown();
  return 0;
}