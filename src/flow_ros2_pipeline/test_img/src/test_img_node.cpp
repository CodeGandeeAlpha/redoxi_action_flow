#include "rclcpp/rclcpp.hpp"
#include "test_img/test_img.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlowRos2Pipeline::TestImg>());
  rclcpp::shutdown();
  return 0;
}