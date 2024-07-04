#include "rclcpp/rclcpp.hpp"
#include "pg_for_ros2/simple_lib.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPSGFlow>());
  rclcpp::shutdown();
  return 0;
}