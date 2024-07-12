#include "rclcpp/rclcpp.hpp"
#include "cache_node/cache.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlowRos2Pipeline::CacheNode>());
    rclcpp::shutdown();
    return 0;
}