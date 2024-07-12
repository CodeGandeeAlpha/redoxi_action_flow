#include "rclcpp/rclcpp.hpp"
#include "video_reader/video_reader.hpp"
#include <rclcpp/executors.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    // auto node = std::make_shared<FlowRos2Pipeline::OpencvVideoReader>();
    // FlowRos2Pipeline::OpencvVideoReaderConfig config;
    // config.from_parameters(node.get());
    // node->set_config(config);
    // node->make_ready();
    // rclcpp::spin(node);
    // rclcpp::shutdown();
    return 0;
}