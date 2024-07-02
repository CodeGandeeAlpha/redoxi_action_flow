#include "rclcpp/rclcpp.hpp"
#include "video_reader/video_reader.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlowRos2Pipeline::VideoReader>());
    rclcpp::shutdown();
    return 0;
}