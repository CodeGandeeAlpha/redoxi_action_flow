#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include <video_reader/video_reader.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    auto node = std::make_shared<FlowRos2Pipeline::OpencvVideoReader>();
    auto init_config = std::make_shared<FlowRos2Pipeline::OpencvVideoReader::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::OpencvVideoReader::RuntimeConfig>();
    init_config->from_parameters(node.get());

    FlowRos2Pipeline::OpencvVideoReader::InitConfig::DownstreamNode downstream_node;
    downstream_node.accept_frame_action = "master_node_process_frame";
    downstream_node.status_query_service = "status_query";
    init_config->downstreams["master_node"] = downstream_node;

    runtime_config->from_parameters(node.get());
    node->init(init_config, runtime_config);
    node->set_image_topic_enable(true);
    node->open();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}