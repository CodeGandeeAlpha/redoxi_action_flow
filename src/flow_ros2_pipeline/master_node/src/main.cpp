#include <rclcpp/rclcpp.hpp>
#include <master_node/master_node.hpp>
#include <rclcpp/executors.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    auto node = std::make_shared<FlowRos2Pipeline::MasterNode>();
    auto init_config = std::make_shared<FlowRos2Pipeline::MasterNode::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::MasterNode::RuntimeConfig>();
    init_config->from_parameters(node.get());
    FlowRos2Pipeline::MasterNode::InitConfig::DownstreamNode detector_in_pipeline_node;
    detector_in_pipeline_node.accept_document_action = "detector_in_process_document_action";
    init_config->downstreams["detector_in"] = detector_in_pipeline_node;

    runtime_config->from_parameters(node.get());
    node->init(init_config, runtime_config);
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}