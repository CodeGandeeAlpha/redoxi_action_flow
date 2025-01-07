#include <master_node/master_node.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    auto node = std::make_shared<FlowRos2Pipeline::MasterNode>();
    auto init_config = std::make_shared<FlowRos2Pipeline::MasterNode::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::MasterNode::RuntimeConfig>();
    init_config->from_parameters(node.get());
    FlowRos2Pipeline::MasterNode::InitConfig::DownstreamNode detector_in_pipeline_node;
    detector_in_pipeline_node.accept_document_action = "detector_pipeline_process_document_action";
    init_config->downstreams["detector_pipeline"] = detector_in_pipeline_node;

    runtime_config->from_parameters(node.get());
    node->init(init_config, runtime_config);
    node->start();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}