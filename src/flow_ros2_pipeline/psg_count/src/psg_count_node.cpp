#include <rclcpp/rclcpp.hpp>
#include <psg_count/psg_count.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    auto node = std::make_shared<FlowRos2Pipeline::PSGCount>();
    auto init_config = std::make_shared<FlowRos2Pipeline::PSGCount::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::PSGCount::RuntimeConfig>();
    init_config->from_parameters(node.get());

    // init_config->process_document_action = "detector_in_process_document_action";
    FlowRos2Pipeline::PSGCountInitConfig::DownstreamPipelineNode event_pipeline_node;
    event_pipeline_node.accept_document_action = "psg_collector_process_document_action";
    init_config->pipeline_downstreams["event"] = event_pipeline_node;

    runtime_config->from_parameters(node.get());

    node->init(init_config, runtime_config);
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}