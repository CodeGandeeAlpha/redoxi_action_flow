#include <rclcpp/rclcpp.hpp>
#include <tracker/pipeline_out.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    auto node = std::make_shared<FlowRos2Pipeline::TrackerOut>();
    auto init_config = std::make_shared<FlowRos2Pipeline::TrackerOut::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::TrackerOut::RuntimeConfig>();
    init_config->from_parameters(node.get());

    // init_config->process_document_action = "tracker_out_process_document_action";
    FlowRos2Pipeline::TrackerOutInitConfig::DownstreamNode downstream_pipeline_node;
    downstream_pipeline_node.accept_document_action = "tracker_process_document_action";
    init_config->downstreams["tracker"] = downstream_pipeline_node;

    runtime_config->from_parameters(node.get());

    node->init(init_config, runtime_config);
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}