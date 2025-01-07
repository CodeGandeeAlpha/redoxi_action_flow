#include <detector/pipeline_out.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    auto node = std::make_shared<FlowRos2Pipeline::DetectorOut>();
    auto init_config = std::make_shared<FlowRos2Pipeline::DetectorOut::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::DetectorOut::RuntimeConfig>();
    init_config->from_parameters(node.get());

    // init_config->process_document_action = "detector_out_process_document_action";
    FlowRos2Pipeline::DetectorOutInitConfig::DownstreamNode downstream_pipeline_node;
    downstream_pipeline_node.accept_document_action = "person_generator_process_document_action";
    init_config->downstreams["person_generator"] = downstream_pipeline_node;

    runtime_config->from_parameters(node.get());

    node->init(init_config, runtime_config);
    node->start();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}