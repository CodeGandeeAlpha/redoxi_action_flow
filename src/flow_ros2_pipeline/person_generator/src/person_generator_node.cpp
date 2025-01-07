#include <person_generator/person_generator.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FlowRos2Pipeline::PersonGenerator>();
    auto init_config = std::make_shared<FlowRos2Pipeline::PersonGenerator::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::PersonGenerator::RuntimeConfig>();
    init_config->from_parameters(node.get());

    // init_config->process_document_action = "detector_in_process_document_action";
    FlowRos2Pipeline::PersonGeneratorInitConfig::DownstreamPipelineNode pose_pipeline_node;
    pose_pipeline_node.accept_document_action = "pose_detector_pipeline_process_document_action";
    // pose_pipeline_node.accept_document_action = "pose_detector_in_process_document_action";
    init_config->pipeline_downstreams["pose_detector_pipeline"] = pose_pipeline_node;


    runtime_config->from_parameters(node.get());

    node->init(init_config, runtime_config);
    node->start();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}