#include <detector/pipeline_in.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    auto node = std::make_shared<FlowRos2Pipeline::DetectorIn>();
    auto init_config = std::make_shared<FlowRos2Pipeline::DetectorIn::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::DetectorIn::RuntimeConfig>();
    init_config->from_parameters(node.get());

    // init_config->process_document_action = "detector_in_process_document_action";
    FlowRos2Pipeline::DetectorInInitConfig::DownstreamPipelineNode detector_out_pipeline_node;
    detector_out_pipeline_node.accept_document_action = "detector_out_process_document_action";
    init_config->pipeline_downstreams["detector_out"] = detector_out_pipeline_node;
    FlowRos2Pipeline::DetectorInInitConfig::DownstreamModelNode model_node;
    model_node.accept_frame_action = "model_process_frame_action";
    init_config->model_downstreams["ddq"] = model_node;

    runtime_config->from_parameters(node.get());

    node->init(init_config, runtime_config);
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}