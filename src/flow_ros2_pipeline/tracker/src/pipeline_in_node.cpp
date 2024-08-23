#include <rclcpp/rclcpp.hpp>
#include <tracker/pipeline_in.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    auto node = std::make_shared<FlowRos2Pipeline::TrackerIn>();
    auto init_config = std::make_shared<FlowRos2Pipeline::TrackerIn::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::TrackerIn::RuntimeConfig>();
    init_config->from_parameters(node.get());

    // init_config->process_document_action = "detector_in_process_document_action";
    FlowRos2Pipeline::TrackerInInitConfig::DownstreamPipelineNode tracker_out_pipeline_node;
    tracker_out_pipeline_node.accept_document_action = "tracker_out_process_document_action";
    init_config->pipeline_downstreams["tracker_out"] = tracker_out_pipeline_node;
    FlowRos2Pipeline::TrackerInInitConfig::DownstreamModelNode model_node;
    model_node.accept_detections_action = "model_process_detections_action";
    init_config->model_downstreams["rtmpose"] = model_node;

    runtime_config->from_parameters(node.get());

    node->init(init_config, runtime_config);
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}