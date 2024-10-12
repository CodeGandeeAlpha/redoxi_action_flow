#include <pose_detector/pipeline.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    auto node = std::make_shared<FlowRos2Pipeline::PoseDetectorPipeline>();
    auto init_config = std::make_shared<FlowRos2Pipeline::PoseDetectorPipeline::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::PoseDetectorPipeline::RuntimeConfig>();
    init_config->from_parameters(node.get());

    // init_config->process_document_action = "detector_in_process_document_action";
    FlowRos2Pipeline::PoseDetectorPipelineInitConfig::DownstreamPipelineNode downstream_pipeline_node;
    downstream_pipeline_node.accept_document_action = "tracker_pipeline_process_document_action";
    init_config->pipeline_downstreams["tracker_pipeline"] = downstream_pipeline_node;
    FlowRos2Pipeline::PoseDetectorPipelineInitConfig::DownstreamModelNode model_node;
    model_node.accept_detections_action = "model_process_detections_action";
    init_config->model_downstreams["rtmpose"] = model_node;

    runtime_config->from_parameters(node.get());

    node->init(init_config, runtime_config);
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}