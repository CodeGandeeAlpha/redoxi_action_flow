#include <pose_detector/pipeline_out.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    auto node = std::make_shared<FlowRos2Pipeline::PoseDetectorOut>();
    auto init_config = std::make_shared<FlowRos2Pipeline::PoseDetectorOut::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::PoseDetectorOut::RuntimeConfig>();
    init_config->from_parameters(node.get());

    // init_config->process_document_action = "pose_detector_out_process_document_action";
    FlowRos2Pipeline::PoseDetectorOutInitConfig::DownstreamNode downstream_pipeline_node;
    downstream_pipeline_node.accept_document_action = "tracker_process_document_action";
    init_config->downstreams["tracker"] = downstream_pipeline_node;

    runtime_config->from_parameters(node.get());

    node->init(init_config, runtime_config);
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}