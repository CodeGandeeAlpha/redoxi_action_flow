#include <rclcpp/rclcpp.hpp>
#include <tracker/tracker.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    auto node = std::make_shared<FlowRos2Pipeline::Tracker>();
    auto init_config = std::make_shared<FlowRos2Pipeline::Tracker::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::Tracker::RuntimeConfig>();
    init_config->from_parameters(node.get());

    // init_config->process_document_action = "detector_in_process_document_action";
    FlowRos2Pipeline::TrackerInitConfig::DownstreamPipelineNode tracker_out_pipeline_node;
    tracker_out_pipeline_node.accept_track_targets_action = "tracker_out_process_track_targets_action";
    init_config->pipeline_downstreams["tracker_out"] = tracker_out_pipeline_node;

    runtime_config->from_parameters(node.get());

    node->init(init_config, runtime_config);
    node->open();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}