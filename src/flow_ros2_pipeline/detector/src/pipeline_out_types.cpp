#include <detector/pipeline_out.hpp>
#include <detector/pipeline_out_types.hpp>

namespace FlowRos2Pipeline
{
void DetectorOutInitConfig::from_parameters(DetectorOut *node)
{
    process_document_action = node->get_parameter("process_document_action").as_string();
    process_detections_action = node->get_parameter("process_detections_action").as_string();

    const auto &logger = node->get_logger();
    // RCLCPP_INFO(logger, "process_document_action: %s", this->process_document_action.c_str());
    // RCLCPP_INFO(logger, "process_detections_action: %s", this->process_detections_action.c_str());
}

void DetectorOutRuntimeConfig::from_parameters(DetectorOut *node)
{
    step_interval_ms = node->get_parameter("step_interval_ms").as_double();
    timeout_ms_send_to_downstream = node->get_parameter("timeout_ms_send_to_downstream").as_double();

    const auto &logger = node->get_logger();
    // RCLCPP_INFO(logger, "step_interval_ms: %lf", this->step_interval_ms);
    // RCLCPP_INFO(logger, "timeout_ms_send_to_downstream: %lf", this->timeout_ms_send_to_downstream);

    buffer_size = node->get_parameter("buffer_size").as_int();
    send_goal_retry = node->get_parameter("send_goal_retry").as_bool();
    RCLCPP_INFO(logger, "send_goal_retry: %d", this->send_goal_retry);
}
} // namespace FlowRos2Pipeline