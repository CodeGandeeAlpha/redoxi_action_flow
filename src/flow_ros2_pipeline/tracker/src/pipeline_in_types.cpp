#include <tracker/pipeline_in.hpp>
#include <tracker/pipeline_in_types.hpp>

namespace FlowRos2Pipeline
{
void TrackerInInitConfig::from_parameters(TrackerIn *node)
{
    process_document_action = node->get_parameter("process_document_action").as_string();

    const auto &logger = node->get_logger();
    // RCLCPP_INFO(logger, "process_document_action: %s", this->process_document_action.c_str());
}

void TrackerInRuntimeConfig::from_parameters(TrackerIn *node)
{
    step_interval_ms = node->get_parameter("step_interval_ms").as_double();
    timeout_ms_send_to_downstream = node->get_parameter("timeout_ms_send_to_downstream").as_double();

    const auto &logger = node->get_logger();
    // RCLCPP_INFO(logger, "step_interval_ms: %lf", this->step_interval_ms);
    // RCLCPP_INFO(logger, "timeout_ms_send_to_downstream: %lf", this->timeout_ms_send_to_downstream);
}
} // namespace FlowRos2Pipeline