#include <detector/pipeline.hpp>
#include <detector/pipeline_types.hpp>

namespace FlowRos2Pipeline
{
void DetectorPipelineInitConfig::from_parameters(DetectorPipeline *node)
{
    process_document_action = node->get_parameter("process_document_action").as_string();
    process_model_results_action = node->get_parameter("process_model_results_action").as_string();

    const auto &logger = node->get_logger();
    // RCLCPP_INFO(logger, "process_document_action: %s", this->process_document_action.c_str());
}

void DetectorPipelineRuntimeConfig::from_parameters(DetectorPipeline *node)
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