#include <psg_count/psg_count.hpp>
#include <psg_count/psg_count_types.hpp>

namespace FlowRos2Pipeline
{
void PSGCountInitConfig::from_parameters(PSGCount *node)
{
    process_document_action = node->get_parameter("process_document_action").as_string();
    passengerflow_config_path = node->get_parameter("passengerflow_config_path").as_string();

    const auto &logger = node->get_logger();
    RCLCPP_INFO(logger, "process_document_action: %s", this->process_document_action.c_str());
    RCLCPP_INFO(logger, "passengerflow_config_path: %s", this->passengerflow_config_path.c_str());
}

void PSGCountRuntimeConfig::from_parameters(PSGCount *node)
{
    step_interval_ms = node->get_parameter("step_interval_ms").as_double();
    timeout_ms_send_to_downstream = node->get_parameter("timeout_ms_send_to_downstream").as_double();

    const auto &logger = node->get_logger();
    RCLCPP_INFO(logger, "step_interval_ms: %lf", this->step_interval_ms);
    RCLCPP_INFO(logger, "timeout_ms_send_to_downstream: %lf", this->timeout_ms_send_to_downstream);
}
} // namespace FlowRos2Pipeline