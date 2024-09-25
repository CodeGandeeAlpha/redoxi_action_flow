#include <master_node/master_node.hpp>
#include <master_node/master_node_types.hpp>

namespace FlowRos2Pipeline
{
void MasterNodeInitConfig::from_parameters(MasterNode *node)
{
    status_query_service = node->get_parameter("status_query_service").as_string();
    process_frame_action = node->get_parameter("process_frame_action").as_string();

    const auto &logger = node->get_logger();
    // RCLCPP_INFO(logger, "status_query_service: %s", this->status_query_service.c_str());
    // RCLCPP_INFO(logger, "process_frame_action: %s", this->process_frame_action.c_str());
}

void MasterNodeRuntimeConfig::from_parameters(MasterNode *node)
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