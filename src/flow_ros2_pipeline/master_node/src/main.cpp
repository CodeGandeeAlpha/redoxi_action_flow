#include <rclcpp/rclcpp.hpp>
#include <master_node/master_node.hpp>
#include <rclcpp/executors.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FlowRos2Pipeline::OpencvVideoReader>());
    auto node = std::make_shared<FlowRos2Pipeline::MasterNode>();
    auto init_config = std::make_shared<FlowRos2Pipeline::MasterNode::InitConfig>();
    auto runtime_config = std::make_shared<FlowRos2Pipeline::MasterNode::RuntimeConfig>();
    init_config->from_parameters(node.get());
    runtime_config->from_parameters(node.get());
    node->init(init_config, runtime_config);
    node->open();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}