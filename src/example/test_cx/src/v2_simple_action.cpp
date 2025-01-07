#include <rclcpp/rclcpp.hpp>
#include <redoxi_samples_nodes/generators/SimpleActionGenerator.hpp>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rdx::SimpleActionGenerator>("simple_action_generator");

    // Create and initialize the node
    auto init_config = std::make_shared<rdx::SimpleActionGenerator::InitConfig_t>();
    auto runtime_config = std::make_shared<rdx::SimpleActionGenerator::RuntimeConfig_t>();

    init_config->from_parameters(node.get());
    runtime_config->from_parameters(node.get());

    node->init(init_config, runtime_config);
    node->open();
    node->start();

    // Spin the node to process callbacks
    rclcpp::spin(node->get_node_base_interface());

    // Clean up and shutdown
    node->stop();
    node->close();
    rclcpp::shutdown();
    return 0;
}