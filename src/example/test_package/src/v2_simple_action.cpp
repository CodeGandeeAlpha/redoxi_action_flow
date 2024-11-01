#include <rclcpp/rclcpp.hpp>
#include <redoxi_samples_nodes/generators/SimpleActionGenerator_v2.hpp>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rdx::SimpleActionGenerator_v2>("simple_action_generator_v2");

    // Create and initialize the node
    rdx::SimpleActionGenerator_v2::InitConfig_t init_config;
    rdx::SimpleActionGenerator_v2::RuntimeConfig_t runtime_config;

    init_config.from_parameters(node.get());
    runtime_config.from_parameters(node.get());

    node->init(init_config, runtime_config);
    node->open();
    node->start();

    // Spin the node to process callbacks
    rclcpp::spin(node);

    // Clean up and shutdown
    node->stop();
    node->close();
    rclcpp::shutdown();
    return 0;
}