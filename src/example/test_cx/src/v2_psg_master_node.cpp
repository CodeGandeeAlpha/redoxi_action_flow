#include <psg_master_node/MasterNode.hpp>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rdx::PSGMasterNode>("psg_master_node");

    //! Initialize the node with default configuration
    auto init_config = std::make_shared<rdx::psg_master_node::InitConfig>();
    init_config->from_parameters(node.get());
    auto runtime_config = std::make_shared<rdx::psg_master_node::RuntimeConfig>();
    runtime_config->from_parameters(node.get());
    node->init(init_config, runtime_config);

    //! Start the node
    node->start();

    //! Keep the node running
    rclcpp::spin(node);

    //! Stop the node before shutdown
    node->stop();

    //! Shutdown ROS
    rclcpp::shutdown();
    return 0;
}