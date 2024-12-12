#include <psg_counter/PSGCounter.hpp>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rdx::PSGCounter>("psg_counter");

    //! Initialize the node with default configuration
    auto init_config = std::make_shared<rdx::psg_counter::InitConfig>();
    init_config->parse_from_node_parameters(init_config.get(), node.get());
    {
        RDX_INFO_DEV(node.get(), __func__, false, "{}", "Converting init config to JSON");
        auto init_config_json = JS::serializeStruct(*init_config);
        RDX_INFO_DEV(node.get(), __func__, false, "InitConfig JSON: {}", init_config_json);
    }
    auto runtime_config = std::make_shared<rdx::psg_counter::RuntimeConfig>();
    runtime_config->parse_from_node_parameters(runtime_config.get(), node.get());
    {
        RDX_INFO_DEV(node.get(), __func__, false, "{}", "Converting runtime config to JSON");
        auto runtime_config_json = JS::serializeStruct(*runtime_config);
        RDX_INFO_DEV(node.get(), __func__, false, "RuntimeConfig JSON: {}", runtime_config_json);
    }
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