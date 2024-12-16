#include <psg_tracker/Driver.hpp>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    //! Create a PSGAllDetectorCppNode with a specific name and options
    auto psg_tracker_node = std::make_shared<rdx::PSGTrackerDriver>("psg_tracker_driver");

    //! Initialize the node with default configuration
    auto init_config = std::make_shared<rdx::PSGTrackerDriver::InitConfig_t>();
    init_config->parse_from_node_parameters(init_config.get(), psg_tracker_node.get());
    {
        RDX_INFO_DEV(psg_tracker_node.get(), __func__, false, "{}", "Converting init config to JSON");
        auto init_config_json = JS::serializeStruct(*init_config);
        RDX_INFO_DEV(psg_tracker_node.get(), __func__, false, "InitConfig JSON: {}", init_config_json);
    }
    auto runtime_config = std::make_shared<rdx::PSGTrackerDriver::RuntimeConfig_t>();
    runtime_config->parse_from_node_parameters(runtime_config.get(), psg_tracker_node.get());
    {
        RDX_INFO_DEV(psg_tracker_node.get(), __func__, false, "{}", "Converting runtime config to JSON");
        auto runtime_config_json = JS::serializeStruct(*runtime_config);
        RDX_INFO_DEV(psg_tracker_node.get(), __func__, false, "RuntimeConfig JSON: {}", runtime_config_json);
    }
    psg_tracker_node->init(init_config, runtime_config);

    //! Start the node
    psg_tracker_node->open();
    psg_tracker_node->start();

    //! Keep the node running
    rclcpp::spin(psg_tracker_node);

    //! Stop the node before shutdown
    psg_tracker_node->stop();
    psg_tracker_node->close();

    //! Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
